#!/usr/bin/env python
# -*- coding: utf-8 -*-

import random
import time
import traceback
from collections import defaultdict, deque
import copy
import os
from os import path
import networkx as nx
import numpy as np
import pyquaternion

import rclpy
from rclpy.qos import qos_profile_sensor_data, QoSDurabilityPolicy
from rclpy.action import ActionClient
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters
from geometry_msgs.msg import Pose, Quaternion, PoseStamped
from sensor_msgs.msg import LaserScan
from action_msgs.msg import GoalStatus
import nav_msgs
from nav_msgs.msg import Odometry, Path
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ManageLifecycleNodes
from slam_toolbox.srv import SaveMap

from performance_modelling_py.environment import ground_truth_map_utils
from performance_modelling_py.utils import backup_file_if_exists, print_info, print_error, nanoseconds_to_seconds
from std_msgs.msg import String


class RunFailException(Exception):
    pass


def main(args=None):
    rclpy.init(args=args)

    node = None

    # noinspection PyBroadException
    try:
        node = LocalizationBenchmarkSupervisor()
        node.start_run()
        rclpy.spin(node)

    except KeyboardInterrupt:
        node.ros_shutdown_callback()
    except RunFailException as e:
        print_error(e)
    except Exception:
        print_error(traceback.format_exc())

    finally:
        if node is not None:
            node.end_run()


class LocalizationBenchmarkSupervisor(Node):
    def __init__(self):
        super().__init__('localization_benchmark_supervisor', automatically_declare_parameters_from_overrides=True)

        # topics, services, actions, entities and frames names
        scan_topic = self.get_parameter('scan_topic').value
        ground_truth_pose_topic = self.get_parameter('ground_truth_pose_topic').value
        lifecycle_manager_service = self.get_parameter('lifecycle_manager_service').value
        global_costmap_get_parameters_service = self.get_parameter('global_costmap_get_parameters_service').value
        save_map_service = self.get_parameter('save_map_service').value
        navigate_to_pose_action = self.get_parameter('navigate_to_pose_action').value
        self.fixed_frame = self.get_parameter('fixed_frame').value
        self.robot_entity_name = self.get_parameter('robot_entity_name').value

        # file system paths
        self.run_output_folder = self.get_parameter('run_output_folder').value
        self.benchmark_data_folder = path.join(self.run_output_folder, "benchmark_data")
        self.ground_truth_map_info_path = self.get_parameter("ground_truth_map_info_path").value
        self.map_file_path = path.join(self.benchmark_data_folder, "map")

        # run parameters
        run_timeout = self.get_parameter('run_timeout').value
        self.ground_truth_map = ground_truth_map_utils.GroundTruthMap(self.ground_truth_map_info_path)
        self.goal_tolerance = self.get_parameter('goal_tolerance').value

        # run variables
        self.received_first_scan = False
        self.latest_ground_truth_pose_msg = None
        self.robot_radius = None
        self.initial_pose = None
        self.traversal_path_poses = None
        self.current_goal = None
        self.num_goals = None
        self.goal_sent_count = 0
        self.goal_succeeded_count = 0
        self.goal_failed_count = 0
        self.goal_rejected_count = 0

        # prepare folder structure
        if not path.exists(self.benchmark_data_folder):
            os.makedirs(self.benchmark_data_folder)

        # file paths for benchmark data
        self.run_events_file_path = path.join(self.benchmark_data_folder, "run_events.csv")
        self.init_run_events_file()

        # setup timers
        self.create_timer(run_timeout, self.run_timeout_callback)

        # setup service clients
        self.lifecycle_manager_service_client = self.create_client(ManageLifecycleNodes, lifecycle_manager_service)
        self.global_costmap_get_parameters_service_client = self.create_client(GetParameters, global_costmap_get_parameters_service)
        self.save_map_service_client = self.create_client(SaveMap, save_map_service)

        # setup publishers
        traversal_path_publisher_qos_profile = copy.copy(qos_profile_sensor_data)
        traversal_path_publisher_qos_profile.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        self.traversal_path_publisher = self.create_publisher(Path, "~/traversal_path", traversal_path_publisher_qos_profile)

        # setup subscribers
        self.create_subscription(LaserScan, scan_topic, self.scan_callback, qos_profile_sensor_data)
        self.create_subscription(Odometry, ground_truth_pose_topic, self.ground_truth_pose_callback, qos_profile_sensor_data)

        # setup action clients
        self.navigate_to_pose_action_client = ActionClient(self, NavigateToPose, navigate_to_pose_action)
        self.navigate_to_pose_action_goal_future = None
        self.navigate_to_pose_action_result_future = None

    def start_run(self):
        print_info("preparing to start run")

        # wait to receive sensor data from the environment (e.g., a simulator may need time to startup)
        waiting_time = 0.0
        waiting_period = 0.5
        while not self.received_first_scan and rclpy.ok():
            time.sleep(waiting_period)
            rclpy.spin_once(self)
            waiting_time += waiting_period
            if waiting_time > 5.0:
                self.get_logger().warning('still waiting to receive first sensor message from environment')
                waiting_time = 0.0

        # get the parameter robot_radius from the global costmap
        parameters_request = GetParameters.Request(names=['robot_radius'])
        parameters_response = self.call_service(self.global_costmap_get_parameters_service_client, parameters_request)
        self.robot_radius = parameters_response.values[0].double_value
        print_info("got robot radius")

        # get deleaved reduced Voronoi graph from ground truth map
        voronoi_graph = self.ground_truth_map.deleaved_reduced_voronoi_graph(minimum_radius=2*self.robot_radius).copy()
        minimum_length_paths = nx.all_pairs_dijkstra_path(voronoi_graph, weight='voronoi_path_distance')
        minimum_length_costs = dict(nx.all_pairs_dijkstra_path_length(voronoi_graph, weight='voronoi_path_distance'))
        costs = defaultdict(dict)
        for i, paths_dict in minimum_length_paths:
            for j in paths_dict.keys():
                if i != j:
                    costs[i][j] = minimum_length_costs[i][j]

        # in case the graph has multiple unconnected components, remove the components with less than two nodes
        too_small_voronoi_graph_components = list(filter(lambda component: len(component) < 2, nx.connected_components(voronoi_graph)))

        for graph_component in too_small_voronoi_graph_components:
            voronoi_graph.remove_nodes_from(graph_component)

        if len(voronoi_graph.nodes) < 2:
            self.write_event(self.get_clock().now(), 'insufficient_number_of_nodes_in_deleaved_reduced_voronoi_graph')
            raise RunFailException("insufficient number of nodes in deleaved_reduced_voronoi_graph, can not generate traversal path")

        # get greedy path traversing the whole graph starting from a random node
        traversal_path_indices = list()
        current_node = random.choice(list(voronoi_graph.nodes))
        nodes_queue = set(nx.node_connected_component(voronoi_graph, current_node))
        while len(nodes_queue):
            candidates = list(filter(lambda node_cost: node_cost[0] in nodes_queue, costs[current_node].items()))
            candidate_nodes, candidate_costs = zip(*candidates)
            next_node = candidate_nodes[int(np.argmin(candidate_costs))]
            traversal_path_indices.append(next_node)
            current_node = next_node
            nodes_queue.remove(next_node)

        # convert path of nodes to list of poses (as deque so they can be popped)
        self.traversal_path_poses = deque()
        for node_index in traversal_path_indices:
            pose = Pose()
            pose.position.x, pose.position.y = voronoi_graph.nodes[node_index]['vertex']
            q = pyquaternion.Quaternion(axis=[0, 0, 1], radians=np.random.uniform(-np.pi, np.pi))
            pose.orientation = Quaternion(w=q.w, x=q.x, y=q.y, z=q.z)
            self.traversal_path_poses.append(pose)

        # publish the traversal path for visualization
        traversal_path_msg = Path()
        traversal_path_msg.header.frame_id = self.fixed_frame
        traversal_path_msg.header.stamp = self.get_clock().now().to_msg()
        for traversal_pose in self.traversal_path_poses:
            traversal_pose_stamped = PoseStamped()
            traversal_pose_stamped.header = traversal_path_msg.header
            traversal_pose_stamped.pose = traversal_pose
            traversal_path_msg.poses.append(traversal_pose_stamped)
        self.traversal_path_publisher.publish(traversal_path_msg)

        self.num_goals = len(self.traversal_path_poses)

        # ask lifecycle_manager to startup all its managed nodes
        startup_request = ManageLifecycleNodes.Request(command=ManageLifecycleNodes.Request.STARTUP)
        startup_response: ManageLifecycleNodes.Response = self.call_service(self.lifecycle_manager_service_client, startup_request)
        print_info("called lifecycle_manager_service_client", startup_response)
        if not startup_response.success:
            self.write_event(self.get_clock().now(), 'failed_to_startup_nodes')
            raise RunFailException("lifecycle manager could not startup nodes")

        self.write_event(self.get_clock().now(), 'run_start')

        self.send_goal()

    def send_goal(self):
        print_info(f"goal {self.goal_sent_count + 1} / {self.num_goals}")

        if not self.navigate_to_pose_action_client.wait_for_server(timeout_sec=5.0):
            self.write_event(self.get_clock().now(), 'failed_to_communicate_with_navigation_node')
            raise RunFailException("navigate_to_pose action server not available")

        if len(self.traversal_path_poses) == 0:
            self.write_event(self.get_clock().now(), 'insufficient_number_of_poses_in_traversal_path')
            raise RunFailException("insufficient number of poses in traversal path, can not send goal")

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.header.frame_id = self.fixed_frame
        goal_msg.pose.pose = self.traversal_path_poses.popleft()
        self.current_goal = goal_msg

        self.navigate_to_pose_action_goal_future = self.navigate_to_pose_action_client.send_goal_async(goal_msg)
        self.navigate_to_pose_action_goal_future.add_done_callback(self.goal_response_callback)
        self.write_event(self.get_clock().now(), 'target_pose_set')
        self.goal_sent_count += 1

    def ros_shutdown_callback(self):
        """
        This function is called when the node receives an interrupt signal (KeyboardInterrupt).
        """
        print_info("asked to shutdown, terminating run")
        self.write_event(self.get_clock().now(), 'ros_shutdown')
        self.write_event(self.get_clock().now(), 'supervisor_finished')

    def end_run(self):
        """
        This function is called after the run has completed, whether the run finished correctly, or there was an exception.
        The only case in which this function is not called is if an exception was raised from self.__init__
        """
        pass

    def goal_response_callback(self, future):
        goal_handle = future.result()

        # if the goal is rejected try with the next goal
        if not goal_handle.accepted:
            print_error('navigation action goal rejected')
            self.write_event(self.get_clock().now(), 'target_pose_rejected')
            self.goal_rejected_count += 1
            self.current_goal = None
            self.send_goal()
            return

        self.write_event(self.get_clock().now(), 'target_pose_accepted')

        self.navigate_to_pose_action_result_future = goal_handle.get_result_async()
        self.navigate_to_pose_action_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        status = future.result().status

        if status == GoalStatus.STATUS_SUCCEEDED:
            goal_position = self.current_goal.pose.pose.position
            current_position = self.latest_ground_truth_pose_msg.pose.pose.position
            distance_from_goal = np.sqrt((goal_position.x - current_position.x) ** 2 + (goal_position.y - current_position.y) ** 2)
            if distance_from_goal < self.goal_tolerance:
                self.write_event(self.get_clock().now(), 'target_pose_reached')
                self.goal_succeeded_count += 1
            else:
                print_error("goal status succeeded but current position farther from goal position than tolerance")
                self.write_event(self.get_clock().now(), 'target_pose_not_reached')
                self.goal_failed_count += 1
        else:
            print_info('navigation action failed with status {}'.format(status))
            self.write_event(self.get_clock().now(), 'target_pose_not_reached')
            self.goal_failed_count += 1

        self.current_goal = None

        # if all goals have been sent end the run, otherwise send the next goal
        if len(self.traversal_path_poses) == 0:
            self.save_map_and_finish_run()
        else:
            self.send_goal()

    def save_map_and_finish_run(self):
        self.write_event(self.get_clock().now(), 'save_map_request_sent')
        self.call_service(self.save_map_service_client, SaveMap.Request(name=String(data=self.map_file_path)))
        self.write_event(self.get_clock().now(), 'run_completed')
        rclpy.shutdown()

    def run_timeout_callback(self):
        print_error("terminating supervisor due to timeout, terminating run")
        self.write_event(self.get_clock().now(), 'run_timeout')
        self.write_event(self.get_clock().now(), 'supervisor_finished')
        raise RunFailException("timeout")

    def scan_callback(self, _):
        self.received_first_scan = True

    def ground_truth_pose_callback(self, odometry_msg: nav_msgs.msg.Odometry):
        self.latest_ground_truth_pose_msg = odometry_msg

    def init_run_events_file(self):
        backup_file_if_exists(self.run_events_file_path)
        try:
            with open(self.run_events_file_path, 'w') as run_events_file:
                run_events_file.write("{t}, {event}\n".format(t='timestamp', event='event'))
        except IOError as e:
            self.get_logger().error("slam_benchmark_supervisor.init_event_file: could not write header to run_events_file")
            self.get_logger().error(e)

    def write_event(self, stamp, event):
        print_info("t: {t}, event: {event}".format(t=nanoseconds_to_seconds(stamp.nanoseconds), event=str(event)))
        try:
            with open(self.run_events_file_path, 'a') as run_events_file:
                run_events_file.write("{t}, {event}\n".format(t=nanoseconds_to_seconds(stamp.nanoseconds), event=str(event)))
        except IOError as e:
            self.get_logger().error("slam_benchmark_supervisor.write_event: could not write event to run_events_file: {t} {event}".format(t=nanoseconds_to_seconds(stamp.nanoseconds), event=str(event)))
            self.get_logger().error(e)

    def call_service(self, service_client, request, fail_timeout=30.0, warning_timeout=5.0):
        time_waited = 0.0
        while not service_client.wait_for_service(timeout_sec=warning_timeout) and rclpy.ok():
            self.get_logger().warning(f'supervisor: still waiting {service_client.srv_name} to become available')
            time_waited += warning_timeout
            if time_waited >= fail_timeout:
                raise RunFailException(f"{service_client.srv_name} was not available")

        srv_future = service_client.call_async(request)
        rclpy.spin_until_future_complete(self, srv_future)
        return srv_future.result()
