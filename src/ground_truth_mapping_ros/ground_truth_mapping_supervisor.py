#!/usr/bin/env python
# -*- coding: utf-8 -*-

import random
import time
import traceback
from collections import defaultdict
import os
import copy
from os import path
import networkx as nx
import numpy as np
import pyquaternion

import rospy
import tf2_ros
from actionlib import SimpleActionClient
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Quaternion, PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, Path
from slam_toolbox_msgs.srv import SaveMap, SerializePoseGraph

from performance_modelling_py.environment import ground_truth_map
from performance_modelling_py.utils import backup_file_if_exists, print_info, print_error
from slam_toolbox_msgs.srv import SaveMapRequest, SerializePoseGraphRequest
from std_msgs.msg import String


class RunFailException(Exception):
    pass


def main():
    rospy.init_node('slam_benchmark_supervisor', anonymous=False)

    node = None

    # noinspection PyBroadException
    try:
        node = GroundTruthMappingSupervisor()
        node.start_run()
        rospy.spin()

    except KeyboardInterrupt:
        node.ros_shutdown_callback()
    except RunFailException as e:
        print_error(e)
    except Exception:
        print_error(traceback.format_exc())

    finally:
        if node is not None:
            node.end_run()
        if not rospy.is_shutdown():
            print_info("calling rospy signal_shutdown")
            rospy.signal_shutdown("run_terminated")


class GroundTruthMappingSupervisor:
    def __init__(self):

        # topics, services, actions, entities and frames names
        scan_topic = rospy.get_param('~scan_topic')
        ground_truth_pose_topic = rospy.get_param('~ground_truth_pose_topic')
        save_map_service = rospy.get_param('~save_map_service')
        serialize_map_service = rospy.get_param('~serialize_map_service')
        navigate_to_pose_action = rospy.get_param('~navigate_to_pose_action')
        self.fixed_frame = rospy.get_param('~fixed_frame')
        self.mapper_odom_frame = rospy.get_param('~mapper_odom_frame')
        self.gt_base_frame = rospy.get_param('~gt_base_frame')
        self.mapper_base_frame = rospy.get_param('~mapper_base_frame')

        # file system paths
        self.run_output_folder = rospy.get_param('~run_output_folder')
        self.benchmark_data_folder = path.join(self.run_output_folder, "benchmark_data")
        self.ground_truth_map_info_path = rospy.get_param('~ground_truth_map_info_path')
        self.output_map_file_path = rospy.get_param('~output_map_file_path')
        self.output_pose_graph_file_path = rospy.get_param('~output_pose_graph_file_path')

        # run parameters
        run_timeout = rospy.get_param('~run_timeout')
        self.waypoint_timeout = rospy.get_param('~waypoint_timeout')
        self.ground_truth_map = ground_truth_map.GroundTruthMap(self.ground_truth_map_info_path)
        self.goal_tolerance = rospy.get_param('~goal_tolerance')
        self.robot_radius = rospy.get_param('~robot_radius')

        # run variables
        self.received_first_scan = False
        self.latest_ground_truth_pose_msg = None
        self.traversal_path_poses = None
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
        rospy.Timer(rospy.Duration.from_sec(run_timeout), self.run_timeout_callback)
        rospy.Timer(rospy.Duration.from_sec(5.0), self.odom_error_timer_callback)

        # setup service clients
        self.save_map_service_client = rospy.ServiceProxy(save_map_service, SaveMap)
        self.serialize_map_client = rospy.ServiceProxy(serialize_map_service, SerializePoseGraph)

        # setup buffers
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # setup publishers
        self.traversal_path_publisher = rospy.Publisher("~/traversal_path", Path, queue_size=1)

        # setup subscribers
        rospy.Subscriber(scan_topic, LaserScan, self.scan_callback, queue_size=1)
        rospy.Subscriber(ground_truth_pose_topic, Odometry, self.ground_truth_pose_callback, queue_size=1)

        # setup action clients
        self.navigate_to_pose_action_client = SimpleActionClient(navigate_to_pose_action, MoveBaseAction)

    def start_run(self):
        print_info("preparing to start run")

        # wait to receive sensor data from the environment (e.g., a simulator may need time to startup)
        waiting_time = 0.0
        waiting_period = 0.5
        while not self.received_first_scan and not rospy.is_shutdown():
            time.sleep(waiting_period)
            waiting_time += waiting_period
            if waiting_time > 5.0:
                rospy.logwarn('still waiting to receive first sensor message from environment')
                waiting_time = 0.0

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
            self.write_event('insufficient_number_of_nodes_in_deleaved_reduced_voronoi_graph')
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

        # convert path of nodes to list of poses
        self.traversal_path_poses = list()
        for node_index in traversal_path_indices:
            pose = Pose()
            pose.position.x, pose.position.y = voronoi_graph.nodes[node_index]['vertex']
            q = pyquaternion.Quaternion(axis=[0, 0, 1], radians=np.random.uniform(-np.pi, np.pi))
            pose.orientation = Quaternion(w=q.w, x=q.x, y=q.y, z=q.z)
            self.traversal_path_poses.append(pose)

        # add the reversed path to make sure all parts of the map are visited in both directions
        reversed_traversal_path_poses = copy.deepcopy(self.traversal_path_poses[::-1])
        for pose in reversed_traversal_path_poses:
            q = pyquaternion.Quaternion(axis=[0, 0, 1], radians=np.random.uniform(-np.pi, np.pi))
            pose.orientation = Quaternion(w=q.w, x=q.x, y=q.y, z=q.z)
        self.traversal_path_poses += reversed_traversal_path_poses

        if len(self.traversal_path_poses) < 2:
            self.write_event('insufficient_number_of_poses_in_traversal_path')
            raise RunFailException("insufficient number of poses in traversal path, can not send goal")

        self.publish_traversal_path()

        self.num_goals = len(self.traversal_path_poses)

        self.write_event('run_start')

        # send goals
        for traversal_path_pose in self.traversal_path_poses:
            print_info("goal {} / {}".format(self.goal_sent_count + 1, self.num_goals))

            if rospy.is_shutdown():
                break

            if not self.navigate_to_pose_action_client.wait_for_server(timeout=rospy.Duration.from_sec(5.0)):
                self.write_event('failed_to_communicate_with_navigation_node')
                raise RunFailException("navigate_to_pose action server not available")

            goal_msg = MoveBaseGoal()
            goal_msg.target_pose.header.stamp = rospy.Time.now()
            goal_msg.target_pose.header.frame_id = self.fixed_frame
            goal_msg.target_pose.pose = traversal_path_pose

            self.navigate_to_pose_action_client.send_goal(goal_msg)
            self.write_event('target_pose_set')
            self.goal_sent_count += 1

            if not self.navigate_to_pose_action_client.wait_for_result(timeout=rospy.Duration.from_sec(self.waypoint_timeout)):
                self.write_event('waypoint_timeout')
                self.write_event('supervisor_finished')
                raise RunFailException("waypoint_timeout")

            if self.navigate_to_pose_action_client.get_state() == GoalStatus.SUCCEEDED:
                goal_position = goal_msg.target_pose.pose.position
                current_position = self.latest_ground_truth_pose_msg.pose.pose.position
                distance_from_goal = np.sqrt((goal_position.x - current_position.x) ** 2 + (goal_position.y - current_position.y) ** 2)
                if distance_from_goal < self.goal_tolerance:
                    self.write_event('target_pose_reached')
                    self.goal_succeeded_count += 1
                else:
                    print_error("goal status succeeded but current position farther from goal position than tolerance")
                    self.write_event('target_pose_not_reached')
                    self.goal_failed_count += 1
            else:
                print_info('navigation action failed with status {}, {}'.format(self.navigate_to_pose_action_client.get_state(), self.navigate_to_pose_action_client.get_goal_status_text()))
                self.write_event('target_pose_not_reached')
                self.goal_failed_count += 1
                if self.goal_failed_count < 10:
                    self.traversal_path_poses.append(traversal_path_pose)
                    self.publish_traversal_path()

            rospy.sleep(1.0)

        # if all goals have been sent end the run
        self.write_event('run_completed')
        self.save_map()
        rospy.signal_shutdown("run_completed")

    def publish_traversal_path(self):
        traversal_path_msg = Path()
        traversal_path_msg.header.frame_id = self.fixed_frame
        traversal_path_msg.header.stamp = rospy.Time.now()
        for traversal_pose in self.traversal_path_poses:
            traversal_pose_stamped = PoseStamped()
            traversal_pose_stamped.header = traversal_path_msg.header
            traversal_pose_stamped.pose = traversal_pose
            traversal_path_msg.poses.append(traversal_pose_stamped)
        self.traversal_path_publisher.publish(traversal_path_msg)

    def ros_shutdown_callback(self):
        """
        This function is called when the node receives an interrupt signal (KeyboardInterrupt).
        """
        print_info("asked to shutdown, terminating run")
        self.write_event('ros_shutdown')
        self.write_event('supervisor_finished')

    @staticmethod
    def end_run():
        """
        This function is called after the run has completed, whether the run finished correctly, or there was an exception.
        The only case in which this function is not called is if an exception was raised from self.__init__
        """
        print_info("end_run: nothing to do")

    def save_map(self):
        print_info("sending save map request")
        self.save_map_service_client.call(SaveMapRequest(name=String(data=self.output_map_file_path)))
        self.serialize_map_client.call(SerializePoseGraphRequest(filename=self.output_pose_graph_file_path))
        self.write_event('save_map_request_sent')

    def run_timeout_callback(self, _):
        print_error("terminating supervisor due to timeout, terminating run")
        self.write_event('run_timeout')
        self.write_event('supervisor_finished')
        raise RunFailException("run_timeout")

    def odom_error_timer_callback(self, _):
        # noinspection PyBroadException
        try:
            transform_msg = self.tf_buffer.lookup_transform(self.gt_base_frame, self.mapper_base_frame, rospy.Time())
            x, y = transform_msg.transform.translation.x, transform_msg.transform.translation.y
            orientation = transform_msg.transform.rotation
            theta, _, _ = pyquaternion.Quaternion(x=orientation.x, y=orientation.y, z=orientation.z, w=orientation.w).yaw_pitch_roll

            r = self.ground_truth_map.resolution/2
            if x > r or y > r or theta > 0.01:
                print_error("map distortion error: x={x:0.4f}, y={y:0.4f}, theta={theta:0.4f}".format(x=x, y=y, theta=theta))
            else:
                print("map distortion error: x={x:0.4f}, y={y:0.4f}, theta={theta:0.4f}".format(x=x, y=y, theta=theta))

        # except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        except:
            print_error(traceback.format_exc())

    def scan_callback(self, _):
        self.received_first_scan = True

    def ground_truth_pose_callback(self, odometry_msg):
        self.latest_ground_truth_pose_msg = odometry_msg

    def init_run_events_file(self):
        backup_file_if_exists(self.run_events_file_path)
        try:
            with open(self.run_events_file_path, 'w') as run_events_file:
                run_events_file.write("{t}, {event}\n".format(t='timestamp', event='event'))
        except IOError:
            rospy.logerr("ground_truth_mapping_supervisor.init_event_file: could not write header to run_events_file")
            rospy.logerr(traceback.format_exc())

    def write_event(self, event):
        t = rospy.Time.now().to_sec()
        print_info("t: {t}, event: {event}".format(t=t, event=str(event)))
        try:
            with open(self.run_events_file_path, 'a') as run_events_file:
                run_events_file.write("{t}, {event}\n".format(t=t, event=str(event)))
        except IOError:
            rospy.logerr("ground_truth_mapping_supervisor.write_event: could not write event to run_events_file: {t} {event}".format(t=t, event=str(event)))
            rospy.logerr(traceback.format_exc())
