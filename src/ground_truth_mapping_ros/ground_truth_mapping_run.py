#!/usr/bin/python
# -*- coding: utf-8 -*-

from __future__ import print_function

import os
import shutil

import yaml
from xml.etree import ElementTree as et
import time
from os import path
import numpy as np

from performance_modelling_py.utils import backup_file_if_exists, print_info, print_error
from performance_modelling_py.component_proxies.ros1_component import Component


class BenchmarkRun(object):
    def __init__(self, run_id, run_output_folder, benchmark_log_path, environment_folder, parameters_combination_dict, benchmark_configuration_dict, show_ros_info, headless):

        # run configuration
        self.run_id = run_id
        self.benchmark_log_path = benchmark_log_path
        self.run_parameters = parameters_combination_dict
        self.benchmark_configuration = benchmark_configuration_dict
        self.components_ros_output = 'screen' if show_ros_info else 'log'
        self.headless = headless

        # environment parameters
        self.environment_folder = environment_folder
        self.map_info_file_path = path.join(environment_folder, "data", "map.yaml")
        laser_scan_max_range = self.run_parameters['laser_scan_max_range']
        laser_scan_fov_deg = self.run_parameters['laser_scan_fov_deg']
        laser_scan_fov_rad = (laser_scan_fov_deg-1)*np.pi/180
        map_resolution = self.run_parameters['map_resolution']
        self.run_output_folder = path.join(environment_folder, "data", "realistic_map", "res_{res}_fov_{fov}_max_range_{max_range}_no_scan_matching".format(res=map_resolution, fov=laser_scan_fov_deg, max_range=laser_scan_max_range))
        backup_file_if_exists(self.run_output_folder)  # backup data to avoid overwriting previous maps, since we are saving the data to the dataset

        self.gazebo_model_path_env_var = ":".join(map(
            lambda p: path.expanduser(p),
            self.benchmark_configuration['gazebo_model_path_env_var'] + [path.dirname(path.abspath(self.environment_folder)), self.run_output_folder]
        ))
        self.gazebo_resource_path_env_var = ":".join(map(
            lambda p: path.expanduser(p),
            self.benchmark_configuration['gazebo_resource_path_env_var']
        ))
        self.gazebo_plugin_path_env_var = ":".join(map(
            lambda p: path.expanduser(p),
            self.benchmark_configuration['gazebo_plugin_path_env_var']
        ))

        # run variables
        self.aborted = False

        # prepare folder structure
        run_configuration_path = path.join(self.run_output_folder, "components_configuration")
        run_info_file_path = path.join(self.run_output_folder, "run_info.yaml")
        backup_file_if_exists(self.run_output_folder)
        os.makedirs(self.run_output_folder)
        os.mkdir(run_configuration_path)
        output_map_file_path = path.join(self.run_output_folder, "map")
        output_pose_graph_file_path = path.join(self.run_output_folder, "pose_graph")

        # components original configuration paths
        components_configurations_folder = path.expanduser(self.benchmark_configuration['components_configurations_folder'])
        original_supervisor_configuration_path = path.join(components_configurations_folder, self.benchmark_configuration['components_configuration']['supervisor'])
        original_slam_toolbox_configuration_path = path.join(components_configurations_folder, self.benchmark_configuration['components_configuration']['slam_toolbox'])
        original_move_base_configuration_path = path.join(components_configurations_folder, self.benchmark_configuration['components_configuration']['move_base'])
        # self.original_rviz_configuration_path = path.join(components_configurations_folder, self.benchmark_configuration['components_configuration']['rviz'])
        original_gazebo_world_model_path = path.join(environment_folder, "gazebo", "gazebo_environment.model")
        original_gazebo_robot_model_config_path = path.join(environment_folder, "gazebo", "robot", "model.config")
        original_gazebo_robot_model_sdf_path = path.join(environment_folder, "gazebo", "robot", "model.sdf")
        original_robot_urdf_path = path.join(environment_folder, "gazebo", "robot.urdf")

        # components configuration relative paths
        supervisor_configuration_relative_path = path.join("components_configuration", self.benchmark_configuration['components_configuration']['supervisor'])
        slam_toolbox_configuration_relative_path = path.join("components_configuration", self.benchmark_configuration['components_configuration']['slam_toolbox'])
        move_base_configuration_relative_path = path.join("components_configuration", self.benchmark_configuration['components_configuration']['move_base'])
        gazebo_world_model_relative_path = path.join("components_configuration", "gazebo", "gazebo_environment.model")
        gazebo_robot_model_config_relative_path = path.join("components_configuration", "gazebo", "robot", "model.config")
        gazebo_robot_model_sdf_relative_path = path.join("components_configuration", "gazebo", "robot", "model.sdf")
        robot_gt_mb_urdf_relative_path = path.join("components_configuration", "gazebo", "robot_gt_mb.urdf")
        robot_gt_st_urdf_relative_path = path.join("components_configuration", "gazebo", "robot_gt_st.urdf")

        # components configuration paths in run folder
        self.supervisor_configuration_path = path.join(self.run_output_folder, supervisor_configuration_relative_path)
        self.slam_toolbox_configuration_path = path.join(self.run_output_folder, slam_toolbox_configuration_relative_path)
        self.move_base_configuration_path = path.join(self.run_output_folder, move_base_configuration_relative_path)
        self.gazebo_world_model_path = path.join(self.run_output_folder, gazebo_world_model_relative_path)
        gazebo_robot_model_config_path = path.join(self.run_output_folder, gazebo_robot_model_config_relative_path)
        gazebo_robot_model_sdf_path = path.join(self.run_output_folder, gazebo_robot_model_sdf_relative_path)
        self.robot_gt_mb_urdf_path = path.join(self.run_output_folder, robot_gt_mb_urdf_relative_path)
        self.robot_gt_st_urdf_path = path.join(self.run_output_folder, robot_gt_st_urdf_relative_path)

        # copy the configuration of the supervisor to the run folder and update its parameters
        with open(original_supervisor_configuration_path) as supervisor_configuration_file:
            supervisor_configuration = yaml.load(supervisor_configuration_file)
        supervisor_configuration['run_output_folder'] = self.run_output_folder
        supervisor_configuration['ground_truth_map_info_path'] = self.map_info_file_path
        supervisor_configuration['output_map_file_path'] = output_map_file_path
        supervisor_configuration['output_pose_graph_file_path'] = output_pose_graph_file_path
        if not path.exists(path.dirname(self.supervisor_configuration_path)):
            os.makedirs(path.dirname(self.supervisor_configuration_path))
        with open(self.supervisor_configuration_path, 'w') as supervisor_configuration_file:
            yaml.dump(supervisor_configuration, supervisor_configuration_file, default_flow_style=False)

        # copy the configuration of slam_toolbox to the run folder and update its parameters
        with open(original_slam_toolbox_configuration_path) as original_slam_toolbox_configuration_file:
            slam_toolbox_configuration = yaml.load(original_slam_toolbox_configuration_file)
        slam_toolbox_configuration['resolution'] = map_resolution
        if not path.exists(path.dirname(self.slam_toolbox_configuration_path)):
            os.makedirs(path.dirname(self.slam_toolbox_configuration_path))
        with open(self.slam_toolbox_configuration_path, 'w') as slam_toolbox_configuration_file:
            yaml.dump(slam_toolbox_configuration, slam_toolbox_configuration_file, default_flow_style=False)

        # copy the configuration of move_base to the run folder
        if not path.exists(path.dirname(self.move_base_configuration_path)):
            os.makedirs(path.dirname(self.move_base_configuration_path))
        shutil.copyfile(original_move_base_configuration_path, self.move_base_configuration_path)

        # copy the configuration of the gazebo world model to the run folder and update its parameters
        gazebo_original_world_model_tree = et.parse(original_gazebo_world_model_path)
        gazebo_original_world_model_root = gazebo_original_world_model_tree.getroot()
        gazebo_original_world_model_root.findall(".//include[@include_id='robot_model']/uri")[0].text = path.join("model://", path.dirname(gazebo_robot_model_sdf_relative_path))
        if not path.exists(path.dirname(self.gazebo_world_model_path)):
            os.makedirs(path.dirname(self.gazebo_world_model_path))
        gazebo_original_world_model_tree.write(self.gazebo_world_model_path)

        # copy the configuration of the gazebo robot sdf model to the run folder and update its parameters
        gazebo_robot_model_sdf_tree = et.parse(original_gazebo_robot_model_sdf_path)
        gazebo_robot_model_sdf_root = gazebo_robot_model_sdf_tree.getroot()
        # realistic sensor, for slam_toolbox (slam_toolbox needs some noise in the lidar to make a decent map)
        gazebo_robot_model_sdf_root.findall(".//sensor[@name='lidar_sensor']/ray/scan/horizontal/samples")[0].text = str(int(laser_scan_fov_deg))
        gazebo_robot_model_sdf_root.findall(".//sensor[@name='lidar_sensor']/ray/scan/horizontal/min_angle")[0].text = str(float(-laser_scan_fov_rad/2))
        gazebo_robot_model_sdf_root.findall(".//sensor[@name='lidar_sensor']/ray/scan/horizontal/max_angle")[0].text = str(float(+laser_scan_fov_rad/2))
        gazebo_robot_model_sdf_root.findall(".//sensor[@name='lidar_sensor']/ray/range/max")[0].text = str(float(laser_scan_max_range))
        gazebo_robot_model_sdf_root.findall(".//sensor[@name='lidar_sensor']/plugin[@name='turtlebot3_laserscan_realistic']/topicName")[0].text = "scan_gt_st"
        gazebo_robot_model_sdf_root.findall(".//sensor[@name='lidar_sensor']/plugin[@name='turtlebot3_laserscan_realistic']/frameName")[0].text = "base_scan_gt_st"
        # gt sensor, for move_base
        gazebo_robot_model_sdf_root.findall(".//sensor[@name='lidar_sensor_gt']/plugin[@name='turtlebot3_laserscan_gt']/topicName")[0].text = "scan_gt_mb"
        gazebo_robot_model_sdf_root.findall(".//sensor[@name='lidar_sensor_gt']/plugin[@name='turtlebot3_laserscan_gt']/frameName")[0].text = "base_scan_gt_mb"
        # ideal odometry, for slam_toolbox
        gazebo_robot_model_sdf_root.findall(".//plugin[@name='turtlebot3_diff_drive']/odometrySource")[0].text = "world"
        gazebo_robot_model_sdf_root.findall(".//plugin[@name='turtlebot3_diff_drive']/odometryTopic")[0].text = "odom"
        gazebo_robot_model_sdf_root.findall(".//plugin[@name='turtlebot3_diff_drive']/odometryFrame")[0].text = "odom_gt_st"
        gazebo_robot_model_sdf_root.findall(".//plugin[@name='turtlebot3_diff_drive']/robotBaseFrame")[0].text = "base_footprint_gt_st"
        # gt odometry, for move_base
        gazebo_robot_model_sdf_root.findall(".//plugin[@name='turtlebot3_diff_drive']/groundTruthParentFrame")[0].text = "odom_gt_mb"
        gazebo_robot_model_sdf_root.findall(".//plugin[@name='turtlebot3_diff_drive']/groundTruthRobotBaseFrame")[0].text = "base_footprint_gt_mb"
        if not path.exists(path.dirname(gazebo_robot_model_sdf_path)):
            os.makedirs(path.dirname(gazebo_robot_model_sdf_path))
        gazebo_robot_model_sdf_tree.write(gazebo_robot_model_sdf_path)

        # copy the configuration of the gazebo robot model to the run folder
        if not path.exists(path.dirname(gazebo_robot_model_config_path)):
            os.makedirs(path.dirname(gazebo_robot_model_config_path))
        shutil.copyfile(original_gazebo_robot_model_config_path, gazebo_robot_model_config_path)

        # copy the configuration of the robot urdf to the run folder and update the link names for ground truth move_base data
        robot_gt_mb_urdf_tree = et.parse(original_robot_urdf_path)
        robot_gt_mb_urdf_root = robot_gt_mb_urdf_tree.getroot()
        for link_element in robot_gt_mb_urdf_root.findall(".//link"):
            # if link_element.attrib['name'] == "base_link":
            #     continue
            link_element.attrib['name'] = "{}_gt_mb".format(link_element.attrib['name'])
        for joint_link_element in robot_gt_mb_urdf_root.findall(".//*[@link]"):
            # if joint_link_element.attrib['link'] == "base_link":
            #     continue
            joint_link_element.attrib['link'] = "{}_gt_mb".format(joint_link_element.attrib['link'])
        if not path.exists(path.dirname(self.robot_gt_mb_urdf_path)):
            os.makedirs(path.dirname(self.robot_gt_mb_urdf_path))
        robot_gt_mb_urdf_tree.write(self.robot_gt_mb_urdf_path)

        # copy the configuration of the robot urdf to the run folder and update the link names for ideal slam_toolbox data
        robot_gt_st_urdf_tree = et.parse(original_robot_urdf_path)
        robot_gt_st_urdf_root = robot_gt_st_urdf_tree.getroot()
        for link_element in robot_gt_st_urdf_root.findall(".//link"):
            link_element.attrib['name'] = "{}_gt_st".format(link_element.attrib['name'])
        for joint_link_element in robot_gt_st_urdf_root.findall(".//*[@link]"):
            joint_link_element.attrib['link'] = "{}_gt_st".format(joint_link_element.attrib['link'])
        if not path.exists(path.dirname(self.robot_gt_st_urdf_path)):
            os.makedirs(path.dirname(self.robot_gt_st_urdf_path))
        robot_gt_st_urdf_tree.write(self.robot_gt_st_urdf_path)

        # write run info to file
        run_info_dict = dict()
        run_info_dict["run_id"] = self.run_id
        run_info_dict["run_folder"] = self.run_output_folder
        run_info_dict["environment_folder"] = environment_folder
        run_info_dict["run_parameters"] = self.run_parameters
        run_info_dict["local_components_configuration"] = {
            'supervisor': supervisor_configuration_relative_path,
            'slam_toolbox': slam_toolbox_configuration_relative_path,
            'move_base': move_base_configuration_relative_path,
            'gazebo_world_model': gazebo_world_model_relative_path,
            'gazebo_robot_model_sdf': gazebo_robot_model_sdf_relative_path,
            'gazebo_robot_model_config': gazebo_robot_model_config_relative_path,
            'robot_gt_mb_urdf': robot_gt_mb_urdf_relative_path,
            'robot_gt_st_urdf': robot_gt_st_urdf_relative_path,
        }

        with open(run_info_file_path, 'w') as run_info_file:
            yaml.dump(run_info_dict, run_info_file, default_flow_style=False)

    def log(self, event):

        if not path.exists(self.benchmark_log_path):
            with open(self.benchmark_log_path, 'a') as output_file:
                output_file.write("timestamp, run_id, event\n")

        t = time.time()

        print_info("t: {t}, run: {run_id}, event: {event}".format(t=t, run_id=self.run_id, event=event))
        try:
            with open(self.benchmark_log_path, 'a') as output_file:
                output_file.write("{t}, {run_id}, {event}\n".format(t=t, run_id=self.run_id, event=event))
        except IOError as e:
            print_error("benchmark_log: could not write event to file: {t}, {run_id}, {event}".format(t=t, run_id=self.run_id, event=event))
            print_error(e)

    def execute_run(self):

        # components parameters
        rviz_params = {
            'headless': self.headless,
        }
        ground_truth_map_server_params = {
            'map': self.map_info_file_path,
        }
        environment_params = {
            'world_model_file': self.gazebo_world_model_path,
            'robot_gt_urdf_file': self.robot_gt_mb_urdf_path,
            'robot_realistic_urdf_file': self.robot_gt_st_urdf_path,
            'headless': True,
        }
        slam_toolbox_params = {
            'params_file': self.slam_toolbox_configuration_path,
        }
        navigation_params = {
            'params_file': self.move_base_configuration_path,
        }
        supervisor_params = {
            'params_file': self.supervisor_configuration_path,
        }
        recorder_benchmark_data_params = {
            'bag_file_path': path.join(self.run_output_folder, "benchmark_data.bag"),
            'topics': "/base_footprint_gt /cmd_vel /initialpose /map_gt /map_gt_metadata /map_gt_updates /map /map_metadata /map_updates /odom /particlecloud /gmapping/entropy /rosout /rosout_agg /scan /scan_gt /tf /tf_static /traversal_path",
        }

        # declare components
        roscore = Component('roscore', 'ground_truth_mapping', 'roscore.launch')
        environment = Component('gazebo', 'ground_truth_mapping', 'gazebo.launch', environment_params)
        ground_truth_map_server = Component('ground_truth_map_server', 'ground_truth_mapping', 'ground_truth_map_server.launch', ground_truth_map_server_params)
        rviz = Component('rviz', 'ground_truth_mapping', 'rviz.launch', rviz_params)
        recorder_benchmark_data = Component('recorder_sensor_data', 'ground_truth_mapping', 'rosbag_recorder.launch', recorder_benchmark_data_params)
        slam_toolbox = Component('slam_toolbox', 'ground_truth_mapping', 'slam_toolbox.launch', slam_toolbox_params)
        navigation = Component('move_base', 'ground_truth_mapping', 'move_base.launch', navigation_params)
        supervisor = Component('supervisor', 'ground_truth_mapping', 'ground_truth_mapping_supervisor.launch', supervisor_params)

        # set gazebo's environment variables
        os.environ['GAZEBO_MODEL_PATH'] = self.gazebo_model_path_env_var
        os.environ['GAZEBO_RESOURCE_PATH'] = self.gazebo_resource_path_env_var
        os.environ['GAZEBO_PLUGIN_PATH'] = self.gazebo_plugin_path_env_var

        # launch components
        roscore.launch()
        environment.launch()
        ground_truth_map_server.launch()
        rviz.launch()
        recorder_benchmark_data.launch()
        slam_toolbox.launch()
        navigation.launch()
        supervisor.launch()

        # launch components and wait for the supervisor to finish
        self.log(event="waiting_supervisor_finish")
        supervisor.wait_to_finish()
        # slam_toolbox.wait_to_finish()
        self.log(event="supervisor_shutdown")

        # shut down components
        navigation.shutdown()
        slam_toolbox.shutdown()
        recorder_benchmark_data.shutdown()
        rviz.shutdown()
        ground_truth_map_server.shutdown()
        environment.shutdown()
        roscore.shutdown()
        print_info("execute_run: components shutdown completed")
