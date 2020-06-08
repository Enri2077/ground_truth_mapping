# controller server crash after target pose is accepted if rviz is already open

```
enrico@enrico-zenbook:~/w/ros2_ws/src/ground_truth_mapping (master) $ rr ground_truth_mapping execute_ground_truth_mapping -e ~/ds/performance_modelling/dataset_v4_n4/airlab -g
environments found: 1
number of environments:           1
number of parameter combinations: 1
number of repetition runs:        1
total number of runs:             1



benchmark: starting run 4
	environment_folder: /home/enrico/ds/performance_modelling/dataset_v4_n4/airlab
	parameters_combination_dict:
		beta: [0.0175, 0.0175, 0.005, 0.005]
		laser_scan_max_range: 60.0
		laser_scan_fov_deg: 270
execute_run: launching components
t: 1591628948.414657, run: 4, event: waiting_supervisor_finish
[INFO] [launch]: All log files can be found below /home/enrico/.ros/log/2020-06-08-17-09-08-414175-enrico-zenbook-32335
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [ground_truth_mapping_supervisor-9]: process started with pid [32386]
[INFO] [gzserver-1]: process started with pid [32341]
[INFO] [robot_state_publisher-2]: process started with pid [32342]
[INFO] [controller_server-3]: process started with pid [32343]
[INFO] [planner_server-4]: process started with pid [32344]
[INFO] [recoveries_server-5]: process started with pid [32345]
[INFO] [bt_navigator-6]: process started with pid [32352]
[INFO] [waypoint_follower-7]: process started with pid [32353]
[INFO] [slam_toolbox-8]: process started with pid [32358]
[INFO] [lifecycle_manager-10]: process started with pid [32398]
[robot_state_publisher-2] Initialize urdf model from file: /home/enrico/ds/performance_modelling/dataset_v4_n4/airlab/gazebo/robot_gt.urdf
[robot_state_publisher-2] Parsing robot urdf xml string.
[robot_state_publisher-2] Link base_link had 7 children
[robot_state_publisher-2] Link camera_link_gt had 2 children
[robot_state_publisher-2] Link camera_depth_frame_gt had 1 children
[robot_state_publisher-2] Link camera_depth_optical_frame_gt had 0 children
[robot_state_publisher-2] Link camera_rgb_frame_gt had 1 children
[robot_state_publisher-2] Link camera_rgb_optical_frame_gt had 0 children
[robot_state_publisher-2] Link caster_back_left_link_gt had 0 children
[robot_state_publisher-2] Link caster_back_right_link_gt had 0 children
[robot_state_publisher-2] Link imu_link_gt had 0 children
[robot_state_publisher-2] Link base_scan_gt had 0 children
[robot_state_publisher-2] Link wheel_left_link_gt had 0 children
[robot_state_publisher-2] Link wheel_right_link_gt had 0 children
[robot_state_publisher-2] got segment base_footprint_gt
[robot_state_publisher-2] got segment base_link
[robot_state_publisher-2] got segment base_scan_gt
[robot_state_publisher-2] got segment camera_depth_frame_gt
[robot_state_publisher-2] got segment camera_depth_optical_frame_gt
[robot_state_publisher-2] got segment camera_link_gt
[robot_state_publisher-2] got segment camera_rgb_frame_gt
[robot_state_publisher-2] got segment camera_rgb_optical_frame_gt
[robot_state_publisher-2] got segment caster_back_left_link_gt
[robot_state_publisher-2] got segment caster_back_right_link_gt
[robot_state_publisher-2] got segment imu_link_gt
[robot_state_publisher-2] got segment wheel_left_link_gt
[robot_state_publisher-2] got segment wheel_right_link_gt
[robot_state_publisher-2] Adding fixed segment from base_footprint_gt to base_link
[robot_state_publisher-2] Adding fixed segment from base_link to camera_link_gt
[robot_state_publisher-2] Adding fixed segment from camera_link_gt to camera_depth_frame_gt
[robot_state_publisher-2] Adding fixed segment from camera_depth_frame_gt to camera_depth_optical_frame_gt
[robot_state_publisher-2] Adding fixed segment from camera_link_gt to camera_rgb_frame_gt
[robot_state_publisher-2] Adding fixed segment from camera_rgb_frame_gt to camera_rgb_optical_frame_gt
[robot_state_publisher-2] Adding fixed segment from base_link to caster_back_left_link_gt
[robot_state_publisher-2] Adding fixed segment from base_link to caster_back_right_link_gt
[robot_state_publisher-2] Adding fixed segment from base_link to imu_link_gt
[robot_state_publisher-2] Adding fixed segment from base_link to base_scan_gt
[robot_state_publisher-2] Adding moving segment from base_link to wheel_left_link_gt
[robot_state_publisher-2] Adding moving segment from base_link to wheel_right_link_gt
[lifecycle_manager-10] [INFO] [lifecycle_manager]: Creating
[slam_toolbox-8] [INFO] [slam_toolbox]: Node using stack size 40000000
^R
[lifecycle_manager-10] [INFO] [lifecycle_manager]: Creating and initializing lifecycle service clients
[slam_toolbox-8] [INFO] [slam_toolbox]: Using solver plugin solver_plugins::CeresSolver
[slam_toolbox-8] [INFO] [slam_toolbox]: CeresSolver: Using SCHUR_JACOBI preconditioner.
[gzserver-1] Gazebo multi-robot simulator, version 9.0.0
[gzserver-1] Copyright (C) 2012 Open Source Robotics Foundation.
[gzserver-1] Released under the Apache 2 License.
[gzserver-1] http://gazebosim.org
[gzserver-1] 
[ground_truth_mapping_supervisor-9] preparing to start run
[gzserver-1] [Msg] Waiting for master.
[gzserver-1] [Msg] Connected to gazebo master @ http://127.0.0.1:11345
[gzserver-1] [Msg] Publicized address: 192.168.1.101
[gzserver-1] [Wrn] [Event.cc:61] Warning: Deleting a connection right after creation. Make sure to save the ConnectionPtr from a Connect call
[gzserver-1] [INFO] [camera_driver]: Publishing camera info to [/camera/camera_info]
[gzserver-1] [INFO] [gazebo_ros_state]: Publishing states of gazebo models at [/model_states]
[gzserver-1] [INFO] [gazebo_ros_state]: Publishing states of gazebo links at [/link_states]
[gzserver-1] [INFO] [turtlebot3_diff_drive]: Wheel pair 1 separation set to [0.287000m]
[gzserver-1] [INFO] [turtlebot3_diff_drive]: Wheel pair 1 diameter set to [0.066000m]
[gzserver-1] [INFO] [turtlebot3_diff_drive]: Subscribed to [/cmd_vel]
[gzserver-1] [INFO] [turtlebot3_diff_drive]: Computing odometry with parametric error model
[gzserver-1] alpha1 [0.017500]
[gzserver-1] alpha2 [0.017500]
[gzserver-1] alpha3 [0.005000]
[gzserver-1] alpha4 [0.005000]
[gzserver-1] 
[gzserver-1] [INFO] [turtlebot3_diff_drive]: Advertise odometry on [/odom]
[gzserver-1] [INFO] [turtlebot3_diff_drive]: Publishing odom transforms between [odom_realistic] and [base_footprint_realistic]
[gzserver-1] [INFO] [turtlebot3_diff_drive]: Publishing ground truth odom transforms between [odom] and [base_footprint_gt]
[gzserver-1] [INFO] [turtlebot3_joint_state]: Going to publish joint [robot::wheel_left_joint]
[gzserver-1] [INFO] [turtlebot3_joint_state]: Going to publish joint [robot::wheel_right_joint]
[slam_toolbox-8] Registering sensor: [Custom Described Lidar]
[ground_truth_mapping_supervisor-9] got robot radius
[lifecycle_manager-10] [INFO] [lifecycle_manager]: Starting managed nodes bringup...
[lifecycle_manager-10] [INFO] [lifecycle_manager]: Configuring controller_server
[lifecycle_manager-10] [INFO] [lifecycle_manager]: Configuring planner_server
[lifecycle_manager-10] [INFO] [lifecycle_manager]: Configuring recoveries_server
[lifecycle_manager-10] [INFO] [lifecycle_manager]: Configuring bt_navigator
[lifecycle_manager-10] [INFO] [lifecycle_manager]: Configuring waypoint_follower
[waypoint_follower-7] [WARN] [rcl.logging_rosout]: Publisher already registered for provided node name. If this is due to multiple nodes with the same name then all logs for that logger name will go out over the existing publisher. As soon as any node with that name is destructed it will unregister the publisher, preventing any further logs for that name from being published on the rosout topic.
[lifecycle_manager-10] [INFO] [lifecycle_manager]: Activating controller_server
[lifecycle_manager-10] [INFO] [lifecycle_manager]: Activating planner_server
[lifecycle_manager-10] [INFO] [lifecycle_manager]: Activating recoveries_server
[lifecycle_manager-10] [INFO] [lifecycle_manager]: Activating bt_navigator
[lifecycle_manager-10] [INFO] [lifecycle_manager]: Activating waypoint_follower
[lifecycle_manager-10] [INFO] [lifecycle_manager]: Managed nodes are active
[ground_truth_mapping_supervisor-9] called lifecycle_manager_service_client nav2_msgs.srv.ManageLifecycleNodes_Response(success=True)
[ground_truth_mapping_supervisor-9] t: 2.596, event: run_start
[ground_truth_mapping_supervisor-9] goal 1 / 18
[ground_truth_mapping_supervisor-9] t: 2.596, event: target_pose_set
[ground_truth_mapping_supervisor-9] t: 2.596, event: target_pose_accepted
[ERROR] [planner_server-4]: process has died [pid 32344, exit code -11, cmd '/opt/ros/eloquent/lib/nav2_planner/planner_server --ros-args -r __node:=planner_server --params-file /tmp/tmpstzcy5ir        '].
[INFO] [lifecycle_manager-10]: sending signal 'SIGINT' to process[lifecycle_manager-10]
[INFO] [ground_truth_mapping_supervisor-9]: sending signal 'SIGINT' to process[ground_truth_mapping_supervisor-9]
[INFO] [slam_toolbox-8]: sending signal 'SIGINT' to process[slam_toolbox-8]
[INFO] [waypoint_follower-7]: sending signal 'SIGINT' to process[waypoint_follower-7]
[INFO] [bt_navigator-6]: sending signal 'SIGINT' to process[bt_navigator-6]
[INFO] [slam_toolbox-8]: process has finished cleanly [pid 32358]
[INFO] [recoveries_server-5]: sending signal 'SIGINT' to process[recoveries_server-5]
[INFO] [controller_server-3]: sending signal 'SIGINT' to process[controller_server-3]
[INFO] [robot_state_publisher-2]: sending signal 'SIGINT' to process[robot_state_publisher-2]
[INFO] [recoveries_server-5]: process has finished cleanly [pid 32345]
[ERROR] [bt_navigator-6]: process has died [pid 32352, exit code -6, cmd '/opt/ros/eloquent/lib/nav2_bt_navigator/bt_navigator --ros-args -r __node:=bt_navigator --params-file /tmp/tmprjgyotrg        '].
[INFO] [gzserver-1]: sending signal 'SIGINT' to process[gzserver-1]
[lifecycle_manager-10] [INFO] [rclcpp]: signal_handler(signal_value=2)
[lifecycle_manager-10] [INFO] [lifecycle_manager]: Destroying
[ground_truth_mapping_supervisor-9] asked to shutdown, terminating run
[ground_truth_mapping_supervisor-9] t: 2.596, event: ros_shutdown
[ground_truth_mapping_supervisor-9] t: 2.596, event: supervisor_finished
[INFO] [controller_server-3]: process has finished cleanly [pid 32343]
[slam_toolbox-8] [INFO] [rclcpp]: signal_handler(signal_value=2)
[bt_navigator-6] terminate called after throwing an instance of 'rclcpp::exceptions::RCLError'
[bt_navigator-6]   what():  Failed to create interrupt guard condition in Executor constructor: the given context is not valid, either rcl_init() was not called or rcl_shutdown() was called., at /tmp/binarydeb/ros-eloquent-rcl-0.8.4/src/rcl/guard_condition.c:69
[gzserver-1] terminate called after throwing an instance of 'std::runtime_error'
[gzserver-1]   what():  could not count subscribers: rcl node's context is invalid, at /tmp/binarydeb/ros-eloquent-rcl-0.8.4/src/rcl/node.c:497
[INFO] [lifecycle_manager-10]: process has finished cleanly [pid 32398]
[ground_truth_mapping_supervisor-9] sys:1: RuntimeWarning: Failed to fini subscription: rcl node implementation is invalid, at /tmp/binarydeb/ros-eloquent-rcl-0.8.4/src/rcl/node.c:483
[INFO] [robot_state_publisher-2]: process has finished cleanly [pid 32342]
[INFO] [waypoint_follower-7]: process has finished cleanly [pid 32353]
[ERROR] [gzserver-1]: process has died [pid 32341, exit code -6, cmd 'gzserver --verbose -s libgazebo_ros_init.so /home/enrico/ds/performance_modelling/output/ground_truth_mapping/run_4/components_configuration/gazebo/gazebo_environment.model'].
[INFO] [ground_truth_mapping_supervisor-9]: process has finished cleanly [pid 32386]
t: 1591628954.3428183, run: 4, event: supervisor_shutdown
execute_run: components shutdown completed
t: 1591628954.3429801, run: 4, event: run_end
run 4 completed
benchmark: run 4 completed
benchmark: finished
enrico@enrico-zenbook:~/w/ros2_ws/src/ground_truth_mapping (master) $ 
```
