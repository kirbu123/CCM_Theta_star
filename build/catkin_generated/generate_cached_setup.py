# -*- coding: utf-8 -*-
from __future__ import print_function

import os
import stat
import sys

# find the import for catkin's python package - either from source space or from an installed underlay
if os.path.exists(os.path.join('/home/vitaly/ros_catkin_ws/src/catkin/cmake', 'catkinConfig.cmake.in')):
    sys.path.insert(0, os.path.join('/home/vitaly/ros_catkin_ws/src/catkin/cmake', '..', 'python'))
try:
    from catkin.environment_cache import generate_environment_script
except ImportError:
    # search for catkin package in all workspaces and prepend to path
    for workspace in '/home/vitaly/ros_catkin_ws/devel_isolated/tf2_tools;/home/vitaly/ros_catkin_ws/devel_isolated/tf2_sensor_msgs;/home/vitaly/ros_catkin_ws/devel_isolated/test_tf2;/home/vitaly/ros_catkin_ws/devel_isolated/tf2_eigen;/home/vitaly/ros_catkin_ws/devel_isolated/tf2_bullet;/home/vitaly/ros_catkin_ws/devel_isolated/geometry2;/home/vitaly/ros_catkin_ws/devel_isolated/visualization_marker_tutorials;/home/vitaly/ros_catkin_ws/devel_isolated/urdf_tutorial;/home/vitaly/ros_catkin_ws/devel_isolated/rviz_python_tutorial;/home/vitaly/ros_catkin_ws/devel_isolated/rviz_plugin_tutorials;/home/vitaly/ros_catkin_ws/devel_isolated/rqt_rviz;/home/vitaly/ros_catkin_ws/devel_isolated/librviz_tutorial;/home/vitaly/ros_catkin_ws/devel_isolated/rviz;/home/vitaly/ros_catkin_ws/devel_isolated/robot_state_publisher;/home/vitaly/ros_catkin_ws/devel_isolated/kdl_parser;/home/vitaly/ros_catkin_ws/devel_isolated/urdf;/home/vitaly/ros_catkin_ws/devel_isolated/turtle_tf2;/home/vitaly/ros_catkin_ws/devel_isolated/turtle_tf;/home/vitaly/ros_catkin_ws/devel_isolated/turtle_actionlib;/home/vitaly/ros_catkin_ws/devel_isolated/turtlesim;/home/vitaly/ros_catkin_ws/devel_isolated/tf_conversions;/home/vitaly/ros_catkin_ws/devel_isolated/tf2_kdl;/home/vitaly/ros_catkin_ws/devel_isolated/laser_geometry;/home/vitaly/ros_catkin_ws/devel_isolated/interactive_marker_tutorials;/home/vitaly/ros_catkin_ws/devel_isolated/interactive_markers;/home/vitaly/ros_catkin_ws/devel_isolated/tf2_geometry_msgs;/home/vitaly/ros_catkin_ws/devel_isolated/tf;/home/vitaly/ros_catkin_ws/devel_isolated/tf2_ros;/home/vitaly/ros_catkin_ws/devel_isolated/stereo_msgs;/home/vitaly/ros_catkin_ws/devel_isolated/rqt_image_view;/home/vitaly/ros_catkin_ws/devel_isolated/map_msgs;/home/vitaly/ros_catkin_ws/devel_isolated/image_transport;/home/vitaly/ros_catkin_ws/devel_isolated/cv_bridge;/home/vitaly/ros_catkin_ws/devel_isolated/sensor_msgs;/home/vitaly/ros_catkin_ws/devel_isolated/rqt_launch;/home/vitaly/ros_catkin_ws/devel_isolated/roswtf;/home/vitaly/ros_catkin_ws/devel_isolated/nodelet_topic_tools;/home/vitaly/ros_catkin_ws/devel_isolated/diagnostic_analysis;/home/vitaly/ros_catkin_ws/devel_isolated/actionlib_tutorials;/home/vitaly/ros_catkin_ws/devel_isolated/actionlib;/home/vitaly/ros_catkin_ws/devel_isolated/rosbag;/home/vitaly/ros_catkin_ws/devel_isolated/topic_tools;/home/vitaly/ros_catkin_ws/devel_isolated/tf2_py;/home/vitaly/ros_catkin_ws/devel_isolated/smach_ros;/home/vitaly/ros_catkin_ws/devel_isolated/self_test;/home/vitaly/ros_catkin_ws/devel_isolated/rqt_reconfigure;/home/vitaly/ros_catkin_ws/devel_isolated/rqt_gui_py;/home/vitaly/ros_catkin_ws/devel_isolated/rqt_gui_cpp;/home/vitaly/ros_catkin_ws/devel_isolated/rostopic;/home/vitaly/ros_catkin_ws/devel_isolated/rospy_tutorials;/home/vitaly/ros_catkin_ws/devel_isolated/rosnode;/home/vitaly/ros_catkin_ws/devel_isolated/rosmsg;/home/vitaly/ros_catkin_ws/devel_isolated/rosbag_storage;/home/vitaly/ros_catkin_ws/devel_isolated/message_filters;/home/vitaly/ros_catkin_ws/devel_isolated/joint_state_publisher;/home/vitaly/ros_catkin_ws/devel_isolated/filters;/home/vitaly/ros_catkin_ws/devel_isolated/dynamic_reconfigure;/home/vitaly/ros_catkin_ws/devel_isolated/diagnostic_common_diagnostics;/home/vitaly/ros_catkin_ws/devel_isolated/diagnostic_updater;/home/vitaly/ros_catkin_ws/devel_isolated/diagnostic_aggregator;/home/vitaly/ros_catkin_ws/devel_isolated/rosout;/home/vitaly/ros_catkin_ws/devel_isolated/roscpp_tutorials;/home/vitaly/ros_catkin_ws/devel_isolated/pluginlib_tutorials;/home/vitaly/ros_catkin_ws/devel_isolated/nodelet_tutorial_math;/home/vitaly/ros_catkin_ws/devel_isolated/nodelet;/home/vitaly/ros_catkin_ws/devel_isolated/bondpy;/home/vitaly/ros_catkin_ws/devel_isolated/bondcpp;/home/vitaly/ros_catkin_ws/devel_isolated/roscpp;/home/vitaly/ros_catkin_ws/devel_isolated/xmlrpcpp;/home/vitaly/ros_catkin_ws/devel_isolated/webkit_dependency;/home/vitaly/ros_catkin_ws/devel_isolated/viz;/home/vitaly/ros_catkin_ws/devel_isolated/visualization_tutorials;/home/vitaly/ros_catkin_ws/devel_isolated/visualization_msgs;/home/vitaly/ros_catkin_ws/devel_isolated/urdf_parser_plugin;/home/vitaly/ros_catkin_ws/devel_isolated/control_msgs;/home/vitaly/ros_catkin_ws/devel_isolated/trajectory_msgs;/home/vitaly/ros_catkin_ws/devel_isolated/tf2;/home/vitaly/ros_catkin_ws/devel_isolated/tf2_msgs;/home/vitaly/ros_catkin_ws/devel_isolated/std_srvs;/home/vitaly/ros_catkin_ws/devel_isolated/smach_msgs;/home/vitaly/ros_catkin_ws/devel_isolated/shape_msgs;/home/vitaly/ros_catkin_ws/devel_isolated/rqt_py_common;/home/vitaly/ros_catkin_ws/devel_isolated/rosgraph_msgs;/home/vitaly/ros_catkin_ws/devel_isolated/nav_msgs;/home/vitaly/ros_catkin_ws/devel_isolated/kdl_conversions;/home/vitaly/ros_catkin_ws/devel_isolated/eigen_conversions;/home/vitaly/ros_catkin_ws/devel_isolated/geometry_msgs;/home/vitaly/ros_catkin_ws/devel_isolated/diagnostic_msgs;/home/vitaly/ros_catkin_ws/devel_isolated/bond;/home/vitaly/ros_catkin_ws/devel_isolated/actionlib_msgs;/home/vitaly/ros_catkin_ws/devel_isolated/std_msgs;/home/vitaly/ros_catkin_ws/devel_isolated/smclib;/home/vitaly/ros_catkin_ws/devel_isolated/smach;/home/vitaly/ros_catkin_ws/devel_isolated/rqt_web;/home/vitaly/ros_catkin_ws/devel_isolated/rqt_topic;/home/vitaly/ros_catkin_ws/devel_isolated/rqt_top;/home/vitaly/ros_catkin_ws/devel_isolated/rqt_tf_tree;/home/vitaly/ros_catkin_ws/devel_isolated/rqt_srv;/home/vitaly/ros_catkin_ws/devel_isolated/rqt_shell;/home/vitaly/ros_catkin_ws/devel_isolated/rqt_service_caller;/home/vitaly/ros_catkin_ws/devel_isolated/rqt_runtime_monitor;/home/vitaly/ros_catkin_ws/devel_isolated/rqt_robot_steering;/home/vitaly/ros_catkin_ws/devel_isolated/rqt_robot_plugins;/home/vitaly/ros_catkin_ws/devel_isolated/rqt_robot_monitor;/home/vitaly/ros_catkin_ws/devel_isolated/rqt_robot_dashboard;/home/vitaly/ros_catkin_ws/devel_isolated/rqt_py_console;/home/vitaly/ros_catkin_ws/devel_isolated/rqt_publisher;/home/vitaly/ros_catkin_ws/devel_isolated/rqt_pose_view;/home/vitaly/ros_catkin_ws/devel_isolated/rqt_plot;/home/vitaly/ros_catkin_ws/devel_isolated/rqt_nav_view;/home/vitaly/ros_catkin_ws/devel_isolated/rqt_msg;/home/vitaly/ros_catkin_ws/devel_isolated/rqt_moveit;/home/vitaly/ros_catkin_ws/devel_isolated/rqt_logger_level;/home/vitaly/ros_catkin_ws/devel_isolated/rqt_gui;/home/vitaly/ros_catkin_ws/devel_isolated/rqt_graph;/home/vitaly/ros_catkin_ws/devel_isolated/rqt_dep;/home/vitaly/ros_catkin_ws/devel_isolated/rqt_console;/home/vitaly/ros_catkin_ws/devel_isolated/rqt_common_plugins;/home/vitaly/ros_catkin_ws/devel_isolated/rqt_bag_plugins;/home/vitaly/ros_catkin_ws/devel_isolated/rqt_bag;/home/vitaly/ros_catkin_ws/devel_isolated/rqt_action;/home/vitaly/ros_catkin_ws/devel_isolated/rostest;/home/vitaly/ros_catkin_ws/devel_isolated/roslz4;/home/vitaly/ros_catkin_ws/devel_isolated/rosconsole_bridge;/home/vitaly/ros_catkin_ws/devel_isolated/resource_retriever;/home/vitaly/ros_catkin_ws/devel_isolated/qt_gui_cpp;/home/vitaly/ros_catkin_ws/devel_isolated/pluginlib;/home/vitaly/ros_catkin_ws/devel_isolated/rosconsole;/home/vitaly/ros_catkin_ws/devel_isolated/angles;/home/vitaly/ros_catkin_ws/devel_isolated/rosunit;/home/vitaly/ros_catkin_ws/devel_isolated/roslaunch;/home/vitaly/ros_catkin_ws/devel_isolated/python_qt_binding;/home/vitaly/ros_catkin_ws/devel_isolated/roscpp_serialization;/home/vitaly/ros_catkin_ws/devel_isolated/rostime;/home/vitaly/ros_catkin_ws/devel_isolated/rosservice;/home/vitaly/ros_catkin_ws/devel_isolated/rospy;/home/vitaly/ros_catkin_ws/devel_isolated/rosparam;/home/vitaly/ros_catkin_ws/devel_isolated/roslib;/home/vitaly/ros_catkin_ws/devel_isolated/rospack;/home/vitaly/ros_catkin_ws/devel_isolated/rosmaster;/home/vitaly/ros_catkin_ws/devel_isolated/rosmake;/home/vitaly/ros_catkin_ws/devel_isolated/roslisp;/home/vitaly/ros_catkin_ws/devel_isolated/roslint;/home/vitaly/ros_catkin_ws/devel_isolated/roslang;/home/vitaly/ros_catkin_ws/devel_isolated/rosgraph;/home/vitaly/ros_catkin_ws/devel_isolated/roscreate;/home/vitaly/ros_catkin_ws/devel_isolated/roscpp_traits;/home/vitaly/ros_catkin_ws/devel_isolated/roscpp_core;/home/vitaly/ros_catkin_ws/devel_isolated/rosclean;/home/vitaly/ros_catkin_ws/devel_isolated/rosbuild;/home/vitaly/ros_catkin_ws/devel_isolated/rosboost_cfg;/home/vitaly/ros_catkin_ws/devel_isolated/rosbash;/home/vitaly/ros_catkin_ws/devel_isolated/rosbag_migration_rule;/home/vitaly/ros_catkin_ws/devel_isolated/ros_tutorials;/home/vitaly/ros_catkin_ws/devel_isolated/ros_environment;/home/vitaly/ros_catkin_ws/devel_isolated/ros_core;/home/vitaly/ros_catkin_ws/devel_isolated/ros_comm;/home/vitaly/ros_catkin_ws/devel_isolated/ros_base;/home/vitaly/ros_catkin_ws/devel_isolated/ros;/home/vitaly/ros_catkin_ws/devel_isolated/robot;/home/vitaly/ros_catkin_ws/devel_isolated/qwt_dependency;/home/vitaly/ros_catkin_ws/devel_isolated/qt_gui_py_common;/home/vitaly/ros_catkin_ws/devel_isolated/qt_gui;/home/vitaly/ros_catkin_ws/devel_isolated/qt_dotgraph;/home/vitaly/ros_catkin_ws/devel_isolated/nodelet_core;/home/vitaly/ros_catkin_ws/devel_isolated/mk;/home/vitaly/ros_catkin_ws/devel_isolated/message_runtime;/home/vitaly/ros_catkin_ws/devel_isolated/message_generation;/home/vitaly/ros_catkin_ws/devel_isolated/media_export;/home/vitaly/ros_catkin_ws/devel_isolated/joint_state_publisher_gui;/home/vitaly/ros_catkin_ws/devel_isolated/gl_dependency;/home/vitaly/ros_catkin_ws/devel_isolated/geometry_tutorials;/home/vitaly/ros_catkin_ws/devel_isolated/geometry;/home/vitaly/ros_catkin_ws/devel_isolated/executive_smach;/home/vitaly/ros_catkin_ws/devel_isolated/diagnostics;/home/vitaly/ros_catkin_ws/devel_isolated/desktop;/home/vitaly/ros_catkin_ws/devel_isolated/cpp_common;/home/vitaly/ros_catkin_ws/devel_isolated/common_tutorials;/home/vitaly/ros_catkin_ws/devel_isolated/common_msgs;/home/vitaly/ros_catkin_ws/devel_isolated/class_loader;/home/vitaly/ros_catkin_ws/devel_isolated/cmake_modules;/home/vitaly/ros_catkin_ws/devel_isolated/bond_core;/home/vitaly/ros_catkin_ws/devel_isolated/genpy;/home/vitaly/ros_catkin_ws/devel_isolated/gennodejs;/home/vitaly/ros_catkin_ws/devel_isolated/genlisp;/home/vitaly/ros_catkin_ws/devel_isolated/geneus;/home/vitaly/ros_catkin_ws/devel_isolated/gencpp;/home/vitaly/ros_catkin_ws/devel_isolated/genmsg;/home/vitaly/ros_catkin_ws/devel_isolated/catkin;/home/vitaly/ros_catkin_ws/devel_isolated/xacro;/home/vitaly/ros_catkin_ws/install_isolated'.split(';'):
        python_path = os.path.join(workspace, 'lib/python3/dist-packages')
        if os.path.isdir(os.path.join(python_path, 'catkin')):
            sys.path.insert(0, python_path)
            break
    from catkin.environment_cache import generate_environment_script

code = generate_environment_script('/home/vitaly/theta_star_ws/devel/env.sh')

output_filename = '/home/vitaly/theta_star_ws/build/catkin_generated/setup_cached.sh'
with open(output_filename, 'w') as f:
    # print('Generate script for cached setup "%s"' % output_filename)
    f.write('\n'.join(code))

mode = os.stat(output_filename).st_mode
os.chmod(output_filename, mode | stat.S_IXUSR)
