#!/bin/bash

# Check if Town is provided
if [ $# -eq 0 ]; then
  echo "Please provide an Town."
  exit 1
fi

source /home/autoware/.bashrc
source /opt/ros/melodic/setup.bash
source /home/autoware/carla_ws/devel/setup.bash
source /home/autoware/Autoware/install/setup.bash

export CARLA_AUTOWARE_CONTENTS=/home/autoware/autoware-contents
export PYTHON2_EGG=$(ls /home/autoware/PythonAPI | grep py2.)
export PYTHONPATH=$PYTHONPATH:/home/autoware/PythonAPI/$PYTHON2_EGG

# We need to kill autoware nodes
rosnode kill /astar_avoid /can_odometry /can_status_translator /config_waypoint_follower_rostopic \
            /config_waypoint_replanner_topic /costmap_generator /decision_maker \
            /detection/fusion_tools/range_fusion_visualization_01 /detection/l_shaped/naive_shape_visualization_01 \
            /detection/lidar_detector/cluster_detect_visualization_01 /detection/lidar_detector/object_roi_filter_clustering \
            /detection/lidar_tracker/ukf_track_visualization_01 /image_tracker_rects /imm_ukf_pda_01 \
            /joint_state_publisher /lane_rule /lane_select /lane_stop /lidar_euclidean_cluster_detect \
            /lidar_naive_l_shape_detect /naive_motion_predict /ndt_matching /op_global_planner /points_map_loader \
            /pose_relay /prediction/motion_predictor/naive_prediction_visualization_01 /pure_pursuit \
            /range_vision_fusion_01 /ray_ground_filter /robot_state_publisher /twist_filter /twist_gate \
            /ukf_track_relay_01 /vector_map_loader /vel_relay /velocity_set /vision_beyond_track_01 \
            /vision_darknet_detect /voxel_grid_filter /wayarea2grid /waypoint_marker_publisher \
            /waypoint_replanner /waypoint_velocity_visualizer /yolo3_rects

# Then reload them 
roslaunch carla_autoware_agent my_agent.launch \
        town:=$1 vehicle_length:=4.54 vehicle_width:=2.00 \
        use_ground_truth_localization:=false \
        use_ground_truth_detections:=false \
        use_manual_control:=false 

