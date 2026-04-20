#!/bin/bash

IDENTIFIER_MAP_SERVER="__ID__MAP_SERVER__"
IDENTIFIER_AMCL="__ID__AMCL__"
IDENTIFIER_PLANNER_SERVER="__ID__PLANNER_SERVER__"
IDENTIFIER_CONTROLLER_SERVER="__ID__CONTROLLER_SERVER__"
IDENTIFIER_BT="__ID__BT_NAVIGATOR__"
IDENTIFIER_SMOOTHER="__ID__SMOOTHER__"
IDENTIFIER_NAV_BEHAVIOR_SERVER="__ID__NAV_BEHAVIOR_SERVER__"
IDENTIFIER_PATH="__ID__MY_PATH_NODE__"
IDENTIFIER_RVIZ="__ID__RVIZ2__"

# === Ctrl+C 처리 ===
trap ctrl_c INT
function ctrl_c() {
  echo ""
  echo "Ctrl+C 감지됨. 노드 종료 중..."
  pkill -f "$IDENTIFIER_MAP_SERVER"
  pkill -f "$IDENTIFIER_AMCL"
  pkill -f "$IDENTIFIER_PLANNER_SERVER"
  pkill -f "$IDENTIFIER_CONTROLLER_SERVER"
  pkill -f "$IDENTIFIER_BT"
  pkill -f "$IDENTIFIER_SMOOTHER"
  pkill -f "$IDENTIFIER_NAV_BEHAVIOR_SERVER"
  pkill -f "$IDENTIFIER_PATH"
  pkill -f "$IDENTIFIER_RVIZ"
  echo "종료 완료."
  exit
}


# NAV_CONFIG_PATH=$(ros2 pkg prefix turtlebot4_navigation)/share/turtlebot4_navigation/config/nav2.yaml
NAV_CONFIG_PATH=/home/jihong/qrrt/nav2.yaml

# === Nav2 노드 실행 ===
gnome-terminal -- bash -c "
  echo '$IDENTIFIER_MAP_SERVER';
  ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=/home/jihong/qrrt/my_map.yaml -p use_sim_time:=true;
  exec bash
" &

gnome-terminal -- bash -c "
  echo '$IDENTIFIER_AMCL';
  ros2 run nav2_amcl amcl --ros-args --params-file $NAV_CONFIG_PATH -p use_sim_time:=true  -p scan_topic:=/scan -p odom_frame_id:=odom -p base_frame_id:=base_link;
  exec bash
" &

gnome-terminal -- bash -c "
  echo '$IDENTIFIER_PLANNER_SERVER';
  ros2 run nav2_planner planner_server --ros-args --params-file $NAV_CONFIG_PATH -p use_sim_time:=true;
  exec bash
" &

gnome-terminal -- bash -c "
  echo '$IDENTIFIER_CONTROLLER_SERVER';
  ros2 run nav2_controller controller_server --ros-args --params-file $NAV_CONFIG_PATH -p use_sim_time:=true;
  exec bash
" &

gnome-terminal -- bash -c "
  echo '$IDENTIFIER_BT';
  ros2 run nav2_bt_navigator bt_navigator --ros-args --params-file $NAV_CONFIG_PATH -p use_sim_time:=true;
  exec bash
" &

gnome-terminal -- bash -c "
  echo '$IDENTIFIER_SMOOTHER';
  ros2 run nav2_smoother smoother_server --ros-args --params-file $NAV_CONFIG_PATH -p use_sim_time:=true;
  exec bash
" &

gnome-terminal -- bash -c "
  echo '$IDENTIFIER_NAV_BEHAVIOR_SERVER';
  ros2 run nav2_behaviors behavior_server --ros-args --params-file $NAV_CONFIG_PATH -p use_sim_time:=true;
  exec bash
" &

gnome-terminal -- bash -c "
  echo '$IDENTIFIER_RVIZ';
  ros2 launch nav2_bringup rviz_launch.py use_sim_time:=true default_nav_to_pose_bt_xml:=/home/jihong/qrrt/navigate_through_poses_custom.xml;
  exec bash
" &

# === Gazebo 시뮬레이터 실행 ===
echo ""
echo "🎮 Gazebo 시뮬레이터를 실행합니다..."
ros2 launch turtlebot4_ignition_bringup turtlebot4_ignition.launch.py slam:=false nav2:=false rviz:=false use_sim_time:=true &
GAZEBO_PID=$!

sleep 5

# === Gazebo 종료 대기 ===
wait $GAZEBO_PID

# === Gazebo 종료 시 모든 노드 종료 ===
echo ""
echo "🛑 Gazebo 종료 감지됨. 노드들도 종료합니다..."
pkill -f "$IDENTIFIER_MAP_SERVER"
pkill -f "$IDENTIFIER_AMCL"
pkill -f "$IDENTIFIER_PLANNER_SERVER"
pkill -f "$IDENTIFIER_CONTROLLER_SERVER"
pkill -f "$IDENTIFIER_BT"
pkill -f "$IDENTIFIER_SMOOTHER"
pkill -f "$IDENTIFIER_NAV_BEHAVIOR_SERVER"
pkill -f "$IDENTIFIER_RVIZ"
pkill -f "$IDENTIFIER_PATH"

echo "✅ 모든 노드가 안전하게 종료되었습니다."
