#!/bin/bash

stop_all() {
    echo -e "\n🛑 正在停止所有服务..."
    pkill -SIGTERM -P $$
    exit 0
}


trap stop_all SIGINT


source /opt/ros/humble/setup.bash
source ./../install/setup.bash


ros2 run arm_motor_control arm_motor_control_node &
# ros2 run arm_mujoco_sim arm_sim_display_node &
ros2 launch rosbridge_server rosbridge_websocket_launch.xml > /dev/null 2>&1 &

sleep 5


python3 -m http.server 8888 --bind 0.0.0.0 > /dev/null 2>&1 &


echo "请访问地址"
echo -e "\033[32m ===== 当前局域网IP ===== \033[0m"
hostname -I
echo -e "\033[32m 端口 =====8888 ===== \033[0m"


wait