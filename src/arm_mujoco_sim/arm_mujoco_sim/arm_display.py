import os
import threading
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy 
from std_msgs.msg import Float64MultiArray, String
from sensor_msgs.msg import JointState
from control_msgs.msg import JointJog

import mujoco
import mujoco.viewer
from ament_index_python.packages import get_package_share_path
from .utils import clip_joint_value, smooth_interpolate, get_joint_limits


PACKAGE_NAME = "arm_mujoco_sim"
MODEL_FILE = "arm_model.xml"  
JOINT_NUM = 6  
DESIRED_FREQ = 200  
SIM_DT = 1.0 / DESIRED_FREQ

MAX_POS_STEP = 0.05 
MAX_VEL = 1.0        
MAX_TORQUE = 5.0    


lock = threading.Lock()

control_cmd = {
    "position": np.zeros(JOINT_NUM),
    "velocity": np.zeros(JOINT_NUM),
    "torque": np.zeros(JOINT_NUM),
    "mode": "position" 
}

sim_state = {
    "qpos": np.zeros(JOINT_NUM),
    "qvel": np.zeros(JOINT_NUM),
    "qtorque": np.zeros(JOINT_NUM)
}


def run_mujoco_sim():
    """独立线程运行Mujoco仿真"""

    try:
        pkg_path = get_package_share_path(PACKAGE_NAME)
        model_path = os.path.join(pkg_path, "mjcf", MODEL_FILE)
        if not os.path.exists(model_path):
            print(f"错误：模型文件不存在 {model_path}")
            return
        model = mujoco.MjModel.from_xml_path(model_path)
        data = mujoco.MjData(model)
    except Exception as e:
        print(f"加载模型失败：{e}")
        return

    joint_limits = get_joint_limits(model, JOINT_NUM)
    data.qpos[:JOINT_NUM] = 0.0 
    current_pos = np.zeros(JOINT_NUM)
    current_vel = np.zeros(JOINT_NUM)
    current_torque = np.zeros(JOINT_NUM)


    with mujoco.viewer.launch_passive(model, data) as viewer:
        print("Mujoco机械臂仿真已启动（支持位置/速度/力矩控制）")
        while viewer.is_running():
            loop_start = time.time()


            with lock:
                cmd_mode = control_cmd["mode"]
                target_pos = control_cmd["position"].copy()
                target_vel = control_cmd["velocity"].copy()
                target_torque = control_cmd["torque"].copy()

            if cmd_mode == "position":
    
                for i in range(JOINT_NUM):
                    current_pos[i] = smooth_interpolate(
                        current_pos[i], target_pos[i], MAX_POS_STEP
                    )
    
                    current_pos[i] = clip_joint_value(current_pos[i], *joint_limits[i])
                    data.qpos[i] = current_pos[i]
 
                current_vel = (current_pos - sim_state["qpos"]) / SIM_DT

                mujoco.mj_forward(model, data)

            elif cmd_mode == "velocity":
  
                for i in range(JOINT_NUM):
         
                    vel = clip_joint_value(target_vel[i], -MAX_VEL, MAX_VEL)
             
                    current_pos[i] += vel * SIM_DT
                
                    current_pos[i] = clip_joint_value(current_pos[i], *joint_limits[i])
                    data.qpos[i] = current_pos[i]
                current_vel = target_vel.copy()
      
                mujoco.mj_forward(model, data)

            elif cmd_mode == "torque":
       
                if model.nu >= JOINT_NUM:
                    for i in range(JOINT_NUM):
                  
                        torque = clip_joint_value(target_torque[i], -MAX_TORQUE, MAX_TORQUE)
            
                    mujoco.mj_step(model, data)
          
                    current_pos = data.qpos[:JOINT_NUM].copy()
                    current_vel = data.qvel[:JOINT_NUM].copy()
                    current_torque = data.qfrc_actuator[:JOINT_NUM].copy()
                else:
                    print("警告：模型执行器数量不足，无法力矩控制")

    
            with lock:
                sim_state["qpos"] = current_pos.copy()
                sim_state["qvel"] = current_vel.copy()
                sim_state["qtorque"] = current_torque.copy()

            viewer.sync()
            loop_duration = time.time() - loop_start
            if loop_duration < SIM_DT:
                time.sleep(SIM_DT - loop_duration)

class ArmControlNode(Node):
    def __init__(self):
        super().__init__("robot_arm_sim_display_node")
        self.get_logger().info("mujoco机械臂仿真显示可视化启动")

  
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.sub_real_arm_pos=self.create_subscription(
            JointState,
            "robot/arm_joint_state",
            self.position_state_callback,
            qos_profile 
        )


        self.pub_state = self.create_publisher(
            JointState,
            "robot/arm_sim_display_joint_state",
            10
        )

        self.timer = self.create_timer(0.01, self.publish_state)


    def position_state_callback(self, msg):
        if len(msg.position) == JOINT_NUM:
            with lock:
                control_cmd["mode"] = "position"
                control_cmd["position"] = np.array(msg.position)  
                control_cmd["velocity"] = np.array(msg.velocity)
                control_cmd["torque"] = np.array(msg.effort)
            # self.get_logger().debug(f"接收位置指令：{msg.data[:2]}...")


    def publish_state(self):
        with lock:
            state = JointState()
            state.header.stamp = self.get_clock().now().to_msg()
            state.name = [f"joint_{i}" for i in range(JOINT_NUM)]
            state.position = sim_state["qpos"].tolist()
            state.velocity = sim_state["qvel"].tolist()
            state.effort = sim_state["qtorque"].tolist()
            self.pub_state.publish(state)


def main(args=None):

    sim_thread = threading.Thread(target=run_mujoco_sim)
    sim_thread.daemon = True
    sim_thread.start()


    rclpy.init(args=args)
    node = ArmControlNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("接收到退出信号，正在关闭...")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        sim_thread.join(timeout=2)
        print("程序已退出")

if __name__ == "__main__":
    main()