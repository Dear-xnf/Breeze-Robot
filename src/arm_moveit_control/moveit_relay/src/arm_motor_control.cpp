#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <robot_msg/msg/motor_cmd.hpp>
#include <robot_msg/msg/motor_single_cmd.hpp>
#include <cmath>
#include <algorithm>

using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
using GoalHandleFJT = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;

class ArmMotorControl : public rclcpp::Node
{
public:
  ArmMotorControl() : Node("moveit_relay")
  {
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/robot/arm_joint_state",
      rclcpp::QoS(1).best_effort(),
      std::bind(&ArmMotorControl::jointStateCallback, this, std::placeholders::_1));

    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
      "/joint_states",
      rclcpp::QoS(1).best_effort());

    motor_cmd_pub_ = this->create_publisher<robot_msg::msg::MotorCmd>(
      "/robot/arm_joint_cmd",
      rclcpp::QoS(1).best_effort());

    action_server_ = rclcpp_action::create_server<FollowJointTrajectory>(
      this,
      "/follow_joint_trajectory",
      std::bind(&ArmMotorControl::handleGoal, this, std::placeholders::_1, std::placeholders::_2),
      std::bind(&ArmMotorControl::handleCancel, this, std::placeholders::_1),
      std::bind(&ArmMotorControl::handleAccepted, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "✅ moveit_relay 节点启动完成");
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<robot_msg::msg::MotorCmd>::SharedPtr motor_cmd_pub_;
  rclcpp_action::Server<FollowJointTrajectory>::SharedPtr action_server_;

  std::shared_ptr<GoalHandleFJT> current_goal_handle_;
  rclcpp::TimerBase::SharedPtr traj_timer_;
  sensor_msgs::msg::JointState current_joint_state_;
  std::mutex joint_state_mutex_; 

  const int joint_num_ = 6;
  const double max_vel_rad_s_ = 1.5;
  const std::vector<uint8_t> joint_id_map_ = {0x1,0x2,0x3,0x4,0x5,0x6};
  double traj_max_velocity_ = 0.0;
  rclcpp::Time traj_start_time_;

  void printFullTrajectory(const trajectory_msgs::msg::JointTrajectory& traj)
  {
    RCLCPP_INFO(this->get_logger(), "📝 接收到 Action 轨迹点数:%d,最大速度:%.3frad/s",(int)traj.points.size(),traj_max_velocity_);
    // RCLCPP_INFO(this->get_logger(), "=====================================");

    for (size_t i = 0; i < traj.points.size(); i++)
    {
      // const auto& p = traj.points[i];
      // RCLCPP_INFO(this->get_logger(), "▶ 点 [%d]", (int)i);
      // RCLCPP_INFO(this->get_logger(), "    位置: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
      //     p.positions[0], p.positions[1], p.positions[2],
      //     p.positions[3], p.positions[4], p.positions[5]);
      // RCLCPP_INFO(this->get_logger(), "    速度: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f]",
      //     p.velocities[0], p.velocities[1], p.velocities[2],
      //     p.velocities[3], p.velocities[4], p.velocities[5]);
    }
    // RCLCPP_INFO(this->get_logger(), "=====================================\n");
  }

  double calculateTrajectoryMaxVelocity(const trajectory_msgs::msg::JointTrajectory& traj)
  {
    double max_v = 0.0;
    for (const auto& point : traj.points) {
      for (double v : point.velocities) {
        max_v = std::max(max_v, std::fabs(v));
      }
    }
    max_v = std::min(max_v, max_vel_rad_s_);
    return max_v;
  }

  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    joint_state_pub_->publish(*msg);
    std::lock_guard<std::mutex> lock(joint_state_mutex_);
    current_joint_state_ = *msg;
  }




  rclcpp_action::GoalResponse handleGoal(const rclcpp_action::GoalUUID &,
    std::shared_ptr<const FollowJointTrajectory::Goal> goal)
  {
    // RCLCPP_INFO(this->get_logger(), "🚀 收到 MoveIt2 轨迹请求");
    if (current_goal_handle_) {
      RCLCPP_WARN(this->get_logger(), "❌ 已有轨迹在执行，拒绝新轨迹");
      return rclcpp_action::GoalResponse::REJECT;
    }

    traj_max_velocity_ = calculateTrajectoryMaxVelocity(goal->trajectory);
    printFullTrajectory(goal->trajectory);
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }


  
  rclcpp_action::CancelResponse handleCancel(const std::shared_ptr<GoalHandleFJT>)
  {
    RCLCPP_INFO(this->get_logger(), "🛑 轨迹取消");
    if (traj_timer_) traj_timer_->cancel();
    current_goal_handle_.reset();
    return rclcpp_action::CancelResponse::ACCEPT;
  }


  void handleAccepted(const std::shared_ptr<GoalHandleFJT> goal_handle)
  {
    current_goal_handle_ = goal_handle;
    traj_start_time_ = this->now();
    // RCLCPP_INFO(this->get_logger(), "▶ 开始执行轨迹...");

    traj_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&ArmMotorControl::trajExecuteLoop, this));
  }

  void trajExecuteLoop()
  {
    if (!current_goal_handle_) return;

    const auto &traj = current_goal_handle_->get_goal()->trajectory;
    double elapsed = (this->now() - traj_start_time_).seconds();

    // 修正：正确查找当前时间对应的轨迹段
    size_t idx = 0;
    for (; idx < traj.points.size(); ++idx)
    {
      double t_pt = rclcpp::Duration(traj.points[idx].time_from_start).seconds();
      if (elapsed <= t_pt) break;
    }

    // 轨迹走完
    if (idx >= traj.points.size())
    {
      auto result = std::make_shared<FollowJointTrajectory::Result>();
      current_goal_handle_->succeed(result);
      current_goal_handle_.reset();
      traj_timer_->cancel();
      RCLCPP_INFO(this->get_logger(), "🏁 轨迹执行完成！\n");
      return;
    }

    // 首尾保护
    if (idx == 0) idx = 1;
    const auto &p_prev = traj.points[idx-1];
    const auto &p_curr = traj.points[idx];

    double t_prev = rclcpp::Duration(p_prev.time_from_start).seconds();
    double t_curr = rclcpp::Duration(p_curr.time_from_start).seconds();
    double alpha = (t_curr == t_prev) ? 1.0 : std::clamp((elapsed - t_prev) / (t_curr - t_prev), 0.0, 1.0);


    // double total_trajectory_time = rclcpp::Duration(traj.points.back().time_from_start).seconds();
    double vel_ratio = 1.0;

    // if (elapsed < total_trajectory_time * 0/1000)
    // {
    //     vel_ratio = 1;
    // }
    // else if (elapsed < total_trajectory_time * 990/1000)
    // {
    //     vel_ratio = 1;
    // }
    // else if (elapsed < total_trajectory_time * 900/1000) 
    // {
    //     vel_ratio = 1;
    // }
    // else if (elapsed < total_trajectory_time * 950/1000)
    // {
    //     vel_ratio = 0.5;
    // }
    // else
    // {
    //     // 最后 
    //     vel_ratio = 0.25;
    // }
    double limited_vel = traj_max_velocity_ * vel_ratio;


    // 发布 Feedback
    auto feedback = std::make_shared<FollowJointTrajectory::Feedback>();
    feedback->header.stamp = this->now();
    feedback->joint_names = traj.joint_names;
    
    {
      std::lock_guard<std::mutex> lock(joint_state_mutex_);
      if(current_joint_state_.position.empty())
      {
        RCLCPP_WARN(this->get_logger(), "⚠️ 暂无关节状态数据，跳过本次控制周期");
        return;
      }

      feedback->actual.positions = current_joint_state_.position;
      feedback->actual.velocities = current_joint_state_.velocity;
    }


    current_goal_handle_->publish_feedback(feedback);

    robot_msg::msg::MotorCmd cmd_msg;
    cmd_msg.commands.resize(joint_num_);

    for (int j = 0; j < joint_num_; ++j)
    {
      double pos_prev = p_prev.positions[j];
      double pos_curr = p_curr.positions[j];
      double pos = pos_prev + alpha * (pos_curr - pos_prev);

      double vel = limited_vel;
      // double vel = traj_max_velocity_;

      if (pos_curr < pos_prev) vel = vel;

      robot_msg::msg::MotorSingleCmd sc;
      sc.id = joint_id_map_[j];
      sc.mode = 2;
      sc.position = static_cast<float>(pos);
      sc.velocity = static_cast<float>(vel);
      sc.torque = 0.0f;
      sc.kp = 0.0f;
      sc.kd = 0.0f;
      cmd_msg.commands[j] = sc;
    }

    motor_cmd_pub_->publish(cmd_msg);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArmMotorControl>());
  rclcpp::shutdown();
  return 0;
}