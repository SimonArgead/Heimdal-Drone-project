#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

using namespace std::chrono_literals;

class RTABMapZigzagNode : public rclcpp::Node {
public:
  RTABMapZigzagNode() : Node("rtabmap_zigzag") {
    // Parametre
    this->declare_parameter<std::string>("pose_topic", "/rtabmap/odom");
    this->declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");
    this->declare_parameter<std::string>("zephyr_cmd_topic", "/zephyr_controller/command");
    this->declare_parameter<double>("takeoff_duration_s", 3.0);
    this->declare_parameter<double>("vz_takeoff", 1.0);
    this->declare_parameter<double>("vx_forward", 1.0);
    this->declare_parameter<double>("vy_amplitude", 1.0);
    this->declare_parameter<double>("zig_period_s", 4.0);
    // Enkle gain-parametre til mapping Twist -> ZephyrController
    this->declare_parameter<double>("flap_gain", 1.0);
    this->declare_parameter<double>("prop_gain", 1.0);

    // Hent parametre
    this->get_parameter("takeoff_duration_s", takeoff_duration_s_);
    this->get_parameter("vz_takeoff", vz_takeoff_);
    this->get_parameter("vx_forward", vx_forward_);
    this->get_parameter("vy_amplitude", vy_amp_);
    this->get_parameter("zig_period_s", zig_T_);
    this->get_parameter("flap_gain", flap_gain_);
    this->get_parameter("prop_gain", prop_gain_);

    std::string pose_topic, cmd_vel_topic, zephyr_cmd_topic;
    this->get_parameter("pose_topic", pose_topic);
    this->get_parameter("cmd_vel_topic", cmd_vel_topic);
    this->get_parameter("zephyr_cmd_topic", zephyr_cmd_topic);

    // Sub til RTAB-Map's pose
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      pose_topic, 10,
      [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        last_pose_ = *msg;
        pose_pub_->publish(*msg);
      });

    // Publishers
    cmd_pub_    = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 10);
    pose_pub_   = create_publisher<geometry_msgs::msg::PoseStamped>("/nav_pkg/pose", 10);
    zephyr_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(zephyr_cmd_topic, 10);

    // Timer til zig-zag kommandoer
    start_time_ = now();
    cmd_timer_ = create_wall_timer(20ms, std::bind(&RTABMapZigzagNode::publishCommand, this));

    RCLCPP_INFO(get_logger(),
      "RTABMap Zigzag-node startet. Lytter på %s, publicerer cmd_vel på %s og zephyr cmds på %s",
      pose_topic.c_str(), cmd_vel_topic.c_str(), zephyr_cmd_topic.c_str());
  }

private:
  void publishCommand() {
    const double t = (now() - start_time_).seconds();
    geometry_msgs::msg::Twist cmd;

    if (t < takeoff_duration_s_) {
      cmd.linear.z = vz_takeoff_;
    } else {
      cmd.linear.x = vx_forward_;
      double phase = std::fmod(t - takeoff_duration_s_, 2.0 * zig_T_);
      cmd.linear.y = (phase < zig_T_) ? vy_amp_ : -vy_amp_;
    }

    // 1) Publicér som Twist (til debugging/nav2-kompatibilitet)
    cmd_pub_->publish(cmd);

    // 2) Direkte mapping til ZephyrController-format (Float64MultiArray)
    // [0]=flap_left_effort, [1]=flap_right_effort, [2]=propeller_velocity
    std_msgs::msg::Float64MultiArray zephyr_cmd;
    zephyr_cmd.data.resize(3);

    const double flap = flap_gain_ * cmd.linear.y;
    zephyr_cmd.data[0] = flap;        // venstre flap effort
    zephyr_cmd.data[1] = -flap;       // højre flap effort (modsat for differential)
    zephyr_cmd.data[2] = prop_gain_ * cmd.linear.x;  // propeller velocity

    zephyr_pub_->publish(zephyr_cmd);
  }

  // ROS interfaces
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr     pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr           cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr    zephyr_pub_;
  rclcpp::TimerBase::SharedPtr                                      cmd_timer_;

  // State
  rclcpp::Time start_time_;
  geometry_msgs::msg::PoseStamped last_pose_;

  // Parametre
  double takeoff_duration_s_{3.0};
  double vz_takeoff_{1.0};
  double vx_forward_{1.0};
  double vy_amp_{1.0};
  double zig_T_{4.0};
  double flap_gain_{1.0};
  double prop_gain_{1.0};
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RTABMapZigzagNode>());
  rclcpp::shutdown();
  return 0;
}
