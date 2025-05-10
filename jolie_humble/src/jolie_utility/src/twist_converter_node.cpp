#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

class TwistConverterNode : public rclcpp::Node
{
public:
  TwistConverterNode()
  : Node("twist_to_twiststamped_converter")
  {
    // Deklarasikan parameter untuk topik input, topik output, frame id, dan faktor pengali k
    this->declare_parameter<std::string>("input_topic", "/twist_mux/cmd_vel");
    this->declare_parameter<std::string>("output_topic", "/omni_wheel_controller/cmd_vel");
    this->declare_parameter<std::string>("frame_id", "base_link");
    this->declare_parameter<double>("k", -1.0);  // Parameter pengali default -1

    input_topic_ = this->get_parameter("input_topic").as_string();
    output_topic_ = this->get_parameter("output_topic").as_string();
    frame_id_ = this->get_parameter("frame_id").as_string();
    k_ = this->get_parameter("k").as_double();

    // Buat publisher untuk TwistStamped
    publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(output_topic_, 10);
    // Buat subscriber untuk Twist
    subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
        input_topic_, 10,
        std::bind(&TwistConverterNode::twist_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "TwistConverterNode started. Subscribing to '%s' and publishing to '%s' with k=%f",
                input_topic_.c_str(), output_topic_.c_str(), k_);
  }

private:
  void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    // Buat pesan TwistStamped baru
    auto twist_stamped_msg = geometry_msgs::msg::TwistStamped();
    twist_stamped_msg.header.stamp = this->now();
    twist_stamped_msg.header.frame_id = frame_id_;

    // Kalikan setiap komponen linear dan angular dengan k_
    twist_stamped_msg.twist.linear.x = msg->linear.x * k_;
    twist_stamped_msg.twist.linear.y = msg->linear.y * k_;
    twist_stamped_msg.twist.linear.z = msg->linear.z * k_;

    twist_stamped_msg.twist.angular.x = msg->angular.x * k_;
    twist_stamped_msg.twist.angular.y = msg->angular.y * k_;
    twist_stamped_msg.twist.angular.z = msg->angular.z * k_;

    publisher_->publish(twist_stamped_msg);

    // Optional: uncomment untuk debug
    // RCLCPP_INFO(this->get_logger(), "Published TwistStamped with scaled values (k=%f)", k_);
  }

  // Variabel untuk menyimpan parameter
  std::string input_topic_;
  std::string output_topic_;
  std::string frame_id_;
  double k_;

  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr publisher_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TwistConverterNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
