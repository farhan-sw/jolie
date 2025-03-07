#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

class TwistConverterNode : public rclcpp::Node
{
public:
  TwistConverterNode()
  : Node("twist_to_twiststamped_converter")
  {
    // Deklarasikan parameter untuk topik input, topik output, dan frame id
    this->declare_parameter<std::string>("input_topic", "/twist_mux/cmd_vel");
    this->declare_parameter<std::string>("output_topic", "/omni_wheel_controller/cmd_vel");
    this->declare_parameter<std::string>("frame_id", "base_link");

    input_topic_ = this->get_parameter("input_topic").as_string();
    output_topic_ = this->get_parameter("output_topic").as_string();
    frame_id_ = this->get_parameter("frame_id").as_string();

    // Buat publisher untuk TwistStamped
    publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(output_topic_, 10);
    // Buat subscriber untuk Twist
    subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
        input_topic_, 10,
        std::bind(&TwistConverterNode::twist_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "TwistConverterNode started. Subscribing to '%s' and publishing to '%s'",
                input_topic_.c_str(), output_topic_.c_str());
  }

private:
  void twist_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    // Buat pesan TwistStamped baru
    auto twist_stamped_msg = geometry_msgs::msg::TwistStamped();
    twist_stamped_msg.header.stamp = this->now();
    twist_stamped_msg.header.frame_id = frame_id_;
    twist_stamped_msg.twist = *msg;  // Copy semua field twist

    publisher_->publish(twist_stamped_msg);
    // RCLCPP_INFO(this->get_logger(), "Converted Twist to TwistStamped with frame_id: %s", frame_id_.c_str());
  }

  // Variabel untuk menyimpan parameter
  std::string input_topic_;
  std::string output_topic_;
  std::string frame_id_;

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
