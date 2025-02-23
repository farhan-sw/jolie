#ifndef JOLIE_BASE_INTERFACE_HPP_
#define JOLIE_BASE_INTERFACE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors.hpp>
#include <hardware_interface/system_interface.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <std_msgs/msg/float32.hpp>
#include <vector>
#include <string>


namespace jolie_firmware
{
    // using CallbackReturn
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    class JolieBaseInterface : public hardware_interface::SystemInterface
    {
        public:
            JolieBaseInterface();
            virtual ~JolieBaseInterface();

            virtual CallbackReturn on_activate(const rclcpp_lifecycle::State &previous_state) override;
            virtual CallbackReturn on_deactivate(const rclcpp_lifecycle::State &previous_state) override;
            virtual CallbackReturn on_init(const hardware_interface::HardwareInfo &hardware_info) override;

            virtual std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
            virtual std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

            virtual hardware_interface::return_type read(const rclcpp::Time &time, const rclcpp::Duration &period) override;
            virtual hardware_interface::return_type write(const rclcpp::Time &time, const rclcpp::Duration &period) override;
            
            
        private:
            std::shared_ptr<rclcpp::Node> node_;
            rclcpp::executors::SingleThreadedExecutor executor;

            rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr from_base_sub_front_left_;
            rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr from_base_sub_front_right_;
            rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr from_base_sub_back_left_;
            rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr from_base_sub_back_right_;

            rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr to_base_pub_front_left_;
            rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr to_base_pub_front_right_;
            rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr to_base_pub_back_left_;
            rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr to_base_pub_back_right_;

            void from_base_callback_front_left(const std_msgs::msg::Float32::SharedPtr msg);
            void from_base_callback_front_right(const std_msgs::msg::Float32::SharedPtr msg);
            void from_base_callback_back_left(const std_msgs::msg::Float32::SharedPtr msg);
            void from_base_callback_back_right(const std_msgs::msg::Float32::SharedPtr msg);

            std::string base_sub_front_left_topic_;
            std::string base_sub_front_right_topic_;
            std::string base_sub_back_left_topic_;
            std::string base_sub_back_right_topic_;

            std::string base_pub_front_left_topic_;
            std::string base_pub_front_right_topic_;
            std::string base_pub_back_left_topic_;
            std::string base_pub_back_right_topic_;

            std::vector<double> velocity_command_;
            std::vector<double> position_state_;
            std::vector<double> velocity_state_;

            rclcpp::Time last_run_front_left_;
            rclcpp::Time last_run_front_right_;
            rclcpp::Time last_run_back_left_;
            rclcpp::Time last_run_back_right_;
    };
}  // namespace jolie_firmware



#endif  // NEBULA_BASE_INTERFACE_HPP_