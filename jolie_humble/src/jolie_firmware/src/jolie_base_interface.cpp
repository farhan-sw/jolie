#include "jolie_firmware/jolie_base_interface.hpp"
#include <hardware_interface/types/hardware_interface_type_values.hpp>


namespace jolie_firmware
{
    JolieBaseInterface::JolieBaseInterface()
    {

    }

    /**
     * @brief Destroy the Jolie Base Interface:: Jolie Base Interface object
     */
    JolieBaseInterface::~JolieBaseInterface()
    {
        // destructor
    }

    CallbackReturn JolieBaseInterface::on_init(const hardware_interface::HardwareInfo &hardware_info)
    {
        // on_init
        RCLCPP_INFO(rclcpp::get_logger("JolieInterface"), "Jolie Base Interface in on_init");

        /* ======================= PREPARE HARDWARE INFO ======================= */
        CallbackReturn result = hardware_interface::SystemInterface::on_init(hardware_info);
        if(result != CallbackReturn::SUCCESS)
        {
            return result;
        }
        /* ======================= END PREPARE HARDWARE INFO ======================= */


        /* ======================= VALIDATE SUM OF JOINTS ======================= */
        if (info_.joints.size() != 4) {
            RCLCPP_ERROR(rclcpp::get_logger("JolieInterface"), "Jolie Base Interface requires 4 joints, but got %zu", info_.joints.size());
            return CallbackReturn::FAILURE;
        }
        /* ======================= END VALIDATE SUM OF JOINTS ======================= */


        /* ======================= GET HARDWARE PARAMETERS ======================= */
        try {
            base_sub_front_left_topic_ = info_.hardware_parameters["base_sub_front_left_topic"];
            base_sub_front_right_topic_ = info_.hardware_parameters["base_sub_front_right_topic"];
            base_sub_back_left_topic_ = info_.hardware_parameters["base_sub_back_left_topic"];
            base_sub_back_right_topic_ = info_.hardware_parameters["base_sub_back_right_topic"];

            base_pub_front_left_topic_ = info_.hardware_parameters["base_pub_front_left_topic"];
            base_pub_front_right_topic_ = info_.hardware_parameters["base_pub_front_right_topic"];
            base_pub_back_left_topic_ = info_.hardware_parameters["base_pub_back_left_topic"];
            base_pub_back_right_topic_ = info_.hardware_parameters["base_pub_back_right_topic"];

            if (base_sub_front_left_topic_.empty() || base_sub_front_right_topic_.empty() ||
                base_sub_back_left_topic_.empty() || base_sub_back_right_topic_.empty() ||
                base_pub_front_left_topic_.empty() || base_pub_front_right_topic_.empty() ||
                base_pub_back_left_topic_.empty() || base_pub_back_right_topic_.empty()) {
                RCLCPP_ERROR(rclcpp::get_logger("JolieInterface"), "One or more topic names are empty");
                return CallbackReturn::FAILURE;
            }

        } catch (const std::out_of_range &e) {
            RCLCPP_ERROR(rclcpp::get_logger("JolieInterface"), "Failed to get required parameters from hardware_parameters: %s", e.what());
            return CallbackReturn::FAILURE;
        } catch (const std::invalid_argument &e) {
            RCLCPP_ERROR(rclcpp::get_logger("JolieInterface"), "Invalid argument while parsing parameters: %s", e.what());
            return CallbackReturn::FAILURE;
        } catch (...) {
            RCLCPP_ERROR(rclcpp::get_logger("JolieInterface"), "Unknown error occurred while getting parameters");
            return CallbackReturn::FAILURE;
        }
        /* ======================= END GET HARDWARE PARAMETERS ======================= */



        /* =========== SUBSCRIBE TO BASE TOPICS AND PUBLISH TO BASE TOPICS ======== */
        try {
            node_ = std::make_shared<rclcpp::Node>("my_base_hardware_interface_node");
            // Create Subscribers
            from_base_sub_front_left_ = node_->create_subscription<std_msgs::msg::Float32>(
            base_sub_front_left_topic_, 10, std::bind(&JolieBaseInterface::from_base_callback_front_left, this, std::placeholders::_1));
            from_base_sub_front_right_ = node_->create_subscription<std_msgs::msg::Float32>(
            base_sub_front_right_topic_, 10, std::bind(&JolieBaseInterface::from_base_callback_front_right, this, std::placeholders::_1));
            from_base_sub_back_left_ = node_->create_subscription<std_msgs::msg::Float32>(
            base_sub_back_left_topic_, 10, std::bind(&JolieBaseInterface::from_base_callback_back_left, this, std::placeholders::_1));
            from_base_sub_back_right_ = node_->create_subscription<std_msgs::msg::Float32>(
            base_sub_back_right_topic_, 10, std::bind(&JolieBaseInterface::from_base_callback_back_right, this, std::placeholders::_1));
            
            // Create Publishers
            to_base_pub_front_left_ = node_->create_publisher<std_msgs::msg::Float32>(base_pub_front_left_topic_, 10);
            to_base_pub_front_right_ = node_->create_publisher<std_msgs::msg::Float32>(base_pub_front_right_topic_, 10);
            to_base_pub_back_left_ = node_->create_publisher<std_msgs::msg::Float32>(base_pub_back_left_topic_, 10);
            to_base_pub_back_right_ = node_->create_publisher<std_msgs::msg::Float32>(base_pub_back_right_topic_, 10);
        } catch (const std::exception &e) {
            RCLCPP_ERROR(node_->get_logger(), "Exception caught while creating subscribers or publishers: %s", e.what());
        } catch (...) {
            RCLCPP_ERROR(node_->get_logger(), "Unknown exception caught while creating subscribers or publishers");
        }
        /* === END SUBSCRIBE TO BASE TOPICS AND PUBLISH TO BASE TOPICS ============== */

        
        /* ======================= INITIALIZE STATE AND COMMAND ======================= */
        RCLCPP_INFO(rclcpp::get_logger("JolieInterface"), "Number of joints: %zu", info_.joints.size());

        try {
            velocity_command_.resize(info_.joints.size(), 0.0);
            position_state_.resize(info_.joints.size(), 0.0);
            velocity_state_.resize(info_.joints.size(), 0.0);
        } catch (const std::exception &e) {
            RCLCPP_ERROR(rclcpp::get_logger("JolieInterface"), "Exception caught during vector reservation: %s", e.what());
            return CallbackReturn::FAILURE;
        } catch (...) {
            RCLCPP_ERROR(rclcpp::get_logger("JolieInterface"), "Unknown exception caught during vector reservation");
            return CallbackReturn::FAILURE;
        }
        /* ======================= END INITIALIZE STATE AND COMMAND ======================= */

        last_run_back_left_ = rclcpp::Clock().now();
        last_run_back_right_ = rclcpp::Clock().now();
        last_run_front_left_ = rclcpp::Clock().now();
        last_run_front_right_ = rclcpp::Clock().now();

        executor.add_node(node_);

        RCLCPP_INFO(rclcpp::get_logger("JolieInterface"), "Jolie Base Interface on_init success");

        return CallbackReturn::SUCCESS;
    }


    std::vector<hardware_interface::StateInterface> JolieBaseInterface::export_state_interfaces()
    {
        // export_state_interfaces
        std::vector<hardware_interface::StateInterface> state_interfaces;
        for(size_t i = 0; i < info_.joints.size(); i++)
        {
            state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, 
                hardware_interface::HW_IF_POSITION, &position_state_[i]));
            state_interfaces.emplace_back(hardware_interface::StateInterface(info_.joints[i].name, 
                hardware_interface::HW_IF_VELOCITY, &velocity_state_[i]));
        }

        return state_interfaces;
    }


    std::vector<hardware_interface::CommandInterface> JolieBaseInterface::export_command_interfaces()
    {
        std::vector<hardware_interface::CommandInterface> command_interfaces;
        for(size_t i = 0; i < info_.joints.size(); i++)
        {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(info_.joints[i].name, 
                hardware_interface::HW_IF_VELOCITY, &velocity_command_[i]));
        }

        return command_interfaces;
    }


    CallbackReturn JolieBaseInterface::on_activate(const rclcpp_lifecycle::State &previous_state)
    {
        RCLCPP_INFO(rclcpp::get_logger("JolieInterface"), "Jolie Base Interface in on_activate");

        // Create initial value
        velocity_command_ = {0.0, 0.0, 0.0, 0.0};
        position_state_ = {0.0, 0.0, 0.0, 0.0};
        velocity_state_ = {0.0, 0.0, 0.0, 0.0};

        // try to start all the pub and sub?

        RCLCPP_INFO(rclcpp::get_logger("JolieInterface"), "Hardware started, ready to publish");
        return CallbackReturn::SUCCESS;
    }


    CallbackReturn JolieBaseInterface::on_deactivate(const rclcpp_lifecycle::State &previous_state)
    {
        RCLCPP_INFO(rclcpp::get_logger("JolieInterface"), "Jolie Base Interface in on_deactivate");
        try{
            from_base_sub_front_left_.reset();
            from_base_sub_front_right_.reset();
            from_base_sub_back_left_.reset();
            from_base_sub_back_right_.reset();

            to_base_pub_front_left_.reset();
            to_base_pub_front_right_.reset();
            to_base_pub_back_left_.reset();
            to_base_pub_back_right_.reset();
            node_.reset();
        } catch (const std::exception &e) {
            RCLCPP_ERROR(node_->get_logger(), "Exception caught while cancelling node: %s", e.what());
            return CallbackReturn::FAILURE;
        } catch (...) {
            RCLCPP_ERROR(node_->get_logger(), "Unknown exception caught while cancelling node");
            return CallbackReturn::FAILURE;
        }
        return CallbackReturn::SUCCESS;

    }

    hardware_interface::return_type JolieBaseInterface::read(const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        executor.spin_some(std::chrono::milliseconds(0));

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type JolieBaseInterface::write(const rclcpp::Time &time, const rclcpp::Duration &period)
    {
        if (velocity_command_.size() < 4) {
            RCLCPP_ERROR(node_->get_logger(), "Velocity command size is less than 4");
            return hardware_interface::return_type::ERROR;
        }

        try
        {
            std_msgs::msg::Float32 msg;
            
            // Publish velocity commands to respective topics
            msg.data = velocity_command_.at(0);
            to_base_pub_front_left_->publish(msg);

            msg.data = velocity_command_.at(1);
            to_base_pub_front_right_->publish(msg);

            msg.data = velocity_command_.at(2);
            to_base_pub_back_left_->publish(msg);

            msg.data = velocity_command_.at(3);
            to_base_pub_back_right_->publish(msg);

        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(node_->get_logger(), "Exception caught while publishing message: %s", e.what());
        }
        catch (...)
        {
            RCLCPP_ERROR(node_->get_logger(), "Unknown exception caught while publishing message");
        }

        return hardware_interface::return_type::OK;
    }

    void JolieBaseInterface::from_base_callback_front_left(const std_msgs::msg::Float32::SharedPtr msg)
    {
        auto dt = (rclcpp::Clock().now() - last_run_front_left_).seconds();
        velocity_state_.at(0) = msg->data;
        position_state_.at(0) += velocity_state_.at(0) * dt;
        last_run_front_left_ = rclcpp::Clock().now();
    }

    void JolieBaseInterface::from_base_callback_front_right(const std_msgs::msg::Float32::SharedPtr msg)
    {
        auto dt = (rclcpp::Clock().now() - last_run_front_right_).seconds();
        velocity_state_.at(1) = msg->data;
        position_state_.at(1) += velocity_state_.at(1) * dt;
        last_run_front_right_ = rclcpp::Clock().now();
    }

    void JolieBaseInterface::from_base_callback_back_left(const std_msgs::msg::Float32::SharedPtr msg)
    {
        auto dt = (rclcpp::Clock().now() - last_run_back_left_).seconds();
        velocity_state_.at(2) = msg->data;
        position_state_.at(2) += velocity_state_.at(2) * dt;
        last_run_back_left_ = rclcpp::Clock().now();
    }

    void JolieBaseInterface::from_base_callback_back_right(const std_msgs::msg::Float32::SharedPtr msg)
    {
        auto dt = (rclcpp::Clock().now() - last_run_back_right_).seconds();
        velocity_state_.at(3) = msg->data;
        position_state_.at(3) += velocity_state_.at(3) * dt;
        last_run_back_right_ = rclcpp::Clock().now();
    }


}


#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(jolie_firmware::JolieBaseInterface, hardware_interface::SystemInterface)