#ifndef ROBOT_CMD_SUBSCRIBER_H
#define ROBOT_CMD_SUBSCRIBER_H

#include <functional>
#include <memory>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "robot_interface/msg/robot_state.hpp"
#include "iceoryx/v2.95.4/iceoryx_posh/popo/subscriber.hpp"
#include "iceoryx/v2.95.4/iceoryx_posh/runtime/posh_runtime.hpp"
#include "../../robot_interface/include/robot_state_protocols.h"

using std::placeholders::_1;

class Robot_State_Publisher : public rclcpp::Node {
public:
    Robot_State_Publisher(): Node("RobotCMD_Subscriber"), iceoryx_state_subscriber_({"ROBOT", "REAL", "ROBOT_STATE"}) {
        publisher_ = this->create_publisher<robot_interface::msg::RobotState>("ROS2_Robot_State", 10); // CHANGE
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(2), std::bind(&Robot_State_Publisher::timer_callback, this));
        receiver_thread_ = std::thread(&Robot_State_Publisher::thread_subscriber_function, this);
    }

private:
    void timer_callback() ; // CHANGE
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<robot_interface::msg::RobotState>::SharedPtr publisher_;
    iox::popo::Subscriber<Robot_State> iceoryx_state_subscriber_;
    std::thread receiver_thread_;
    void thread_subscriber_function();
    robot_interface::msg::RobotState robot_state_;
    std::mutex mutex_;
};

#endif
