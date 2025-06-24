#ifndef ROBOT_CMD_SUBSCRIBER_H
#define ROBOT_CMD_SUBSCRIBER_H

#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "robot_interface/msg/robot_cmd.hpp"
#include "iceoryx/v2.95.4/iceoryx_posh/popo/publisher.hpp"
#include "iceoryx/v2.95.4/iceoryx_posh/runtime/posh_runtime.hpp"
#include "../../robot_interface/include/robot_state_protocols.h"

using std::placeholders::_1;

class RobotCMD_Subscriber : public rclcpp::Node {
public:
    RobotCMD_Subscriber(): Node("RobotCMD_Subscriber"), iceoryx_motor_publisher_({"ROBOT","REAL","MOTOR_CMD"}){
        subscription_ = this->create_subscription<robot_interface::msg::RobotCMD>( // CHANGE
            "RobotCMD", 100, std::bind(&RobotCMD_Subscriber::topic_callback, this, _1));
    }

private:
    void topic_callback(const robot_interface::msg::RobotCMD &msg); // CHANGE

    rclcpp::Subscription<robot_interface::msg::RobotCMD>::SharedPtr subscription_; // CHANGE
    iox::popo::Publisher<Robot_Control_Motor_Cmd> iceoryx_motor_publisher_;
};

#endif
