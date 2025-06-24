#include "../include/robot_state_publisher.hpp"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    iox::runtime::PoshRuntime::initRuntime("ROS2_Robot_State_Node");
    rclcpp::spin(std::make_shared<Robot_State_Publisher>());
    rclcpp::shutdown();
    return 0;
}

void Robot_State_Publisher::timer_callback()  {
    std::lock_guard<std::mutex> lock(mutex_);
    RCLCPP_INFO_STREAM(this->get_logger(), "Publishing:");
    publisher_->publish(robot_state_);
}

void Robot_State_Publisher::thread_subscriber_function() {
    while (true) {
        iceoryx_state_subscriber_.take().and_then([this](auto &sample) {
            std::lock_guard<std::mutex> lock(mutex_);
            for (int i = 0; i < 3; i++) {
                robot_state_.acc[i] = sample->acc[i];
                robot_state_.gyro[i] = sample->gyro[i];
                robot_state_.quat[i] = sample->quat[i];
            }
            robot_state_.quat[3] = sample->quat[3];
            for (int i = 0; i < 18; i++) {
                robot_state_.q[i] = sample->q[i];
                robot_state_.qd[i] = sample->qd[i];
            }
        }).or_else([](auto &result) {
            if (result != iox::popo::ChunkReceiveResult::NO_CHUNK_AVAILABLE) {
                std::cout << "Error receiving chunk." << std::endl;
            }
        });
    }
}
