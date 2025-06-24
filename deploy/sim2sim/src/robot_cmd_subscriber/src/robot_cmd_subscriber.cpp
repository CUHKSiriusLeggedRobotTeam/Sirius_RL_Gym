#include "../include/robot_cmd_subscriber.hpp"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    iox::runtime::PoshRuntime::initRuntime("ROS2_Robot_CMD_Node");
    rclcpp::spin(std::make_shared<RobotCMD_Subscriber>());
    rclcpp::shutdown();
    return 0;
}

void RobotCMD_Subscriber::topic_callback(const robot_interface::msg::RobotCMD &msg) {
    std::cout << "Get into callback\n";
    iceoryx_motor_publisher_.loan()
            .and_then([this, msg](auto &sample) {
                for (int i = 0; i < 18; i++) {
                    sample->q[i] = msg.q[i];
                    sample->qd[i] = msg.qd[i];
                    sample->kp[i] = msg.kp[i];
                    sample->kd[i] = msg.kd[i];
                    sample->tau_ff[i] = msg.tau_ff[i];
                }
                sample.publish();
            })
            .or_else([](auto &result) {
                std::cerr << "Unable to loan sample, error: " << result << std::endl;
            });
}
