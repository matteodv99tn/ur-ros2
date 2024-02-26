#include <cstdlib>
#include <fstream>
#include <map>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2/buffer_core.h"
#include "tf2_ros/transform_listener.h"

class ReadDataNode : public rclcpp::Node {
    std::ofstream                                                 joint_file;
    std::ofstream                                                 cart_file;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub;

    tf2::BufferCore            buffer;
    tf2_ros::TransformListener listener;

    const std::map<std::string, int> joint_map = {
            {"shoulder_pan_joint",  0},
            {"shoulder_lift_joint", 1},
            {"elbow_joint",         2},
            {"wrist_1_joint",       3},
            {"wrist_2_joint",       4},
            {"wrist_3_joint",       5}
    };

    void on_joint_state_received(const sensor_msgs::msg::JointState::SharedPtr msg) {
        std::vector<double> joint_values(6);
        for (int i = 0; i < 6; i++) {
            int idx           = joint_map.at(msg->name[i]);
            joint_values[idx] = msg->position[i];
        }

        joint_file << msg->header.stamp.sec * 1e9 + msg->header.stamp.nanosec << ", ";
        joint_file << joint_values[0] << ", " << joint_values[1] << ", "
                   << joint_values[2] << ", " << joint_values[3] << ", "
                   << joint_values[4] << ", " << joint_values[5] << std::endl;

        geometry_msgs::msg::TransformStamped transform;
        transform = buffer.lookupTransform("world", "ee_link", tf2::TimePointZero);
        cart_file << transform.header.stamp.sec * 1e9 + transform.header.stamp.nanosec
                  << ", ";
        cart_file << transform.transform.translation.x << ", "
                  << transform.transform.translation.y << ", "
                  << transform.transform.translation.z << ", "
                  << transform.transform.rotation.x << ", "
                  << transform.transform.rotation.y << ", "
                  << transform.transform.rotation.z << ", "
                  << transform.transform.rotation.w << std::endl;
    }

public:
    ReadDataNode() : Node("read_data"), listener(buffer, this) {
        std::string home_dir       = std::getenv("HOME");
        std::string joint_filename = home_dir + std::string("/joint_states.csv");
        std::string cart_filename  = home_dir + std::string("/cartesian.csv");

        joint_file.open(joint_filename);
        cart_file.open(cart_filename);

        sub = this->create_subscription<sensor_msgs::msg::JointState>(
                "/joint_states",
                10,
                std::bind(
                        &ReadDataNode::on_joint_state_received,
                        this,
                        std::placeholders::_1
                )
        );
    }

    ~ReadDataNode() {
        joint_file.close();
        cart_file.close();
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReadDataNode>());
    rclcpp::shutdown();
    return 0;
}
