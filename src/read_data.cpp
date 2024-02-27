#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <map>
#include <string>
#include <sys/stat.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "tf2/buffer_core.h"
#include "tf2_ros/transform_listener.h"

class ReadDataNode : public rclcpp::Node {
    std::uint64_t                                                 first_joint_time = 0;
    std::uint64_t                                                 first_cart_time  = 0;
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
        std::vector<double> joint_velocities(6);
        for (int i = 0; i < 6; i++) {
            int idx               = joint_map.at(msg->name[i]);
            joint_values[idx]     = msg->position[i];
            joint_velocities[idx] = msg->velocity[i];
        }

        std::uint64_t curr_time =
                msg->header.stamp.sec * 1e9 + msg->header.stamp.nanosec;
        if (first_joint_time == 0) { first_joint_time = curr_time; }

        joint_file << curr_time - first_joint_time << ", ";
        joint_file << joint_values[0] << ", " << joint_values[1] << ", "
                   << joint_values[2] << ", " << joint_values[3] << ", "
                   << joint_values[4] << ", " << joint_values[5] << ", "
                   << joint_velocities[0] << ", " << joint_velocities[1] << ", "
                   << joint_velocities[2] << ", " << joint_velocities[3] << ", "
                   << joint_velocities[4] << ", " << joint_velocities[5] << std::endl;

        geometry_msgs::msg::TransformStamped transform;
        transform = buffer.lookupTransform("world", "tool0", tf2::TimePointZero);
        curr_time = transform.header.stamp.sec * 1e9 + msg->header.stamp.nanosec;
        if (first_cart_time == 0) { first_cart_time = curr_time; }
        cart_file << curr_time - first_cart_time << ", ";
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
        std::string dmps_dir       = home_dir + std::string("/dmps");
        std::string data_dir       = dmps_dir + std::string("/data");
        mkdir(dmps_dir.c_str(), 0777);
        mkdir(data_dir.c_str(), 0777);

        std::string joint_filename = data_dir + std::string("/joint_states.csv");
        std::string cart_filename  = data_dir + std::string("/cartesian.csv");

        joint_file.open(joint_filename);
        cart_file.open(cart_filename);

        joint_file << "time, q1, q2, q3, q4, q5, q6, v1, v2, v3, v4, v5, v6"
                   << std::endl;
        cart_file << "time, x, y, z, qx, qy, qz, qw" << std::endl;

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
