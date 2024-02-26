#include "rclcpp/rclcpp.hpp"

class ReadDataNode : public rclcpp::Node {
public:
    ReadDataNode() : Node("read_data") {

    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReadDataNode>());
    rclcpp::shutdown();
    return 0;
}
