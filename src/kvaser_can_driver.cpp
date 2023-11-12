#include "kvaser_can_driver_node.h"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<KvaserCanDriver>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}