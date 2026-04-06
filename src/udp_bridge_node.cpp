#include "capra_udp_bridge/udp_bridge.hpp"
#include <memory>

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);

    auto udp_bridge_node = std::make_shared<UdpBridgeNode>();
    
    rclcpp::spin(udp_bridge_node);
    
    rclcpp::shutdown();
    
    return EXIT_SUCCESS;
}