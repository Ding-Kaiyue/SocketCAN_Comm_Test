#include <rclcpp/rclcpp.hpp>
#include "can_msgs/msg/frame.hpp"
#include "ros2_socketcan/socket_can_id.hpp"
#include "ros2_socketcan/socket_can_sender.hpp"

using namespace drivers::socketcan;

// 06 01 04 00 00 00 00 enable
class SocketCanSenderNode : public rclcpp :: Node 
{
    public:
        SocketCanSenderNode(const std::string& node_name) : Node(node_name)
        {
            SocketCanSender sender("can0", false);
            CanId canid(0x01, 0, FrameType::DATA, ExtendedFrame);

            uint8_t rawData[] = {0x06, 0x01, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00};
            std::size_t dataLength = sizeof(rawData);
            std::chrono::nanoseconds timeout = std::chrono::seconds(1);
            // while (rclcpp::ok()) {
            //     sender.send(rawData, dataLength, canid, timeout);
            //     rclcpp::sleep_for(std::chrono::seconds(1));
            // }
            sender.send(rawData, dataLength, canid, timeout);
        }
};

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SocketCanSenderNode>("bsp_cantx_node");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
