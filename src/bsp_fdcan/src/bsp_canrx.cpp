#include <rclcpp/rclcpp.hpp>
#include "can_msgs/msg/frame.hpp"
#include "ros2_socketcan/socket_can_id.hpp"
#include "ros2_socketcan/socket_can_receiver.hpp"

using namespace drivers::socketcan;

class SocketCanReceiverNode : public rclcpp :: Node
{
    public:
        SocketCanReceiverNode(const std::string& node_name) : Node(node_name)
        {
            SocketCanReceiver receiver("can0", false);
            
            // Set CAN Filters
            try {
                receiver.SetCanFilters(filters);
            } catch (const std::runtime_error & e) {
                RCLCPP_ERROR(this->get_logger(), "Failed to set filters: %s", e.what());
            }

            // CAN Message Receive
            std::array<uint8_t, 8> data;
            while (rclcpp::ok()) {
                try {
                    CanId can_id = receiver.receive(data, std::chrono::seconds(10));
                    RCLCPP_INFO(this->get_logger(), "Received CAN Data: %d, %d, %d, %d, %d, %d, %d, %d", 
                                                                        data[0], data[1], data[2], data[3], 
                                                                        data[4], data[5], data[6], data[7]);
                    RCLCPP_INFO(this->get_logger(), "Received CAN ID: %d", can_id.identifier());
                    RCLCPP_INFO(this->get_logger(), "Received CAN Message Length: %d", can_id.length());
                } catch (const SocketCanTimeout & e) {
                    RCLCPP_ERROR(this->get_logger(), "Timeout: %s", e.what());
                } catch (const std::runtime_error & e) {
                    RCLCPP_ERROR(this->get_logger(), "Error: %s", e.what());
                }
            }
        }
    private:
        std::string filter_str = "0x00000000~0x000000FF";
        SocketCanReceiver::CanFilterList filters = SocketCanReceiver::CanFilterList::ParseFilters(filter_str);
};

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SocketCanReceiverNode>("bsp_canrx_node");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
