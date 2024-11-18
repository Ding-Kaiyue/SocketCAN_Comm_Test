#include <rclcpp/rclcpp.hpp>
#include "can_msgs/msg/frame.hpp"
#include "ros2_socketcan/socket_can_id.hpp"
#include "ros2_socketcan/socket_can_sender.hpp"


using namespace drivers::socketcan;

class SocketCanSenderNode : public rclcpp :: Node 
{
    public:
        SocketCanSenderNode(const std::string& node_name) : Node(node_name)
        {
            // SocketCanSender sender("can0", true);
            sender_ = std::make_unique<SocketCanSender>("can0", true);
            canid_ = CanId(0x1F, 0, FrameType::DATA, ExtendedFrame);

            uint8_t rawData[] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};
            dataLength_ = sizeof(rawData);
            timeout_ = std::chrono::seconds(1);
            
            timer_ = this->create_wall_timer(std::chrono::seconds(2), 
                                             [this, rawData]() {
                                                this->timer_callback(rawData);
                                             }
            );
        }

    private:
        rclcpp::TimerBase::SharedPtr timer_;
        std::unique_ptr<SocketCanSender> sender_;
        CanId canid_;
        size_t dataLength_;
        std::chrono::nanoseconds timeout_;

        void timer_callback(const uint8_t* rawData_)
        {
            sender_->send_fd(rawData_, dataLength_, canid_, timeout_);
            RCLCPP_INFO(this->get_logger(), "CAN FD frame sent.");
        }
};

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SocketCanSenderNode>("bsp_fdcantx_node");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

