#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <iostream>
#include <memory>
#include <thread>
#include <type_traits>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "std_msgs/msg/u_int16.hpp"
#include "can_interface/msg/can_frame.hpp"
#include "dwm1001_interface/msg/position.hpp"

using std::placeholders::_1;

class DWM1001CAN : public rclcpp_lifecycle::LifecycleNode
{
    public:
        explicit DWM1001CAN()
        : LifecycleNode("DWM1001CAN")
        {
            RCLCPP_INFO(rclcpp::get_logger("Constructor"), "Node Created");
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_configure(const rclcpp_lifecycle::State &)
        {
            RCLCPP_INFO(rclcpp::get_logger("on_configure"), "Configuring...");

            this->declare_parameter<std::int32_t>("can_id", 1);

            this->get_parameter("can_id", canId);

            RCLCPP_INFO(rclcpp::get_logger("on_configure"), "Using Can Id: %d", canId);

            createInterfaces();

            RCLCPP_INFO(rclcpp::get_logger("on_configure"), "Configuration Complete");
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_activate(const rclcpp_lifecycle::State &)
        {
            RCLCPP_INFO(rclcpp::get_logger("on_activate"), "Activating...");

            positionPublisher->on_activate();

            RCLCPP_INFO(rclcpp::get_logger("on_activate"), "Activation completed successfully");
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_deactivate(const rclcpp_lifecycle::State &)
        {
            RCLCPP_INFO(rclcpp::get_logger("on_deactivate"), "Deactivating...");

            positionPublisher->on_deactivate();

            RCLCPP_INFO(rclcpp::get_logger("on_deactivate"), "Deactivation completed successfully");
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_cleanup(const rclcpp_lifecycle::State &)
        {
            RCLCPP_INFO(rclcpp::get_logger("on_cleanup"), "Cleaning Up...");

            resetVariables();
            
            RCLCPP_INFO(rclcpp::get_logger("on_cleanup"), "Cleanup completed successfully");

            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_shutdown(const rclcpp_lifecycle::State &)
        {
            RCLCPP_INFO(rclcpp::get_logger("on_shutdown"), "Shutting Down...");

            resetVariables();

            RCLCPP_INFO(rclcpp::get_logger("on_shutdown"), "Shut down completed successfully");

            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        ~DWM1001CAN()
        {
            RCLCPP_INFO(rclcpp::get_logger("Destructor"), "Shutting Down");

            resetVariables();
            
            RCLCPP_INFO(rclcpp::get_logger("Destructor"), "Shutdown complete. Goodbye!");
        }

    private:
        void createInterfaces()
        {
            canDataSubscription = this->create_subscription<can_interface::msg::CanFrame>(
                "battery/input/can", 50, std::bind(&DWM1001CAN::canDataReceived, this, _1));
            positionPublisher = this->create_publisher<dwm1001_interface::msg::Position>("dwm1001/output/position", 10);
        }

        void resetVariables()
        {
            canDataSubscription.reset();
            positionPublisher.reset();
        }

        enum messageTypes
        {
            DWM1001_CAN_MESSAGE_TYPE_VERSION = 0,
            DWM1001_CAN_MESSAGE_TYPE_POS     = 1
        };

        void canDataReceived(const can_interface::msg::CanFrame & message)
        {
            uint32_t messageType;
            dwm1001_interface::msg::Position msgOut;

            if(this->get_current_state().id() != 3)     //3 is the "active" lifecycle state
            {
                RCLCPP_DEBUG(rclcpp::get_logger("canDataReceived"), "Ignoring data while in current state");
                return;
            }

            if(message.can_id >> 5 != (uint32_t) canId)
            {
                RCLCPP_DEBUG(rclcpp::get_logger("canDataReceived"), "Ignoring data that is not for this node");
                return;
            }

            messageType = message.can_id & 0b00011111;

            switch((messageTypes) messageType)
            {
                case DWM1001_CAN_MESSAGE_TYPE_VERSION:
                    RCLCPP_INFO(rclcpp::get_logger("canDataReceived"), "DWM1001 CAN Interface Version: %d.%d.%d build %d",
                            message.data[3], message.data[2], message.data[1], message.data[0]);
                    break;
                case DWM1001_CAN_MESSAGE_TYPE_POS:
                    msgOut.x = message.data[0] << 8 | message.data[1];
                    msgOut.y = message.data[2] << 8 | message.data[3];
                    msgOut.z = message.data[4] << 8 | message.data[5];
                    msgOut.delta_time = message.data[6] << 8 | message.data[7];

                    positionPublisher->publish(msgOut);
                    break;
                default:
                    break;
            }
        }

        rclcpp::Subscription<can_interface::msg::CanFrame>::SharedPtr canDataSubscription;

        rclcpp_lifecycle::LifecyclePublisher<dwm1001_interface::msg::Position>::SharedPtr positionPublisher;

        int32_t canId;
        uint16_t xPos   = 0;
        uint16_t yPos   = 0;
        uint16_t zPos   = 0;
        uint16_t deltaT = 0;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor exe;

    std::shared_ptr<DWM1001CAN> lc_node =
        std::make_shared<DWM1001CAN>();

    exe.add_node(lc_node->get_node_base_interface());

    exe.spin();
  rclcpp::shutdown();
    return 0;
}