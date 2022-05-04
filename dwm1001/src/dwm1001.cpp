#include <unistd.h>
#include <stdio.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "std_msgs/msg/int16.hpp"
#include "dwm1001_interface/msg/general_configuration.hpp"
#include "dwm1001_interface/msg/status.hpp"
#include "dwm1001_interface/msg/position.hpp"

#include "dwm1001/rs232.hpp"
#include "dwm1001/dwm1001_api.hpp"

#define TX_BUFFER_SIZE 64
#define RX_BUFFER_SIZE 512
#define COM_PORT_POLL_TIMER_DURATION 30
#define LOCATION_POLL_TIMER_DURATION 100
#define FLOW_CONTROL_ENABLED 0

class DWM1001 : public rclcpp_lifecycle::LifecycleNode
{
    public:
        DWM1001()
        : LifecycleNode("DWM1001")
        {
            RCLCPP_INFO(rclcpp::get_logger("Constructor"), "Node created");
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_configure(const rclcpp_lifecycle::State &)
        {
            RCLCPP_INFO(rclcpp::get_logger("on_configure"), "Configuring...");

            this->declare_parameter<std::string>("port_name");
            this->declare_parameter<std::int32_t>("baud_rate", 115200);
            this->declare_parameter<std::string>("port_mode", "8N1");

            this->get_parameter("port_name", portName);
            this->get_parameter("baud_rate", baudRate);
            this->get_parameter("port_mode", portMode);

            portNum = RS232_GetPortnr(portName.c_str());

            if(portNum == -1)
            {
                RCLCPP_FATAL(rclcpp::get_logger("on_configure"), "Unable to find serial port");
                return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
            }

            RCLCPP_INFO(rclcpp::get_logger("on_configure"), "Attempting to open port: %s\n\tNumber: %d", portName.c_str(), portNum);
            if(RS232_OpenComport(portNum, baudRate, portMode.c_str(), FLOW_CONTROL_ENABLED))
            {
                RCLCPP_FATAL(rclcpp::get_logger("on_configure"), "Failed to open serial port");
                return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
            }
            RCLCPP_INFO(rclcpp::get_logger("on_configure"), "Successfully opened serial port");

            createInterfaces();

            comPortPollTimer = rclcpp::create_timer(this, get_clock(), std::chrono::milliseconds(COM_PORT_POLL_TIMER_DURATION),
                    std::bind(&DWM1001::comPortTimerCallback, this));

            locationGetTimer = rclcpp::create_timer(this, get_clock(), std::chrono::milliseconds(LOCATION_POLL_TIMER_DURATION),
                    std::bind(&DWM1001::locationUpdateTimerCallback, this));

            RCLCPP_INFO(rclcpp::get_logger("on_configure"), "Configuration Complete");
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_activate(const rclcpp_lifecycle::State &)
        {
            RCLCPP_INFO(rclcpp::get_logger("on_activate"), "Activating...");

            errorCodePublisher->on_activate();
            configurationPublisher->on_activate();
            statusPublisher->on_activate();
            positionPublisher->on_activate();

            RCLCPP_INFO(rclcpp::get_logger("on_activate"), "Activation completed successfully");
            return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
        }

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
        on_deactivate(const rclcpp_lifecycle::State &)
        {
            RCLCPP_INFO(rclcpp::get_logger("on_deactivate"), "Deactivating...");

            errorCodePublisher->on_deactivate();
            configurationPublisher->on_deactivate();
            statusPublisher->on_deactivate();
            positionPublisher->on_deactivate();

            resetVariables();

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

        ~DWM1001()
        {
            RCLCPP_INFO(rclcpp::get_logger("Destructor"), "Shutting Down");

            resetVariables();
            
            RCLCPP_INFO(rclcpp::get_logger("Destructor"), "Shutdown complete. Goodbye!");
        }
    
    private:
        void createInterfaces()
        {
            errorCodePublisher = this->create_publisher<std_msgs::msg::Int16>("dwm1001/output/error", 10);
            configurationPublisher = this->create_publisher<dwm1001_interface::msg::GeneralConfiguration>("dwm1001/output/configuration", 10);
            statusPublisher = this->create_publisher<dwm1001_interface::msg::Status>("dwm1001/output/status", 10);
            positionPublisher = this->create_publisher<dwm1001_interface::msg::Position>("dwm1001/output/position", 10);
        }

        void resetVariables()
        {
            errorCodePublisher.reset();
            configurationPublisher.reset();
            statusPublisher.reset();
            positionPublisher.reset();

            if(portNum != -1)
            {
                RCLCPP_INFO(rclcpp::get_logger("resetVariables"), "Closing port");
                RS232_CloseComport(portNum);
            }
        }

        void comPortTimerCallback()
        {
            int len = 0;
            unsigned char val;

            if(this->get_current_state().id() != 3)     //3 is the "active" lifecycle state
            {
                RCLCPP_DEBUG(rclcpp::get_logger("comPortTimerCallback"), "Ignoring data while in current state");
                return;
            }

            if(activeReceiveType == DWM1001_TLV_TYPE_NONE)
            {
                len = RS232_PollComport(portNum, &val, 1);
                activeReceiveType = static_cast<DWM1001TLVTypes>(val);
                RCLCPP_DEBUG(rclcpp::get_logger("comPortTimerCallback"), "New Type: %d", activeReceiveType);
            }
            else if(activeReceiveLength == -1)
            {
                len = RS232_PollComport(portNum, &val, 1);
                activeReceiveLength = static_cast<int>(val);
                RCLCPP_DEBUG(rclcpp::get_logger("comPortTimerCallback"), "New Length: %d", activeReceiveLength);
            }
            else
            {
                if(activeReceiveLength == 0)
                {
                    resetReceiveDataVariables();
                    return;
                }
                else
                {
                    len = RS232_PollComport(portNum, activeReceiveValueBuffer + activeReceiveValueIndex, activeReceiveLength);
                    activeReceiveValueIndex += len;
                }

                if(activeReceiveValueIndex == activeReceiveLength)
                {
                    processResponseData();
                    resetReceiveDataVariables();
                }
            }
        }

        void locationUpdateTimerCallback()
        {
            unsigned char buf[2] = {DWM1001_TLV_TYPE_COMMAND_LOCATION_GET, 0x00};

            RS232_SendBuf(portNum, buf, 2);
        }

        void processResponseData()
        {
            switch(activeReceiveType)
            {
                case DWM1001_TLV_TYPE_RESPONSE_RETURN_VALUE:
                    /*std_msgs::msg::Int16 msg;
                    msg.data = static_cast<int16_t>(activeReceiveValueBuffer[0]);
                    errorCodePublisher->publish(msg);*/
                    break;
                case DWM1001_TLV_TYPE_RESPONSE_POSITION_XYZ:
                    DWM1001PositionData position;
                    position.fillData(activeReceiveValueBuffer);
                    positionPublisher->publish(fillPositionMessage(position));
                    break;
                case DWM1001_TLV_TYPE_RESPONSE_GENERAL_CONFIG:
                    DWM1001Configuration config;
                    config.fillData(activeReceiveValueBuffer);
                    configurationPublisher->publish(fillConfigurationMessage(config));
                    break;
                case DWM1001_TLV_TYPE_RESPONSE_RANGING_ANCHOR_DISTANCES:
                    break;
                case DWM1001_TLV_TYPE_RESPONSE_RANGING_ANCHOR_DISTANCES_AND_POSITIONS:
                    break;
                case DWM1001_TLV_TYPE_RESPONSE_USER_DATA:
                    break;
                case DWM1001_TLV_TYPE_RESPONSE_LABEL:
                    break;
                case DWM1001_TLV_TYPE_RESPONSE_ANCHOR_LIST:
                    break;
                case DWM1001_TLV_TYPE_RESPONSE_STATUS:
                    DWM1001Status status;
                    status.fillData(activeReceiveValueBuffer);
                    statusPublisher->publish(fillStatusMessage(status));
                    break;
                default:
                    break;
            }
        }

        dwm1001_interface::msg::GeneralConfiguration fillConfigurationMessage(DWM1001Configuration config)
        {
            dwm1001_interface::msg::GeneralConfiguration msg;

            msg.uwb_mode = config.uwbMode;
            msg.firmware_update_enabled = config.firmwareUpdateEnabled;
            msg.ble_enabled = config.bleEnabled;
            msg.led_enabled = config.ledEnabled;
            msg.encryption_enabled = config.encryptionEnabled;
            msg.location_engine_enabled = config.locationEngineEnabled;
            msg.low_power_mode = config.lowPowerMode;
            msg.measurement_mode = config.measurementMode;
            msg.stationary_mode_enabled = config.stationaryModeEnabled;
            msg.bridge_mode = config.bridgeMode;
            msg.initiator_mode = config.initiatorMode;
            msg.is_anchor = config.isAnchor;

            return msg;
        }

        dwm1001_interface::msg::Status fillStatusMessage(DWM1001Status status)
        {
            dwm1001_interface::msg::Status msg;

            msg.location_ready = status.locationReady;
            msg.uwb_mac_joined_network = status.uwbmacJoinedNetwork;
            msg.backhaul_data_ready = status.backhaulDataReady;
            msg.backhaul_status_changed = status.backhaulStatusChanged;
            msg.uwb_scan_ready = status.uwbScanReady;
            msg.user_data_ready = status.userDataReady;
            msg.user_data_sent = status.userDataSent;
            msg.firmware_update_in_progress = status.firmwareUpdateInProgress;

            return msg;
        }

        dwm1001_interface::msg::Position fillPositionMessage(DWM1001PositionData position)
        {
            dwm1001_interface::msg::Position msg;

            msg.x = position.x;
            msg.y = position.y;
            msg.z = position.z;
            msg.quality = position.quality;

            return msg;
        }

        void resetReceiveDataVariables()
        {
            activeReceiveType = DWM1001_TLV_TYPE_NONE;
            activeReceiveLength = -1;
            activeReceiveValueIndex = 0;
        }

        rclcpp::TimerBase::SharedPtr comPortPollTimer;
        rclcpp::TimerBase::SharedPtr locationGetTimer;
        rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Int16>::SharedPtr errorCodePublisher;
        rclcpp_lifecycle::LifecyclePublisher<dwm1001_interface::msg::GeneralConfiguration>::SharedPtr configurationPublisher;
        rclcpp_lifecycle::LifecyclePublisher<dwm1001_interface::msg::Status>::SharedPtr statusPublisher;
        rclcpp_lifecycle::LifecyclePublisher<dwm1001_interface::msg::Position>::SharedPtr positionPublisher;

        std::string portName;
        int portNum;
        int baudRate;
        std::string portMode;

        bool commandRunning;
        DWM1001TLVTypes activeReceiveType;
        int16_t activeReceiveLength;
        unsigned char activeReceiveValueBuffer[RX_BUFFER_SIZE];
        int16_t activeReceiveValueIndex = 0;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::executors::MultiThreadedExecutor exe;

    std::shared_ptr<DWM1001> lc_node =
        std::make_shared<DWM1001>();

    exe.add_node(lc_node->get_node_base_interface());

    exe.spin();
  rclcpp::shutdown();
    return 0;
}