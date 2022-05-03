#include <unistd.h>
#include <stdio.h>

#include "rclcpp/rclcpp.hpp"
#include "dwm1001/rs232.hpp"
#include "dwm1001/dwm1001_api.hpp"

#define TX_BUFFER_SIZE 64
#define RX_BUFFER_SIZE 512
#define POLL_TIMER_DURATION 30
#define FLOW_CONTROL_ENABLED 0

class DWM1001 : public rclcpp::Node
{
    public:
        DWM1001()
        : Node("DWM1001")
        {
            std::string portName = "ttyS0";
            portNum = RS232_GetPortnr(portName.c_str());
            baudRate = 115200;
            portMode = "8N1";

            RCLCPP_INFO(rclcpp::get_logger("Constructor"), "Attempting to open port: %s\n\tNumber: %d", portName.c_str(), portNum);
            if(RS232_OpenComport(portNum, baudRate, portMode.c_str(), FLOW_CONTROL_ENABLED))
            {
                RCLCPP_ERROR(rclcpp::get_logger("Constructor"), "Failed to open serial port");
                return;
            }
            RCLCPP_INFO(rclcpp::get_logger("Constructor"), "Successfully opened serial port");

            unsigned char cmd[] = {DWM1001_TLV_TYPE_COMMAND_VERSION_GET, 0x00};
            RCLCPP_INFO(rclcpp::get_logger("Constructor"), "Sending command");
            RS232_SendBuf(portNum, cmd, 2);

            timer = this->create_wall_timer(std::chrono::milliseconds(POLL_TIMER_DURATION), std::bind(&DWM1001::timer_callback, this));
        }

        ~DWM1001()
        {
            RCLCPP_INFO(rclcpp::get_logger("Destructor"), "Closing serial port");
            RS232_CloseComport(0);
        }
    
    private:
        void timer_callback()
        {
            unsigned char c[RX_BUFFER_SIZE];
            int len = 0;

            if((len = RS232_PollComport(0, c, RX_BUFFER_SIZE)) > 0)
            {
                for(int i = 0; i < len; i++)
                {
                    RCLCPP_INFO(rclcpp::get_logger("timer_callback"), "Received: 0x%X", c[i]);
                }
            }
            
        }

        rclcpp::TimerBase::SharedPtr timer;
        int portNum;
        int baudRate;
        std::string portMode;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DWM1001>());
    rclcpp::shutdown();
    return 0;
}