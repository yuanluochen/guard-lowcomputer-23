#include "rclcpp/rclcpp.hpp"
#include "guard_interface/srv/serial_msg.hpp"
#include "serial.h"

class SerialMessageService : public rclcpp::Node
{
public:
    SerialMessageService(std::string serialName) : rclcpp::Node("serial_message_service_node")
    {
        SerialService_ = this->create_service<guard_interface::srv::SerialMsg>(serialName, std::bind(&SerialMessageService::MessageToSerialMessage, this, std::placeholders::_1, std::placeholders::_2));
        if(SerialService_ != nullptr)
        {
            //报告服务端建立
            RCLCPP_INFO(this->get_logger(), "串口信息服务端建立,服务端的service_name为 >> %s << ", serialName.c_str());
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "串口信息服务端建立失败，>> %s << 节点退出", serialName.c_str());
            exit(-1);
        }
    }
private:
    //真实数据转为串口发送数据
    void MessageToSerialMessage(guard_interface::srv::SerialMsg::Request::SharedPtr serialRequest, guard_interface::srv::SerialMsg::Response::SharedPtr serialResponse)
    {
        //处理数据转为串口发送数据
        serialResponse->serial_message = message.messageToSerialMessage(serialRequest->message);

    }
private:
    //创建服务指针
    rclcpp::Service<guard_interface::srv::SerialMsg>::SharedPtr SerialService_;
    //串口数据处理对象
    SerialSpace::SerialMessage message;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SerialMessageService>("serial_message_service"));
    rclcpp::shutdown();   
    return 0;
}
