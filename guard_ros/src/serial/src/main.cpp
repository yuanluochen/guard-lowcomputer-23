#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "serial.h"

class serialTx : public rclcpp::Node
{
private:
    SerialSpace::Serial serial;
    std::string topicName;
public:
    serialTx(std::string topicName) : Node("serial_cpp_node"), serial(SerialSpace::SERIAL_BAUDRATE_115200, SerialSpace::NO_FLOW_CTRL, SerialSpace::DATA_8BIT, SerialSpace::NO_PARITY, SerialSpace::NO_STOP_BIT)
    {
        this->topicName = topicName;
        //创建订阅者
        this->create_subscription<std_msgs::msg::String>(this->topicName, 10, std::bind(&serialTx::messageHandle, this, std::placeholders::_1));
        //串口初始化
        serial.init();
        RCLCPP_INFO(this->get_logger(), "串口节点创建完毕，节点订阅话题为 >> %s <<", this->topicName.c_str());
    }

private:
    void messageHandle(std_msgs::msg::String topicMessage)
    {
        //读取获取节点数据，节点数据发送到串口
        serial.sendMessage(topicMessage.data, strlen(topicMessage.data.c_str()));
        //打印发送的数据
        RCLCPP_INFO(this->get_logger(), "发送的数据为 >> %s <<\n", topicMessage.data.c_str());
    }
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<serialTx>((char*)"serial"));
    rclcpp::shutdown();
    return 0;
}