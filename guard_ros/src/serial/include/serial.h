/**
 * @file serial.h
 * @author yuanluochen
 * @brief linux下串口基础配置,配置一个串口的类驱动串口，实现串口读写
 * @version 0.1
 * @date 2023-01-05
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#ifndef SERIAL_H
#define SERIAL_H

//c++ 库
#include <iostream>
#include <string>

//c库
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <assert.h>
//系统串口库
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <termios.h>

#define SERIAL_PATH "/dev/ttyUSB0"

//建立串口命名空间
namespace SerialSpace
{
    //串口波特率
    typedef enum
    {
        SERIAL_BAUDRATE_115200 = B115200,
        SERIAL_BAUDRATE_19200 = B19200,
        SERIAL_BAUDRATE_9600 = B9600,
        SERIAL_BAUDRATE_4800 = B4800,
        SERIAL_BAUDRATE_2400 = B2400,
        SERIAL_BAUDRATE_1200 = B1200,
        SERIAL_BAUDRATE_300 = B300,
    } BaudRate;
     

    // 串口数据流控制配置
    typedef enum
    {
        NO_FLOW_CTRL = 0,       // 不使用流控制
        HARDWARE_FLOW_CTRL = 1, // 硬件流控制
        SOFTWARE_FLOW_CTRL = 2, // 软件流控制
    } FlowCtrl;

    // 数据位的位数
    typedef enum
    {
        DATA_5BIT = CS5,
        DATA_6BIT = CS6,
        DATA_7BIT = CS7,
        DATA_8BIT = CS8,
    } DataBit;

    // 校验位
    typedef enum
    {
        NO_PARITY = 'N', // 无奇偶校验
        ODD_CHECK = 'O', // 奇校验
        EVENNESS = 'E',  // 偶校验
        SPACE = 'S',     // 空格
    } Check;

    // 停止位
    typedef enum
    {
        NO_STOP_BIT = 0,
        STOP_BIT = 1
    } StopBit;


    class Serial
    {
    private:
        // 串口地址
        std::string serialPath;
        // 串口参数
        BaudRate baudRate; // 波特率
        FlowCtrl flowCtrl; // 控制类型
        DataBit dataBit;   // 数据位的位数
        Check check;       // 校验位
        StopBit stopBit;   // 停止位
        // 串口文件标识符
        int fd;
        //串口配置结构体
        struct termios serialOptions;  

        //串口发送数据
        std::string txMessage;
        //串口接收数据
        std::string rxMessage;
    public:
        //构造函数,赋值串口属性
        Serial(BaudRate baudRate,
               FlowCtrl flowCtrl,
               DataBit dataBit,
               Check check,
               StopBit stopBit,
               std::string serialPath = SERIAL_PATH);
    public:
        // 设置串口地址
        void setSerialPath(std::string serialPath);
        // 串口初始化,主要设置串口参数
        void init(void);//默认开辟串口路径为/dev/ttyUSB0
        //串口发送
        void sendMessage(std::string txMessage, int Datalen);
        //串口接收
        std::string receiveMessage(int dataLen);

        //串口打开和关闭
        void serialOpen();
        void serialClose();

        //读取发送
        std::string getTxMessage(void);

    };
}; // namespace Serial

#endif // !SERIAL_H
