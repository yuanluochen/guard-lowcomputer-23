#include "serial.h"

static int SerialOpen(const char* path);
void SetBaudRate(struct termios& setSerialOptions, SerialSpace::BaudRate setBaudRate);
void SetFlowCtrl(struct termios& setSerialOptions, SerialSpace::FlowCtrl setFlowCtrl);
void SetDataBit(struct termios& setSerialOptions, SerialSpace::DataBit setDatabit);
void SetCheck(struct termios& setSerialOptions, SerialSpace::Check setCheck);
void SetStopBit(struct termios& setSerialOptions, SerialSpace::StopBit setStopBit);

SerialSpace::Serial::Serial(BaudRate baudRate,
                       FlowCtrl flowCtrl,
                       DataBit dataBit,
                       Check check,
                       StopBit stopBit,
                       std::string serialPath)
{
    this->baudRate = baudRate;
    this->flowCtrl = flowCtrl;
    this->dataBit = dataBit;
    this->check = check;
    this->stopBit = stopBit;
    this->serialPath = serialPath;
}

/**
 * @brief 设置串口地址
 * 
 * @param serialPath 串口地址
 */
void SerialSpace::Serial::setSerialPath(std::string serialPath)
{
    this->serialPath = serialPath;
    std::cout << "设置串口地址为 >> %s << " << this->serialPath.c_str() << std::endl;
}

/**
 * @brief 串口初始化，主要配置串口参数
 * 
 * @param baudRate 串口波特率
 * @param flowCtrl 流控制
 * @param databit 数据位长度
 * @param check 校验位
 * @param stopBit 停止位
 */
void SerialSpace::Serial::init(void)
{
    //打开串口
    this->fd = SerialOpen(this->serialPath.c_str());
    //配置串口
    if (tcgetattr(this->fd, &this->serialOptions) != 0)//判断串口是否调用成功
    {
        //调用失败
        perror("Setup Serial 1");
        exit(-1);//终止程序
    }
    //配置串口波特率
    SetBaudRate(this->serialOptions, this->baudRate);
    //配置串口数据流控制类型
    SetFlowCtrl(this->serialOptions, this->flowCtrl);
    //设置串口数据位
    SetDataBit(this->serialOptions, this->dataBit);
    //设置串口校验位
    SetCheck(this->serialOptions, this->check);
    //配置停止位
    SetStopBit(this->serialOptions, this->stopBit);

    // 修改输出模式，原始数据输出
    this->serialOptions.c_oflag &= ~OPOST;

    this->serialOptions.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    //设置等待时间和最小接收字节
    this->serialOptions.c_cc[VTIME] = 1; // 读取一个字节等待 1*(1 / 10)s
    this->serialOptions.c_cc[VMIN] = 1;  //读取字符最小个数为1

    //如果发生数据溢出，接收数据，但是不再读取，刷新收到数据但是不读
    tcflush(this->fd, TCIFLUSH);

    //激活配置
    if (tcsetattr(this->fd, TCSANOW, &this->serialOptions) != 0)
    {
        perror("com set error!\n");
        exit(-1);//报错直接退出
    }

}
//配置串口波特率
void SetBaudRate(struct termios& setSerialOptions, SerialSpace::BaudRate setBaudRate)
{
    cfsetispeed(&setSerialOptions, setBaudRate);     
    cfsetospeed(&setSerialOptions, setBaudRate);     
}
//配置串口控制模式
void SetFlowCtrl(struct termios& setSerialOptions, SerialSpace::FlowCtrl setFlowCtrl)
{
    // 修改控制模式，保证程序不会占用串口
    setSerialOptions.c_cflag |= CLOCAL;
    //修改控制模式，使得能够从串口中读取数据
    setSerialOptions.c_cflag |= CREAD;

    //设置数据流控制
    switch (setFlowCtrl)
    {
    case SerialSpace::NO_FLOW_CTRL: //不使用流控制
        setSerialOptions.c_cflag &= ~CRTSCTS;
        break;
    case SerialSpace::HARDWARE_FLOW_CTRL: //使用硬件流控制
        setSerialOptions.c_cflag |= CRTSCTS;
        break;
    case SerialSpace::SOFTWARE_FLOW_CTRL: //使用软件流控制
        setSerialOptions.c_cflag |= IXON | IXOFF | IXANY;
        break;
    }
}
//配置串口数据位
void SetDataBit(struct termios& setSerialOptions, SerialSpace::DataBit setDatabit)
{
    //屏蔽其他标志位
    setSerialOptions.c_cflag &= ~CSIZE;
    //设置数据位
    setSerialOptions.c_cflag |= setDatabit;
}
//配置串口校验位
void SetCheck(struct termios& setSerialOptions, SerialSpace::Check setCheck)
{
    switch (setCheck)
    {
    case SerialSpace::NO_PARITY: //无奇偶校验
        setSerialOptions.c_cflag &= ~PARENB;
        setSerialOptions.c_iflag &= ~INPCK;
        break;
    case SerialSpace::ODD_CHECK: //奇校验
        setSerialOptions.c_cflag |= (PARODD | PARENB);
        setSerialOptions.c_iflag |= INPCK;
        break;
    case SerialSpace::EVENNESS: //偶校验
        setSerialOptions.c_cflag |= PARENB;
        setSerialOptions.c_cflag &= ~PARODD;
        setSerialOptions.c_iflag |= INPCK;
        break;
    case SerialSpace::SPACE: //空格
        setSerialOptions.c_cflag &= ~PARENB;
        setSerialOptions.c_cflag &= ~CSTOPB;
        break;
    }
}
//配置停止位
void SetStopBit(struct termios& setSerialOptions, SerialSpace::StopBit setStopBit)
{
    // 设置停止位
    switch (setStopBit)
    {
    case SerialSpace::NO_STOP_BIT: //无停止位
        setSerialOptions.c_cflag &= ~CSTOPB;
        break;
    case SerialSpace::STOP_BIT: //设置停止位
        setSerialOptions.c_cflag &= CSTOPB;
        break;
    }
}
/**
 * @brief 串口开启函数
 * 
 * @param path 串口地址
 * @return 如果打开成功返回文件描述符，如果初始化失败，程序结束
 */
static int SerialOpen(const char* path)
{
    int fd = open(path, O_RDWR | O_NOCTTY | O_NDELAY);
    //验证串口文件是否打开
    if(fd < 0)
    {
        perror("Can't Open Serial port");
        exit(-1);//程序终止
    }
    // 恢复串口为阻塞状态
    if (fcntl(fd, F_SETFL, 0) < 0) { printf("fcntl failed!\n"); exit(-1);//终止程序
    }
    else
    {
        printf("fcntl=%d\n", fcntl(fd, F_SETFL, 0));
    }
    std::cout << "串口开辟成功开辟串口为 >> " << path << " <<" << 
    std::endl;
    return fd;
}

//串口数据发送函数
void SerialSpace::Serial::sendMessage(std::string txMessage, int dataLen)
{
    //将要发送的数据赋值到类内
    this->txMessage = txMessage;

    //发送数据
    int len = write(fd, this->txMessage.c_str(), dataLen);

    //判断数据是否发送成功
    if (dataLen == len)
    {
        std::cout << "message send successful" << std::endl;
        std::cout << "send message is " << this->txMessage << std::endl;
    }

}
std::string SerialSpace::Serial::receiveMessage(int dataLen)
{
    read(this->fd, (void *)this->rxMessage.c_str(), dataLen);
    return this->rxMessage;
}
//串口打开
void SerialSpace::Serial::serialOpen()
{
    this->fd = SerialOpen(this->serialPath.c_str());
}

//串口关闭
void SerialSpace::Serial::serialClose()
{
    close(this->fd);
}
// 读取串口发送数据
std::string SerialSpace::Serial::getTxMessage(void)
{
    return this->txMessage;
}
// 读取串口接收数据
std::string SerialSpace::Serial::getRxMessage(void)
{
    return this->rxMessage;
}



//默认将0xFF 0X42 赋值到字符串头
SerialSpace::SerialMessage::SerialMessage(float* serialBegin, int size)
{
    //判断指针是否为空
    if (serialBegin == nullptr)
    {
        std::cout << "指针为空" << std::endl;
        exit(-1);
    }
    //float 转为 char
    for(int i = 0; i < size; i++)
    {
        this->messageSwitch.floatMessage = serialBegin[i];
        for(int j = 0; j < sizeof(float);j++)
        {
            this->serialMessageBegin += this->messageSwitch.charMessage[j];
        }
    }
}

SerialSpace::SerialMessage::SerialMessage(void)
{
    const int beginsize(2); 
    float begin[beginsize]{0xff, 0x42};
    for(int i = 0; i < beginsize;i++)
    {
        this->messageSwitch.floatMessage = begin[i];
        for(int j = 0; j < sizeof(float);j++)
        {
            this->serialMessageBegin += this->messageSwitch.charMessage[j];
        }
    }
    std::cout << this->serialMessage << std::endl;

}
std::string SerialSpace::SerialMessage::messageToSerialMessage(std::string message)
{
    //数据转换
    this->message = message;
    this->serialMessage = this->serialMessageBegin + this->message;
    return this->serialMessage;
}

std::string SerialSpace::SerialMessage::serialMessageToMessage(std::string serialMessage)
{
    //提取信息
    this->serialMessage = serialMessage;
    this->message = this->serialMessage.substr(this->serialMessageBegin.size(), this->serialMessage.size());
    return this->message;
}