/**
 * @file vision_task.h
 * @author yuanluochen
 * @brief 发送自身yaw轴pitch轴roll轴绝对角度给上位机视觉，并解算视觉接收数据，由于hal库自身原因对全双工串口通信支持不是特别好，
 *        为处理这个问题将串口数据分析，与串口发送分离，并延长串口发送时间
 * @version 0.1
 * @date 2023-03-11
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#ifndef VISION_TASK_H
#define VISION_TASK_H

#include "usart.h"
#include "dma.h"
#include "INS_task.h"
#include "arm_math.h"
#include "rm_usart.h"

//允许发弹角度误差
#define ALLOW_ATTACK_ERROR 7

//发弹判断计数次数
#define JUDGE_ATTACK_COUNT 2
//发弹停止判断计数次数
#define JUDGE_STOP_ATTACK_COUNT 100

//延时等待
#define VISION_SEND_TASK_INIT_TIME 401
//系统延时时间
#define VISION_SEND_CONTROL_TIME_MS 2

//延时等待
#define VISION_TASK_INIT_TIME 450
//系统延时时间
#define VISION_CONTROL_TIME_MS 2

//加倍
#define DOUBLE_ 10000
//弧度制转角度制
#define RADIAN_TO_ANGLE (360 / (2 * PI))


//数据分离uint8_t的数组的大小
#define UINT8_T_DATA_SIZE 4

//向上位机发送数据为处理符号做加pi处理
#define SEND_MESSAGE_ERROR PI

//最大未接受到上位机数据的时间
#define MAX_UNRX_TIME 400

//发送尾帧数据
#define END_DATA 0x0A

//数据起始帧类型
typedef enum
{
    //下位机发送到上位机
    LOWER_TO_HIGH_HEAD = 0x34,
    //上位机发送到下位机
    HIGH_TO_LOWER_HEAD = 0X44,
}data_head_type_e;


//上位机模式,包括装甲板模式,能量机关模式,前哨站模式
typedef enum
{
    ARMOURED_PLATE_MODE = 1, //装甲板模式
    ENERGY_ORGAN_MODE = 2,   //能量机关
    OUTPOST_MODE = 3,        //前哨站
}highcomputer_mode_e;

//发送数据偏移量
enum
{
    HEAD_ADDRESS_OFFSET = 0,                                                   // 起始帧
    DATA_QUAT_REAL_ADDRESS_OFFSET,                                             // 数据段 四元数实部
    DATA_QUAT_X_ADDRESS_OFFSET = DATA_QUAT_REAL_ADDRESS_OFFSET + sizeof(fp32), // 数据段 四元数x轴虚部
    DATA_QUAT_Y_ADDRESS_OFFSET = DATA_QUAT_X_ADDRESS_OFFSET + sizeof(fp32),    // 数据段 四元数y轴虚部
    DATA_QUAT_Z_ADDRESS_OFFSET = DATA_QUAT_Y_ADDRESS_OFFSET + sizeof(fp32),    // 数据段 四元数z轴虚部
    MODE_SWITCH_ADDRESS_OFFSET = DATA_QUAT_Z_ADDRESS_OFFSET + sizeof(fp32),    // 模式切换段
    CHECK_BIT_ADDRESS_OFFSET,                                                  // 校验段
    END_ADDRESS_OFFSET,                                                        // 尾帧
    SUM_SEND_MESSAGE,                                                          // 总值
};



//上下位机通信数据
typedef struct 
{
    //起始帧
    uint8_t head;
    //数据段-四元数段
    fp32 quat[4];
    //模式切换段
    uint8_t mode_change;
    //校验段
    uint8_t check;
    //尾
    uint8_t end;
}communication_data_t;


typedef struct 
{
    fp32 yaw;
    fp32 pitch;
    fp32 roll;
}eular_angle_t;

//哨兵云台电机运动命令,经滤波处理后的数值
typedef struct 
{
    //本次云台yaw轴数值
    fp32 gimbal_yaw;
    //本次云台pitch轴数值
    fp32 gimbal_pitch;
}gimbal_vision_control_t;

//哨兵发射命令
typedef enum
{
    SHOOT_ATTACK,       // 袭击
    SHOOT_READY_ATTACK, // 准备袭击
    SHOOT_STOP_ATTACK,  // 停止袭击
} shoot_command_e;

//哨兵发射电机运动控制命令
typedef struct 
{
    //自动发射命令
    shoot_command_e shoot_command;
}shoot_vision_control_t;

//视觉任务结构体
typedef struct
{
    // 上位机视觉指针
    vision_rxfifo_t *vision_rxfifo;
    
    // 自身绝对角指针
    const fp32* vision_angle_point;
    
    // 绝对角
    eular_angle_t absolution_angle;

    // 云台电机运动命令
    gimbal_vision_control_t gimbal_vision_control;    

    //发射机构发射命令
    shoot_vision_control_t shoot_vision_control;

} vision_control_t;

//视觉发送任务结构体
typedef struct
{
    // 四元数指针
    const fp32* vision_quat_point;
    //发送数据结构体
    communication_data_t send_msg_struct;
    // 配置串口发送数据
    uint8 send_message[SUM_SEND_MESSAGE];
} vision_send_t;

//视觉接收任务结构体
typedef struct 
{
    //接收数据结构体
    communication_data_t receive_msg_struct;
    //接收到数据标志位
    bool_t rx_flag;
}vision_receive_t;


//视觉通信任务
void vision_send_task(void const *pvParameters);

//视觉数据处理任务
void vision_task(void const* pvParameters);

//上位机原始数据解码
void highcomputer_rx_decode(uint8_t* rx_Buf, uint32_t* rx_buf_Len);

// 获取上位机云台命令
gimbal_vision_control_t* get_vision_gimbal_point(void);

// 获取上位机发射命令
shoot_vision_control_t* get_vision_shoot_point(void);

/**
 * @brief 判断未接收到上位机数据
 * 
 * @return 返回1 未接收到， 返回0 接收到 
 */
bool_t judge_not_rx_vision_data(void);

#endif // !VISION_TASK_H
