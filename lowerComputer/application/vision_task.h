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
#include "kalman.h"
#include "rm_usart.h"
#include "pid.h"

//外部变量
extern DMA_HandleTypeDef hdma_usart1_tx;

/**
 * @brief kalman filter 类型
 *        0 为二阶
 *        1 为一阶 
 * 
 */
#define KALMAN_FILTER_TYPE 0

//运算时间差
#define DT 2
//矩阵大小
#define MATRIX_SIZE 4

//pitch轴位移方差
#define PITCH_DP 1
//pitch轴速度方差
#define PITCH_DV 1
//pitch轴加速度方差
#define PITCH_DA 1

//yaw轴位移方差
#define YAW_DP 1
//yaw轴速度方差
#define YAW_DV 1
//yaw轴加速度方差
#define YAW_DA 1

//滤波后的角度偏移量
#define GIMBAL_ANGLE_ADDRESS_OFFSET 0


//一阶kalman filter参数
#define GIMBAL_YAW_MOTOR_KALMAN_Q 400
#define GIMBAL_YAW_MOTOR_KALMAN_R 400

#define GIMBAL_PITCH_MOTOR_KALMAN_Q 200
#define GIMBAL_PITCH_MOTOR_KALMAN_R 400


//允许发弹角度误差
#define ALLOW_ATTACK_ERROR 5

//发弹判断计数次数
#define JUDGE_ATTACK_COUNT 20
//发弹停止判断计数次数
#define JUDGE_STOP_ATTACK_COUNT 20

//延时等待
#define VISION_SEND_TASK_INIT_TIME 401
//系统延时时间
#define VISION_SEND_CONTROL_TIME_MS 10

//延时等待
#define VISION_TASK_INIT_TIME 450
//系统延时时间
#define VISION_CONTROL_TIME_MS 2

//串口发送数据大小
#define SERIAL_SEND_MESSAGE_SIZE 28
//加倍
#define DOUBLE_ 10000
//上位机串口结构体
#define VISION_USART huart1
//上位机串口dma结构体
#define VISION_TX_DMA hdma_usart1_tx
//串口阻塞时间
#define VISION_USART_TIME_OUT 1000
//弧度制转角度制
#define RADIAN_TO_ANGle (360 / (2 * PI))

#define HEAD1_DATA 0x34
#define HEAD2_DATA 0X43

//起始位1偏移量
#define HEAD1_ADDRESS_OFFSET 0
//起始位2偏移量
#define HEAD2_ADDRESS_OFFSET 1
//yaw轴偏移量
#define YAW_ADDRESS_OFFSET 2
//pitch轴偏移量
#define PITCH_ADDRESS_OFFSET 3
//roll轴偏移量
#define ROLL_ADDRESS_OFFSET 4
//模式转换偏移量
#define SWITCH_ADDRESS_OFFSET 5
//结尾数据偏移量
#define END_ADDRESS_OFFSET 6

//数据分离uint8_t的数组的大小
#define UINT8_T_DATA_SIZE 4

//向上位机发送数据为处理符号做加pi处理
#define SEND_MESSAGE_ERROR PI


//上位机模式,包括装甲板模式,能量机关模式,前哨站模式
typedef enum
{
    ARMOURED_PLATE = 1, //装甲板模式
    ENERGY_ORGAN = 2,   //能量机关
    OUTPOST = 3,        //前哨站
}vision_mode_e;

//32位数据转8位数据
typedef union 
{
    float fp32_val;
    uint32_t uint32_val;
    uint8_t uin8_value[UINT8_T_DATA_SIZE];
    
}date32_to_date8_t;

typedef struct 
{
    fp32 yaw;
    fp32 pitch;
    fp32 roll;
}eular_angle_t;


//视觉kalman filter结构体
typedef struct 
{
    //一维kalman
    kalman gimbal_pitch_kalman;  //pitch轴电机kalman filter结构体    
    kalman gimbal_yaw_kalman;    //云台yaw轴电机kalman filter结构体
    kalman disdance_kalman;      //距离结构体    

    //二阶kalman filter,主要处理云台电机数据
    kalman_filter_t gimbal_pitch_second_order_kalman;
    kalman_filter_t gimbal_yaw_second_order_kalman;

    //二阶kalman filter 初始化结构体，用于给矩阵运算库的矩阵赋值
    kalman_filter_init_t gimbal_pitch_second_order_kalman_init;
    kalman_filter_init_t gimbal_yaw_second_order_kalman_init;
}vision_kalman_filter_t;

//哨兵云台电机运动命令,经滤波处理后的数值
typedef struct 
{
    //本次云台yaw轴增量
    fp32 gimbal_yaw_add;
    //本次云台pitch轴增量
    fp32 gimbal_pitch_add;
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

//哨兵云台pid结构体
typedef struct 
{
    //云台pitch轴pid
    pid_type_def gimbal_pitch_pid;
    //云台yaw轴pid
    pid_type_def gimbal_yaw_pid;
}vision_pid_t;

//视觉任务结构体
typedef struct
{
    // 上位机视觉指针
    vision_rxfifo_t *vision_rxfifo;
    
    // 自身绝对角指针
    const fp32* vision_angle_point;
    
    // 绝对角
    eular_angle_t absolution_angle;

    // kalman filter 结构体，用于处理视觉上位机发来的数据
    vision_kalman_filter_t vision_kalman_filter;

    // 云台电机运动命令
    gimbal_vision_control_t gimbal_vision_control;    

    //发射机构发射命令
    shoot_vision_control_t shoot_vision_control;

} vision_control_t;

//视觉发送任务结构体
typedef struct
{
    
    // 自身绝对角指针
    const fp32 *vision_angle_point;
    // 绝对角
    eular_angle_t absolution_angle;
    // 发送的绝对角数据，为消除负号对原始数据做加pi处理
    eular_angle_t send_absolution_angle;
    // 配置串口发送数据
    uint8 send_message[SERIAL_SEND_MESSAGE_SIZE];
    // 配置串口名称(该为地址)
    UART_HandleTypeDef *send_message_usart;
    // 配置串口发送dma
    DMA_HandleTypeDef *send_message_dma;

}vision_send_t;


//视觉通信任务
void vision_send_task(void const *pvParameters);

//视觉数据处理任务
void vision_task(void const* pvParameters);

// 获取上位机云台命令
gimbal_vision_control_t* get_vision_gimbal_point(void);

// 获取上位机发射命令
shoot_vision_control_t* get_vision_shoot_point(void);

#endif // !VISION_TASK_H
