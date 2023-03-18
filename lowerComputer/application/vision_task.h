/**
 * @file vision_task.h
 * @author yuanluochen
 * @brief 视觉任务
 * @version 0.1
 * @date 2023-03-11
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#ifndef VISION_TASK_H
#define VISION_TASK_H

#include "usart.h"
#include "INS_task.h"
#include "arm_math.h"
#include "kalman.h"
#include "rm_usart.h"
#include "pid.h"

//延时等待
#define VISION_TASK_INIT_TIME 401
//系统延时时间
#define VISION_CONTROL_TIME_MS 2

//串口发送数据大小
#define SERIAL_SEND_MESSAGE_SIZE 28
//加倍
#define DOUBLE_ 10000
//上位机串口结构体
#define VISION_USART huart1
//串口阻塞时间
#define VISION_USART_TIME_OUT 1000
//弧度制转角度制
#define RADIAN_TO_ANGle (360 / (2 * PI))

//一阶kalman filter参数
#define GIMBAL_YAW_MOTOR_KALMAN_Q 400
#define GIMBAL_YAW_MOTOR_KALMAN_R 400

#define GIMBAL_PITCH_MOTOR_KALMAN_Q 200
#define GIMBAL_PITCH_MOTOR_KALMAN_R 400


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
    kalman_filter_init_t gimbal_pitch_second_order_kalman;
    kalman_filter_init_t gimbal_yaw_second_order_kalman;
}vision_kalman_filter_t;

//哨兵云台电机运动命令,经滤波处理后的数值
typedef struct 
{
    //本次云台yaw轴增量
    fp32 gimbal_yaw_add;
    //本次云台pitch轴增量
    fp32 gimbal_pitch_add;
}gimbal_motor_command_t;

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
    //自身绝对角指针
    const fp32* vision_angle_point;
    //绝对角
    eular_angle_t absolution_angle; 
    //配置串口发送数据
    uint8 send_message[SERIAL_SEND_MESSAGE_SIZE];

    //上位机视觉指针
    vision_rxfifo_t* vision_rxfifo;

    //kalman filter 结构体，用于处理视觉上位机发来的数据
    vision_kalman_filter_t vision_kalman_filter;

    //pid 结构体，用于提高上位机视觉响应
    vision_pid_t vision_pid;
    
    //云台电机运动命令
    gimbal_motor_command_t gimbal_motor_command;    
    
}vision_t;


//视觉通信任务
void vision_task(void const *pvParameters);
//获取上位机命令
vision_t* get_vision_point(void);


#endif // !VISION_TASK_H
