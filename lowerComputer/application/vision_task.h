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

//允许发弹角度误差
#define ALLOW_ATTACK_ERROR 1

//发弹判断计数次数
#define JUDGE_ATTACK_COUNT 2
//发弹停止判断计数次数
#define JUDGE_STOP_ATTACK_COUNT 10

//延时等待
#define VISION_SEND_TASK_INIT_TIME 401
//系统延时时间
#define VISION_SEND_CONTROL_TIME_MS 2

//延时等待
#define VISION_TASK_INIT_TIME 450
//系统延时时间
#define VISION_CONTROL_TIME_MS 2

//弧度制转角度制
#define RADIAN_TO_ANGLE (360 / (2 * PI))

//最大未接受到上位机数据的时间
#define MAX_UNRX_TIME 400


//数据起始帧类型
typedef enum
{
    //下位机发送到上位机
    LOWER_TO_HIGH_HEAD = 0x5A,
    //上位机发送到下位机
    HIGH_TO_LOWER_HEAD = 0XA5,
}data_head_type_e;

//哨兵发射命令
typedef enum
{
    SHOOT_ATTACK,       // 袭击
    SHOOT_READY_ATTACK, // 准备袭击
    SHOOT_STOP_ATTACK,  // 停止袭击
} shoot_command_e;

//欧拉角结构体
typedef struct
{
    fp32 yaw;
    fp32 pitch;
    fp32 roll;
} eular_angle_t;

// 发送数据包(紧凑模式下的结构体，防止因数据对齐引发的数据错位)
typedef struct __attribute__((packed))
{
    uint8_t header;
    uint8_t robot_color : 1;
    uint8_t task_mode : 2;
    uint8_t reserved : 5;
    float pitch;
    float yaw;
    float aim_x;
    float aim_y;
    float aim_z;
    uint16_t checksum;
} send_packet_t;

// 接收数据包(紧凑模式下的结构体，防止因数据对齐引发的数据错位)
typedef struct __attribute__((packed))
{
    uint8_t header;
    bool_t tracking;
    float x;
    float y;
    float z;
    float yaw;
    float vx;
    float vy;
    float vz;
    float v_yaw;
    float r1;
    float r2;
    float z_2;
    uint16_t checksum;
} receive_packet_t;

// 视觉发送结构体
typedef struct
{
    //陀螺仪绝对角指针
    const fp32* INS_angle_point;
    // 欧拉角
    eular_angle_t eular_angle;
    // 发送数据包
    send_packet_t send_packet;
} vision_send_t;

// 视觉接收结构体
typedef struct
{
    // 接收标志位
    bool_t rx_flag;
    // 接收数据包
    receive_packet_t receive_packet;
} vision_receive_t;

// 哨兵云台电机运动命令,经滤波处理后的数值
typedef struct
{
    // 本次云台yaw轴数值
    fp32 gimbal_yaw;
    // 本次云台pitch轴数值
    fp32 gimbal_pitch;
} gimbal_vision_control_t;

// 哨兵发射电机运动控制命令
typedef struct
{
    // 自动发射命令
    shoot_command_e shoot_command;
} shoot_vision_control_t;

// 视觉任务结构体
typedef struct
{

    // 自身绝对角指针
    const fp32 *vision_angle_point;

    // 绝对角
    eular_angle_t absolution_angle;

    //接收的数据包指针
    const receive_packet_t* receive_packet_point; 

    // 云台电机运动命令
    gimbal_vision_control_t gimbal_vision_control;

    // 发射机构发射命令
    shoot_vision_control_t shoot_vision_control;

} vision_control_t;

// 视觉通信任务
void vision_send_task(void const *pvParameters);

// 视觉数据处理任务
void vision_task(void const *pvParameters);

// 获取上位机云台命令
gimbal_vision_control_t *get_vision_gimbal_point(void);

// 获取上位机发射命令
shoot_vision_control_t *get_vision_shoot_point(void);

/**
 * @brief 判断未接收到上位机数据
 *
 * @return 返回1 未接收到， 返回0 接收到
 */
bool_t judge_not_rx_vision_data(void);

/**
 * @brief 分析视觉原始增加数据，根据原始数据，判断是否要进行发射
 * 
 * @param shoot_judge 视觉结构体
 * @param vision_begin_add_yaw_angle 上位机视觉yuw轴原始增加角度
 * @param vision_begin_add_pitch_angle 上位机视觉pitch轴原始增加角度
 */
void vision_shoot_judge(vision_control_t* shoot_judge, fp32 vision_begin_add_yaw_angle, fp32 vision_begin_add_pitch_angle);

/**
 * @brief 接收数据解码
 * 
 * @param buf 接收到的数据
 * @param len 接收到的数据长度
 */
void receive_decode(uint8_t* buf, uint32_t len);

#endif // !VISION_TASK_H
