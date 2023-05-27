/**
 * @file vision_task.h
 * @author yuanluochen
 * @brief 解析视觉数据包，处理视觉观测数据，预测装甲板位置，以及计算弹道轨迹，进行弹道补偿
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
#include "referee.h"

//允许发弹角度误差
#define ALLOW_ATTACK_ERROR 0.04

//发弹判断计数次数
#define JUDGE_ATTACK_COUNT 2
//发弹停止判断计数次数
#define JUDGE_STOP_ATTACK_COUNT 5

//延时等待
#define VISION_SEND_TASK_INIT_TIME 401
//系统延时时间
#define VISION_SEND_CONTROL_TIME_MS 1

//延时等待
#define VISION_TASK_INIT_TIME 450
//系统延时时间
#define VISION_CONTROL_TIME_MS 1

//弧度制转角度制
#define RADIAN_TO_ANGLE (360 / (2 * PI))

//机器人红蓝id分界值，大于该值则机器人自身为蓝色，小于这个值机器人自身为红色
#define ROBOT_RED_AND_BLUE_DIVIDE_VALUE 100

//最大未接受到上位机数据的时间
#define MAX_UNRX_TIME 100

//IMU 到 枪口之间的竖直距离
#define IMU_TO_GUMPOINT_VERTICAK 0.05f
//IMU 到 枪口之间的竖直距离
#define IMU_TO_GUNPOINT_DISTANCE 0.20f

//弹速
#define BULLET_SPEED 25.0f

//空气阻力系数 K1 = (0.5 * density * C * S) / m
#define AIR_K1 0.09f
//初始子弹飞行迭代数值
#define T_0 0.0f
//迭代精度
#define PRECISION 0.000001f
//最小迭代差值
#define MIN_DELTAT 0.001f
//最大迭代次数
#define MAX_ITERATE_COUNT 20
//视觉计算时间
#define VISION_CALC_TIME 0.003f

//比例补偿器比例系数
#define ITERATE_SCALE_FACTOR 0.3f
//重力加速度
#define GRAVITY 9.78f

//固有时间偏移即上位机计算时间单位ms
#define TIME_BIAS 15

//ms转s
#ifndef TIME_MS_TO_S
#define TIME_MS_TO_S(ms) (fp32)(ms / 1000.0f)

#endif // !TIME_MS_TO_S(x)

//全圆弧度
#define ALL_CIRCLE (2 * PI)

//yaw轴电机到枪口的竖直高度
#define Z_STATIC 0.15f
//枪口前推距离
#define DISTANCE_STATIC 0.25f
//初始飞行时间
#define INIT_FILIGHT_TIME 0.5f

//发射枪管id
#define SHOOT_ID 1
//子弹类型
#define BULLET_TYPE 1




//接收数据状态
typedef enum
{
    //未读取
    UNLOADED,
    //已读取
    LOADED,
}receive_state_e;

//数据起始帧类型
typedef enum
{
    //下位机发送到上位机
    LOWER_TO_HIGH_HEAD = 0x5A,
    //上位机发送到下位机
    HIGH_TO_LOWER_HEAD = 0XA5,
}data_head_type_e;

//装甲板颜色
typedef enum
{
    RED = 0,
    BLUE = 1,
}robot_armor_color_e;

//id
typedef enum
{
    ID_OUTPOST = 0, // 前哨站
    ID_GUADE = 6,   // 哨兵
    ID_BASE = 7,    // 基地
} armor_id_e;

//装甲板数值
typedef enum
{
    ARMOR_NUM_BALANCE = 2,
    ARMOR_NUM_OUTPOST = 3,
    ARMOR_NUM_NORMAL = 4
}armor_num_e;


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

//向量结构体 
typedef struct
{
    fp32 x;
    fp32 y;
    fp32 z;
} vector_t;

// 发送数据包(紧凑模式下的结构体，防止因数据对齐引发的数据错位)
typedef struct __attribute__((packed))
{
    uint8_t header;
    uint8_t detect_color : 1; // 0-red 1-blue
    bool_t reset_tracker : 1;
    uint8_t reserved : 6;
    float roll;
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
    bool_t tracking : 1;
    uint8_t id : 3;         // 0-outpost 6-guard 7-base
    uint8_t armors_num : 3; // 装甲板数量 2-balance 3-outpost 4-normal
    uint8_t reserved : 1;
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
    float dz;
    uint16_t checksum;
} receive_packet_t;

//类型转化
typedef receive_packet_t target_data_t;

// 视觉接收结构体
typedef struct
{
    // 接收标志位
    uint8_t receive_state : 1;
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

//哨兵底盘控制命令
typedef struct 
{
    //我方机器人距离敌方机器人的距离
    fp32 distance;
}chassis_vision_control_t;



// 目标位置结构体
typedef struct
{
    float x;
    float y;
    float z;
    float yaw;
} target_position_t;

//弹道计算结构体
typedef struct
{
    // 当前弹速
    fp32 current_bullet_speed;
    // 当前pitch
    fp32 current_pitch;
    // 当前yaw
    fp32 current_yaw;
    // 弹道系数
    fp32 k1;
    //子弹飞行时间
    fp32 flight_time;
    //预测时间
    fp32 predict_time;

    // 目标yaw
    fp32 target_yaw;

    // 装甲板数量
    uint8_t armor_num;

    //yaw轴电机到枪口水平面的垂直距离
    fp32 z_static;
    //枪口前推距离
    fp32 distance_static;

    //所有装甲板位置指针
    target_position_t* all_target_position_point;

} solve_trajectory_t;



// 视觉任务结构体
typedef struct
{
    // 绝对角指针
    const fp32* vision_angle_point;
    // 当前弹速
    fp32 current_bullet_speed;
    // 检测装甲板的颜色(敌方装甲板的颜色)
    uint8_t detect_armor_color;

    //敌方机器人数据
    target_data_t target_data;
    //弹道解算
    solve_trajectory_t solve_trajectory;
    //目标位置
    target_position_t target_position;

    // 自身imu绝对角
    eular_angle_t imu_absolution_angle;
    // 视觉解算的绝对角
    eular_angle_t vision_absolution_angle;

    //机器人状态指针
    const ext_game_robot_state_t* robot_state_point;
    //发射机构弹速指针
    const ext_shoot_data_t* shoot_data_point;

    // 机器人云台瞄准位置向量
    vector_t robot_gimbal_aim_vector;

    //接收的数据包指针
    vision_receive_t* vision_receive_point; 
    //发送数据包
    send_packet_t send_packet;

    // 云台电机运动命令
    gimbal_vision_control_t gimbal_vision_control;
    // 发射机构发射命令
    shoot_vision_control_t shoot_vision_control;
    //底盘运动命令
    chassis_vision_control_t chassis_vision_control;

} vision_control_t;

// 视觉数据处理任务
void vision_task(void const *pvParameters);

// 获取上位机云台命令
const gimbal_vision_control_t *get_vision_gimbal_point(void);

// 获取上位机发射命令
const shoot_vision_control_t *get_vision_shoot_point(void);

// 获取上位机底盘控制命令
const chassis_vision_control_t* get_vision_chassis_point(void);
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

/**
 * @brief 将数据包通过usb发送到nuc
 * 
 * @param send 发送数据包
 */
void send_packet(vision_control_t* send);




#endif // !VISION_TASK_H
