/**
 * @file connect_task.h
 * @author yuanluochen 
 * @brief 哨兵通信任务，接收上位机数据，并对上位机数据进行处理
 * @version 0.1
 * @date 2023-02-21
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#ifndef CONNECT_TASK_H
#define CONNECT_TASK_H

#include "rm_usart.h"
#include "kalman.h"

//初始化延时时间
#define CONNECT_TASK_INIT_TIME 150
//系统延时
#define CONNECT_CONTROL_TIME_MS 2

//连接模式
typedef enum
{
    fail,
}connect_mode_t;

//底盘运动数据结构体
typedef struct 
{
    fp32 vx_set;
    fp32 vy_set;
    fp32 angel_set;
}chassis_move_cmd_t;

//云台运动数据结构体
typedef struct 
{
    fp32 pitch_add_angle;
    fp32 yaw_add_angle;

    //kalman filer 结构体
    kalman pitch_kalman_filter;//pitch轴电机kalman filter结构体
    

}gimbal_movd_cmd_t;



//哨兵自动模式结构体
typedef struct 
{
    //底盘运动命令
    chassis_move_cmd_t chassis_cmd;
    //云台运动命令
    gimbal_movd_cmd_t gimbal_cmd;

    //上位机指针
    const vision_rxfifo_t* master_computer;


}connect_control_t;


void connect_task(void const* pvParameters);


#endif // !CONNECT_TASK_H
