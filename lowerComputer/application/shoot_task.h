/**
 * @file shoot_task.h
 * @author yuanluochen 
 * @brief 哨兵发弹任务接收视觉任务指令发送弹丸
 * @version 0.1
 * @date 2023-05-11
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#ifndef SHOOT_TASK_H
#define SHOOT_TASK_H


#include "struct_typedef.h"

#include "CAN_receive.h"
#include "gimbal_task.h"
#include "remote_control.h"
#include "user_lib.h"
#include "vision_task.h"

//射击遥控器控制摩擦轮通道
#define SHOOT_MODE_CHANNEL 1
//射击遥控器控制拨弹盘通道
#define SHOOT_TRIGGER_CHANNEL 1
//射击遥控器控制射击控制方式的通道
#define SHOOT_CONTROL_CHANNEL 0

//发射任务延时时间 1ms
#define SHOOT_TASK_DELAY_TIME 1

// 发射任务时间转化 秒转毫秒
#define SHOOT_TASK_S_TO_MS(x) ((int32_t)((x * 1000.0f) / (SHOOT_TASK_DELAY_TIME)))

//发射任务最大时间，以秒为单位 20 s
#define SHOOT_TASK_MAX_INIT_TIME 10

//摩擦轮电机转速
#define FRIC_MOTOR_RUN_SPEED 2.9
//摩擦轮电机停止转速
#define FRIC_MOTOR_STOP_SPEED 0

//拨弹盘电机转速
#define TRIGGER_MOTOR_RUN_SPEED 10.0
//拨弹盘电机停转
#define TRIGGER_MOTOR_STOP_SPEED 0

#define GIMBAL_ModeChannel  1

#define SHOOT_CONTROL_TIME  0.02

#define RC_S_LONG_TIME 2000

#define PRESS_LONG_TIME 400

#define SHOOT_ON_KEYBOARD KEY_PRESSED_OFFSET_Q
#define SHOOT_OFF_KEYBOARD KEY_PRESSED_OFFSET_E


#define MAX_SPEED 15.0f //-12.0f
#define MID_SPEED 12.0f //-12.0f
#define MIN_SPEED 10.0f //-12.0f
#define Ready_Trigger_Speed 6.0f

#define Motor_RMP_TO_SPEED 0.00290888208665721596153948461415f
#define Motor_ECD_TO_ANGLE 0.000021305288720633905968306772076277f
#define FULL_COUNT 18

#define TRIGGER_ANGLE_PID_KP 900///2450.0f
#define TRIGGER_ANGLE_PID_KI 0.0f
#define TRIGGER_ANGLE_PID_KD 100.0f

#define TRIGGER_READY_PID_MAX_OUT 8000.0f
#define TRIGGER_READY_PID_MAX_IOUT 2500.0f

#define TRIGGER_BULLET_PID_MAX_OUT 15000.0f
#define TRIGGER_BULLET_PID_MAX_IOUT 5000.0f
// 摩擦轮电机最大输入转速
#define FRIC_MOTOR_PID_MAX_OUT 15000.0f
#define FRIC_MOTOR_PID_MAX_IOUT 5000.0f

#define Half_ecd_range 395  //395  7796
#define ecd_range 8191

#define S3505_MOTOR_SPEED_PID_KP 8700.f
#define S3505_MOTOR_SPEED_PID_KI  0.0f
#define S3505_MOTOR_SPEED_PID_KD  10.f
#define S3505_MOTOR_SPEED_PID_MAX_OUT 11000.0f
#define S3505_MOTOR_SPEED_PID_MAX_IOUT 1000.0f

#define PI_Four 0.78539816339744830961566084581988f
#define PI_Three 1.0466666666666f
#define PI_Ten 0.314f

#define TRIGGER_SPEED 3.0f
#define SWITCH_TRIGGER_ON 0

//哨兵热量上限
#define GUARD_MAX_MUZZLE_HEAT 240

//哨兵枪口热量距离最大值最大允许误差
#define GUARD_MAX_ALLOW_MUZZLE_HEAT_ERR0R 50

//卡弹电流
#define STUCK_CURRENT 7000
//卡弹时间
#define STUCK_TIME 800
//回拨时间
#define REVERSE_TIME 110

typedef enum
{
    SHOOT_STOP = 0,
    SHOOT_READY,
    SHOOT_BULLET,
    SHOOT_BULLET_ONE,
    SHOOT_DONE,
    SHOOT_INIT, //初始化模式
} shoot_mode_e;

//电机控制模式
typedef enum
{
    SHOOT_MOTOR_RUN,  // 电机运行
    SHOOT_MOTOR_STOP, // 电机停止
} shoot_motor_control_mode_e;


typedef enum
{
    SHOOT_AUTO_CONTROL,    //自动控制模式
    SHOOT_RC_CONTROL,      //遥控器控制模式
    SHOOT_INIT_CONTROL,    //初始化控制模式
    SHOOT_STOP_CONTROL,    //停止控制模式
}shoot_control_mode_e;

// 卡弹状态
typedef enum
{
    STUCK,   // 卡弹
    UNSTUCK, // 不卡弹
} stuck_state_e;

typedef struct
{
    //PID结构体

    PidTypeDef motor_pid;

    const motor_measure_t *shoot_motor_measure;
    fp32 speed;
    fp32 speed_set;
    fp32 angle;
    int8_t ecd_count;
    fp32 set_angle;
    int16_t given_current;

    bool_t press_l;
    bool_t press_r;
    bool_t last_press_l;
    bool_t last_press_r;
    uint16_t press_l_time;
    uint16_t press_r_time;
    uint16_t rc_s_time;

    uint32_t run_time;
    uint32_t cmd_time;
    int16_t move_flag;
    int16_t move_flag_ONE;
    bool_t key;
    int16_t BulletShootCnt;
    int16_t last_butter_count;
    fp32 shoot_CAN_Set_Current;
    fp32 blocking_angle_set;
    fp32 blocking_angle_current;
    int8_t blocking_ecd_count;

    stuck_state_e stuck_state; // 卡弹状态
    fp32 stuck_time;           // 卡弹时间
    fp32 reverse_time;         // 卡弹回拨时间

} Shoot_Motor_t;

typedef struct
{
    const motor_measure_t *fric_motor_measure;
    fp32 accel;
    fp32 speed;
    fp32 speed_set;
    int16_t give_current;
    uint16_t rc_key_time;
} fric_Motor_t;

//微动开关状态
typedef enum
{
    PRESS = GPIO_PIN_RESET,   // 按下
    RELEASE = GPIO_PIN_SET, // 松开
} micro_switch_state_e;

//初始化状态
typedef enum
{
    SHOOT_INIT_FINISH,   // 初始化完成
    SHOOT_INIT_UNFINISH, // 初始化未完成
} shoot_init_state_e;

typedef struct
{
    const RC_ctrl_t *shoot_rc;            // 遥控器
    const shoot_vision_control_t *shoot_vision_control; // 视觉控制指针

    shoot_mode_e fric_mode;               // 发射模式
    shoot_mode_e last_fric_mode;          // 上一次的发射模式
    fric_Motor_t motor_fric[2];           // 左右摩擦轮
    fp32 fric_CAN_Set_Current[2];         // can发射电流
    // PidTypeDef motor_speed_pid[4];        // 电机速度pid


    first_order_filter_type_t fric1_cmd_slow_set_speed; // 摩擦轮一阶低通
    first_order_filter_type_t fric2_cmd_slow_set_speed; // 摩擦轮一阶低通

    fp32 angle[2];
    int16_t ecd_count[2];
    int16_t given_current[2];
    fp32 set_angle[2];
    fp32 speed[2];
    fp32 speed_set[2];
    fp32 current_set[2];
    bool_t move_flag;

    fp32 min_speed;
    fp32 max_speed;
    int flag[2];
    int laster_add;
    
} fric_move_t;

extern void shoot_init(void);
extern void shoot_control_loop(void);



/**
 * @brief 射击任务函数
 * 
 * @param pvParameters 
 */
void shoot_task(void const *pvParameters);

//射击控制视觉任务
bool_t shoot_control_vision_task(void);


#endif
