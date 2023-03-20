/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       shoot_task.c/h
  * @brief      射击功能.
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-21-2022     LYH              1. 完成
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  */

#include "shoot_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "bsp_laser.h"
#include "bsp_fric.h"
#include "arm_math.h"
#include "user_lib.h"
#include "referee.h"
#include "CAN_receive.h"
#include "gimbal_behaviour.h"
#include "detect_task.h"
#include "pid.h"
#include "stm32.h"
/*----------------------------------宏定义---------------------------*/
#define shoot_laser_on() laser_on()                                              // 激光开启宏定义
#define shoot_laser_off() laser_off()                                            // 激光关闭宏定义
#define BUTTEN_TRIG_PIN HAL_GPIO_ReadPin(BUTTON_TRIG_GPIO_Port, BUTTON_TRIG_Pin) // 微动开关IO
#define trigger_motor(speed) trigger_control.trigger_speed_set = speed           // 开启拨弹电机
/*----------------------------------内部函数---------------------------*/
/**
 * @brief   射击状态机
 */
// void Choose_Shoot_Mode(void);
static void Shoot_Set_Mode(void);
static void shoot_level(void);
/**
 * @brief   射击数据更新
 */
static void Shoot_Feedback_Update(void);
/** @brief   摩擦轮控制循环
 */
static void fric_control_loop(fric_move_t *fric_move_control_loop);
static void shoot_fric_off(fric_move_t *fric1_off);

/**
 * @brief  射击控制，控制拨弹电机角度，完成一次发射
 */
static void shoot_bullet_control(void);
/**
 * @brief  射击初始化
 */
void shoot_init(void);

/*----------------------------------内部变量---------------------------*/
fp32 fric;
int jam_flag = 0;
// uint16_t ShootSpeed;
fp32 KH = 0;
int flag = 0;
int time_l = 0;
int flag1 = 0;
int trigger_flag = 0, trigger_flag1 = 0;
int add_t = 0;
/*----------------------------------结构体------------------------------*/
static PidTypeDef trigger_motor_pid;
Shoot_Motor_t trigger_motor;              // 拨弹轮数据
fric_move_t fric_move;                    // 摩擦轮数据
shoot_mode_e shoot_mode = SHOOT_STOP;     // 此次射击模式
shoot_mode_e last_fric_mode = SHOOT_STOP; // 上次射击模式
/*----------------------------------外部变量---------------------------*/
extern ExtY_stm32 stm32_Y;
extern ExtU_stm32 stm32_U;
/*---------------------------------------------------------------------*/

/**
 * @brief          射击任务，间隔 GIMBAL_CONTROL_TIME 1ms
 * @param[in]      pvParameters: 空
 * @retval         none
 */
void shoot_task(void const *pvParameters)
{
    vTaskDelay(GIMBAL_TASK_INIT_TIME);
    shoot_init();
    while (1)
    {
        // shoot_laser_on();
        //设置发射等级
        shoot_level();
        //设置发射模式
        Shoot_Set_Mode();
        Shoot_Feedback_Update();
        shoot_control_loop();
        CAN_cmd_shoot(fric_move.fric_CAN_Set_Current[0], fric_move.fric_CAN_Set_Current[1], trigger_motor.given_current, 0);
        vTaskDelay(1);
    }
}
/**
 * @brief          射击初始化，初始化PID，遥控器指针，电机指针
 * @param[in]      void
 * @retval         返回空
 */
void shoot_init(void)
{
    fric_move.laster_add = 0;
    trigger_motor.move_flag = 1;
    trigger_motor.move_flag_ONE = 1;
    // 初始化PID
    stm32_shoot_pid_init();
    static const fp32 Trigger_speed_pid[3] = {900, 0, 100};
    PID_Init(&trigger_motor_pid, PID_POSITION, Trigger_speed_pid, TRIGGER_READY_PID_MAX_OUT, TRIGGER_READY_PID_MAX_IOUT);
    const static fp32 motor_speed_pid[3] = {S3505_MOTOR_SPEED_PID_KP, S3505_MOTOR_SPEED_PID_KI, S3505_MOTOR_SPEED_PID_KD};
    PID_Init(&fric_move.motor_speed_pid[0], PID_POSITION, motor_speed_pid, S3505_MOTOR_SPEED_PID_MAX_OUT, S3505_MOTOR_SPEED_PID_MAX_IOUT);
    PID_Init(&fric_move.motor_speed_pid[1], PID_POSITION, motor_speed_pid, S3505_MOTOR_SPEED_PID_MAX_OUT, S3505_MOTOR_SPEED_PID_MAX_IOUT);
    fric_move.motor_speed_pid[0].mode_again = KI_SEPRATE;
    fric_move.motor_speed_pid[1].mode_again = KI_SEPRATE;
    // 数据指针获取
    fric_move.shoot_rc = get_remote_control_point();
    trigger_motor.shoot_motor_measure = get_trigger_motor_measure_point();
    trigger_motor.blocking_angle_set = 0;
    fric_move.motor_fric[0].fric_motor_measure = get_shoot_motor_measure_point(0); // 右摩擦轮
    fric_move.motor_fric[1].fric_motor_measure = get_shoot_motor_measure_point(1); // 左摩擦轮
    // 滤波初始化
    const static fp32 fric_1_order_filter[1] = {0.1666666667f};
    const static fp32 fric_2_order_filter[1] = {0.1666666667f};
    first_order_filter_init(&fric_move.fric1_cmd_slow_set_speed, SHOOT_CONTROL_TIME, fric_1_order_filter);
    first_order_filter_init(&fric_move.fric2_cmd_slow_set_speed, SHOOT_CONTROL_TIME, fric_2_order_filter);
    // 速度限幅
    fric_move.max_speed = 4.75f;
    fric_move.min_speed = -4.75f;
    Shoot_Feedback_Update();
    trigger_motor.set_angle = trigger_motor.angle;
}
/**
 * @brief          射击等级设置
 * @param[in]      void
 * @retval         返回空
 */
void shoot_level(void)
{
    KH = 1.0;
    // ShootSpeed = 30;
    fric = 2.9f;
    trigger_motor.speed_set = 8.3;
}

/**
 * @brief          射击数据更新
 * @param[in]      void
 * @retval         void
 */
static void Shoot_Feedback_Update(void)
{
    uint8_t i;
    // 长按计时，更新标志位，控制单点单发
    if (shoot_mode != SHOOT_STOP && (abs(fric_move.shoot_rc->rc.ch[4]) >= 100 || fric_move.shoot_rc->mouse.press_l))
    {
        shoot_mode = SHOOT_BULLET;
        if (last_fric_mode != SHOOT_BULLET)
        {
            last_fric_mode = SHOOT_BULLET;
        }
        time_l++;
    }
    else
    {
        time_l = 0, flag1 = 0;
        flag = 0;
    }

    if (time_l > 350)
    {
        flag = 0;
        flag1 = 0;
    }
    else if (shoot_mode == SHOOT_BULLET)
    {
        flag = 1;
    }

    if (flag == 1 && flag1 == 0)
    {
        trigger_motor.set_angle = rad_format(trigger_motor.set_angle + PI_Four);
        flag1 = 1;
    }
    // 滤波————>拨弹轮
    static fp32 speed_fliter_1 = 0.0f;
    static fp32 speed_fliter_2 = 0.0f;
    static fp32 speed_fliter_3 = 0.0f;
    static const fp32 fliter_num[3] = {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f};
    speed_fliter_1 = speed_fliter_2;
    speed_fliter_2 = speed_fliter_3;
    speed_fliter_3 = speed_fliter_2 * fliter_num[0] + speed_fliter_1 * fliter_num[1] + (trigger_motor.shoot_motor_measure->speed_rpm * Motor_RMP_TO_SPEED) * fliter_num[2];
    trigger_motor.speed = speed_fliter_3;
    // 电机圈数重置， 因为输出轴旋转一圈， 电机轴旋转 36圈，将电机轴数据处理成输出轴数据，用于控制输出轴角度
    if (trigger_motor.shoot_motor_measure->ecd - trigger_motor.shoot_motor_measure->last_ecd > Half_ecd_range)
    {
        trigger_motor.ecd_count--;
    }
    else if (trigger_motor.shoot_motor_measure->ecd - trigger_motor.shoot_motor_measure->last_ecd < -Half_ecd_range)
    {
        trigger_motor.ecd_count++;
    }
    if (trigger_motor.ecd_count == FULL_COUNT)
    {
        trigger_motor.ecd_count = -(FULL_COUNT - 1);
    }
    else if (trigger_motor.ecd_count == -FULL_COUNT)
    {
        trigger_motor.ecd_count = FULL_COUNT - 1;
    }
    // 计算输出轴角度
    trigger_motor.angle = (trigger_motor.ecd_count * ecd_range + trigger_motor.shoot_motor_measure->ecd) * Motor_ECD_TO_ANGLE;
    // 摩擦轮——>速度处理
    for (i = 0; i < 2; i++)
    {
        fric_move.motor_fric[i].speed = 0.000415809748903494517209f * fric_move.motor_fric[i].fric_motor_measure->speed_rpm;
        fric_move.motor_fric[i].accel = fric_move.motor_speed_pid[i].Dbuf[0] * 500.0f;
    }
    // speed_t = -1 * fric_move.motor_fric[0].fric_motor_measure->speed_rpm;
    // speed_ = fric_move.motor_fric[1].fric_motor_measure->speed_rpm;
}

/**
 * @brief          射击模式设置
 * @param[in]      void
 * @retval         返回无
 */
static void Shoot_Set_Mode(void)
{
    
    // //判断哨兵模式
    // if(switch_is_up(fric_move.shoot_rc->rc.s[SHOOT_CONTROL_CHANNEL]))
    // {
    //     //此时哨兵为自动模式，射击自动
    //     shoot_mode = SHOOT_AUTO;
    // }
    // else
    // {
        //此时哨兵为遥控器控制模式，射击手动
        if (switch_is_up(fric_move.shoot_rc->rc.s[SHOOT_MODE_CHANNEL]))
        {
            shoot_mode = SHOOT_READY;
        }
        else
        {
            shoot_mode = SHOOT_STOP;
            last_fric_mode = SHOOT_STOP;
        }
    // }

}
/**
 * @brief          拨弹轮循环
 * @param[in]      void
 * @retval         返回无
 */
void shoot_control_loop(void)
{
    if (shoot_mode == SHOOT_BULLET)
    {
        trigger_motor_pid.max_out = TRIGGER_BULLET_PID_MAX_OUT;
        trigger_motor_pid.max_iout = TRIGGER_BULLET_PID_MAX_IOUT;
        if (trigger_flag1 == 0)
        {
            shoot_bullet_control();
        }
        fric_control_loop(&fric_move);
        if (trigger_motor.speed > 2)
        {
            trigger_flag = 1;
            jam_flag = 0;
        }
        if (trigger_motor.speed < 0.1f && trigger_flag == 1)
        {
            jam_flag++;
            if (jam_flag > 20)
            {
                trigger_flag1 = 1;
                trigger_flag = 0;
                jam_flag = 0;
            }
        }
    }
    else if (shoot_mode == SHOOT_READY)
    {
        trigger_motor.speed_set = 0;
        fric_control_loop(&fric_move);
        trigger_motor.move_flag = 1;
        trigger_motor.move_flag_ONE = 1;
    }
    else if (shoot_mode == SHOOT_STOP)
    {
        shoot_fric_off(&fric_move);
        trigger_motor.speed_set = 0;
        fric = 0.0f;
        fric_control_loop(&fric_move);
    }

    if (trigger_flag1 == 0)
    {
        PID_Calc(&trigger_motor_pid, trigger_motor.speed, trigger_motor.speed_set);
    }
    else if (trigger_flag1 == 1)
    {
        PID_Calc(&trigger_motor_pid, trigger_motor.speed, -4.0);
        add_t++;
        if (add_t > 190)
        {
            add_t = 0;
            trigger_flag1 = 0;
        }
        trigger_motor.set_angle = trigger_motor.angle;
    }
    trigger_motor.given_current = (int16_t)(trigger_motor_pid.out);
}
/**
 * @brief   射击控制，控制拨弹电机角度，完成一次发射
 */
static void shoot_bullet_control(void)
{
    if (shoot_mode == SHOOT_BULLET && trigger_motor.move_flag == 1 && flag == 0)
    {
        trigger_motor.set_angle = rad_format(trigger_motor.set_angle + PI_Four);
        trigger_motor.cmd_time = xTaskGetTickCount();
        trigger_motor.move_flag = 0;
    }
    if (rad_format(trigger_motor.set_angle - trigger_motor.angle) > 0.05f)
    {
    }
    else
    {
        if (shoot_mode == SHOOT_BULLET)
        {
            trigger_motor.move_flag = 1;
        }
    }
}
/**
 * @brief  摩擦轮关闭
 */
static void shoot_fric_off(fric_move_t *fric_off)
{
    fric_off->speed_set[0] = 0.0f;
    fric_off->speed_set[1] = 0.0f;
}
/**
 * @brief  摩擦轮控制循环
 */
static void fric_control_loop(fric_move_t *fric_move_control_loop)
{
    uint8_t i = 0;
    // PID输出限幅
    for (i = 0; i < 2; i++)
    {
        fric_move_control_loop->motor_speed_pid[i].max_out = TRIGGER_BULLET_PID_MAX_OUT;
        fric_move_control_loop->motor_speed_pid[i].max_iout = TRIGGER_BULLET_PID_MAX_IOUT;
    }
    // 速度设置
    fric_move_control_loop->speed_set[0] = fric;
    fric_move_control_loop->speed_set[1] = -fric;
    for (i = 0; i < 2; i++)
    {
        fric_move_control_loop->motor_fric[i].speed_set = fric_move.speed_set[i];
        PID_Calc(&fric_move_control_loop->motor_speed_pid[i], fric_move_control_loop->motor_fric[i].speed, fric_move_control_loop->motor_fric[i].speed_set);
    }
    stm32_step_shoot_0(fric_move_control_loop->speed_set[0], fric_move_control_loop->motor_fric[0].speed);
    stm32_step_shoot_1(fric_move_control_loop->speed_set[1], fric_move_control_loop->motor_fric[1].speed);
    fric_move.fric_CAN_Set_Current[0] = stm32_Y.out_shoot;
    fric_move.fric_CAN_Set_Current[1] = stm32_Y.out_shoot1;
}
