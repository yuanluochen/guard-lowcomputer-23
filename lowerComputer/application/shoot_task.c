/**
 * @file shoot_task.c
 * @author yuanluochen 
 * @brief 哨兵发弹任务，接收视觉任务指令发送弹丸
 * @version 0.1
 * @date 2023-05-11
 * 
 * @copyright Copyright (c) 2023
 * 
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
static void Shoot_Set_Mode(void);
/**
 * @brief   射击数据更新
 */
static void Shoot_Feedback_Update(void);
/** @brief   摩擦轮控制循环
 */
static void fric_control_loop(fric_move_t *fric_move_control_loop);
/**
 * @brief 拨弹盘控制循环
 * 
 */
static void trigger_control_loop(Shoot_Motor_t* trigger_move_control_loop);

/**
 * @brief 设置发射控制模式
 * 
 */
static void shoot_set_control_mode(fric_move_t* fric_set_control);

/**
 * @brief 射击模式切换数据过渡 
 * 
 */
static void shoot_mode_change_transit(void);

/**
 * @brief 判断是否卡弹
 * 
 * @param trigger_stuck 卡弹结构体
 * @return bool_t 返回1 卡弹 返回0 不卡弹
 */
static bool_t judge_bullet_is_stuck(Shoot_Motor_t* trigger_stuck);

/**
 * @brief  射击初始化
 */
void shoot_init(void);



/*----------------------------------内部变量---------------------------*/
fp32 fric;
/*----------------------------------结构体------------------------------*/
Shoot_Motor_t trigger_motor;                                  // 拨弹轮数据
fric_move_t fric_move;                                        // 发射控制
/*----------------------------------外部变量---------------------------*/
extern ExtY_stm32 stm32_Y_shoot;
extern ext_power_heat_data_t power_heat_data_t;//机器人当前的功率状态，主要判断枪口热量
/*---------------------------------------------------------------------*/
// 控制模式
shoot_mode_e shoot_mode = SHOOT_STOP;                             // 此次射击模式
shoot_mode_e last_shoot_mode = SHOOT_STOP;                        // 上次射击模式
shoot_control_mode_e shoot_control_mode = SHOOT_STOP_CONTROL;     // 射击控制模式
shoot_init_state_e shoot_init_state = SHOOT_INIT_UNFINISH;        // 射击初始化枚举体
shoot_motor_control_mode_e fric_motor_mode = SHOOT_MOTOR_STOP;    // 摩擦轮电机
shoot_motor_control_mode_e trigger_motor_mode = SHOOT_MOTOR_STOP; // 拨弹盘电机


/**
 * @brief          射击任务，间隔 GIMBAL_CONTROL_TIME 1ms
 * @param[in]      pvParameters: 空
 * @retval         none
 */
void shoot_task(void const *pvParameters)
{
    vTaskDelay(GIMBAL_TASK_INIT_TIME);
    //射击初始化
    shoot_init();
    while (1)
    {
        //设置发射模式
        Shoot_Set_Mode();
        // 模式切换数据过渡,主要PID清除，防止数据积累引发电机反转
        shoot_mode_change_transit();
        //发射数据更新
        Shoot_Feedback_Update();
        //射击控制循环
        shoot_control_loop();
        //发送控制电流
        if (!(toe_is_error(TRIGGER_MOTOR_TOE) && !toe_is_error(FRIC_LEFT_MOTOR_TOE) && !toe_is_error(FRIC_RIGHT_MOTOR_TOE)))
        {
            // 发送控制指令
            CAN_cmd_shoot(fric_move.fric_CAN_Set_Current[0], fric_move.fric_CAN_Set_Current[1], trigger_motor.given_current, 0);
        }
        vTaskDelay(SHOOT_TASK_DELAY_TIME);
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
    // 初始化PID
    stm32_shoot_pid_init();
    //摩擦轮电机PID清除
    stm32_step_shoot_pid_clear();
    //拨弹盘pid
    static const fp32 Trigger_speed_pid[3] = {900, 0, 100};
    PID_Init(&trigger_motor.motor_pid, PID_POSITION, Trigger_speed_pid, TRIGGER_READY_PID_MAX_OUT, TRIGGER_READY_PID_MAX_IOUT);
    // 数据指针获取
    fric_move.shoot_rc = get_remote_control_point();
    // 获取视觉控制指针
    fric_move.shoot_vision_control = get_vision_shoot_point();
    trigger_motor.shoot_motor_measure = get_trigger_motor_measure_point();
    trigger_motor.blocking_angle_set = 0;
    fric_move.motor_fric[0].fric_motor_measure = get_shoot_motor_measure_point(0); // 右摩擦轮
    fric_move.motor_fric[1].fric_motor_measure = get_shoot_motor_measure_point(1); // 左摩擦轮
    // 滤波初始化
    const static fp32 fric_1_order_filter[1] = {0.1666666667f};
    const static fp32 fric_2_order_filter[1] = {0.1666666667f};
    first_order_filter_init(&fric_move.fric1_cmd_slow_set_speed, SHOOT_CONTROL_TIME, fric_1_order_filter);
    first_order_filter_init(&fric_move.fric2_cmd_slow_set_speed, SHOOT_CONTROL_TIME, fric_2_order_filter);

    //初始化卡弹状态为不卡弹
    trigger_motor.stuck_state = UNSTUCK;
    // 初始化卡弹时间为0
    trigger_motor.stuck_time = 0;
    // 初始化卡弹回拨时间为0
    trigger_motor.reverse_time = 0;
    Shoot_Feedback_Update();
}

static void shoot_mode_change_transit(void)
{
    if (last_shoot_mode != shoot_mode)
    {
        //模式发生切换,pid清除
        stm32_step_shoot_pid_clear();
    }
}

static bool_t judge_bullet_is_stuck(Shoot_Motor_t* trigger_stuck)
{
    //判断播弹电机电流值是否过大
    if (abs(trigger_stuck->given_current) > STUCK_CURRENT)
    {
        //计数卡弹时间
        trigger_motor.stuck_time++;
        //判断是否卡弹时间过长
        if (trigger_motor.stuck_time > STUCK_TIME)
        {
            //标志卡弹
            trigger_stuck->stuck_state = STUCK;
            //卡弹时间归零
            trigger_motor.stuck_time = 0;
        }
        else
        {
            //卡弹时间归零
            trigger_motor.stuck_time = 0;
        }

        //判断是否卡弹
        if (trigger_motor.stuck_state == STUCK)
        {
            trigger_motor.reverse_time++;
            if (trigger_motor.reverse_time > REVERSE_TIME)
            {
                //标志不卡弹
                trigger_motor.stuck_state = UNSTUCK;
                //卡弹回拨时间归零
                trigger_motor.reverse_time = 0;
            }
        }
    }

    return trigger_motor.stuck_state == STUCK;
}

/**
 * @brief          射击数据更新
 * @param[in]      void
 * @retval         void
 */
static void Shoot_Feedback_Update(void)
{
    uint8_t i;
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
    }
}

static void shoot_set_control_mode(fric_move_t *fric_set_control)
{

    // 运行模式

    // 判断初始化是否完成
    if (shoot_control_mode == SHOOT_INIT_CONTROL)
    {
        static uint32_t init_time = 0;
        // 判断拨杆是否拨到下档
        if (switch_is_down(fric_set_control->shoot_rc->rc.s[SHOOT_CONTROL_CHANNEL]))
        {
            // 拨到下档停止初始化
            init_time = 0;
        }
        else
        {
            // 判断是否初始化完成
            if (shoot_init_state == SHOOT_INIT_UNFINISH)
            {
                // 初始化未完成

                // 判断初始化时间是否过长
                if (init_time >= SHOOT_TASK_S_TO_MS(SHOOT_TASK_MAX_INIT_TIME))
                {
                    // 初始化时间过长不进行初始化，进入其他模式
                    init_time = 0;
                }
                else
                {
                    // 判断微动开关是否打开
                    if (BUTTEN_TRIG_PIN == PRESS)
                    {
                        // 按下
                        // 设置初始化完成
                        shoot_init_state = SHOOT_INIT_FINISH;
                        init_time = 0;
                        // 进入其他模式
                    }
                    else
                    {
                        // 初始化模式保持原状，初始化时间增加
                        init_time++;
                        return;
                    }
                }
            }
            else
            {
                // 进入其他模式
                init_time = 0;
            }
        }
    }
    // 根据遥控器开关设置发射控制模式
    if (switch_is_up(fric_set_control->shoot_rc->rc.s[SHOOT_CONTROL_CHANNEL]))
    {
        // 自动控制模式
        shoot_control_mode = SHOOT_AUTO_CONTROL;
    }
    else if (switch_is_mid(fric_set_control->shoot_rc->rc.s[SHOOT_CONTROL_CHANNEL]))
    {
        // 遥控器控制模式
        shoot_control_mode = SHOOT_RC_CONTROL;
    }
    else if (switch_is_down(fric_set_control->shoot_rc->rc.s[SHOOT_CONTROL_CHANNEL]))
    {
        // 停止
        shoot_control_mode = SHOOT_STOP_CONTROL;
    }
    else if (toe_is_error(DBUS_TOE))
    {
        // 遥控器报错处理
        shoot_control_mode = SHOOT_STOP_CONTROL;
    }
    else
    {
        shoot_control_mode = SHOOT_STOP_CONTROL;
    }

    // 云台在某些模式下发射停止
    if (gimbal_cmd_to_shoot_stop())
    {
        shoot_control_mode = SHOOT_STOP_CONTROL;
    }

    // 判断进入初始化模式
    static shoot_control_mode_e last_shoot_control_mode = SHOOT_STOP_CONTROL;
    if (shoot_control_mode != SHOOT_STOP_CONTROL && last_shoot_control_mode == SHOOT_STOP_CONTROL)
    {
        // 进入初始化模式
        shoot_control_mode = SHOOT_INIT_CONTROL;
    }
    last_shoot_control_mode = shoot_control_mode;
}

/**
 * @brief          射击模式设置
 * @param[in]      void
 * @retval         返回无
 */
static void Shoot_Set_Mode(void)
{

    
    
    //设置发射控制模式
    shoot_set_control_mode(&fric_move);

    //保留上次射击模式
    last_shoot_mode = shoot_mode;

    //更新当前射击模式
    
    
    //判断当前枪口热量是否即将到达最大值
    if (GUARD_MAX_MUZZLE_HEAT - power_heat_data_t.shooter_id1_17mm_cooling_heat >= GUARD_MAX_ALLOW_MUZZLE_HEAT_ERR0R &&
        GUARD_MAX_MUZZLE_HEAT - power_heat_data_t.shooter_id2_17mm_cooling_heat >= GUARD_MAX_ALLOW_MUZZLE_HEAT_ERR0R)
    {
        // 未即将达到最大值，发射任务正常进行

        // 根据控制模式设置发射模式
        if (shoot_control_mode == SHOOT_AUTO_CONTROL)
        {
            if (fric_move.shoot_vision_control->shoot_command == SHOOT_ATTACK)
            {
                // 设置发射模式，开摩擦轮，拨弹盘
                shoot_mode = SHOOT_BULLET;
            }
            else
            {
                // 其他状态摩擦轮一直开启
                // 设置准备发射模式，开摩擦轮
                shoot_mode = SHOOT_READY;
            }
        }
        else if (shoot_control_mode == SHOOT_RC_CONTROL)
        {
            // 此时哨兵为遥控器控制模式，射击手动
            if (switch_is_up(fric_move.shoot_rc->rc.s[SHOOT_MODE_CHANNEL]))
            {
                shoot_mode = SHOOT_READY;
                // 控制发射
                if (abs(fric_move.shoot_rc->rc.ch[4]) >= 100)
                {
                    shoot_mode = SHOOT_BULLET;
                }
            }
            else
            {
                shoot_mode = SHOOT_STOP;
            }
        }
        else if (shoot_control_mode == SHOOT_INIT_CONTROL)
        {
            // 此时哨兵初始化控制模式
            shoot_mode = SHOOT_INIT;
        }
        else if (shoot_control_mode == SHOOT_STOP_CONTROL)
        {
            shoot_mode = SHOOT_STOP;
        }
        else
        {
            shoot_mode = SHOOT_STOP;
        }
    }
    else
    {
        //即将到达最大值，停止发射
        if (shoot_control_mode == SHOOT_AUTO_CONTROL)
        {
            //如果为自动控制模式则停止拨弹
            shoot_mode = SHOOT_READY;
        }
        else if (shoot_control_mode == SHOOT_RC_CONTROL)
        {
            //如果为遥控器控制模式，则停止一切
            shoot_mode = SHOOT_STOP;
        }
        else
        {
            shoot_mode = SHOOT_STOP;
        }
    }
    
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
        //配置摩擦轮电机和拨弹盘电机转动
        fric_motor_mode = SHOOT_MOTOR_RUN;
        trigger_motor_mode = SHOOT_MOTOR_RUN;
        
    }
    else if (shoot_mode == SHOOT_READY)
    {
        // 配置摩擦轮电机转动，拨弹盘电机不转
        fric_motor_mode = SHOOT_MOTOR_RUN;
        trigger_motor_mode = SHOOT_MOTOR_STOP;
    }
    else if (shoot_mode == SHOOT_STOP)
    {
        //配置摩擦轮电机和拨弹盘电机停转
        fric_motor_mode = SHOOT_MOTOR_STOP;
        trigger_motor_mode = SHOOT_MOTOR_STOP;
    }
    else if (shoot_mode == SHOOT_INIT)
    {
       
        //判断是否初始化完成
        if(shoot_init_state == SHOOT_INIT_UNFINISH)
        {
            //初始化未完成
            //微动开关是否命中
            if (BUTTEN_TRIG_PIN == RELEASE)
            {
                shoot_init_state = SHOOT_INIT_UNFINISH;
                //未按下,配置拨弹盘转动,摩擦轮停转
                fric_motor_mode = SHOOT_MOTOR_STOP;
                trigger_motor_mode = SHOOT_MOTOR_RUN;
            }
            else
            {
                //按下
                shoot_init_state = SHOOT_INIT_FINISH;
                //电机停转
                fric_motor_mode = SHOOT_MOTOR_STOP;
                trigger_motor_mode = SHOOT_MOTOR_STOP;
            }
        }
        else
        {
            shoot_init_state = SHOOT_INIT_FINISH;
            // 电机停转
            fric_motor_mode = SHOOT_MOTOR_STOP;
            trigger_motor_mode = SHOOT_MOTOR_STOP;
        }
    }


    //根据电机模式配置电机转动速度
    switch (fric_motor_mode)
    {
    case SHOOT_MOTOR_RUN:
        //配置电机转动
        fric = FRIC_MOTOR_RUN_SPEED;
        break;
    case SHOOT_MOTOR_STOP:
        //配置电机停转
        fric = FRIC_MOTOR_STOP_SPEED;
        break;
    }


    switch (trigger_motor_mode)
    {
    case SHOOT_MOTOR_RUN:
        trigger_motor.speed_set = TRIGGER_MOTOR_RUN_SPEED;
        break;
    case SHOOT_MOTOR_STOP:

        trigger_motor.speed_set = TRIGGER_MOTOR_STOP_SPEED;
        break;
    }
 
    // pid计算
    fric_control_loop(&fric_move);        // 摩擦轮控制
    trigger_control_loop(&trigger_motor); // 拨弹盘控制

    //发弹停止
    

    //判断是否卡弹
    if (judge_bullet_is_stuck(&trigger_motor))
    {
        //回拨
        trigger_motor.given_current = -5;
    }
}

/**
 * @brief  摩擦轮控制循环
 */
static void fric_control_loop(fric_move_t *fric_move_control_loop)
{
    // 速度设置
    fric_move_control_loop->speed_set[0] = fric;
    fric_move_control_loop->speed_set[1] = -fric;
    // pid 计算
    stm32_step_shoot_0(fric_move_control_loop->speed_set[0], fric_move_control_loop->motor_fric[0].speed);
    stm32_step_shoot_1(fric_move_control_loop->speed_set[1], fric_move_control_loop->motor_fric[1].speed);
    fric_move.fric_CAN_Set_Current[0] = stm32_Y_shoot.out_shoot_0;
    fric_move.fric_CAN_Set_Current[1] = stm32_Y_shoot.out_shoot_1;
}


static void trigger_control_loop(Shoot_Motor_t* trigger_move_control_loop)
{
    trigger_move_control_loop->motor_pid.max_out = TRIGGER_BULLET_PID_MAX_OUT;
    trigger_move_control_loop->motor_pid.max_iout = TRIGGER_BULLET_PID_MAX_IOUT;
    PID_Calc(&trigger_move_control_loop->motor_pid, trigger_move_control_loop->speed, trigger_move_control_loop->speed_set); // 拨弹盘
    trigger_move_control_loop->given_current = (int16_t)(trigger_move_control_loop->motor_pid.out);
}


//射击控制视觉
bool_t shoot_control_vision_task(void)
{
    if (shoot_mode == SHOOT_INIT || shoot_control_mode == SHOOT_INIT || toe_is_error(DBUS_TOE))
    {
        //射击初始化模式，遥控器无信号，停止运行
        return 0;
    }   
    else
    {
        return 1;
    }
}
