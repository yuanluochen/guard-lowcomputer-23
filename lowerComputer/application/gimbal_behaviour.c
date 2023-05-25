/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       gimbal_task.c/h
  * @brief          
  *             完成云台控制任务，由于云台使用陀螺仪解算出的角度，其范围在（-pi,pi）
  *             故而设置目标角度均为范围，存在许多对角度计算的函数。云台主要分为2种
  *             状态，陀螺仪控制状态是利用板载陀螺仪解算的姿态角进行控制，编码器控制
  *             状态是通过电机反馈的编码值控制的校准，此外还有校准状态，停止状态等。
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *
  @verbatim
  ==============================================================================
        
    如果要添加一个新的行为模式
    1.首先，在gimbal_behaviour.h文件中， 添加一个新行为名字在 gimbal_behaviour_e
    erum
    {  
        ...
        ...
        GIMBAL_XXX_XXX, // 新添加的
    }gimbal_behaviour_e,

    2. 实现一个新的函数 gimbal_xxx_xxx_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);
        "yaw, pitch" 参数是云台运动控制输入量
        第一个参数: 'yaw' 通常控制yaw轴移动,通常是角度增量,正值是逆时针运动,负值是顺时针
        第二个参数: 'pitch' 通常控制pitch轴移动,通常是角度增量,正值是逆时针运动,负值是顺时针
        在这个新的函数, 你能给 "yaw"和"pitch"赋值想要的参数
    3.  在"gimbal_behavour_set"这个函数中，添加新的逻辑判断，给gimbal_behaviour赋值成GIMBAL_XXX_XXX
        在gimbal_behaviour_mode_set函数最后，添加"else if(gimbal_behaviour == GIMBAL_XXX_XXX)" ,然后选择一种云台控制模式
        3种:
        GIMBAL_MOTOR_RAW : 使用'yaw' and 'pitch' 作为电机电流设定值,直接发送到CAN总线上.
        GIMBAL_MOTOR_ENCONDE : 'yaw' and 'pitch' 是角度增量,  控制编码相对角度.
        GIMBAL_MOTOR_GYRO : 'yaw' and 'pitch' 是角度增量,  控制陀螺仪绝对角度.
    4.  在"gimbal_behaviour_control_set" 函数的最后，添加
        else if(gimbal_behaviour == GIMBAL_XXX_XXX)
        {
            gimbal_xxx_xxx_control(&rc_add_yaw, &rc_add_pit, gimbal_control_set);
        }
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "gimbal_behaviour.h"
#include "arm_math.h"
#include "bsp_buzzer.h"
#include "detect_task.h"
#include "chassis_behaviour.h"
#include "user_lib.h"

#define int_abs(x) ((x) > 0 ? (x) : (-x))

/**
  * @brief          遥控器的死区判断，因为遥控器的拨杆在中位的时候，不一定为0，
  */
#define rc_deadband_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }



/**
  * @brief          通过判断角速度来判断云台是否到达极限位置
  */
#define gimbal_cali_gyro_judge(gyro, cmd_time, angle_set, angle, ecd_set, ecd, step) \
    {                                                                                \
        if ((gyro) < GIMBAL_CALI_GYRO_LIMIT)                                         \
        {                                                                            \
            (cmd_time)++;                                                            \
            if ((cmd_time) > GIMBAL_CALI_STEP_TIME)                                  \
            {                                                                        \
                (cmd_time) = 0;                                                      \
                (angle_set) = (angle);                                               \
                (ecd_set) = (ecd);                                                   \
                (step)++;                                                            \
            }                                                                        \
        }                                                                            \
    }




/**
  * @brief          云台行为状态机设置.
  * @param[in]      gimbal_mode_set: 云台数据指针
  * @retval         none
  */
static void gimbal_behavour_set(gimbal_control_t *gimbal_mode_set);
/**
  * @brief          当云台行为模式是GIMBAL_ZERO_FORCE, 这个函数会被调用,云台控制模式是raw模式.原始模式意味着
  *                 设定值会直接发送到CAN总线上,这个函数将会设置所有为0.
  */
static void gimbal_zero_force_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);

/**
  * @brief          云台初始化控制，电机是陀螺仪角度控制，云台先抬起pitch轴，后旋转yaw轴
  */
static void gimbal_init_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);

/**
  * @brief          云台控制，电机是角度控制，
  * @param[out]     yaw: yaw轴角度控制，为角度的增量 单位 rad
           * @param[out]     pitch:pitch轴角度控制，为角度的增量 单位 rad
  * @param[in]      gimbal_control_set:云台数据指针
  */
static void gimbal_RC_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);
/**
  * @brief          云台进入遥控器无输入控制，电机是相对角度控制，
  * @author         RM
  */
static void gimbal_motionless_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);

/**
 * @brief                     云台进入自动模式，云台姿态受视觉上位机控制，电机是绝对角度控制
 * 
 * @param yaw                 yaw轴角度增量
 * @param pitch               pitch轴角度增量
 * @param gimbal_control_set  云台数据指针
 * @author                    yuanluochen
 */
static void gimbal_auto_control(fp32* yaw, fp32* pitch, gimbal_control_t* gimbal_control_set);

/*----------------------------------结构体---------------------------*/
//云台行为状态机
static gimbal_behaviour_e gimbal_behaviour = GIMBAL_ZERO_FORCE;
/*----------------------------------内部变量---------------------------*/
int yaw_flag=0;  
fp32 Pitch_Set[8]={0};
/*----------------------------------外部变量---------------------------*/
extern chassis_behaviour_e chassis_behaviour_mode;
//云台初始化完毕标志位
bool_t gimbal_init_finish_flag = 0;
/**
 * @brief 标志变量，云台从其他模式切入自瞄模式标志位
 * 
 *        该位为1， 则发生了从其他模式切入自瞄模式
 * 
 */
bool_t other_mode_transform_auto_mode_flag = 0;

/**
 * @brief 保存云台自动扫描中值 
 * 
 */
bool_t save_auto_scan_center_value_flag = 0;



/**
  * @brief          gimbal_set_mode函数调用在gimbal_task.c,云台行为状态机以及电机状态机设置
  * @param[out]     gimbal_mode_set: 云台数据指针
  * @retval         none
  */

void gimbal_behaviour_mode_set(gimbal_control_t *gimbal_mode_set)
{
    if (gimbal_mode_set == NULL)
    {
        return;
    }
    //set gimbal_behaviour variable
    //云台行为状态机设置
    gimbal_behavour_set(gimbal_mode_set);
    //accoring to gimbal_behaviour, set motor control mode
    //根据云台行为状态机设置电机状态机
    switch (gimbal_behaviour)
    {
        // 无力模式下,设置为电机原始值控制，方便让电机处于无力态
    case GIMBAL_ZERO_FORCE:
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_RAW;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_RAW;
        break;

    // 相对角度控制和遥控器和初始化模式以及自动模式都采用一种控制模式
    case GIMBAL_RC:
    case GIMBAL_INIT:
    case GIMBAL_AUTO:
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_GYRO;   // yaw轴通过陀螺仪的绝对角控制
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_GYRO; // pitch轴通过陀螺仪的绝对角控制
        break;

    case GIMBAL_MOTIONLESS:
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
        break;
    }
}

/**
 * @brief                          (修改版)云台行为模式控制，云台pitch轴采用相对角度控制，云台yaw轴采用绝对角度控制,原因为pitch绝对角难以限制
 * 
 * @param add_yaw                  yaw 轴的角度增量 为指针
 * @param add_pitch                pitch 轴的角度增量 为指针 
 * @param gimbal_control_set       云台结构体指针 
 */
void gimbal_behaviour_control_set(fp32 *add_yaw, fp32 *add_pitch, gimbal_control_t *gimbal_control_set)
{
    if (add_yaw == NULL || add_pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }
    // 判断底盘模式，根据底盘模式选择底盘控制方式
    switch (gimbal_behaviour)
    {
    case GIMBAL_ZERO_FORCE: // 无力模式云台无力
        gimbal_zero_force_control(add_yaw, add_pitch, gimbal_control_set);
        break;

    case GIMBAL_INIT: // 初始化模式，云台初始化
        gimbal_init_control(add_yaw, add_pitch, gimbal_control_set);
        break;

    case GIMBAL_RC: // 遥控器控制模式,绝对角控制
        gimbal_RC_control(add_yaw, add_pitch, gimbal_control_set);
        break;

    case GIMBAL_MOTIONLESS: // 无信号下的控制,即无力
        gimbal_motionless_control(add_yaw, add_pitch, gimbal_control_set);
        break;

    case GIMBAL_AUTO: // 自动模式，上位机控制
        gimbal_auto_control(add_yaw, add_pitch, gimbal_control_set);
        break;
    }
}

/**
 * @brief          云台在某些行为下，需要底盘不动
 * @param[in]      none
 * @retval         1: no move 0:normal
 */

bool_t gimbal_cmd_to_chassis_stop(void)
{
    if (gimbal_behaviour == GIMBAL_INIT || gimbal_behaviour == GIMBAL_MOTIONLESS || gimbal_behaviour == GIMBAL_ZERO_FORCE)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

/**
 * @brief          云台在某些行为下，需要射击停止
 * @param[in]      none
 * @retval         1: no move 0:normal
 */

bool_t gimbal_cmd_to_shoot_stop(void)
{
    if (gimbal_behaviour == GIMBAL_INIT || gimbal_behaviour == GIMBAL_ZERO_FORCE)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}


/**
 * @brief          云台行为状态机设置.
 * @param[in]      gimbal_mode_set: 云台数据指针
 * @retval         none
 */
static void gimbal_behavour_set(gimbal_control_t *gimbal_mode_set)
{
    if (gimbal_mode_set == NULL)
    {
        return;
    }
    //判断imu是否校准完毕
    if (judge_imu_offset_calc_finish())
    {
        //校准完毕云台进入其他模式
        // 判断是否为初始化模式
        if (gimbal_behaviour == GIMBAL_INIT)
        {
            // 初始化时间
            static int init_time = 0;
            if (switch_is_down(gimbal_mode_set->gimbal_rc_ctrl->rc.s[GIMBAL_MODE_CHANNEL]))
            {
                //停止初始化
                init_time = 0;
            }
            else
            {
                init_time++; // 初始化时间增加
                // 是否初始化完成
                if ((fabs(gimbal_mode_set->gimbal_pitch_motor.absolute_angle - INIT_PITCH_SET) > GIMBAL_INIT_ANGLE_ERROR) ||
                    (fabs(gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_measure->ecd - gimbal_mode_set->gimbal_yaw_motor.frist_ecd) * MOTOR_ECD_TO_RAD > GIMBAL_INIT_ANGLE_ERROR))
                {
                    // 初始化未完成，判断初始化时间
                    if (init_time >= GIMBAL_INIT_TIME)
                    {
                        // 不进行任何行为，直接判断进入其他模式,计时归零
                        init_time = 0;
                    }
                    else
                    {
                        // 退出模式选择，依旧为初始化模式
                        return;
                    }
                }
                else
                {
                    // 初始化完成,计时归零,重新计时
                    init_time = 0;
                    // 标志初始化完成
                    gimbal_init_finish_flag = 1;
                }
                // 数值仅保存一次
                if (save_auto_scan_center_value_flag == 0)
                {
                    save_auto_scan_center_value_flag = 1;
                    // 初始化完成保存扫描中心点
                    gimbal_mode_set->gimbal_auto_scan.yaw_center_value = gimbal_mode_set->gimbal_yaw_motor.absolute_angle_set;
                    gimbal_mode_set->gimbal_auto_scan.pitch_center_value = gimbal_mode_set->gimbal_pitch_motor.absolute_angle_set;
                }
            }
       }

        if (switch_is_down(gimbal_mode_set->gimbal_rc_ctrl->rc.s[GIMBAL_MODE_CHANNEL]))
        {
            // 遥控器控制模式
            gimbal_behaviour = GIMBAL_ZERO_FORCE;
        }
        if (switch_is_mid(gimbal_mode_set->gimbal_rc_ctrl->rc.s[GIMBAL_MODE_CHANNEL]))
        {
            // 切换到遥控器控制模式
            gimbal_behaviour = GIMBAL_RC;
        }
        else if (switch_is_up(gimbal_mode_set->gimbal_rc_ctrl->rc.s[GIMBAL_MODE_CHANNEL]))
        {
            // 切换到云台自动模式
            gimbal_behaviour = GIMBAL_AUTO;
        }
        // 遥控器报错处理
        if (toe_is_error(DBUS_TOE))
        {
            gimbal_behaviour = GIMBAL_ZERO_FORCE;
        }

        // 判断进入初始化模式
        static gimbal_behaviour_e last_gimbal_behaviour = GIMBAL_ZERO_FORCE;
        // 判断云台从无力模式转为其他模式,进入初始化状态
        if (last_gimbal_behaviour == GIMBAL_ZERO_FORCE && gimbal_behaviour != GIMBAL_ZERO_FORCE)
        {
            gimbal_behaviour = GIMBAL_INIT;
            // 标志初始化未完成
            gimbal_init_finish_flag = 0;
        }

        // 判断是否发生云台从其他模式切入自瞄模式
        if (last_gimbal_behaviour != GIMBAL_AUTO && gimbal_behaviour == GIMBAL_AUTO)
        {
            other_mode_transform_auto_mode_flag = 1;
            // 保存当前云台位置
            gimbal_mode_set->gimbal_auto_scan.yaw_center_value = gimbal_mode_set->gimbal_yaw_motor.absolute_angle;
            gimbal_mode_set->gimbal_auto_scan.pitch_center_value = gimbal_mode_set->gimbal_pitch_motor.absolute_angle;
        }
        // 保存历史数据
        last_gimbal_behaviour = gimbal_behaviour;
    }
    else
    {
        //imu校准未完毕，云台无力
        gimbal_behaviour = GIMBAL_ZERO_FORCE;
    }
    
}
/**
 * @brief          当云台行为模式是GIMBAL_ZERO_FORCE, 这个函数会被调用,云台控制模式是raw模式.原始模式意味着
 *                 设定值会直接发送到CAN总线上,这个函数将会设置所有为0.
 */
static void gimbal_zero_force_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }

    *yaw = 0.0f;
    *pitch = 0.0f;
}
/**
 * @brief          云台控制，电机是角度控制，遥控器控制数据
 */

static void gimbal_RC_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
{
    static int16_t yaw_channel_RC;
    static int16_t pitch_channel_RC;
    
    // 遥控器控制
    rc_deadband_limit(gimbal_control_set->gimbal_rc_ctrl->rc.ch[YAW_CHANNEL], yaw_channel_RC, RC_DEADBAND);
    rc_deadband_limit(gimbal_control_set->gimbal_rc_ctrl->rc.ch[PITCH_CHANNEL], pitch_channel_RC, RC_DEADBAND);

    *yaw = yaw_channel_RC * YAW_RC_SEN;
    *pitch = -pitch_channel_RC * PITCH_RC_SEN;
}

/**
 * @brief          云台进入遥控器无输入控制，电机是相对角度控制，
 */
static void gimbal_motionless_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }
    *yaw = 0.0f;
    *pitch = 0.0f;
}

/**
 * @brief                     云台进入自动模式，云台姿态受视觉上位机控制，电机是绝对角度控制
 *
 * @param yaw                 yaw 轴角度增量
 * @param pitch               pitch 轴角度增量
 * @param gimbal_control_set  云台指针
 */
static void gimbal_auto_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
{
    // yaw pitch 轴设定值与当前值的差值
    fp32 pitch_error = 0; 
    fp32 yaw_error = 0;

    // pitch轴yaw轴设定角度
    fp32 pitch_set_angle = 0;
    fp32 yaw_set_angle = 0;

    // 计算过去设定角度与当前角度之间的差值
    yaw_error = gimbal_control_set->gimbal_yaw_motor.absolute_angle_set - gimbal_control_set->gimbal_yaw_motor.absolute_angle;
    pitch_error = gimbal_control_set->gimbal_pitch_motor.absolute_angle_set - gimbal_control_set->gimbal_pitch_motor.absolute_angle;

    // 判断数据是否长久未更新
    if (judge_not_rx_vision_data())
    {
        // 长久未更新

        // 自动扫描设置浮动值
        static fp32 auto_scan_AC_set_yaw = 0;
        static fp32 auto_scan_AC_set_pitch = 0;
        // 计算运行时间
        gimbal_control_set->gimbal_auto_scan.scan_run_time = TIME_MS_TO_S(HAL_GetTick()) - gimbal_control_set->gimbal_auto_scan.scan_begin_time;
        //云台自动扫描,设置浮动值
        scan_control_set(&auto_scan_AC_set_yaw, gimbal_control_set->gimbal_auto_scan.yaw_range, gimbal_control_set->gimbal_auto_scan.scan_yaw_period, gimbal_control_set->gimbal_auto_scan.scan_run_time);
        scan_control_set(&auto_scan_AC_set_pitch, gimbal_control_set->gimbal_auto_scan.pitch_range, gimbal_control_set->gimbal_auto_scan.scan_pitch_period, gimbal_control_set->gimbal_auto_scan.scan_run_time);
        // 赋值控制值  = 中心值 + 加上浮动函数
        pitch_set_angle = auto_scan_AC_set_pitch;
        yaw_set_angle = auto_scan_AC_set_yaw + gimbal_control_set->gimbal_auto_scan.yaw_center_value;
        // yaw_set_angle = gimbal_control_set->gimbal_auto_scan.yaw_center_value;
        // pitch_set_angle = gimbal_control_set->gimbal_auto_scan.pitch_center_value;

    }
    else
    {
        //  获取上位机视觉数据
        pitch_set_angle = gimbal_control_set->gimbal_vision_point->gimbal_pitch;
        yaw_set_angle = gimbal_control_set->gimbal_vision_point->gimbal_yaw;
        // 云台自动扫描更新初始时间
        gimbal_control_set->gimbal_auto_scan.scan_begin_time = TIME_MS_TO_S(HAL_GetTick());
   }
    // 赋值增量
    *yaw = yaw_set_angle - gimbal_control_set->gimbal_yaw_motor.absolute_angle - yaw_error;
    *pitch = pitch_set_angle - gimbal_control_set->gimbal_pitch_motor.absolute_angle - pitch_error;
}
/**
 * @brief          云台初始化控制，电机是陀螺仪角度控制，云台先抬起pitch轴，后旋转yaw轴
 */
static void gimbal_init_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }
    // 初始化状态控制量计算
    if (fabs(INIT_PITCH_SET - gimbal_control_set->gimbal_pitch_motor.absolute_angle) > GIMBAL_INIT_ANGLE_ERROR) // pitch轴回正
    {
        *pitch = (INIT_PITCH_SET - gimbal_control_set->gimbal_pitch_motor.absolute_angle) * GIMBAL_INIT_PITCH_SPEED;
        *yaw = 0.0f;
    }
    else // yaw轴回归初始值
    {
        static fp32 yaw_error = 0;
        yaw_error = gimbal_control_set->gimbal_yaw_motor.absolute_angle_set - gimbal_control_set->gimbal_yaw_motor.absolute_angle;
        // pitch轴保持不变yaw轴回归中值
        *pitch = (INIT_PITCH_SET - gimbal_control_set->gimbal_pitch_motor.absolute_angle) * GIMBAL_INIT_PITCH_SPEED;
        // yaw轴绝对角计算控制yaw轴正方向
        *yaw = (gimbal_control_set->gimbal_yaw_motor.frist_ecd - gimbal_control_set->gimbal_yaw_motor.gimbal_motor_measure->ecd) * MOTOR_ECD_TO_RAD - yaw_error;
    }
}



void scan_control_set(fp32* gimbal_set, fp32 range, fp32 period, fp32 run_time)
{
    // 计算单次运行的步长
    fp32 step = 4.0f * range / period;

    // 判断云台设置浮动角度是否超过最大值,限制最大值
    if (*gimbal_set >= range)
    {
        *gimbal_set = range;
    }
    else if (*gimbal_set <= -range)
    {
        *gimbal_set = -range;
    }

    // 处理运行时间，将运行时间处理到一个周期内
    fp32 calc_time = run_time - period * ((int16_t)(run_time / period));
    // 判断当前时间所处的位置，根据当前位置，判断数值计算方向
    if (calc_time < period / 2.0f)
    {
        // 上半周期,数值为向上递增函数,step为正值
        *gimbal_set = step * calc_time - range;
    }
    else if (calc_time >= period / 2.0f)
    {
        // 下半周期，数据为向下递减函数，step为赋值
        *gimbal_set = -(step * calc_time) + 3 * range;
    }
}

bool_t judge_gimbal_mode_is_auto_mode(void)
{
    // 判断当前云台模式是否为自动模式
    return (gimbal_behaviour == GIMBAL_AUTO) ? 1 : 0;
}

bool_t judge_other_mode_transform_auto_mode(void)
{
    // 读取数值，判断事件是否发生
    bool_t temp = other_mode_transform_auto_mode_flag;
    // 数值归零
    other_mode_transform_auto_mode_flag = 0;
    return temp;
}

bool_t gimbal_control_vision_task(void)
{
    return gimbal_init_finish_flag;
}
