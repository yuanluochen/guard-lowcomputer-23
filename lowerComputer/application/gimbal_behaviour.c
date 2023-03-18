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


/*----------------------------------内部函数---------------------------*/
/**
  * @brief          pitch轴滤波.
  */
void Fiter(fp32 pitch);   
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
  * @brief          云台校准控制，电机是raw控制，云台先抬起pitch，放下pitch，在正转yaw，最后反转yaw，记录当时的角度和编码值
  */
static void gimbal_cali_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);

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
    //无力模式下,设置为电机原始值控制，方便让电机处于无力态
		case GIMBAL_ZERO_FORCE:
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_RAW;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_RAW;
        break;
        
    //相对角度控制和遥控器和初始化模式以及自动模式都采用一种控制模式
    case GIMBAL_RC:
    case GIMBAL_INIT:
    case GIMBAL_RELATIVE_ANGLE: 
    case GIMBAL_AUTO:
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_GYRO;      // yaw轴通过陀螺仪的绝对角控制
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_GYRO;    // pitch轴通过陀螺仪的绝对角控制
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

    // 修改版的代码,pitch相对角度控制， yaw绝对角度控制
    if (add_yaw == NULL || add_pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }
    //判断底盘模式，根据底盘模式选择底盘控制方式
    switch(gimbal_behaviour)
    {
    case GIMBAL_ZERO_FORCE://无力模式云台无力
        gimbal_zero_force_control(add_yaw, add_pitch, gimbal_control_set);
        break;
    case GIMBAL_INIT:      //初始化模式，云台初始化
        gimbal_init_control(add_yaw, add_pitch, gimbal_control_set);
        break;
    case GIMBAL_CALI:      //校准模式，云台校准
        gimbal_cali_control(add_yaw, add_pitch, gimbal_control_set);
        break;
    case GIMBAL_RC: //相对角度控制和绝对角度控制都采用统一的控制方式，即pitch轴相对角度控制，yaw轴绝对角度控制
    case GIMBAL_RELATIVE_ANGLE:
        gimbal_RC_control(add_yaw, add_pitch, gimbal_control_set);
        break;

    case GIMBAL_MOTIONLESS: //无信号下的控制,即无力
        gimbal_motionless_control(add_yaw, add_pitch, gimbal_control_set);
        break;

    case GIMBAL_AUTO:       //自动模式，上位机控制
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
    if (gimbal_behaviour == GIMBAL_INIT || gimbal_behaviour == GIMBAL_CALI || gimbal_behaviour == GIMBAL_MOTIONLESS || gimbal_behaviour == GIMBAL_ZERO_FORCE)
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
    if (gimbal_behaviour == GIMBAL_INIT || gimbal_behaviour == GIMBAL_CALI || gimbal_behaviour == GIMBAL_ZERO_FORCE)
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
    //判断初始化是否完成
    if(gimbal_behaviour == GIMBAL_INIT)
    {
        //初始化时间
        static int init_time = 0;
        init_time++;//初始化时间增加
        //是否初始化完成
        if ((fabs(gimbal_mode_set->gimbal_pitch_motor.absolute_angle - INIT_PITCH_SET) > GIMBAL_INIT_ANGLE_ERROR) &&
            (fabs(gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_measure->ecd - gimbal_mode_set->gimbal_pitch_motor.frist_ecd) > GIMBAL_INIT_ANGLE_ERROR)) 
        {
            //初始化未完成，判断初始化时间
            if(init_time >= GIMBAL_INIT_TIME)
            {
                //不进行任何行为，直接判断进入其他模式,计时归零
                init_time = 0;

            }
            else
            {
                //退出模式选择，依旧为初始化模式
                return;
            }
        }
        else
        {
            
            //初始化完成,计时归零,重新计时
            init_time = 0;
            //初始化完成，保存初始化后的绝对角,该绝对角用于哨兵云台yaw轴摇摆
            gimbal_mode_set->gimbal_yaw_absolute_offset_angle = gimbal_mode_set->gimbal_yaw_motor.absolute_angle;
        }

        //判断yaw轴pitch轴角度是否符合要求 
    }
    
    if (switch_is_down(gimbal_mode_set->gimbal_rc_ctrl->rc.s[GIMBAL_MODE_CHANNEL]))
    {
        //遥控器控制模式
        gimbal_behaviour = GIMBAL_ZERO_FORCE;
    }
    if (switch_is_mid(gimbal_mode_set->gimbal_rc_ctrl->rc.s[GIMBAL_MODE_CHANNEL]))
    {
        //切换到遥控器控制模式
        gimbal_behaviour = GIMBAL_RC;
    }
    else if (switch_is_up(gimbal_mode_set->gimbal_rc_ctrl->rc.s[GIMBAL_MODE_CHANNEL]))
    {
        //切换到云台自动模式
        gimbal_behaviour = GIMBAL_AUTO;
    }
    //遥控器报错处理
    if (toe_is_error(DBUS_TOE))
    {
        gimbal_behaviour = GIMBAL_ZERO_FORCE;
    }

    //判断进入初始化模式
    static gimbal_behaviour_e last_gimbal_behaviour = GIMBAL_ZERO_FORCE;
    //判断云台从无力模式转为其他模式,进入初始化状态
    if (last_gimbal_behaviour == GIMBAL_ZERO_FORCE && gimbal_behaviour != GIMBAL_ZERO_FORCE)
    {
        gimbal_behaviour = GIMBAL_INIT;
    }
    //保存历史数据
    last_gimbal_behaviour = gimbal_behaviour;
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
    static fp32 rc_add_yaw, rc_add_pit;
    static fp32 rc_add_yaw_RC, rc_add_pit_RC;
    volatile fp32 rc_add_z = 0;
    static int16_t yaw_channel = 0, pitch_channel = 0;
    static int16_t yaw_channel_RC;
    static int16_t pitch_channel_RC;
    fp32 yaw_turn = 0;
    fp32 yaw_turn1 = 0;
    fp32 pitch_turn = 0;
    if (yaw_flag == 0)
    {
        if (gimbal_control_set->gimbal_rc_ctrl->key.v & KEY_PRESSED_OFFSET_X)
        {
            yaw_turn = 3.1415926f;
            yaw_flag = 1;
        }
    }
    if ((gimbal_control_set->gimbal_rc_ctrl->key.v & KEY_PRESSED_OFFSET_X) == 0)
    {
        yaw_flag = 0;
    }
    if (gimbal_control_set->gimbal_rc_ctrl->key.v & KEY_PRESSED_OFFSET_Q)
    {
        yaw_turn1 = 0.0025f;
    }
    else if (gimbal_control_set->gimbal_rc_ctrl->key.v & KEY_PRESSED_OFFSET_E)
    {
        yaw_turn1 = -0.0025f;
    }
    // 遥控器控制
    rc_deadband_limit(gimbal_control_set->gimbal_rc_ctrl->rc.ch[YAW_CHANNEL], yaw_channel_RC, 15);
    rc_deadband_limit(gimbal_control_set->gimbal_rc_ctrl->rc.ch[PITCH_CHANNEL], pitch_channel_RC, 15);

    rc_add_yaw_RC = yaw_channel_RC * YAW_RC_SEN;
    rc_add_pit_RC = pitch_channel_RC * PITCH_RC_SEN;
    // 一阶低通滤波代替斜波作为输入
    first_order_filter_cali(&gimbal_control_set->gimbal_cmd_slow_set_vx_RC, rc_add_yaw_RC);
    first_order_filter_cali(&gimbal_control_set->gimbal_cmd_slow_set_vy_RC, rc_add_pit_RC);

    // 键盘控制
    rc_deadband_limit(gimbal_control_set->gimbal_rc_ctrl->mouse.x, yaw_channel, 5);
    rc_deadband_limit(gimbal_control_set->gimbal_rc_ctrl->mouse.y, pitch_channel, 5);

    rc_add_yaw = -yaw_channel * Yaw_Mouse_Sen;
    rc_add_pit = -pitch_channel * Pitch_Mouse_Sen;
    // 滑动滤波
    Fiter(rc_add_pit);
    rc_add_pit = (rc_add_pit * 8 + Pitch_Set[1] - Pitch_Set[7]) / 8;
    // 一阶低通滤波代替斜波作为输入
    first_order_filter_cali(&gimbal_control_set->gimbal_cmd_slow_set_vx, rc_add_yaw);
    first_order_filter_cali(&gimbal_control_set->gimbal_cmd_slow_set_vy, rc_add_pit);

    if (gimbal_control_set->gimbal_cmd_slow_set_vx.out > 2.f)
        gimbal_control_set->gimbal_cmd_slow_set_vx.out = 2.f;
    else if (gimbal_control_set->gimbal_cmd_slow_set_vx.out < -2.f)
        gimbal_control_set->gimbal_cmd_slow_set_vx.out = -2.f;

    *yaw = gimbal_control_set->gimbal_cmd_slow_set_vx.out + yaw_turn + yaw_turn1 + gimbal_control_set->gimbal_cmd_slow_set_vx_RC.out;
    *pitch = (gimbal_control_set->gimbal_cmd_slow_set_vy.out + pitch_turn + gimbal_control_set->gimbal_cmd_slow_set_vz.out + gimbal_control_set->gimbal_cmd_slow_set_vy_RC.out);
    
}



void Fiter(fp32 pitch)
{
	Pitch_Set[7]=Pitch_Set[6];
	Pitch_Set[6]=Pitch_Set[5];
	Pitch_Set[5]=Pitch_Set[4];
	Pitch_Set[4]=Pitch_Set[3];
	Pitch_Set[3]=Pitch_Set[2];
	Pitch_Set[2]=Pitch_Set[1];
	Pitch_Set[1]=Pitch_Set[0];
	Pitch_Set[0]=pitch;
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
static void gimbal_auto_control(fp32* yaw, fp32* pitch, gimbal_control_t* gimbal_control_set)
{
    

#if 0
    //yaw轴， pitch轴上位机原始数值
    fp32 yaw_value = 0;
    fp32 pitch_value = 0;


    //yaw轴数据保存标志位
    static int save_yaw_offset_flag = 0;
    if(save_yaw_offset_flag == 0)
    {
        //保存yaw轴数据
        gimbal_control_set->gimbal_yaw_absolute_offset_angle = gimbal_control_set->gimbal_yaw_motor.absolute_angle;
        save_yaw_offset_flag = 1;
    }

    //判断是否接收到上位机发送的数据
    if (gimbal_control_set->gimbal_vision_control->rx_flag)
    { 
        //接收到上位机数据

        // 数据接收位置零，等待下一次数据的接收
        gimbal_control_set->gimbal_vision_control->rx_flag = 0;

    }
    else
    {
#if 1
        //未接收到上位机数据,不移动
        *yaw = 0;
        *pitch = 0;
#else

        static int init_count = 0; // 云台初始化时间,初始化一次计数一次，为了防止初始化时间过长
        // 未接收到上位机数据，云台自主运动
        // 判断云台是否到达中值
        if (fabs(gimbal_control_set->gimbal_pitch_motor.absolute_angle - INIT_PITCH_SET) <  GIMBAL_INIT_ANGLE_ERROR
            && fabs(gimbal_control_set->gimbal_yaw_motor.absolute_angle - gimbal_control_set->gimbal_yaw_absolute_offset_angle) < GIMBAL_INIT_ANGLE_ERROR)
        {
            if ( init_count <= INIT_STOP_COUNT)
            {
                //计数小于100次继续初始化
                gimbal_init_control(yaw, pitch, gimbal_control_set);  //云台初始化
                //初始化计数
                init_count++;
            }
            else
            {
                //计数大于INIT_STOP_COUNT次,停止计数，停止初始化
                init_count = 0;
                return;

            }
            
        } 
        else
        {
            init_count = 0; // 计数归零

            //  云台pitch轴yaw轴在一定范围内摇摆
            // 同时设置yaw轴pitch轴转动角度,以中点为中心进行,yaw轴中心为初始角度，pitch轴中心为绝对角零度
            // yaw轴首先角度提升，提升到最大值后，角度减少，减少到最大值
            static fp32 yaw_set_angle = 0;                      // yaw设置角度，该角度为相对于yaw轴gimbal_yaw_absolute_offset_angle_angle 的角度
            static fp32 yaw_relative_to_first_yaw = 0;                     // 云台yaw轴相对于初始角度的相对角度
            yaw_relative_to_first_yaw = gimbal_control_set->gimbal_yaw_motor.absolute_angle - gimbal_control_set->gimbal_yaw_absolute_offset_angle;
            static gimbal_swing_direction_e yaw_swing_direction = POSITIVE;//摇摆yaw轴运动方向
            static int yaw_set_count = 0;                                      //yaw轴设定角度计数
            // 判断当前是否为正向最大角度
            if (yaw_swing_direction == POSITIVE && yaw_set_angle >= GIMBAL_YAW_SWING_RANGE && fabs(yaw_relative_to_first_yaw) >= GIMBAL_YAW_SWING_RANGE)
            {
                // 角度设置转化为反向角度
                yaw_swing_direction = NEGATIVE;//此时为减法
            }
            //判断当前是否为反向最大角度
            else if (yaw_swing_direction == NEGATIVE && yaw_set_angle <= -GIMBAL_YAW_SWING_RANGE && fabs(yaw_relative_to_first_yaw) >= GIMBAL_YAW_SWING_RANGE)
            {
                //角度设置转化为正向角度
                yaw_swing_direction = POSITIVE;//此时为加法
            }
            else
            {
                //判断当前是否到达设定角度
                if (fabs(yaw_relative_to_first_yaw - yaw_set_angle)  <= GIMBAL_INIT_ANGLE_ERROR)
                {
                    //到达，设置数值
                    //计数值置零
                    yaw_set_count = 0;
                    // 根据yaw_swing_direction 设置yaw轴运动的加法减法
                    switch (yaw_swing_direction)
                    {
                    case POSITIVE:
                        yaw_set_angle += GIMBAL_YAW_SWING_STEP;
                        break;
                    case NEGATIVE:
                        yaw_set_angle -= GIMBAL_YAW_SWING_STEP;
                        break;
                    }
                }
                else
                {
                    //未到达目标数值，等待
                    //判断计数时间是否过长
                    if (yaw_set_count >= GIMBAL_SWING_STOP_COUNT)
                    {
                        //等待时间过长返回
                        return;
                    }
                    yaw_set_count++;

                }
           }
           //赋值yaw轴pitch轴转动数值,数值为角度增量
           *yaw = (yaw_set_angle + gimbal_control_set->gimbal_yaw_absolute_offset_angle) - gimbal_control_set->gimbal_yaw_motor.absolute_angle;
        }
#endif
    }
#else

    *yaw = gimbal_control_set->gimbal_vision_point->gimbal_motor_command.gimbal_yaw_add * MOTOR_ECD_TO_RAD;
    *pitch = -gimbal_control_set->gimbal_vision_point->gimbal_motor_command.gimbal_pitch_add * MOTOR_ECD_TO_RAD;

#endif
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

    //初始化状态控制量计算
    if (fabs(INIT_PITCH_SET - gimbal_control_set->gimbal_pitch_motor.absolute_angle) > GIMBAL_INIT_ANGLE_ERROR)  //pitch轴回正
    {
        *pitch = (INIT_PITCH_SET - gimbal_control_set->gimbal_pitch_motor.absolute_angle) * GIMBAL_INIT_PITCH_SPEED;
        *yaw = 0.0f;
    }
    else //yaw轴回归初始值
    {
        //pitch轴保持不变yaw轴回归中值
        *pitch = (INIT_PITCH_SET - gimbal_control_set->gimbal_pitch_motor.absolute_angle);
        //yaw轴绝对角计算控制yaw轴正方向
        *yaw = (gimbal_control_set->gimbal_yaw_motor.frist_ecd - gimbal_control_set->gimbal_yaw_motor.gimbal_motor_measure->ecd) * MOTOR_ECD_TO_RAD;
    }
}

/**
  * @brief          云台校准控制，电机是raw控制，云台先抬起pitch，放下pitch，在正转yaw，最后反转yaw，记录当时的角度和编码值
  */
static void gimbal_cali_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }
    static uint16_t cali_time = 0;

    if (gimbal_control_set->gimbal_cali.step == GIMBAL_CALI_PITCH_MAX_STEP)
    {

        *pitch = GIMBAL_CALI_MOTOR_SET;
        *yaw = 0;

        //判断陀螺仪数据， 并记录最大最小角度数据
        gimbal_cali_gyro_judge(gimbal_control_set->gimbal_pitch_motor.motor_gyro, cali_time, gimbal_control_set->gimbal_cali.max_pitch,
                               gimbal_control_set->gimbal_pitch_motor.absolute_angle, gimbal_control_set->gimbal_cali.max_pitch_ecd,
                               gimbal_control_set->gimbal_pitch_motor.gimbal_motor_measure->ecd, gimbal_control_set->gimbal_cali.step);
    }
    else if (gimbal_control_set->gimbal_cali.step == GIMBAL_CALI_PITCH_MIN_STEP)
    {
        *pitch = -GIMBAL_CALI_MOTOR_SET;
        *yaw = 0;

        gimbal_cali_gyro_judge(gimbal_control_set->gimbal_pitch_motor.motor_gyro, cali_time, gimbal_control_set->gimbal_cali.min_pitch,
                               gimbal_control_set->gimbal_pitch_motor.absolute_angle, gimbal_control_set->gimbal_cali.min_pitch_ecd,
                               gimbal_control_set->gimbal_pitch_motor.gimbal_motor_measure->ecd, gimbal_control_set->gimbal_cali.step);
    }
    else if (gimbal_control_set->gimbal_cali.step == GIMBAL_CALI_YAW_MAX_STEP)
    {
        *pitch = 0;
        *yaw = GIMBAL_CALI_MOTOR_SET;

        gimbal_cali_gyro_judge(gimbal_control_set->gimbal_yaw_motor.motor_gyro, cali_time, gimbal_control_set->gimbal_cali.max_yaw,
                               gimbal_control_set->gimbal_yaw_motor.absolute_angle, gimbal_control_set->gimbal_cali.max_yaw_ecd,
                               gimbal_control_set->gimbal_yaw_motor.gimbal_motor_measure->ecd, gimbal_control_set->gimbal_cali.step);
    }

    else if (gimbal_control_set->gimbal_cali.step == GIMBAL_CALI_YAW_MIN_STEP)
    {
        *pitch = 0;
        *yaw = -GIMBAL_CALI_MOTOR_SET;

        gimbal_cali_gyro_judge(gimbal_control_set->gimbal_yaw_motor.motor_gyro, cali_time, gimbal_control_set->gimbal_cali.min_yaw,
                               gimbal_control_set->gimbal_yaw_motor.absolute_angle, gimbal_control_set->gimbal_cali.min_yaw_ecd,
                               gimbal_control_set->gimbal_yaw_motor.gimbal_motor_measure->ecd, gimbal_control_set->gimbal_cali.step);
    }
    else if (gimbal_control_set->gimbal_cali.step == GIMBAL_CALI_END_STEP)
    {
        cali_time = 0;
    }
}
