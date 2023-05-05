/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       gimbal_task.c/h
  * @brief      gimbal control task, because use the euler angle calculated by
  *             gyro sensor, range (-pi,pi), angle set-point must be in this
  *             range.gimbal has two control mode, gyro mode and enconde mode
  *             gyro mode: use euler angle to control, encond mode: use enconde
  *             angle to control. and has some special mode:cali mode, motionless
  *             mode.
  *             完成云台控制任务，由于云台使用陀螺仪解算出的角度，其范围在（-pi,pi）
  *             故而设置目标角度均为范围，存在许多对角度计算的函数。云台主要分为2种
  *             状态，陀螺仪控制状态是利用板载陀螺仪解算的姿态角进行控制，编码器控制
  *             状态是通过电机反馈的编码值控制的校准，此外还有校准状态，停止状态等。
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. add some annotation
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "gimbal_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "arm_math.h"
#include "CAN_receive.h"
#include "user_lib.h"
#include "detect_task.h"
#include "remote_control.h"
#include "gimbal_behaviour.h"
#include "INS_task.h"
#include "shoot_task.h"
#include "pid.h"
#include "stm32.h"
#include "stm32_private.h"
// motor enconde value format, range[0-8191]
// 电机编码值规整 0—8191
#define ecd_format(ecd)         \
    {                           \
        if ((ecd) > ECD_RANGE)  \
            (ecd) -= ECD_RANGE; \
        else if ((ecd) < 0)     \
            (ecd) += ECD_RANGE; \
    }

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t gimbal_high_water;
#endif

/**
 * @brief          初始化"gimbal_control"变量，包括pid初始化， 遥控器指针初始化，云台电机指针初始化，陀螺仪角度指针初始化
 * @param[out]     init:"gimbal_control"变量指针.
 * @retval         none
 */
static void gimbal_init(gimbal_control_t *init);

/**
 * @brief          设置云台控制模式，主要在'gimbal_behaviour_mode_set'函数中改变
 * @param[out]     gimbal_set_mode:"gimbal_control"变量指针.
 * @retval         none
 */
static void gimbal_set_mode(gimbal_control_t *set_mode);

/**
 * @brief          底盘测量数据更新，包括电机速度，欧拉角度，机器人速度
 * @param[out]     gimbal_feedback_update:"gimbal_control"变量指针.
 * @retval         none
 */
static void gimbal_feedback_update(gimbal_control_t *feedback_update);

/**
 * @brief          云台模式改变，有些参数需要改变，例如控制yaw角度设定值应该变成当前yaw角度
 * @param[out]     mode_change:"gimbal_control"变量指针.
 * @retval         none
 */
static void gimbal_mode_change_control_transit(gimbal_control_t *mode_change);

/**
 * @brief          计算ecd与offset_ecd之间的相对角度
 * @param[in]      ecd: 电机当前编码
 * @param[in]      offset_ecd: 电机中值编码
 * @retval         相对角度，单位rad
 */
static fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd);

/**
 * @brief          设置云台控制设定值，控制值是通过gimbal_behaviour_control_set函数设置的
 * @param[out]     gimbal_set_control:"gimbal_control"变量指针.
 * @retval         none
 */
static void gimbal_set_control(gimbal_control_t *set_control);

/**
 * @brief          控制循环，根据控制设定值，计算电机电流值，进行控制
 * @param[out]     gimbal_control_loop:"gimbal_control"变量指针.
 * @retval         none
 */
static void gimbal_control_loop(gimbal_control_t *control_loop);

/**
 * @brief          云台控制模式:GIMBAL_MOTOR_GYRO，使用陀螺仪计算的欧拉角进行控制
 * @param[out]     gimbal_motor:yaw电机或者pitch电机
 * @retval         none
 */
static void gimbal_motor_absolute_angle_control(gimbal_motor_t *gimbal_motor);

/**
 * @brief          云台控制模式:GIMBAL_MOTOR_ENCONDE，使用编码相对角进行控制
 * @param[out]     gimbal_motor:yaw电机或者pitch电机
 * @retval         none
 */
static void gimbal_motor_relative_angle_control(gimbal_motor_t *gimbal_motor);

/**
 * @brief          云台控制模式:GIMBAL_MOTOR_RAW，电流值直接发送到CAN总线.
 * @param[out]     gimbal_motor:yaw电机或者pitch电机
 * @retval         none
 */
static void gimbal_motor_raw_angle_control(gimbal_motor_t *gimbal_motor);

/**
 * @brief          在GIMBAL_MOTOR_GYRO模式，限制角度设定,防止超过最大
 * @param[out]     gimbal_motor:yaw电机或者pitch电机
 * @retval         none
 */
static void gimbal_absolute_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add);

/**
 * @brief          在GIMBAL_MOTOR_ENCONDE模式，限制角度设定,防止超过最大
 * @param[out]     gimbal_motor:yaw电机或者pitch电机
 * @retval         none
 */
static void gimbal_relative_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add);

/*----------------------------------内部变量---------------------------*/
fp32 fuzzy_pid_speed[3] = {50, 1, 5};
fp32 fuzzy_pid_angle[3] = {1200, 1, 10};
/*----------------------------------结构体------------------------------*/
gimbal_control_t gimbal_control;
/*----------------------------------外部变量---------------------------*/
extern ExtY_stm32 stm32_Y_yaw;
extern ExtY_stm32 stm32_Y_pitch;

/**
 * @brief          云台任务，间隔 GIMBAL_CONTROL_TIME 1ms
 * @param[in]      pvParameters: 空
 * @retval         none
 */

void gimbal_task(void const *pvParameters)
{
    // 等待陀螺仪任务更新陀螺仪数据
    // wait a time
    vTaskDelay(GIMBAL_TASK_INIT_TIME);
    // gimbal init
    // 云台初始化
    gimbal_init(&gimbal_control);
    // 判断电机是否都上线
    while (toe_is_error(YAW_GIMBAL_MOTOR_TOE) || toe_is_error(PITCH_GIMBAL_MOTOR_TOE))
    {
        vTaskDelay(GIMBAL_CONTROL_TIME);
        gimbal_feedback_update(&gimbal_control); // 云台数据反馈
    }
    while (1)
    {
        gimbal_set_mode(&gimbal_control);                    // 设置云台控制模式
        gimbal_mode_change_control_transit(&gimbal_control); // 控制模式切换 控制数据过渡
        gimbal_feedback_update(&gimbal_control);             // 云台数据反馈
        gimbal_set_control(&gimbal_control);                 // 设置云台控制量
        gimbal_control_loop(&gimbal_control);                // 云台控制计算

        if (!(toe_is_error(YAW_GIMBAL_MOTOR_TOE) && toe_is_error(PITCH_GIMBAL_MOTOR_TOE)))
        {
            if (toe_is_error(DBUS_TOE))
            {
                // 判断遥控器是否掉线
                CAN_cmd_gimbal(0, 0, 0);
            }
            else
            {
                // 由于安装位置问题pitch轴电机控制值置负
                CAN_cmd_gimbal(gimbal_control.gimbal_yaw_motor.given_current, -gimbal_control.gimbal_pitch_motor.given_current, 0);
            }
        }
        vTaskDelay(GIMBAL_CONTROL_TIME);
#if INCLUDE_uxTaskGetStackHighWaterMark
        gimbal_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

/**
 * @brief          返回yaw 电机数据指针
 * @param[in]      none
 * @retval         yaw电机指针
 */
const gimbal_motor_t *get_yaw_motor_point(void)
{
    return &gimbal_control.gimbal_yaw_motor;
}

/**
 * @brief          返回pitch 电机数据指针
 * @param[in]      none
 * @retval         pitch
 */
const gimbal_motor_t *get_pitch_motor_point(void)
{
    return &gimbal_control.gimbal_pitch_motor;
}

/**
 * @brief          初始化"gimbal_control"变量，包括pid初始化， 遥控器指针初始化，云台电机指针初始化，陀螺仪角度指针初始化
 * @param[out]     init:"gimbal_control"变量指针.
 * @retval         none
 */
static void gimbal_init(gimbal_control_t *init)
{
    const static fp32 gimbal_x_order_filter[1] = {GIMBAL_ACCEL_X_NUM};
    const static fp32 gimbal_y_order_filter[1] = {GIMBAL_ACCEL_Y_NUM + 7};
    const static fp32 gimbal_y_gyro_order_filter[1] = {GIMBAL_ACCEL_Y_GYRO_NUM};
    const static fp32 gimbal_z_order_filter[1] = {GIMBAL_ACCEL_Z_NUM};
    const static fp32 gimbal_x_order_filter_RC[1] = {GIMBAL_ACCEL_X_NUM};
    const static fp32 gimbal_y_order_filter_RC[1] = {GIMBAL_ACCEL_Y_NUM};
    const static fp32 gimbal_x_order_filter_auto[1] = {GIMBAL_ACCEL_X_NUM - 50};
    const static fp32 gimbal_y_order_filter_auto[1] = {GIMBAL_ACCEL_Y_NUM};
    const static fp32 gimbal_vision_yaw_filter[1] = {GIMBAL_VISION_YAW_NUM};
    const static fp32 gimbal_vision_pitch_filter[1] = {GIMBAL_VISION_PITCH_NUM};
    // 给底盘跟随云台模式用的
    gimbal_control.gimbal_yaw_motor.frist_ecd = GIMBAL_YAW_OFFSET_ENCODE;
    gimbal_control.gimbal_yaw_motor.LAST_ZERO_ECD = GIMBAL_YAW_LAST_OFFSET_ENCODE;

    // 电机数据指针获取
    init->gimbal_yaw_motor.gimbal_motor_measure = get_yaw_gimbal_motor_measure_point();
    init->gimbal_pitch_motor.gimbal_motor_measure = get_pitch_gimbal_motor_measure_point();
    // 陀螺仪数据指针获取
    init->gimbal_INT_angle_point = get_INS_angle_point();
    init->gimbal_INT_gyro_point = get_gyro_data_point();
    // 遥控器数据指针获取
    init->gimbal_rc_ctrl = get_remote_control_point();
    // 获取上位机视觉数据指针
    init->gimbal_vision_point = get_vision_gimbal_point();
    // 初始化电机模式
    init->gimbal_yaw_motor.gimbal_motor_mode = init->gimbal_yaw_motor.last_gimbal_motor_mode = GIMBAL_MOTOR_RAW;
    init->gimbal_pitch_motor.gimbal_motor_mode = init->gimbal_pitch_motor.last_gimbal_motor_mode = GIMBAL_MOTOR_RAW;
    // 一阶低通滤波初始化
    first_order_filter_init(&init->gimbal_cmd_slow_set_vx, GIMBAL_CONTROL_TIME, gimbal_x_order_filter);
    first_order_filter_init(&init->gimbal_cmd_slow_set_vy, GIMBAL_CONTROL_TIME, gimbal_y_order_filter);
    first_order_filter_init(&init->gimbal_cmd_slow_set_vy_gyro, GIMBAL_CONTROL_TIME, gimbal_y_gyro_order_filter);
    first_order_filter_init(&init->gimbal_cmd_slow_set_vx_RC, GIMBAL_CONTROL_TIME, gimbal_x_order_filter_RC);
    first_order_filter_init(&init->gimbal_cmd_slow_set_vy_RC, GIMBAL_CONTROL_TIME, gimbal_y_order_filter_RC);
    first_order_filter_init(&init->gimbal_cmd_slow_set_vz, GIMBAL_CONTROL_TIME, gimbal_z_order_filter);
    first_order_filter_init(&init->gimbal_cmd_slow_set_vx_auto, GIMBAL_CONTROL_TIME, gimbal_x_order_filter_auto);
    first_order_filter_init(&init->gimbal_cmd_slow_set_vy_auto, GIMBAL_CONTROL_TIME, gimbal_y_order_filter_auto);
    // 视觉数据处理
    first_order_filter_init(&init->gimbal_vision_control_pitch, GIMBAL_CONTROL_TIME, gimbal_vision_pitch_filter);
    first_order_filter_init(&init->gimbal_vision_control_yaw, GIMBAL_CONTROL_TIME, gimbal_vision_yaw_filter);

    // 初始化yaw电机pid
    stm32_pid_init_yaw();
    // 初始化pitch轴电机pid
    stm32_pid_init_pitch();

    // 云台PID清除
    stm32_step_gimbal_pid_clear();
    // 云台数据更新
    gimbal_feedback_update(init);
    // yaw轴电机初始化
    init->gimbal_yaw_motor.absolute_angle_set = init->gimbal_yaw_motor.absolute_angle;
    init->gimbal_yaw_motor.relative_angle_set = init->gimbal_yaw_motor.relative_angle;
    init->gimbal_yaw_motor.motor_gyro_set = init->gimbal_yaw_motor.motor_gyro;
    // pitch轴电机初始化
    init->gimbal_pitch_motor.absolute_angle_set = init->gimbal_pitch_motor.absolute_angle;
    init->gimbal_pitch_motor.relative_angle_set = init->gimbal_pitch_motor.relative_angle;
    init->gimbal_pitch_motor.motor_gyro_set = init->gimbal_pitch_motor.motor_gyro;

    init->gimbal_yaw_motor.offset_ecd = GIMBAL_YAW_OFFSET_ENCODE;
    init->gimbal_pitch_motor.offset_ecd = GIMBAL_PITCH_OFFSET_ENCODE; // pitch轴云台初始化相对角度

    // 初始化云台自动扫描结构体的扫描范围
    init->gimbal_auto_scan.pitch_range = PITCH_SCAN_RANGE;
    init->gimbal_auto_scan.yaw_range = YAW_SCAN_RANGE;

    // 初始化云台自动扫描周期
    init->gimbal_auto_scan.scan_pitch_period = PITCH_SCAN_PERIOD;
    init->gimbal_auto_scan.scan_yaw_period = YAW_SCAN_PERIOD;

    // 设置pitch轴相对角最大值
    init->gimbal_pitch_motor.max_relative_angle = -motor_ecd_to_angle_change(GIMBAL_PITCH_MAX_ENCODE, init->gimbal_pitch_motor.offset_ecd);
    init->gimbal_pitch_motor.min_relative_angle = -motor_ecd_to_angle_change(GIMBAL_PITCH_MIN_ENCODE, init->gimbal_pitch_motor.offset_ecd);
}

/**
 * @brief          设置云台控制模式，主要在'gimbal_behaviour_mode_set'函数中改变
 * @param[out]     gimbal_set_mode:"gimbal_control"变量指针.
 * @retval         none
 */
static void gimbal_set_mode(gimbal_control_t *set_mode)
{
    if (set_mode == NULL)
    {
        return;
    }
    gimbal_behaviour_mode_set(set_mode);
}

/**
 * @brief          底盘测量数据更新，包括电机速度，欧拉角度，机器人速度
 * @param[out]     gimbal_feedback_update:"gimbal_control"变量指针.
 * @retval         none
 */
static void gimbal_feedback_update(gimbal_control_t *feedback_update)
{
    if (feedback_update == NULL)
    {
        return;
    }
    // 云台数据更新
    feedback_update->gimbal_pitch_motor.absolute_angle = *(feedback_update->gimbal_INT_angle_point + INS_PITCH_ADDRESS_OFFSET);
    feedback_update->gimbal_pitch_motor.relative_angle = -motor_ecd_to_angle_change(feedback_update->gimbal_pitch_motor.gimbal_motor_measure->ecd,
                                                                                    feedback_update->gimbal_pitch_motor.offset_ecd);
    feedback_update->gimbal_pitch_motor.motor_gyro = *(feedback_update->gimbal_INT_gyro_point + INS_GYRO_Y_ADDRESS_OFFSET);

    feedback_update->gimbal_yaw_motor.absolute_angle = *(feedback_update->gimbal_INT_angle_point + INS_YAW_ADDRESS_OFFSET);
    feedback_update->gimbal_yaw_motor.relative_angle = motor_ecd_to_angle_change(feedback_update->gimbal_yaw_motor.gimbal_motor_measure->ecd, feedback_update->gimbal_yaw_motor.frist_ecd);
    feedback_update->gimbal_yaw_motor.motor_gyro = arm_cos_f32(feedback_update->gimbal_pitch_motor.relative_angle) * (*(feedback_update->gimbal_INT_gyro_point + INS_GYRO_Z_ADDRESS_OFFSET)) - arm_sin_f32(feedback_update->gimbal_pitch_motor.relative_angle) * (*(feedback_update->gimbal_INT_gyro_point + INS_GYRO_X_ADDRESS_OFFSET));
}

/**
 * @brief          calculate the relative angle between ecd and offset_ecd
 * @param[in]      ecd: motor now encode
 * @param[in]      offset_ecd: gimbal offset encode
 * @retval         relative angle, unit rad
 */
/**
 * @brief          计算ecd与offset_ecd之间的相对角度
 * @param[in]      ecd: 电机当前编码
 * @param[in]      offset_ecd: 电机中值编码
 * @retval         相对角度，单位rad
 */
static fp32 motor_ecd_to_angle_change(uint16_t ecd, uint16_t offset_ecd)
{
    fp32 relative_ecd = ecd - offset_ecd;

    if (relative_ecd > gimbal_control.gimbal_yaw_motor.frist_ecd + 4096)
    {
        // relative_ecd -= 8191;
    }
    return relative_ecd * MOTOR_ECD_TO_RAD;
}

/**
 * @brief          when gimbal mode change, some param should be changed, suan as  yaw_set should be new yaw
 * @param[out]     gimbal_mode_change: "gimbal_control" valiable point
 * @retval         none
 */
/**
 * @brief          云台模式改变，有些参数需要改变，例如控制yaw角度设定值应该变成当前yaw角度
 * @param[out]     gimbal_mode_change:"gimbal_control"变量指针.
 * @retval         none
 */
static void gimbal_mode_change_control_transit(gimbal_control_t *gimbal_mode_change)
{
    if (gimbal_mode_change == NULL)
    {
        return;
    }
    // yaw电机状态机切换保存数据
    if (gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_RAW && gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        gimbal_mode_change->gimbal_yaw_motor.raw_cmd_current = gimbal_mode_change->gimbal_yaw_motor.current_set = gimbal_mode_change->gimbal_yaw_motor.given_current;
    }
    else if (gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_GYRO && gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        gimbal_mode_change->gimbal_yaw_motor.absolute_angle_set = gimbal_mode_change->gimbal_yaw_motor.absolute_angle;
    }
    else if (gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_ENCONDE && gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        gimbal_mode_change->gimbal_yaw_motor.relative_angle_set = gimbal_mode_change->gimbal_yaw_motor.relative_angle;
    }
    gimbal_mode_change->gimbal_yaw_motor.last_gimbal_motor_mode = gimbal_mode_change->gimbal_yaw_motor.gimbal_motor_mode;

    // pitch电机状态机切换保存数据
    if (gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_RAW && gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        gimbal_mode_change->gimbal_pitch_motor.raw_cmd_current = gimbal_mode_change->gimbal_pitch_motor.current_set = gimbal_mode_change->gimbal_pitch_motor.given_current;
    }
    else if (gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_GYRO && gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        gimbal_mode_change->gimbal_pitch_motor.absolute_angle_set = gimbal_mode_change->gimbal_pitch_motor.absolute_angle;
    }
    else if (gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode != GIMBAL_MOTOR_ENCONDE && gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        gimbal_mode_change->gimbal_pitch_motor.relative_angle_set = gimbal_mode_change->gimbal_pitch_motor.relative_angle;
    }

    gimbal_mode_change->gimbal_pitch_motor.last_gimbal_motor_mode = gimbal_mode_change->gimbal_pitch_motor.gimbal_motor_mode;
}
/**
 * @brief          set gimbal control set-point, control set-point is set by "gimbal_behaviour_control_set".
 * @param[out]     gimbal_set_control: "gimbal_control" valiable point
 * @retval         none
 */
/**
 * @brief          设置云台控制设定值，控制值是通过gimbal_behaviour_control_set函数设置的
 * @param[out]     gimbal_set_control:"gimbal_control"变量指针.
 * @retval         none
 */
static void gimbal_set_control(gimbal_control_t *set_control)
{
    if (set_control == NULL)
    {
        return;
    }

    fp32 add_yaw_angle = 0.0f;
    fp32 add_pitch_angle = 0.0f;

    gimbal_behaviour_control_set(&add_yaw_angle, &add_pitch_angle, set_control);
    // yaw电机模式控制
    if (set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        // raw模式下，直接发送控制值
        set_control->gimbal_yaw_motor.raw_cmd_current = add_yaw_angle;
    }
    else if (set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        // gyro模式下，陀螺仪角度控制
        gimbal_absolute_angle_limit(&set_control->gimbal_yaw_motor, add_yaw_angle);
    }
    else if (set_control->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        // enconde模式下，电机编码角度控制
        gimbal_relative_angle_limit(&set_control->gimbal_yaw_motor, add_yaw_angle);
    }

    // pitch电机模式控制
    if (set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        // raw模式下，直接发送控制值
        set_control->gimbal_pitch_motor.raw_cmd_current = add_pitch_angle;
    }
    else if (set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        // gyro模式下，陀螺仪角度控制
        gimbal_absolute_angle_limit(&set_control->gimbal_pitch_motor, add_pitch_angle);
    }
    else if (set_control->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        // enconde模式下，电机编码角度控制
        gimbal_relative_angle_limit(&set_control->gimbal_pitch_motor, add_pitch_angle);
    }
}
/**
 * @brief          gimbal control mode :GIMBAL_MOTOR_GYRO, use euler angle calculated by gyro sensor to control.
 * @param[out]     gimbal_motor: yaw motor or pitch motor
 * @retval         none
 */
/**
 * @brief          云台控制模式:GIMBAL_MOTOR_GYRO，使用陀螺仪计算的欧拉角进行控制
 * @param[out]     gimbal_motor:yaw电机或者pitch电机
 * @retval         none
 */
static void gimbal_absolute_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add)
{
    static fp32 angle_set_yaw = 0;

    if (gimbal_motor == NULL)
    {
        return;
    }
    if (gimbal_motor == &gimbal_control.gimbal_yaw_motor)
    {
        angle_set_yaw = gimbal_motor->absolute_angle_set + INS_YAW_ERROR; // 陀螺仪问题
        gimbal_motor->absolute_angle_set = rad_format(angle_set_yaw + add);
    }
    else
    {
        // 当前误差角度
        static fp32 error_angle = 0;
        static fp32 angle_set = 0;
        error_angle = rad_format(gimbal_motor->absolute_angle_set - gimbal_motor->absolute_angle);
        // 云台相对角度+ 误差角度 + 新增角度 如果大于 最大机械角度
        if (gimbal_motor->relative_angle + error_angle + add > gimbal_motor->max_relative_angle)
        {
            // 如果是往最大机械角度控制方向
            if (add > 0.0f)
            {
                // 计算出一个最大的添加角度，
                add = gimbal_motor->max_relative_angle - gimbal_motor->relative_angle - error_angle;
            }
        }
        else if (gimbal_motor->relative_angle + error_angle + add < gimbal_motor->min_relative_angle)
        {
            if (add < 0.0f)
            {
                add = gimbal_motor->min_relative_angle - gimbal_motor->relative_angle - error_angle;
            }
        }
        angle_set = gimbal_motor->absolute_angle_set;
        gimbal_motor->absolute_angle_set = rad_format(angle_set + add);
    }
}
/**
 * @brief          gimbal control mode :GIMBAL_MOTOR_ENCONDE, use the encode relative angle  to control.
 * @param[out]     gimbal_motor: yaw motor or pitch motor
 * @retval         none
 */
/**
 * @brief          云台控制模式:GIMBAL_MOTOR_ENCONDE，使用编码相对角进行控制
 * @param[out]     gimbal_motor:yaw电机或者pitch电机
 * @retval         none
 */
static void gimbal_relative_angle_limit(gimbal_motor_t *gimbal_motor, fp32 add)
{
    if (gimbal_motor == NULL)
    {
        return;
    }

    gimbal_motor->relative_angle_set += add;

    if (gimbal_motor == &gimbal_control.gimbal_yaw_motor)
    {
        // 云台yaw轴相对角度限制
        if (gimbal_motor->relative_angle_set < 0)
        {
            gimbal_motor->relative_angle_set = 2 * PI + gimbal_motor->relative_angle_set;
        }
        else if (gimbal_motor->relative_angle_set > 2 * PI)
        {
            gimbal_motor->relative_angle_set = gimbal_motor->relative_angle_set - 2 * PI;
        }
    }
    else if (gimbal_motor == &gimbal_control.gimbal_pitch_motor)
    {
        // 云台pitch轴相对角度限制，防止pitch轴转动过度
        if (gimbal_motor->relative_angle_set >= motor_ecd_to_angle_change(GIMBAL_PITCH_MAX_ENCODE, gimbal_motor->offset_ecd))
        {
            gimbal_motor->relative_angle_set = motor_ecd_to_angle_change(GIMBAL_PITCH_MAX_ENCODE, gimbal_motor->offset_ecd);
        }
        else if (gimbal_motor->relative_angle_set <= motor_ecd_to_angle_change(GIMBAL_PITCH_MIN_ENCODE, gimbal_motor->offset_ecd))
        {
            gimbal_motor->relative_angle_set = motor_ecd_to_angle_change(GIMBAL_PITCH_MIN_ENCODE, gimbal_motor->offset_ecd);
        }
    }
}

/**
 * @brief          控制循环，根据控制设定值，计算电机电流值，进行控制
 * @param[out]     gimbal_control_loop:"gimbal_control"变量指针.
 * @retval         none
 */
static void gimbal_control_loop(gimbal_control_t *control_loop)
{
    if (control_loop == NULL)
    {
        return;
    }

    if (control_loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        gimbal_motor_raw_angle_control(&control_loop->gimbal_yaw_motor);
    }
    else if (control_loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        gimbal_motor_absolute_angle_control(&control_loop->gimbal_yaw_motor);
    }
    else if (control_loop->gimbal_yaw_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        gimbal_motor_relative_angle_control(&control_loop->gimbal_yaw_motor);
    }

    if (control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_RAW)
    {
        gimbal_motor_raw_angle_control(&control_loop->gimbal_pitch_motor);
    }
    else if (control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_GYRO)
    {
        gimbal_motor_absolute_angle_control(&control_loop->gimbal_pitch_motor);
    }
    else if (control_loop->gimbal_pitch_motor.gimbal_motor_mode == GIMBAL_MOTOR_ENCONDE)
    {
        gimbal_motor_relative_angle_control(&control_loop->gimbal_pitch_motor);
    }
}

/**
 * @brief          云台控制模式:GIMBAL_MOTOR_GYRO，使用陀螺仪计算的欧拉角进行控制
 * @param[out]     gimbal_motor:yaw电机或者pitch电机
 * @retval         none
 */
static void gimbal_motor_absolute_angle_control(gimbal_motor_t *gimbal_motor)
{
    if (gimbal_motor == NULL)
    {
        return;
    }
    if (gimbal_motor == &gimbal_control.gimbal_yaw_motor)
    {
        stm32_step_yaw(gimbal_motor->absolute_angle_set, gimbal_motor->absolute_angle, gimbal_motor->motor_gyro);
        gimbal_motor->current_set = stm32_Y_yaw.Out1;
        gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
    }
    else if (gimbal_motor == &gimbal_control.gimbal_pitch_motor)
    {
        stm32_step_pitch(gimbal_motor->absolute_angle_set, gimbal_motor->absolute_angle, gimbal_motor->motor_gyro);
        gimbal_motor->current_set = stm32_Y_pitch.Out1;
        gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
    }
}
/**
 * @brief          云台控制模式:GIMBAL_MOTOR_ENCONDE，使用编码相对角进行控制
 * @param[out]     gimbal_motor:yaw电机或者pitch电机
 * @retval         none
 */
static void gimbal_motor_relative_angle_control(gimbal_motor_t *gimbal_motor)
{
    if (gimbal_motor == NULL)
    {
        return;
    }
    if (gimbal_motor == &gimbal_control.gimbal_yaw_motor)
    {
        stm32_step_yaw(gimbal_motor->relative_angle_set, gimbal_motor->relative_angle, 0);
        gimbal_motor->current_set = stm32_Y_yaw.Out1;
        gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
    }
    else if (gimbal_motor == &gimbal_control.gimbal_pitch_motor)
    {
        stm32_step_pitch(gimbal_motor->relative_angle_set, gimbal_motor->relative_angle, gimbal_motor->motor_gyro);
        gimbal_motor->current_set = stm32_Y_pitch.Out1;
        // 控制值赋值
        gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
    }
}

/**
 * @brief          云台控制模式:GIMBAL_MOTOR_RAW，电流值直接发送到CAN总线.
 * @param[out]     gimbal_motor:yaw电机或者pitch电机
 * @retval         none
 */
static void gimbal_motor_raw_angle_control(gimbal_motor_t *gimbal_motor)
{
    if (gimbal_motor == NULL)
    {
        return;
    }
    gimbal_motor->current_set = gimbal_motor->raw_cmd_current;
    gimbal_motor->given_current = (int16_t)(gimbal_motor->current_set);
}

#if GIMBAL_TEST_MODE
int32_t yaw_ins_int_1000, pitch_ins_int_1000;
int32_t yaw_ins_set_1000, pitch_ins_set_1000;
int32_t pitch_relative_set_1000, pitch_relative_angle_1000;
int32_t yaw_speed_int_1000, pitch_speed_int_1000;
int32_t yaw_speed_set_int_1000, pitch_speed_set_int_1000;
static void J_scope_gimbal_test(void)
{
    yaw_ins_int_1000 = (int32_t)(gimbal_control.gimbal_yaw_motor.absolute_angle * 1000);
    yaw_ins_set_1000 = (int32_t)(gimbal_control.gimbal_yaw_motor.absolute_angle_set * 1000);
    yaw_speed_int_1000 = (int32_t)(gimbal_control.gimbal_yaw_motor.motor_gyro * 1000);
    yaw_speed_set_int_1000 = (int32_t)(gimbal_control.gimbal_yaw_motor.motor_gyro_set * 1000);

    pitch_ins_int_1000 = (int32_t)(gimbal_control.gimbal_pitch_motor.absolute_angle * 1000);
    pitch_ins_set_1000 = (int32_t)(gimbal_control.gimbal_pitch_motor.absolute_angle_set * 1000);
    pitch_speed_int_1000 = (int32_t)(gimbal_control.gimbal_pitch_motor.motor_gyro * 1000);
    pitch_speed_set_int_1000 = (int32_t)(gimbal_control.gimbal_pitch_motor.motor_gyro_set * 1000);
    pitch_relative_angle_1000 = (int32_t)(gimbal_control.gimbal_pitch_motor.relative_angle * 1000);
    pitch_relative_set_1000 = (int32_t)(gimbal_control.gimbal_pitch_motor.relative_angle_set * 1000);
}

#endif
