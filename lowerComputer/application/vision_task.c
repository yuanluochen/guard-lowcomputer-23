/**
 * @file vision_task.c
 * @author yuanluochen
 * @brief 发送自身yaw轴pitch轴roll轴绝对角度给上位机视觉，并解算视觉接收数据，由于hal库自身原因对全双工串口通信支持不是特别好，
 *        为处理这个问题将串口数据分析，与串口发送分离，并延长串口发送时间
 * @version 0.1
 * @date 2023-03-11
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "vision_task.h"
#include "FreeRTOS.h"
#include "task.h"


//视觉发送任务结构体初始化
static void vision_send_task_init(vision_send_t* init);
//视觉任务初始化
static void vision_task_init(vision_control_t* init);
//视觉发送任务数据更新
static void vision_send_task_feedback_update(vision_send_t* update);
//视觉人物数据更新
static void vision_task_feedback_update(vision_control_t* update);
//数据编码
static void vision_tx_encode(uint8_t* buf, float yaw, float pitch, float roll, uint8_t mode_switch);
//设置yaw轴pitch轴增量
static void vision_set_add_value(vision_control_t* vision_set);
//发送数据
static void send_message_to_vision(UART_HandleTypeDef* send_message_usart, DMA_HandleTypeDef* send_message_dma, uint8_t* send_message, uint16_t send_message_size);
/**
 * @brief 二阶kalman filter 初始化 针对yaw轴pitch轴角度运动模型
 *  
 * @param kalman_filter_struct kalman filter运算矩阵
 * @param kalman_filter_init_struct kalman filter 初始化赋值矩阵，用于给kalman filter运算矩阵赋值
 * @param Dp 位移方差
 * @param Dv 速度方差
 * @param Da 加速度方差
 * @param Dt 运算间隔
 */
static void second_order_kalman_filter_init(kalman_filter_t* kalman_filter_struct, 
                                            kalman_filter_init_t* kalman_filter_init_struct, 
                                            float Dp, float Dv, float Da, float Dt);

/**
 * @brief 分析视觉原始增加数据，根据原始数据，判断是否要进行发射
 * 
 * @param shoot_judge 视觉结构体
 * @param vision_begin_add_yaw_angle 上位机视觉yuw轴原始增加角度
 * @param vision_begin_add_pitch_angle 上位机视觉pitch轴原始增加角度
 */
static void vision_shoot_judge(vision_control_t* shoot_judge, fp32 vision_begin_add_yaw_angle, fp32 vision_begin_add_pitch_angle);

/**
 * @brief 上位机数据发送
 * 
 * @param vision_send 上位机数据发送结构体
 */
static void vision_send_msg(vision_send_t* vision_send);
//视觉任务结构体
vision_control_t vision_control = { 0 };
//视觉发送任务结构体
vision_send_t vision_send = { 0 };


void vision_send_task(void const *pvParameters)
{
    // 延时等待，等待陀螺仪任务更新陀螺仪数据
    vTaskDelay(VISION_SEND_TASK_INIT_TIME);
    //视觉发送任务结构体初始化
    vision_send_task_init(&vision_send);
    while(1)
    {
        //发送当前位姿传递给上位机视觉
        vision_send_msg(&vision_send);
        //系统延时
        vTaskDelay(VISION_SEND_CONTROL_TIME_MS);
        
    }
}


void vision_task(void const* pvParameters)
{
    //延时等待，等待上位机发送数据成功
    vTaskDelay(VISION_TASK_INIT_TIME);
    //视觉任务初始化
    vision_task_init(&vision_control);
    while(1)
    {
        // 更新数据
        vision_task_feedback_update(&vision_control);
        // 解析上位机数据,配置yaw轴pitch轴增量
        vision_set_add_value(&vision_control);
        //系统延时
        vTaskDelay(VISION_CONTROL_TIME_MS);
    }
    
}

static void vision_send_msg(vision_send_t* vision_send)
{
    //数据更新
    vision_send_task_feedback_update(vision_send);

    //配置串口发送数据,编码
    vision_tx_encode(vision_send->send_message, vision_send->send_absolution_angle.yaw * RADIAN_TO_ANGle,
                                                vision_send->send_absolution_angle.pitch * RADIAN_TO_ANGle,
                                                vision_send->send_absolution_angle.roll * RADIAN_TO_ANGle,
                                                ARMOURED_PLATE);
    //串口发送
    send_message_to_vision(vision_send->send_message_usart, vision_send->send_message_dma, vision_send->send_message, SERIAL_SEND_MESSAGE_SIZE);
    
}

static void send_message_to_vision(UART_HandleTypeDef* send_message_usart, DMA_HandleTypeDef* send_message_dma, 
                                  uint8_t* send_message, uint16_t send_message_size)
{
    //等待上一次数据发送
    while (HAL_DMA_GetState(send_message_dma) == HAL_DMA_STATE_BUSY)
    {
        static int count = 0;
        if (count ++ >= 1000)
        {
            break;
        }
        
    }
    //关闭DMA
    __HAL_DMA_DISABLE(send_message_dma);

    //发送数据
    HAL_UART_Transmit_DMA(send_message_usart, send_message, send_message_size);
}

static void vision_send_task_init(vision_send_t* init)
{
    //获取陀螺仪绝对角指针                                                                                                                                                                                                                                                                                                                                                           init->vision_angle_point = get_INS_angle_point();
    init->vision_angle_point = get_INS_angle_point();
    //初始化发送串口名称
    init->send_message_usart = &VISION_USART;
    //初始化发送dma
    init->send_message_dma = &VISION_TX_DMA;

    // 数据更新
    vision_send_task_feedback_update(init);
}

static void vision_task_init(vision_control_t* init)
{

    // 获取上位机视觉指针
    init->vision_rxfifo = get_vision_rxfifo_point();
#if KALMAN_FILTER_TYPE
    //初始化一维kalman filter
    kalmanCreate(&init->vision_kalman_filter.gimbal_pitch_kalman, GIMBAL_PITCH_MOTOR_KALMAN_Q, GIMBAL_PITCH_MOTOR_KALMAN_R);
    kalmanCreate(&init->vision_kalman_filter.gimbal_yaw_kalman, GIMBAL_YAW_MOTOR_KALMAN_Q, GIMBAL_YAW_MOTOR_KALMAN_R);
#else

    //初始化二阶kalman filter 
    second_order_kalman_filter_init(&init->vision_kalman_filter.gimbal_pitch_second_order_kalman, &init->vision_kalman_filter.gimbal_pitch_second_order_kalman_init, 
                                    PITCH_DP,
                                    PITCH_DV,
                                    PITCH_DA,
                                    DT);
    second_order_kalman_filter_init(&init->vision_kalman_filter.gimbal_yaw_second_order_kalman, &init->vision_kalman_filter.gimbal_pitch_second_order_kalman_init,
                                    YAW_DP,
                                    YAW_DV,
                                    YAW_DA,
                                    DT);
#endif
    //初始化发射模式为停止袭击
    init->shoot_vision_control.shoot_command = SHOOT_STOP_ATTACK;

    vision_task_feedback_update(init);
 
}
static void vision_send_task_feedback_update(vision_send_t* update)
{
    // 获取原始数据
    update->absolution_angle.yaw = *(update->vision_angle_point + INS_YAW_ADDRESS_OFFSET);
    update->absolution_angle.pitch = -*(update->vision_angle_point + INS_PITCH_ADDRESS_OFFSET);
    update->absolution_angle.roll = *(update->vision_angle_point + INS_ROLL_ADDRESS_OFFSET);

    //更新发送数据,为处理负号，数据加180
    update->send_absolution_angle.yaw = update->absolution_angle.yaw + SEND_MESSAGE_ERROR;
    update->send_absolution_angle.pitch = update->absolution_angle.pitch + SEND_MESSAGE_ERROR;
    update->send_absolution_angle.roll = update->absolution_angle.roll + SEND_MESSAGE_ERROR;
}
static void vision_task_feedback_update(vision_control_t* update)
{
    // 获取原始数据
    update->absolution_angle.yaw = *(update->vision_angle_point + INS_YAW_ADDRESS_OFFSET);
    update->absolution_angle.pitch = -*(update->vision_angle_point + INS_PITCH_ADDRESS_OFFSET);
    update->absolution_angle.roll = *(update->vision_angle_point + INS_ROLL_ADDRESS_OFFSET);

}
static void vision_tx_encode(uint8_t* buf, float yaw, float pitch, float roll, uint8_t mode_switch)
{
    //数据起始
    date32_to_date8_t head1_temp = { 0 };
    date32_to_date8_t head2_temp = { 0 };
    //yaw轴数值转化
    date32_to_date8_t yaw_temp = { 0 };
    //pitch轴数值转化
    date32_to_date8_t pitch_temp = { 0 };
    //roll轴数据转化
    date32_to_date8_t roll_temp = { 0 };
    //模式转换
    date32_to_date8_t switch_temp = { 0 };
    //中止位
    date32_to_date8_t end_temp = { 0 };

    //赋值数据
    head1_temp.uint32_val = HEAD1_DATA;
    head2_temp.uint32_val = HEAD2_DATA;
    yaw_temp.uint32_val = yaw * DOUBLE_;
    pitch_temp.uint32_val = pitch * DOUBLE_;
    roll_temp.uint32_val = roll * DOUBLE_;  
    switch_temp.uint32_val = mode_switch;
    end_temp.uint32_val = 0x0A;//"/n"

    for (int i = 3; i >= 0; i--)
    {
        int j = -(i - 3);//数据位置转换
        buf[HEAD1_ADDRESS_OFFSET * UINT8_T_DATA_SIZE + j] = head1_temp.uin8_value[i];
        buf[HEAD2_ADDRESS_OFFSET * UINT8_T_DATA_SIZE + j] = head2_temp.uin8_value[i];
        buf[YAW_ADDRESS_OFFSET * UINT8_T_DATA_SIZE + j] = yaw_temp.uin8_value[i];
        buf[PITCH_ADDRESS_OFFSET * UINT8_T_DATA_SIZE + j] = pitch_temp.uin8_value[i];
        buf[ROLL_ADDRESS_OFFSET * UINT8_T_DATA_SIZE + j] = roll_temp.uin8_value[i];
        buf[SWITCH_ADDRESS_OFFSET * UINT8_T_DATA_SIZE + j] = switch_temp.uin8_value[i]; 
        buf[END_ADDRESS_OFFSET * UINT8_T_DATA_SIZE + j] = end_temp.uin8_value[i];
    }
}

static void vision_set_add_value(vision_control_t* vision_set)
{
    // 上位机视觉原始数据
    static fp32 vision_gimbal_yaw = 0;   // yaw轴绝对角
    static fp32 vision_gimbal_pitch = 0; // pitch轴绝对角

    // kalman filter 迭代开始标志位、
    static bool_t kalman_filter_start_flag = false;

    // 上位机
    //  判断是否接收到上位机数据
    if (vision_set->vision_rxfifo->rx_flag)//识别到目标
    {
        // 接收到上位机数据
        // 接收位置零
        vision_set->vision_rxfifo->rx_flag = 0;

        // 获取上位机视觉数据
        vision_gimbal_pitch = vision_set->vision_rxfifo->pitch_fifo;
        vision_gimbal_yaw = vision_set->vision_rxfifo->yaw_fifo;

        // 判断发射
        vision_shoot_judge(vision_set, (vision_gimbal_yaw - vision_set->absolution_angle.yaw), (vision_gimbal_pitch - vision_set->absolution_angle.pitch));

        if (kalman_filter_start_flag == false)
        {
            // 命令 kalman filter 迭代开始
            kalman_filter_start_flag = true;
        }
        
    }
    else
    {
        //设置发射命令为停止发射
        vision_set->shoot_vision_control.shoot_command = SHOOT_STOP_ATTACK;
    }
    
    // 等待上位机第一次发送数据，判断开始迭代
    if (kalman_filter_start_flag == true)
    {

        // kalman filter 
#if KALMAN_FILTER_TYPE
        // 处理上位机视觉数据,对视觉数据进行kalman filter
        KalmanFilter(&vision_set->vision_kalman_filter.gimbal_yaw_kalman, vision_gimbal_yaw);
        KalmanFilter(&vision_set->vision_kalman_filter.gimbal_pitch_kalman, vision_gimbal_pitch);

        // 读取上位机kalman filter处理数据
        vision_set->gimbal_vision_control.gimbal_pitch_add = vision_set->vision_kalman_filter.gimbal_pitch_kalman.X_now - vision_set->absolution_angle.pitch;
        vision_set->gimbal_vision_control.gimbal_yaw_add = vision_set->vision_kalman_filter.gimbal_yaw_kalman.X_now - vision_set->absolution_angle.yaw;
#else
        // 计算二阶kalamn filer,速度自我迭代
        kalman_filter_calc(&vision_set->vision_kalman_filter.gimbal_pitch_second_order_kalman, vision_gimbal_pitch);
        kalman_filter_calc(&vision_set->vision_kalman_filter.gimbal_yaw_second_order_kalman, vision_gimbal_yaw);

        // 读取上位机kalamn filter处理后的数值
        vision_set->gimbal_vision_control.gimbal_pitch_add = vision_set->vision_kalman_filter.gimbal_pitch_second_order_kalman.filtered_value[GIMBAL_ANGLE_ADDRESS_OFFSET] - vision_set->absolution_angle.pitch;
        vision_set->gimbal_vision_control.gimbal_yaw_add = vision_set->vision_kalman_filter.gimbal_yaw_second_order_kalman.filtered_value[GIMBAL_ANGLE_ADDRESS_OFFSET] - vision_set->absolution_angle.yaw;
#endif
    }

}

/**
 * @brief 二阶kalman filter 初始化 针对yaw轴pitch轴角度运动模型
 * 
 * @param kalman_filter_struct kalman filter运算矩阵
 * @param kalman_filter_init_struct kalman filter 初始化赋值矩阵，用于给kalman filter运算矩阵赋值
 * @param Dp 位移方差
 * @param Dv 速度方差
 * @param Da 加速度方差
 * @param Dt 运算间隔
 */
static void second_order_kalman_filter_init(kalman_filter_t* kalman_filter_struct, 
                                            kalman_filter_init_t* kalman_filter_init_struct, 
                                            float Dp, float Dv, float Da, float Dt)
{
    //状态矩阵
    float X[STATUS_MATRIX_SIZE] = {
        1,
        1
    };

    //状态转移矩阵
    float H[H_MATRIX_SIZE] = {
        1,
        1
    };

    //状态转移矩阵
    float A[MATRIX_SIZE] = {
        1.0f, Dt * 6.0f,
        0.0f, 1.0f
    };

    //状态协方差矩阵,数值不需要太精准,但绝对不可以太小，如果太小会使响应过慢
    float P[MATRIX_SIZE] = {
        20.0f, 0.0f,
        0.0f, 20.0f
    };

    //过程噪声协方差矩阵
    float Q[MATRIX_SIZE] = {
        0.25f * pow((double)Dt, 4) * Da, 0.5f * pow((double)Dt, 3) * Da,
        0.5f  * pow((double)Dt, 3) * Da,        pow((double)Dt, 2) * Da
    };

    //观测噪声协方差矩阵
    float R[MATRIX_SIZE] = {
        Dp, 0.0f,
        0.0f, Dv
    };
    
    //赋值数组
    memcpy(kalman_filter_init_struct->xhat_data, X, STATUS_MATRIX_SIZE * sizeof(float));
    memcpy(kalman_filter_init_struct->H_data, H, H_MATRIX_SIZE * sizeof(float));
    memcpy(kalman_filter_init_struct->A_data, A, MATRIX_SIZE * sizeof(float));
    memcpy(kalman_filter_init_struct->P_data, P, MATRIX_SIZE * sizeof(float));
    memcpy(kalman_filter_init_struct->Q_data, Q, MATRIX_SIZE * sizeof(float));
    memcpy(kalman_filter_init_struct->R_data, R, MATRIX_SIZE * sizeof(float));
    


    //初始化kalman filter结构体
    kalman_filter_init(kalman_filter_struct, kalman_filter_init_struct);
}

/**
 * @brief 分析视觉原始增加数据，根据原始数据，判断是否要进行发射，判断yaw轴pitch的角度，如果在一定范围内，则计算值增加，增加到一定数值则判断发射，如果yaw轴pitch轴角度大于该范围，则计数归零
 * 
 * @param shoot_judge 视觉结构体
 * @param vision_begin_add_yaw_angle 上位机视觉yuw轴原始增加角度
 * @param vision_begin_add_pitch_angle 上位机视觉pitch轴原始增加角度
 */
static void vision_shoot_judge(vision_control_t* shoot_judge, fp32 vision_begin_add_yaw_angle, fp32 vision_begin_add_pitch_angle)
{
    // 判断击打计数
    static int attack_count = 0;
    // 判断停止击打的次数 
    static int stop_attack_count = 0;

    
    // 上位机发送角度到一定方位内计数值增加
    if (fabs(vision_begin_add_pitch_angle) <= ALLOW_ATTACK_ERROR && fabs(vision_begin_add_yaw_angle) <= ALLOW_ATTACK_ERROR)
    {
        // 停止击打计数值归零
        stop_attack_count = 0;

        // 判断计数值是否到达判断击打的计数值
        if (attack_count >= JUDGE_ATTACK_COUNT)
        {
            // 到达可击打的次数
            // 设置击打
            shoot_judge->shoot_vision_control.shoot_command = SHOOT_ATTACK;
        }
        else
        {
            // 未到达可击打的次数
            // 计数值增加
            attack_count++;
        }
    }
    // 上位机发射角度大于该范围计数值归零
    else if (fabs(vision_begin_add_pitch_angle) >= ALLOW_ATTACK_ERROR || fabs(vision_begin_add_yaw_angle) >= ALLOW_ATTACK_ERROR)
    {
        // 判断击打计数值归零
        attack_count = 0;

        if (stop_attack_count >= JUDGE_STOP_ATTACK_COUNT)
        {
            //达到停止击打的计数
            //设置准备击打
            shoot_judge->shoot_vision_control.shoot_command = SHOOT_READY_ATTACK;
        }
        else
        {
            // 未到达停止击打的次数
            // 计数值增加
            stop_attack_count ++;
        }
        
    }
}

// 获取上位机云台命令
gimbal_vision_control_t* get_vision_gimbal_point(void)
{
    return &vision_control.gimbal_vision_control;
}

// 获取上位机发射命令
shoot_vision_control_t* get_vision_shoot_point(void)
{
    return &vision_control.shoot_vision_control;
}

