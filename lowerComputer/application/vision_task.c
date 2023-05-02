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
#include "shoot_task.h"
#include "gimbal_behaviour.h"

#include "usbd_cdc_if.h"


//视觉发送任务结构体初始化
static void vision_send_task_init(vision_send_t* init);
//视觉任务初始化
static void vision_task_init(vision_control_t* init);
//视觉发送任务数据更新
static void vision_send_task_feedback_update(vision_send_t* update);
//设置自瞄模式
static void vision_set_mode(vision_send_t* vision_mode);
//视觉人物数据更新
static void vision_task_feedback_update(vision_control_t* update);
//数据编码
static void vision_tx_encode(vision_send_t* vision_tx_data_encode);
//设置yaw轴pitch轴增量
static void vision_analysis_date(vision_control_t* vision_set);
/**
 * @brief 分析视觉原始增加数据，根据原始数据，判断是否要进行发射
 * 
 * @param shoot_judge 视觉结构体
 * @param vision_begin_add_yaw_angle 上位机视觉yuw轴原始增加角度
 * @param vision_begin_add_pitch_angle 上位机视觉pitch轴原始增加角度
 */
static void vision_shoot_judge(vision_control_t* shoot_judge, fp32 vision_begin_add_yaw_angle, fp32 vision_begin_add_pitch_angle);


//视觉任务结构体
vision_control_t vision_control = { 0 };
//视觉发送任务结构体
vision_send_t vision_send = { 0 };
//视觉接收结构体
vision_receive_t vision_receive = { 0 };

//未接收到视觉数据标志位，该位为1 则未接收
bool_t not_rx_vision_data_flag = 1;


void vision_send_task(void const *pvParameters)
{
    // 延时等待，等待陀螺仪任务更新陀螺仪数据
    vTaskDelay(VISION_SEND_TASK_INIT_TIME);
    //视觉发送任务结构体初始化
    vision_send_task_init(&vision_send);
    while(1)
    {
        // 数据更新
        vision_send_task_feedback_update(&vision_send);
        // 配置自瞄模式
        vision_set_mode(&vision_send);
        // 配置串口发送数据,编码
        vision_tx_encode(&vision_send);

        //发送数据
        CDC_Transmit_FS(vision_send.send_message, SUM_SEND_MESSAGE);
        //系统延时
        vTaskDelay(VISION_SEND_CONTROL_TIME_MS);
    }
}


void vision_task(void const* pvParameters)
{
    // 延时等待，等待上位机发送数据成功
    vTaskDelay(VISION_TASK_INIT_TIME);
    // 视觉任务初始化
    vision_task_init(&vision_control);
    while (1)
    {
        // 等待预装弹完毕，以及云台底盘初始化完毕，判断任务是否进行
        if (shoot_control_vision_task() && gimbal_control_vision_task())
        {
            // 更新数据
            vision_task_feedback_update(&vision_control);
            // 解析上位机数据,配置yaw轴pitch轴增量,以及判断是否发射
            vision_analysis_date(&vision_control);
        }
        else
        {
            //任务不运行
        }
        // 系统延时
        vTaskDelay(VISION_CONTROL_TIME_MS);
    }
}

static void vision_send_task_init(vision_send_t* init)
{
    //获取陀螺仪绝对角指针                                                                                                                                                                                                                                                                                                                                                           init->vision_angle_point = get_INS_angle_point();
    init->vision_quat_point = get_INS_quat_point();
    //初始化发送数据结构体的头帧尾帧
    init->send_msg_struct.head = LOWER_TO_HIGH_HEAD;
    init->send_msg_struct.end = END_DATA;
    // 数据更新
    vision_send_task_feedback_update(init);
}

static void vision_task_init(vision_control_t* init)
{
    // 获取陀螺仪绝对角指针                                                                                                                                                                                                                                                                                                                                                           init->vision_angle_point = get_INS_angle_point();
    init->vision_angle_point = get_INS_angle_point();
	
    // 获取上位机视觉指针
    init->vision_rxfifo = get_vision_rxfifo_point();
    //初始化发射模式为停止袭击
    init->shoot_vision_control.shoot_command = SHOOT_STOP_ATTACK;

    vision_task_feedback_update(init);
 
}

static void vision_send_task_feedback_update(vision_send_t* update)
{
    //获取四元数数值
    memcpy(update->send_msg_struct.quat, update->vision_quat_point, 4 * sizeof(fp32));
}

static void vision_task_feedback_update(vision_control_t* update)
{
    // 获取原始数据并转化为角度制
    update->absolution_angle.yaw = *(update->vision_angle_point + INS_YAW_ADDRESS_OFFSET) * RADIAN_TO_ANGLE;
    update->absolution_angle.pitch = *(update->vision_angle_point + INS_PITCH_ADDRESS_OFFSET) * RADIAN_TO_ANGLE;
    update->absolution_angle.roll = *(update->vision_angle_point + INS_ROLL_ADDRESS_OFFSET) * RADIAN_TO_ANGLE;

}


static void vision_set_mode(vision_send_t* vision_mode)
{
    //设置自瞄模式为装甲板模式
    vision_mode->send_msg_struct.mode_change = ARMOURED_PLATE_MODE;
}

static void vision_tx_encode(vision_send_t* vision_tx_data_encode)
{
    //头
    memcpy((void*)(vision_tx_data_encode->send_message + (int)HEAD_ADDRESS_OFFSET), (void*)&vision_tx_data_encode->send_msg_struct.head, DATA_QUAT_REAL_ADDRESS_OFFSET - HEAD_ADDRESS_OFFSET);
    //数据段 四元数
    memcpy((void*)(vision_tx_data_encode->send_message + (int)DATA_QUAT_REAL_ADDRESS_OFFSET), (void*)vision_tx_data_encode->send_msg_struct.quat, MODE_SWITCH_ADDRESS_OFFSET - DATA_QUAT_REAL_ADDRESS_OFFSET);
    //数据段模式转换
    memcpy((void*)(vision_tx_data_encode->send_message + (int)MODE_SWITCH_ADDRESS_OFFSET), (void*)&vision_tx_data_encode->send_msg_struct.mode_change, CHECK_BIT_ADDRESS_OFFSET - MODE_SWITCH_ADDRESS_OFFSET);
    //校验位
    memcpy((void*)(vision_tx_data_encode->send_message + (int)CHECK_BIT_ADDRESS_OFFSET), (void*)&vision_tx_data_encode->send_msg_struct.check, END_ADDRESS_OFFSET - CHECK_BIT_ADDRESS_OFFSET);
    //尾
    memcpy((void*)(vision_tx_data_encode->send_message + (int)END_ADDRESS_OFFSET), (void*)&vision_tx_data_encode->send_msg_struct.end, SUM_SEND_MESSAGE - END_ADDRESS_OFFSET);
}

static void vision_analysis_date(vision_control_t *vision_set)
{

    // 上位机视觉版本
    static fp32 vision_gimbal_yaw = 0;   // yaw轴绝对角
    static fp32 vision_gimbal_pitch = 0; // pitch轴绝对角
    // 未接收到上位机的时间
    static int32_t unrx_time = MAX_UNRX_TIME;

    // 判断当前云台模式为自瞄模式
    if (judge_gimbal_mode_is_auto_mode())
    {
        // 是自瞄模式，设置角度为上位机设置角度

        // 判断是否接收到上位机数据
        if (vision_set->vision_rxfifo->rx_flag) // 识别到目标
        {
            // 接收到数据标志位为0
            not_rx_vision_data_flag = 0;

            unrx_time = 0;
            // 接收到上位机数据
            // 接收标志位 置零
            vision_set->vision_rxfifo->rx_flag = 0;

            // 获取上位机视觉数据
            vision_gimbal_pitch = vision_set->vision_rxfifo->pitch_fifo;
            vision_gimbal_yaw = vision_set->vision_rxfifo->yaw_fifo;

            // 判断发射
            vision_shoot_judge(vision_set, (vision_gimbal_yaw - vision_set->absolution_angle.yaw), (vision_gimbal_pitch - vision_set->absolution_angle.pitch));
        }
        else
        {
            unrx_time++;
        }

        // 判断上位机视觉停止发送指令
        if (unrx_time >= MAX_UNRX_TIME)
        {
            // 数据置零
            unrx_time = 0;
            // 停止发弹
            vision_set->shoot_vision_control.shoot_command = SHOOT_STOP_ATTACK;
            not_rx_vision_data_flag = 1;
        }
    }

    // 赋值控制值
    // 判断是否控制值被赋值
    if (vision_gimbal_pitch == 0 && vision_gimbal_yaw == 0)
    {

        // 未赋值依旧为当前值
        vision_set->gimbal_vision_control.gimbal_pitch = vision_set->absolution_angle.pitch;
        vision_set->gimbal_vision_control.gimbal_yaw = vision_set->absolution_angle.yaw;
    }
    else
    {
        // 已赋值，用设置值
        vision_set->gimbal_vision_control.gimbal_pitch = vision_gimbal_pitch;
        vision_set->gimbal_vision_control.gimbal_yaw = vision_gimbal_yaw;
    }
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
    else if (fabs(vision_begin_add_pitch_angle) > ALLOW_ATTACK_ERROR || fabs(vision_begin_add_yaw_angle) > ALLOW_ATTACK_ERROR)
    {
        

        if (stop_attack_count >= JUDGE_STOP_ATTACK_COUNT)
        {
            //达到停止击打的计数
            // 判断击打计数值归零
            attack_count = 0;
            //设置停止击打
            shoot_judge->shoot_vision_control.shoot_command = SHOOT_STOP_ATTACK;
        }
        else
        {
            // 未到达停止击打的次数
            // 计数值增加
            stop_attack_count ++;
        }

        
    }
}


void highcomputer_rx_decode(uint8_t* rx_Buf, uint32_t* rx_buf_Len)
{
    // 串口数据解码
    //头
    memcpy((void *)vision_receive.receive_msg_struct.head, (void *)(rx_Buf + (int)HEAD_ADDRESS_OFFSET), (DATA_QUAT_REAL_ADDRESS_OFFSET - HEAD_ADDRESS_OFFSET));
    // 数据段
    memcpy((void *)&vision_receive.receive_msg_struct.quat, (void *)(rx_Buf + (int)DATA_QUAT_REAL_ADDRESS_OFFSET), (MODE_SWITCH_ADDRESS_OFFSET - DATA_QUAT_REAL_ADDRESS_OFFSET));
    // 模式转化位
    memcpy((void *)&vision_receive.receive_msg_struct.mode_change, (void *)(rx_Buf + (int)MODE_SWITCH_ADDRESS_OFFSET), (CHECK_BIT_ADDRESS_OFFSET - MODE_SWITCH_ADDRESS_OFFSET));
    // 校验位
    memcpy((void *)&vision_receive.receive_msg_struct.check, (void *)(rx_Buf + (int)CHECK_BIT_ADDRESS_OFFSET), (END_ADDRESS_OFFSET - CHECK_BIT_ADDRESS_OFFSET));
    // 尾
    memcpy((void *)&vision_receive.receive_msg_struct.end, (void *)(rx_Buf + (int)END_ADDRESS_OFFSET), (SUM_SEND_MESSAGE - END_ADDRESS_OFFSET));
}

bool_t judge_not_rx_vision_data(void)
{
    return not_rx_vision_data_flag;
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
