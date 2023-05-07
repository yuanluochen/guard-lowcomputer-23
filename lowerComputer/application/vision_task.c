/**
 * @file vision_task.c
 * @author yuanluochen
 * @brief 
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
#include "CRC8_CRC16.h"
#include "usbd_cdc_if.h"
#include "arm_math.h"



//视觉发送任务结构体初始化
static void vision_send_task_init(vision_send_t* init);
//视觉发送任务数据更新
static void vision_send_task_feedback_update(vision_send_t* update);
//配置发送数据包
static void set_vision_send_packet(vision_send_t* set_send_packet);
//发送数据包
static void send_packet(vision_send_t* send);

//视觉任务初始化
static void vision_task_init(vision_control_t* init);
//视觉任务数据更新
static void vision_task_feedback_update(vision_control_t* update);
//处理视觉数据包
static void vision_data_process(vision_control_t* vision_data);
//设置yaw轴pitch轴增量
static void vision_analysis_date(vision_control_t* vision_set);

//惯性系下，机器人中心坐标转装甲板坐标
// static void robot_center_vector_to_armor_vector()

//获取接收数据包指针
static vision_receive_t* get_vision_receive_point(void);

//视觉任务结构体
vision_control_t vision_control = { 0 };
//视觉发送结构体
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
        // 配置发送数据包
        set_vision_send_packet(&vision_send);
        //发送数据包
        send_packet(&vision_send);
        //系统延时
        vTaskDelay(VISION_SEND_CONTROL_TIME_MS);
    }
}

static void vision_send_task_init(vision_send_t* init)
{
    //获取陀螺仪绝对角指针
    init->INS_angle_point = get_INS_angle_point();
    // 数据更新
    vision_send_task_feedback_update(init);
}

static void vision_send_task_feedback_update(vision_send_t* update)
{
    //更新自身欧拉角
    update->eular_angle.yaw = *(update->INS_angle_point + INS_YAW_ADDRESS_OFFSET); 
    update->eular_angle.pitch = *(update->INS_angle_point + INS_PITCH_ADDRESS_OFFSET); 
    update->eular_angle.roll = *(update->INS_angle_point + INS_ROLL_ADDRESS_OFFSET); 
    //更新瞄准位置
    update->aim_position.x = vision_control.target_robot_vector.x;
    update->aim_position.y = vision_control.target_robot_vector.y;
    update->aim_position.z = vision_control.target_robot_vector.z;
    
}

static void set_vision_send_packet(vision_send_t* set_send_packet)
{
    set_send_packet->send_packet.header = LOWER_TO_HIGH_HEAD;
    set_send_packet->send_packet.robot_color = 0;
    set_send_packet->send_packet.task_mode = 0;
    set_send_packet->send_packet.reserved = 0;
    set_send_packet->send_packet.pitch = set_send_packet->eular_angle.pitch;
    set_send_packet->send_packet.yaw = set_send_packet->eular_angle.yaw;
    set_send_packet->send_packet.aim_x = set_send_packet->aim_position.x;
    set_send_packet->send_packet.aim_y = set_send_packet->aim_position.y;
    set_send_packet->send_packet.aim_z = set_send_packet->aim_position.z;
}

static void send_packet(vision_send_t* send)
{
    if (send == NULL)
    {
        return;
    }
    //添加CRC16到结尾
    append_CRC16_check_sum((uint8_t*)&send->send_packet, sizeof(send->send_packet));
    //发送数据
    CDC_Transmit_FS((uint8_t*)&send->send_packet, sizeof(send->send_packet));
}


void receive_decode(uint8_t* buf, uint32_t len)
{
    if (buf == NULL || len < 2)
    {
        return;
    }
    //CRC校验
    if (verify_CRC16_check_sum(buf, len))
    {
        receive_packet_t temp_packet = {0};
        // 拷贝接收到的数据到临时内存中
        memcpy(&temp_packet, buf, sizeof(temp_packet));
        if (temp_packet.header == HIGH_TO_LOWER_HEAD)
        {
            //数据正确，将临时数据拷贝到接收数据包中
            memcpy(&vision_receive.receive_packet, &temp_packet, sizeof(receive_packet_t));
            //接收数据位置1
            vision_receive.rx_flag = 1;
        }
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
        if (shoot_control_vision_task() /*&& gimbal_control_vision_task()*/)
        {
            // 更新数据
            vision_task_feedback_update(&vision_control);
            // 处理上位机数据
            vision_data_process(&vision_control);
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



static void vision_task_init(vision_control_t* init)
{
    // 获取陀螺仪绝对角指针                                                                                                                                                                                                                                                                                                                                                           init->vision_angle_point = get_INS_angle_point();
    init->vision_angle_point = get_INS_angle_point();
    // 获取接收数据包指针
    init->vision_receive_point = get_vision_receive_point();
    //初始化发射模式为停止袭击
    init->shoot_vision_control.shoot_command = SHOOT_STOP_ATTACK;

    //更新数据
    vision_task_feedback_update(init);
 
}

static void vision_task_feedback_update(vision_control_t* update)
{
    // 获取云台位姿数据
    update->imu_absolution_angle.yaw = *(update->vision_angle_point + INS_YAW_ADDRESS_OFFSET);
    update->imu_absolution_angle.pitch = *(update->vision_angle_point + INS_PITCH_ADDRESS_OFFSET);
    update->imu_absolution_angle.roll = *(update->vision_angle_point + INS_ROLL_ADDRESS_OFFSET);
    
    //获取目标地球坐标系下的空间坐标点
    update->target_robot_vector.x = update->vision_receive_point->receive_packet.x;
    update->target_robot_vector.y = update->vision_receive_point->receive_packet.y;
    update->target_robot_vector.z = update->vision_receive_point->receive_packet.z;
}

static void vision_data_process(vision_control_t* vision_data)
{
    //获取当前临时数据
    receive_packet_t robot_temp;
    memcpy(&robot_temp, &vision_data->vision_receive_point->receive_packet, sizeof(robot_temp));
    //惯性系下机器人中心的空间坐标转装甲板空间坐标
    vision_data->target_armor_vector.x = robot_temp.x - robot_temp.r1 * cosf(robot_temp.yaw);
    vision_data->target_armor_vector.y = robot_temp.y - robot_temp.r1 * sinf(robot_temp.yaw);
    vision_data->target_armor_vector.z = robot_temp.z;
    //反解欧拉角
    vision_data->vision_absolution_angle.yaw = atan2(vision_data->target_armor_vector.y, vision_data->target_armor_vector.x);
    // vision_data->vision_absolution_angle.yaw = 0;
    // vision_data->vision_absolution_angle.pitch = atan2(vision_data->target_armor_vector.z, sqrt(vision_data->target_armor_vector.x * vision_data->target_armor_vector.x + vision_data->target_armor_vector.y * vision_data->target_armor_vector.y));   
    vision_data->vision_absolution_angle.pitch = atan2(vision_data->target_armor_vector.z, vision_data->target_armor_vector.x);   
    // vision_data->vision_absolution_angle.pitch = atan2(vision_data->target_armor_vector.z, vision_data->target_armor_vector.x);   
    // vision_data->vision_absolution_angle.pitch = 0;        
}

static void vision_analysis_date(vision_control_t *vision_set)
{
    static fp32 vision_gimbal_yaw = 0;   // yaw轴绝对角
    static fp32 vision_gimbal_pitch = 0; // pitch轴绝对角
    // 未接收到上位机的时间
    static int32_t unrx_time = MAX_UNRX_TIME;

    // 判断当前云台模式为自瞄模式
    if (judge_gimbal_mode_is_auto_mode())
    {
        // 自瞄模式，设置角度为上位机设置角度

        // 判断是否接收到上位机数据
        if (vision_set->vision_receive_point->rx_flag) // 识别到目标
        {
            // 接收到数据标志位为0
            not_rx_vision_data_flag = 0;

            unrx_time = 0;
            // 接收到上位机数据
            // 接收标志位 置零
            vision_set->vision_receive_point->rx_flag = 0;

            // 获取上位机视觉数据
            vision_gimbal_pitch = vision_set->vision_absolution_angle.pitch;
            vision_gimbal_yaw = vision_set->vision_absolution_angle.yaw;

            // 判断发射
            vision_shoot_judge(vision_set, (vision_gimbal_yaw - vision_set->imu_absolution_angle.yaw), (vision_gimbal_pitch - vision_set->imu_absolution_angle.pitch));
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
        vision_set->gimbal_vision_control.gimbal_pitch = vision_set->imu_absolution_angle.pitch;
        vision_set->gimbal_vision_control.gimbal_yaw = vision_set->imu_absolution_angle.yaw;
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
void vision_shoot_judge(vision_control_t* shoot_judge, fp32 vision_begin_add_yaw_angle, fp32 vision_begin_add_pitch_angle)
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


static vision_receive_t* get_vision_receive_point(void)
{
    return &vision_receive;
}

/**
 * @brief 相对坐标系转世界坐标系 
 * 
 * @param vecRF 相对坐标系下的空间向量
 * @param vecEF 世界坐标系下的空间向量
 * @param q 四元数
 */
void relativeFrame_to_earthFrame(const float *vecRF, float *vecEF, float *q)
{
    vecEF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecRF[0] +
                       (q[1] * q[2] - q[0] * q[3]) * vecRF[1] +
                       (q[1] * q[3] + q[0] * q[2]) * vecRF[2]);

    vecEF[1] = 2.0f * ((q[1] * q[2] + q[0] * q[3]) * vecRF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecRF[1] +
                       (q[2] * q[3] - q[0] * q[1]) * vecRF[2]);

    vecEF[2] = 2.0f * ((q[1] * q[3] - q[0] * q[2]) * vecRF[0] +
                       (q[2] * q[3] + q[0] * q[1]) * vecRF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecRF[2]);
}

/**
 * @brief 世界坐标系转相对坐标系 
 *  
 * @param vecEF 世界坐标系下的空间向量
 * @param vecRF 相对坐标系下的空间向量
 * @param q 四元数
 */
void earthFrame_to_relativeFrame(const float *vecEF, float *vecRF, float *q)
{
    vecRF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecEF[0] +
                       (q[1] * q[2] + q[0] * q[3]) * vecEF[1] +
                       (q[1] * q[3] - q[0] * q[2]) * vecEF[2]);

    vecRF[1] = 2.0f * ((q[1] * q[2] - q[0] * q[3]) * vecEF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecEF[1] +
                       (q[2] * q[3] + q[0] * q[1]) * vecEF[2]);

    vecRF[2] = 2.0f * ((q[1] * q[3] + q[0] * q[2]) * vecEF[0] +
                       (q[2] * q[3] - q[0] * q[1]) * vecEF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecEF[2]);
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
