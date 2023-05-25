/**
 * @file vision_task.c
 * @author yuanluochen
 * @brief 解析视觉数据包，处理视觉观测数据，预测装甲板位置，以及计算弹道轨迹，进行弹道补偿
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

//视觉任务初始化
static void vision_task_init(vision_control_t* init);
//视觉任务数据更新
static void vision_task_feedback_update(vision_control_t* update);
// 处理上位机数据,计算弹道的空间落点，并反解空间绝对角
static void vision_data_process(vision_control_t* vision_data);
//设置yaw轴pitch轴增量
static void vision_analysis_date(vision_control_t* vision_set);
//配置发送数据包
static void set_vision_send_packet(vision_control_t* set_send_packet);
//判断敌方机器人装甲板颜色，返回0 则敌方为红色，返回1 则敌方为蓝色
static uint8_t judge_enemy_robot_armor_color(ext_game_robot_state_t* person_robor_state);

//获取接收数据包指针
static vision_receive_t* get_vision_receive_point(void);


//视觉任务结构体
vision_control_t vision_control = { 0 };
//视觉接收结构体
vision_receive_t vision_receive = { 0 };

//未接收到视觉数据标志位，该位为1 则未接收
bool_t not_rx_vision_data_flag = 1;

void vision_task(void const* pvParameters)
{
    // 延时等待，等待上位机发送数据成功
    vTaskDelay(VISION_TASK_INIT_TIME);
    // 视觉任务初始化
    vision_task_init(&vision_control);
    // 等待云台射击初始化完成
    while(shoot_control_vision_task() && gimbal_control_vision_task())
    {
        //系统延时
        vTaskDelay(VISION_CONTROL_TIME_MS);
    }

    while (1)
    {
        // 更新数据
        vision_task_feedback_update(&vision_control);
        // 处理上位机数据,计算弹道的空间落点，并反解空间绝对角
        vision_data_process(&vision_control);
        // 解析上位机数据,配置yaw轴pitch轴增量,以及判断是否发射
        vision_analysis_date(&vision_control);

        // 配置发送数据包
        set_vision_send_packet(&vision_control);
        // 发送数据包
        send_packet(&vision_control);

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
    // 获取机器人状态指针
    init->robot_state_point = get_game_robot_status_point();
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
}

static void vision_data_process(vision_control_t* vision_data)
{
    //机器人数据
    receive_packet_t robot_data = {0};
    memcpy(&robot_data, &vision_data->vision_receive_point->receive_packet, sizeof(robot_data));
    //初始化计算结构体
    GimbalControlInit(vision_data->imu_absolution_angle.pitch, vision_data->imu_absolution_angle.yaw, robot_data.yaw, robot_data.v_yaw, robot_data.r1, robot_data.r2, robot_data.dz, 0, BULLET_SPEED, 0.076);
    //计算数据
    GimbalControlTransform(robot_data.x, robot_data.y, robot_data.z, robot_data.vx, robot_data.vy, robot_data.vz, TIME_BIAS, &vision_data->vision_absolution_angle.pitch, &vision_data->vision_absolution_angle.yaw, &vision_data->robot_gimbal_aim_vector.x, &vision_data->robot_gimbal_aim_vector.y, &vision_data->robot_gimbal_aim_vector.z);
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

        // 判断接收内存中是否存在未读取的数据
        if (vision_set->vision_receive_point->receive_state == UNLOADED) //存在未读取的数据
        {
            // 接收到数据标志位为0
            not_rx_vision_data_flag = 0;

            unrx_time = 0;
            // 标记数据已经读取
            vision_set->vision_receive_point->receive_state = LOADED;

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
    else
    {
        //非自瞄模式，发弹模式为停止发弹
        vision_set->shoot_vision_control.shoot_command = SHOOT_STOP_ATTACK;
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

static void set_vision_send_packet(vision_control_t* set_send_packet)
{
    //判断敌方机器人颜色

    set_send_packet->send_packet.header = LOWER_TO_HIGH_HEAD;
    set_send_packet->send_packet.detect_color = judge_enemy_robot_armor_color(set_send_packet->robot_state_point);
    set_send_packet->send_packet.roll = set_send_packet->imu_absolution_angle.roll;
    set_send_packet->send_packet.pitch = set_send_packet->imu_absolution_angle.pitch;
    set_send_packet->send_packet.yaw = set_send_packet->imu_absolution_angle.yaw;
    set_send_packet->send_packet.aim_x = set_send_packet->robot_gimbal_aim_vector.x;
    set_send_packet->send_packet.aim_y = set_send_packet->robot_gimbal_aim_vector.y;
    set_send_packet->send_packet.aim_z = set_send_packet->robot_gimbal_aim_vector.z;
}

static uint8_t judge_enemy_robot_armor_color(ext_game_robot_state_t* person_robor_state)
{
    //判断机器人id是否大于ROBOT_RED_AND_BULE_DIVIDE_VALUE这个值
    if (person_robor_state->robot_id > ROBOT_RED_AND_BLUE_DIVIDE_VALUE)
    {
        //自己为蓝色，返回1，输出敌方为红色
        return RED;
    }
    else
    {
        return BLUE;
    }
}

void send_packet(vision_control_t* send)
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
            if (!(temp_packet.vx == 0 && temp_packet.vy == 0 && temp_packet.vz == 0))
            {
                // 数据正确，将临时数据拷贝到接收数据包中
                memcpy(&vision_receive.receive_packet, &temp_packet, sizeof(receive_packet_t));
                // 接收数据数据状态标志为未读取
                vision_receive.receive_state = UNLOADED;
            }
       }
    }
}

static vision_receive_t* get_vision_receive_point(void)
{
    return &vision_receive;
}



bool_t judge_not_rx_vision_data(void)
{
    return not_rx_vision_data_flag;
}

// 获取上位机云台命令
gimbal_vision_control_t *get_vision_gimbal_point(void)
{
    return &vision_control.gimbal_vision_control;
}

// 获取上位机发射命令
shoot_vision_control_t *get_vision_shoot_point(void)
{
    return &vision_control.shoot_vision_control;
}
