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

//类型转化
typedef receive_packet_t target_data_t;

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
static void judge_enemy_robot_armor_color(vision_control_t* judge_detect_color);
//实时计算弹速
static void calc_current_bullet_speed(vision_control_t* calc_cur_bullet_speed);

//初始化弹道解算的参数
static void solve_trajectory_param_init(solve_trajectory_t* solve_trajectory, fp32 k1, fp32 init_flight_time, fp32 z_static, fp32 distance_static);
//赋值弹道解算的一些可变参数
static void assign_solve_trajectory_param(solve_trajectory_t* solve_trajectory, fp32 current_pitch, fp32 current_yaw, fp32 current_bullet_speed);
//选择最优击打目标
static void select_optimal_target(solve_trajectory_t* solve_trajectory, target_data_t* vision_data, target_position_t* optimal_target_position);
//赋值云台瞄准位置
static void calc_robot_gimbal_aim_vector(vector_t* robot_gimbal_aim_vector, target_position_t* target_position, fp32 vx, fp32 vy, fp32 vz, fp32 predict_time);
// 计算子弹落点
float calc_bullet_drop(solve_trajectory_t* solve_trajectory, float x, float bullet_speed, float pitch);
// 二维平面弹道模型，计算pitch轴的高度
float calc_target_position_pitch_angle(solve_trajectory_t* solve_trajectory, fp32 x, fp32 z);

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

    while(1)
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
    // 获取发射机构弹速指针
    init->shoot_data_point = get_shoot_data_point();
    //初始化发射模式为停止袭击
    init->shoot_vision_control.shoot_command = SHOOT_STOP_ATTACK;
    //初始化一些基本的弹道参数
    solve_trajectory_param_init(&init->solve_trajectory, AIR_K1, INIT_FILIGHT_TIME, Z_STATIC, DISTANCE_STATIC);
    //更新数据
    vision_task_feedback_update(init);
}

static void vision_task_feedback_update(vision_control_t* update)
{
    // 获取云台位姿数据
    update->imu_absolution_angle.yaw = *(update->vision_angle_point + INS_YAW_ADDRESS_OFFSET);
    update->imu_absolution_angle.pitch = *(update->vision_angle_point + INS_PITCH_ADDRESS_OFFSET);
    update->imu_absolution_angle.roll = *(update->vision_angle_point + INS_ROLL_ADDRESS_OFFSET);
    // 判断敌方装甲板颜色
    judge_enemy_robot_armor_color(update);
    // 赋值当前弹速
    calc_current_bullet_speed(update);
}

static void vision_data_process(vision_control_t* vision_data)
{
    //机器人数据
    target_data_t robot_data = {0};
    memcpy(&robot_data, &vision_data->vision_receive_point->receive_packet, sizeof(target_data_t));
    // //初始化计算结构体
    // GimbalControlInit(vision_data->imu_absolution_angle.pitch, vision_data->imu_absolution_angle.yaw, robot_data.yaw, robot_data.v_yaw, robot_data.r1, robot_data.r2, robot_data.dz, 0, BULLET_SPEED, 0.076);
    // //计算数据
    // GimbalControlTransform(robot_data.x, robot_data.y, robot_data.z, robot_data.vx, robot_data.vy, robot_data.vz, TIME_BIAS, &vision_data->vision_absolution_angle.pitch, &vision_data->vision_absolution_angle.yaw, &vision_data->robot_gimbal_aim_vector.x, &vision_data->robot_gimbal_aim_vector.y, &vision_data->robot_gimbal_aim_vector.z);

    //赋值弹道计算的可变参数
    assign_solve_trajectory_param(&vision_data->solve_trajectory, vision_data->imu_absolution_angle.pitch, vision_data->imu_absolution_angle.yaw, vision_data->current_bullet_speed);
    //选择最优装甲板
    select_optimal_target(&vision_data->solve_trajectory, &robot_data, &vision_data->target_position);
    //计算机器人瞄准位置
    calc_robot_gimbal_aim_vector(&vision_data->robot_gimbal_aim_vector, &vision_data->target_position, robot_data.vx, robot_data.vy, robot_data.vz, vision_data->solve_trajectory.predict_time);
    //计算机器人pitch轴与yaw轴角度
    vision_data->vision_absolution_angle.pitch = calc_target_position_pitch_angle(&vision_data->solve_trajectory, sqrt(pow(vision_data->robot_gimbal_aim_vector.x, 2) + pow(vision_data->robot_gimbal_aim_vector.y, 2)) + vision_data->solve_trajectory.distance_static, vision_data->robot_gimbal_aim_vector.z - vision_data->solve_trajectory.z_static);
    vision_data->vision_absolution_angle.yaw = atan2(vision_data->robot_gimbal_aim_vector.y, vision_data->robot_gimbal_aim_vector.x);

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
    set_send_packet->send_packet.detect_color = set_send_packet->detect_armor_color;
    set_send_packet->send_packet.roll = set_send_packet->imu_absolution_angle.roll;
    set_send_packet->send_packet.pitch = set_send_packet->imu_absolution_angle.pitch;
    set_send_packet->send_packet.yaw = set_send_packet->imu_absolution_angle.yaw;
    set_send_packet->send_packet.aim_x = set_send_packet->robot_gimbal_aim_vector.x;
    set_send_packet->send_packet.aim_y = set_send_packet->robot_gimbal_aim_vector.y;
    set_send_packet->send_packet.aim_z = set_send_packet->robot_gimbal_aim_vector.z;
}

static void judge_enemy_robot_armor_color(vision_control_t* judge_detect_color)
{
    //判断机器人id是否大于ROBOT_RED_AND_BULE_DIVIDE_VALUE这个值
    if (judge_detect_color->robot_state_point->robot_id > ROBOT_RED_AND_BLUE_DIVIDE_VALUE)
    {
        //自己为蓝色，返回1，输出敌方为红色
        judge_detect_color->detect_armor_color = RED;
    }
    else
    {
        judge_detect_color->detect_armor_color = BLUE;
    }
}

/**
 * @brief 由于受天气温度影响，弹速可能会在直流信号上产生较大的变化，所以实时计算弹速 
 * 
 * @param calc_cur_bullet_speed 视觉控制结构体
 */
static void calc_current_bullet_speed(vision_control_t* calc_cur_bullet_speed)
{
    if (calc_cur_bullet_speed->shoot_data_point->bullet_type == BULLET_TYPE)
    {
        if (calc_cur_bullet_speed->shoot_data_point->shooter_id == SHOOT_ID)
        {
            calc_cur_bullet_speed->current_bullet_speed = calc_cur_bullet_speed->shoot_data_point->bullet_speed;
        }
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
        memcpy(&temp_packet, buf, sizeof(receive_packet_t));
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

/**
 * @brief 初始化弹道计算的参数
 * 
 * @param solve_trajectory 弹道计算结构体
 * @param k1 弹道参数
 * @param init_flight_time 初始飞行时间估计值
 * @param z_static yaw轴电机到枪口水平面的垂直距离
 * @param distance_static /枪口前推距离 
 */
static void solve_trajectory_param_init(solve_trajectory_t* solve_trajectory, fp32 k1, fp32 init_flight_time, fp32 z_static, fp32 distance_static)\
{
    solve_trajectory->k1 = k1;
    solve_trajectory->flight_time = init_flight_time;
    solve_trajectory->z_static = z_static;
    solve_trajectory->distance_static = distance_static;
    solve_trajectory->all_target_position_point = NULL;
    solve_trajectory->current_bullet_speed = BULLET_SPEED;
}

/**
 * @brief 赋值弹道解算的一些可变参数
 *
 * @param solve_trajectory 弹道计算结构体
 * @param current_pitch 当前云台的pitch
 * @param current_yaw 当前云台的yaw
 * @param current_bullet_speed 当前弹速
 */
static void assign_solve_trajectory_param(solve_trajectory_t* solve_trajectory, fp32 current_pitch, fp32 current_yaw, fp32 current_bullet_speed)
{
    solve_trajectory->current_yaw = current_yaw;
    solve_trajectory->current_pitch = current_pitch;
    solve_trajectory->current_bullet_speed = current_bullet_speed;
}

/**
 * @brief 选择最优击打目标
 * 
 * @param solve_trajectory 弹道计算结构体 
 * @param vision_data 接收视觉数据
 * @param optimal_target_position 最优目标位置 
 */
static void select_optimal_target(solve_trajectory_t* solve_trajectory, target_data_t* vision_data, target_position_t* optimal_target_position)
{
    //计算预测时间 = 上一次的子弹飞行时间 + 固有偏移时间, 时间可能不正确，但可以接受
    solve_trajectory->predict_time = solve_trajectory->flight_time + TIME_MS_TO_S(TIME_BIAS);
    //计算子弹到达目标时的yaw角度
    solve_trajectory->target_yaw = vision_data->yaw + vision_data->v_yaw * solve_trajectory->predict_time;

    //赋值装甲板数量
    solve_trajectory->armor_num = vision_data->armors_num;
    
    //开辟装甲板数量的位置变量的空间
    if (solve_trajectory->all_target_position_point == NULL)
    {
       solve_trajectory->all_target_position_point = malloc(solve_trajectory->armor_num * sizeof(target_position_t));
    }

    //选择目标的数组编号
    uint8_t select_targrt_num = 0;
    
    //计算所有装甲板的位置
    for (int i = 0; i < solve_trajectory->armor_num; i++)
    {
        //由于四块装甲板距离机器人中心距离不同，但是一般两辆对称，所以进行计算装甲板位置时，第0 2块用当前半径，第1 3块用上一次半径
        fp32 r = (i % 2 == 0) ? vision_data->r1 : vision_data->r2;
        solve_trajectory->all_target_position_point[i].yaw = solve_trajectory->target_yaw + i * (ALL_CIRCLE / solve_trajectory->armor_num);
        solve_trajectory->all_target_position_point[i].x = vision_data->x - r * cos(solve_trajectory->all_target_position_point[i].yaw);
        solve_trajectory->all_target_position_point[i].y = vision_data->y - r * sin(solve_trajectory->all_target_position_point[i].yaw);
        solve_trajectory->all_target_position_point[i].z = (i % 2 == 0) ? vision_data->z : vision_data->z + vision_data->dz;
    }

    // 选择与机器人自身yaw差值最小的目标,冒泡排序选择最小目标
    fp32 yaw_error_min = fabsf(solve_trajectory->current_yaw - solve_trajectory->all_target_position_point[0].yaw);

    for (int i = 0; i < solve_trajectory->armor_num; i++)
    {
        fp32 yaw_error_temp = fabsf(solve_trajectory->current_yaw - solve_trajectory->all_target_position_point[i].yaw);
        if (yaw_error_temp < yaw_error_min)
        {
            yaw_error_min = yaw_error_temp;
            select_targrt_num = i;
        }
    }

    //将选择的装甲板数据，拷贝打最优目标中去
    memcpy(optimal_target_position, &solve_trajectory->all_target_position_point[select_targrt_num], sizeof(target_position_t));
    //释放开辟的内存
    free(solve_trajectory->all_target_position_point);
    //指针置空
    solve_trajectory->all_target_position_point = NULL;
}

/**
 * @brief 计算装甲板瞄准位置
 * 
 * @param robot_gimbal_aim_vector 机器人云台瞄准向量
 * @param target_position 目标位置
 * @param vx 机器人中心速度
 * @param vy 机器人中心速度
 * @param vz 机器人中心速度
 * @param predict_time 预测时间
 */
static void calc_robot_gimbal_aim_vector(vector_t* robot_gimbal_aim_vector, target_position_t* target_position, fp32 vx, fp32 vy, fp32 vz, fp32 predict_time)
{
    //由于目标与观测中心处于同一系，速度相同
    robot_gimbal_aim_vector->x = target_position->x + vx * predict_time;
    robot_gimbal_aim_vector->y = target_position->y + vy * predict_time;
    robot_gimbal_aim_vector->z = target_position->z + vz * predict_time;
}

/**
 * @brief 计算子弹落点
 * 
 * @param solve_trajectory 弹道计算结构体
 * @param x 水平距离
 * @param bullet_speed 弹速
 * @param pitch 仰角
 * @return 子弹落点
 */
float calc_bullet_drop(solve_trajectory_t* solve_trajectory, float x, float bullet_speed, float pitch)
{
    solve_trajectory->flight_time = (float)((exp(solve_trajectory->k1 * x) - 1) / (solve_trajectory->k1 * bullet_speed * cos(pitch)));
    //计算子弹落点高度
    fp32 bullet_drop_z = (float)(bullet_speed * sin(pitch) * solve_trajectory->flight_time - 0.5f * GRAVITY * pow(solve_trajectory->flight_time, 2));
    return bullet_drop_z;
}

/**
 * @brief 二维平面弹道模型，计算pitch轴的高度
 * 
 * @param solve_tragectory 弹道计算结构体
 * @param x 水平距离
 * @param z 竖直距离
 * @param bullet_speed 弹速
 * @return 返回pitch轴数值
 */
float calc_target_position_pitch_angle(solve_trajectory_t* solve_trajectory, fp32 x, fp32 z)
{
    // 计算落点高度
    float bullet_drop_z = 0;
    // 瞄准高度
    float aim_z = z;

    // 仰角
    float pitch = 0;
    // 计算值与真实值之间的误差
    float calc_and_actual_error = 0;
    // 比例迭代法
    for (int i = 0; i < MAX_ITERATE_COUNT; i++)
    {
        // 计算仰角
        pitch = atan2(aim_z, x);
        // 计算子弹落点高度
        bullet_drop_z = calc_bullet_drop(solve_trajectory, x, solve_trajectory->current_bullet_speed, pitch);
        // 计算误差
        calc_and_actual_error = z - bullet_drop_z;
        // 对瞄准高度进行补偿
        aim_z += calc_and_actual_error * ITERATE_SCALE_FACTOR;
        // 判断误差是否符合精度要求
        if (fabs(calc_and_actual_error) < PRECISION)
        {
            break;
        }
    }
    //由于为右手系，pitch为向下为正，所以置负
    return -pitch;
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
