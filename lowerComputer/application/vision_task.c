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
// 判断是否识别到目标
static void vision_judge_appear_target(vision_control_t* judge_appear_target);
// 判断目标装甲板数据
static void vision_judge_target_armor_data(vision_control_t* judge_target_armor_id);
// 处理上位机数据,计算弹道的空间落点，并反解空间绝对角
static void vision_data_process(vision_control_t* vision_data);
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
static float calc_bullet_drop(solve_trajectory_t* solve_trajectory, float x, float bullet_speed, float pitch);
// 二维平面弹道模型，计算pitch轴的高度
static float calc_target_position_pitch_angle(solve_trajectory_t* solve_trajectory, fp32 x, fp32 z);

//获取接收数据包指针
static vision_receive_t* get_vision_receive_point(void);


//视觉任务结构体
vision_control_t vision_control = { 0 };
//视觉接收结构体
vision_receive_t vision_receive = { 0 };

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
        //判断是否识别到目标
        vision_judge_appear_target(&vision_control);
        //判断目标装甲板编号
        vision_judge_target_armor_data(&vision_control);
        // 处理上位机数据,计算弹道的空间落点，并反解空间绝对角,并设置控制命令
        vision_data_process(&vision_control);

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
    //初始化当前弹速
    init->current_bullet_speed = MIN_SET_BULLET_SPEED;
    //初始化视觉目标状态为未识别到目标
    init->vision_target_appear_state = TARGET_UNAPPEAR;
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
    // 修正当前弹速
    calc_current_bullet_speed(update);
    //获取目标数据
    if (update->vision_receive_point->receive_state == UNLOADED)
    {
        //拷贝数据
        memcpy(&update->target_data, &update->vision_receive_point->receive_packet, sizeof(target_data_t));
        //接收数值状态置为已读取
        update->vision_receive_point->receive_state = LOADED;
    }
}

static void vision_judge_appear_target(vision_control_t* judge_appear_target)
{
    //根据接收数据判断是否为识别到目标
    if (judge_appear_target->vision_receive_point->receive_packet.x == 0 &&
        judge_appear_target->vision_receive_point->receive_packet.y == 0 &&
        judge_appear_target->vision_receive_point->receive_packet.z == 0 &&
        judge_appear_target->vision_receive_point->receive_packet.yaw == 0 &&
        judge_appear_target->vision_receive_point->receive_packet.vx == 0 &&
        judge_appear_target->vision_receive_point->receive_packet.vy == 0 &&
        judge_appear_target->vision_receive_point->receive_packet.vz == 0 &&
        judge_appear_target->vision_receive_point->receive_packet.v_yaw == 0
        )
    {
        //未识别到目标
        judge_appear_target->vision_target_appear_state = TARGET_UNAPPEAR;
    }
    else
    {
        //更新当前时间
        judge_appear_target->vision_receive_point->current_time = TIME_MS_TO_S(HAL_GetTick());
        //判断当前时间是否距离上次接收的时间过长
        if (fabs(judge_appear_target->vision_receive_point->current_time - judge_appear_target->vision_receive_point->current_receive_time) > MAX_NOT_RECEIVE_DATA_TIME)
        {
            //判断为未识别目标
            judge_appear_target->vision_target_appear_state = TARGET_UNAPPEAR;
        }
        else
        {
            // 识别到目标
            judge_appear_target->vision_target_appear_state = TARGET_APPEAR;
        }
    }
}


static void vision_judge_target_armor_data(vision_control_t* judge_target_armor_id)
{
    //赋值装甲板id
    judge_target_armor_id->target_armor_id = (armor_id_e)judge_target_armor_id->target_data.id; 
}

static void vision_data_process(vision_control_t* vision_data)
{
    //判断是否识别到目标
    if (vision_data->vision_target_appear_state == TARGET_APPEAR)
    {
        //识别到目标
        // 赋值弹道计算的可变参数
        assign_solve_trajectory_param(&vision_data->solve_trajectory, vision_data->imu_absolution_angle.pitch, vision_data->imu_absolution_angle.yaw, vision_data->current_bullet_speed);
        // 选择最优装甲板
        select_optimal_target(&vision_data->solve_trajectory, &vision_data->target_data, &vision_data->target_position);
        // 计算机器人瞄准位置
        calc_robot_gimbal_aim_vector(&vision_data->robot_gimbal_aim_vector, &vision_data->target_position, vision_data->target_data.vx, vision_data->target_data.vy, vision_data->target_data.vz, vision_data->solve_trajectory.predict_time);
        // 计算机器人pitch轴与yaw轴角度
        vision_data->gimbal_vision_control.gimbal_pitch = calc_target_position_pitch_angle(&vision_data->solve_trajectory, sqrt(pow(vision_data->robot_gimbal_aim_vector.x, 2) + pow(vision_data->robot_gimbal_aim_vector.y, 2)) - vision_data->solve_trajectory.distance_static, vision_data->robot_gimbal_aim_vector.z + vision_data->solve_trajectory.z_static);
        vision_data->gimbal_vision_control.gimbal_yaw = atan2(vision_data->robot_gimbal_aim_vector.y, vision_data->robot_gimbal_aim_vector.x);
        //判断发射
        vision_shoot_judge(vision_data, vision_data->gimbal_vision_control.gimbal_yaw - vision_data->imu_absolution_angle.yaw, vision_data->gimbal_vision_control.gimbal_pitch - vision_data->imu_absolution_angle.pitch, sqrt(pow(vision_data->target_data.x, 2) + pow(vision_data->target_data.y, 2)));
        //赋值底盘控制命令 -- 我方机器人与目标的距离
        vision_data->chassis_vision_control.distance = sqrt(pow(vision_data->target_data.x, 2) + pow(vision_data->target_data.y, 2));
    }
    else
    {
        //未识别到目标 -- 控制值清零
        vision_data->gimbal_vision_control.gimbal_yaw = 0;
        vision_data->gimbal_vision_control.gimbal_pitch = 0;
        vision_data->chassis_vision_control.distance = 0;
        //设置停止发射
        vision_data->shoot_vision_control.shoot_command = SHOOT_STOP_ATTACK;
    }
}



/**
 * @brief 分析视觉原始增加数据，根据原始数据，判断是否要进行发射，判断yaw轴pitch的角度，如果在一定范围内，则计算值增加，增加到一定数值则判断发射，如果yaw轴pitch轴角度大于该范围，则计数归零
 * 
 * @param shoot_judge 视觉结构体
 * @param vision_begin_add_yaw_angle 上位机视觉yuw轴原始增加角度
 * @param vision_begin_add_pitch_angle 上位机视觉pitch轴原始增加角度
 * @param target_distance 目标距离
 */
void vision_shoot_judge(vision_control_t* shoot_judge, fp32 vision_begin_add_yaw_angle, fp32 vision_begin_add_pitch_angle, fp32 target_distance)
{ 
    //判断目标距离
    if (target_distance <= ALLOW_ATTACK_DISTANCE)
    {
        // 小于一定角度开始击打
        if (fabs(vision_begin_add_pitch_angle) <= ALLOW_ATTACK_ERROR && fabs(vision_begin_add_yaw_angle) <= ALLOW_ATTACK_ERROR)
        {
            shoot_judge->shoot_vision_control.shoot_command = SHOOT_ATTACK;
        }
        else
        {
            shoot_judge->shoot_vision_control.shoot_command = SHOOT_STOP_ATTACK;
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
    //判断子弹类型
    if (calc_cur_bullet_speed->shoot_data_point->bullet_type == BULLET_17)
    {
        if (calc_cur_bullet_speed->shoot_data_point->shooter_id == SHOOTER_17_1)
        {
            //判断弹速是否低于一定数值，低于这一数值，认为其不合法
            if (calc_cur_bullet_speed->shoot_data_point->bullet_speed >= MIN_SET_BULLET_SPEED)
            {
                  calc_cur_bullet_speed->current_bullet_speed = (int16_t)calc_cur_bullet_speed->shoot_data_point->bullet_speed;
            }
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
            // 数据正确，将临时数据拷贝到接收数据包中
            memcpy(&vision_receive.receive_packet, &temp_packet, sizeof(receive_packet_t));
            // 接收数据数据状态标志为未读取
            vision_receive.receive_state = UNLOADED;
            // 保存时间
            vision_receive.last_receive_time = vision_receive.current_receive_time;
            // 记录当前接收数据的时间
            vision_receive.current_receive_time = TIME_MS_TO_S(HAL_GetTick());
            //计算时间间隔
            vision_receive.interval_time = vision_receive.current_receive_time - vision_receive.last_receive_time;
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
 * @param distance_static 枪口前推距离 
 */
static void solve_trajectory_param_init(solve_trajectory_t* solve_trajectory, fp32 k1, fp32 init_flight_time, fp32 z_static, fp32 distance_static)\
{
    solve_trajectory->k1 = k1;
    solve_trajectory->flight_time = init_flight_time;
    solve_trajectory->z_static = z_static;
    solve_trajectory->distance_static = distance_static;
    solve_trajectory->all_target_position_point = NULL;
    solve_trajectory->current_bullet_speed = MIN_SET_BULLET_SPEED;
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
        //由于四块装甲板距离机器人中心距离不同，但是一般两两对称，所以进行计算装甲板位置时，第0 2块用当前半径，第1 3块用上一次半径
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
static float calc_bullet_drop(solve_trajectory_t* solve_trajectory, float x, float bullet_speed, float pitch)
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
static float calc_target_position_pitch_angle(solve_trajectory_t* solve_trajectory, fp32 x, fp32 z)
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

//获取当前视觉是否识别到目标
bool_t judge_vision_appear_target(void)
{
    return vision_control.vision_target_appear_state == TARGET_APPEAR;
}

// 获取上位机云台命令
const gimbal_vision_control_t *get_vision_gimbal_point(void)
{
    return &vision_control.gimbal_vision_control;
}

// 获取上位机发射命令
const shoot_vision_control_t *get_vision_shoot_point(void)
{
    return &vision_control.shoot_vision_control;
}

//获取底盘控制命令
const chassis_vision_control_t* get_vision_chassis_point(void)
{
    return &vision_control.chassis_vision_control;
}


queue_t* queue_create(int16_t capacity)
{
    //创建空间
    queue_t* queue = malloc(sizeof(queue_t));
    if (queue == NULL)
    {
        return NULL;
    }
    //数值清空
    memset(queue, 0, sizeof(queue_t));
    //赋值容量
    queue->capacity = capacity;
    //开辟空间
    queue->date = malloc(queue->capacity * sizeof(fp32));
    //赋值当前存储值
    queue->cur_size = 0; 
    return queue;
}


void queue_append_data(queue_t* queue, fp32 append_data)
{
    
    if (queue->date != NULL && queue->capacity != 0)
    {
        //判断是否已满
        if (queue->cur_size < queue->capacity)
        {
            //未满 -- 添加数据

            //数据后移
            for (int i = (queue->cur_size - 1); i >= 0; i--)
            {
                queue->date[i + 1] = queue->date[i];   
            }

            //添加最新数据
            queue->date[0] = append_data;

            //存储量加1
            queue->cur_size += 1;
        }
        else
        {
            // 已满 -- 清空最后一位数据

            //数据后移动
            for (int i = queue->cur_size - 2; i >= 0; i--)
            {
                queue->date[i + 1] = queue->date[i];
            }

            //添加最新数据
            queue->date[0] = append_data;

            //存储量不变
        }
    }
    else
    {
        return;
    }
}



fp32 queue_data_calc_average(queue_t* queue)
{
    if (queue == NULL)
    {
        //存在问题，返回-1
        return -1;
    }
    fp32 sum = 0;
    if (queue->cur_size > 0)
    {
        for (int i = 0; i < queue->cur_size - 1; i++)
        {
            sum += queue->date[i];
        }
    }
    else
    {
        //无数据返回0
        return 0;
    }

    return sum / queue->cur_size;
}


void queue_delete(queue_t* queue)
{
    if (queue == NULL)
    {
        return;
    }
    //释放存储空间
    free(queue->date);
    queue->date = NULL;
    //释放自身空间
    free(queue);
    queue = NULL;
}

