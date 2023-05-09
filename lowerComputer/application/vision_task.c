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

//imu转枪口的平移矩阵
#define TRANSLATION_IMU_TO_GUNPOINT {0, 0, -fabs(IMU_TO_GUMPOINT_DISTANCE)}

//视觉任务初始化
static void vision_task_init(vision_control_t* init);
//视觉任务数据更新
static void vision_task_feedback_update(vision_control_t* update);
// 处理上位机数据,计算弹道的空间落点，并反解空间绝对角
static void vision_data_process(vision_control_t* vision_data);
//惯性系下，机器人中心坐标转装甲板中心坐标
static void robot_center_vector_to_armor_center_vetcor(fp32 robot_center_x, fp32 robot_center_y, fp32 robot_center_z, 
                                                       fp32 armor_to_robot_center_r, fp32 armor_to_robot_center_theta, 
                                                       vector_t* armor_center_vector);
// //惯性系下，装甲板坐标的空间原点由以imu为空间原点转化为以枪口为空间原点
// static void imu_origin_to_gunpoint_origin_in_earth_frame(vector_t* armor_centor_vector, const fp32 quat[4]);
// //相对坐标系下，装甲板坐标的空间原点由以imu为空间原点转化为以枪口为空间原点
// static void imu_origin_to_gunpoint_origin_in_relative_frame(vector_t* armor_centor_vector);
//惯性系云台瞄准向量计算
static void calc_gimbal_aim_target_vector(vector_t* armor_target_vector, vector_t* aim_vector, fp32 observe_vx, fp32 observe_vy, fp32 observe_vz, fp32 bullet_speed);
//设置yaw轴pitch轴增量
static void vision_analysis_date(vision_control_t* vision_set);
//配置发送数据包
static void set_vision_send_packet(vision_control_t* set_send_packet);

//获取接收数据包指针
static vision_receive_t* get_vision_receive_point(void);
/**
 * @brief 牛顿迭代法求解子弹飞行时间
 * 
 * @param t_0 初始迭代时间
 * @param precision 迭代精度
 * @param min_deltat 最小迭代差值
 * @param max_iterate_count 最大迭代次数 
 * @param target_x 目标的x轴水平距离
 * @param target_vx 目标x轴观测速度
 * @param bullet_speed 弹速
 * @return 迭代最终值，子弹飞行时间 
 */
static float newton_iterate_to_calc_bullet_flight_time(float t_0, float precision, float min_deltat, int max_iterate_count,
                                           float target_x, float target_vx, float bullet_speed);
/**
 * @brief 子弹函数
 * 
 * @param t 飞行时间
 * @param target_x 水平位移
 * @param target_vx 水平速度
 * @param bullet_speed 子弹弹速
 * @return float
 */
static float bullet_flight_function(float t, float target_x, float target_vx, float bullet_speed);

/**
 * @brief 子弹微分函数
 * 
 * @param t 飞行时间
 * @param target_vx 水平速度
 * @param bullet_speed 子弹弹速
 * @return float
 */
static float bullet_flight_diff_function(float t, float target_vx, float bullet_speed);
/**
 * @brief 弹道补偿 比例迭代器进行迭代
 * 
 * @param bullet_flight_time 子弹飞行时间
 * @param armor_position_x 装甲板空间x距离
 * @param armor_position_z 装甲板空间y轴距离
 * @param bullet_speed 弹速
 * @param precision 迭代精度
 * @param max_iterate_count 最大迭代次数
 * @return z轴补偿瞄准位置 
 */
static float calc_gimbal_aim_z_compensation(float bullet_flight_time, float armor_position_x, float armor_position_z, float bullet_speed, float precision, float max_iterate_count);


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
    // 获取四元数指针
    init->vision_quat_point = get_INS_quat_point();
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
    // 获取四元数
    memcpy(update->quat, update->vision_quat_point, sizeof(update->vision_quat_point));
}

static void vision_data_process(vision_control_t* vision_data)
{
    //获取当前临时数据
    receive_packet_t robot_temp;
    memcpy(&robot_temp, &vision_data->vision_receive_point->receive_packet, sizeof(robot_temp));
    
    // 惯性系下机器人中心的空间坐标转装甲板空间坐标
    robot_center_vector_to_armor_center_vetcor(robot_temp.x, robot_temp.y, robot_temp.z, robot_temp.r1, robot_temp.yaw, &vision_data->target_armor_vector);

    //计算弹道空间落点
    calc_gimbal_aim_target_vector(&vision_data->target_armor_vector, &vision_data->robot_gimbal_aim_vector, robot_temp.vx, robot_temp.vy, robot_temp.vz, BULLET_SPEED);


    //反解欧拉角
    vision_data->vision_absolution_angle.yaw = atan2(vision_data->robot_gimbal_aim_vector.y, vision_data->robot_gimbal_aim_vector.x);
    vision_data->vision_absolution_angle.pitch = atan2(vision_data->robot_gimbal_aim_vector.z, vision_data->robot_gimbal_aim_vector.x);   
}

static void robot_center_vector_to_armor_center_vetcor(fp32 robot_center_x, fp32 robot_center_y, fp32 robot_center_z, 
                                                       fp32 armor_to_robot_center_r, fp32 armor_to_robot_center_theta, 
                                                       vector_t* armor_center_vector)
{
    armor_center_vector->x = robot_center_x - armor_to_robot_center_r * cosf(armor_to_robot_center_theta);
    armor_center_vector->y = robot_center_y - armor_to_robot_center_r * sinf(armor_to_robot_center_theta);
    armor_center_vector->z = robot_center_z;
}

// static void imu_origin_to_gunpoint_origin_in_earth_frame(vector_t* armor_centor_vector, const fp32 quat[4])
// {
//     //装甲板空间坐标由惯性系转为以imu为原点的机体相对坐标系
//     earthFrame_to_relativeFrame(armor_centor_vector, quat);
//     //相对坐标系下空间原点从imu转到枪口
//     imu_origin_to_gunpoint_origin_in_relative_frame(armor_centor_vector);
//     //装甲板空间坐标由相对坐标转为以枪口为原点的惯性坐标系
//     relativeFrame_to_earthFrame(armor_centor_vector, quat);    
// }

// static void imu_origin_to_gunpoint_origin_in_relative_frame(vector_t* armor_centor_vector)
// {
//     //平移矩阵
//     fp32 translation_imu_to_gunpoint[3] = TRANSLATION_IMU_TO_GUNPOINT;

//     //矩阵平移
//     armor_centor_vector->x += translation_imu_to_gunpoint[0];
//     armor_centor_vector->y += translation_imu_to_gunpoint[1];
//     armor_centor_vector->z += translation_imu_to_gunpoint[2];
// }


static void calc_gimbal_aim_target_vector(vector_t* armor_target_vector, vector_t* aim_vector, fp32 observe_vx, fp32 observe_vy, fp32 observe_vz, fp32 bullet_speed)
{
    //估计子弹飞行时间
    fp32 bullet_flight_time = newton_iterate_to_calc_bullet_flight_time(T_0, PRECISION, MIN_DELTAT, MAX_ITERATE_COUNT, armor_target_vector->x, observe_vx, BULLET_SPEED);

    //计算瞄准位置
    aim_vector->x = armor_target_vector->x + observe_vx * bullet_flight_time;
    aim_vector->y = armor_target_vector->y + observe_vy * bullet_flight_time;
    aim_vector->z = armor_target_vector->z + observe_vz * bullet_flight_time;

    //计算弹道补偿
    aim_vector->z = calc_gimbal_aim_z_compensation(bullet_flight_time, armor_target_vector->x, armor_target_vector->z, bullet_speed, PRECISION, MAX_ITERATE_COUNT);

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

static void set_vision_send_packet(vision_control_t* set_send_packet)
{
    set_send_packet->send_packet.header = LOWER_TO_HIGH_HEAD;
    set_send_packet->send_packet.robot_color = 0;
    set_send_packet->send_packet.task_mode = 0;
    set_send_packet->send_packet.reserved = 0;
    set_send_packet->send_packet.pitch = set_send_packet->imu_absolution_angle.pitch;
    set_send_packet->send_packet.yaw = set_send_packet->imu_absolution_angle.yaw;
    set_send_packet->send_packet.aim_x = set_send_packet->robot_gimbal_aim_vector.x;
    set_send_packet->send_packet.aim_y = set_send_packet->robot_gimbal_aim_vector.y;
    set_send_packet->send_packet.aim_z = set_send_packet->robot_gimbal_aim_vector.z;
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
            //数据正确，将临时数据拷贝到接收数据包中
            memcpy(&vision_receive.receive_packet, &temp_packet, sizeof(receive_packet_t));
            //接收数据位置1
            vision_receive.rx_flag = 1;
        }
    }
}

static vision_receive_t* get_vision_receive_point(void)
{
    return &vision_receive;
}


/**
 * @brief 相对坐标系转惯性坐标系
 * 
 * @param vector 相对坐标系下的空间向量
 * @param q 四元数
 */
void relativeFrame_to_earthFrame(vector_t* vector, const float* q)
{
    //相对坐标临时变量
    vector_t relativeFrame_temp;
    memcpy(&relativeFrame_temp, vector, sizeof(vector));
    //相对坐标转绝对坐标
    vector->x = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * relativeFrame_temp.x +
                       (q[1] * q[2] - q[0] * q[3]) * relativeFrame_temp.y +
                       (q[1] * q[3] + q[0] * q[2]) * relativeFrame_temp.z);

    vector->y = 2.0f * ((q[1] * q[2] + q[0] * q[3]) * relativeFrame_temp.x +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * relativeFrame_temp.y +
                       (q[2] * q[3] - q[0] * q[1]) * relativeFrame_temp.z);

    vector->z = 2.0f * ((q[1] * q[3] - q[0] * q[2]) * relativeFrame_temp.x +
                       (q[2] * q[3] + q[0] * q[1]) * relativeFrame_temp.y +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * relativeFrame_temp.z);
}

/**
 * @brief 惯性坐标系转相对坐标系 
 *  
 * @param vector 惯性坐标系下的空间向量
 * @param q 四元数
 */
void earthFrame_to_relativeFrame(vector_t* vector, const float* q)
{
    //惯性坐标系下的临时变脸 
    vector_t earthFrame_temp;
    memcpy(&earthFrame_temp, vector, sizeof(vector));
    //惯性坐标系转相对坐标系
    vector->x = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * earthFrame_temp.x +
                       (q[1] * q[2] + q[0] * q[3]) * earthFrame_temp.y +
                       (q[1] * q[3] - q[0] * q[2]) * earthFrame_temp.z);

    vector->y = 2.0f * ((q[1] * q[2] - q[0] * q[3]) * earthFrame_temp.x +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * earthFrame_temp.y +
                       (q[2] * q[3] + q[0] * q[1]) * earthFrame_temp.z);

    vector->z = 2.0f * ((q[1] * q[3] + q[0] * q[2]) * earthFrame_temp.x +
                       (q[2] * q[3] - q[0] * q[1]) * earthFrame_temp.y +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * earthFrame_temp.z);
}
/**
 * @brief 牛顿迭代法求解子弹飞行时间
 * 
 * @param t_0 初始迭代时间
 * @param precision 迭代精度
 * @param min_deltat 最小迭代差值
 * @param max_iterate_count 最大迭代次数 
 * @param target_x 目标的x轴水平距离
 * @param target_vx 目标x轴观测速度
 * @param bullet_speed 弹速
 * @return 迭代最终值，子弹飞行时间 
 */
static float newton_iterate_to_calc_bullet_flight_time(float t_0, float precision, float min_deltat, int max_iterate_count,
                                                       float target_x, float target_vx, float bullet_speed)
{
    //当前解
    float t_n = t_0;
    // 迭代后的解
    float t_n1 = t_0;
    // 迭代后的函数值
    float f_n1 = 0;
    // 函数值
    float f_n = 0;
    // 微分函数值
    float diff_f_n = 0;
    // 前后两次迭代的差值
    float deltat = 0;
    // 迭代次数
    int iterate_count = 0;
    do
    {
        // 更新迭代数值
        t_n = t_n1;

        //更新函数值
        f_n = bullet_flight_function(t_n, target_x, target_vx, bullet_speed);
        diff_f_n = bullet_flight_diff_function(t_n, target_vx, bullet_speed);

        //判断微分值是否合法
        if (diff_f_n < 1e-6f)
        {
            if (fabs(f_n) > precision)
            {
                return NAN;
            }
            else
            {
                return f_n;
            }
        }
        //迭代
        t_n1 = t_n - (f_n / diff_f_n);

        f_n1 = bullet_flight_function(t_n1, target_x, target_vx, bullet_speed);

        deltat = fabs(t_n1 - t_n);

        // 判断是否超过最大迭代次数
        if (iterate_count++ > max_iterate_count)
        {
            // 判断迭代数据是否符合精度要求
            if (fabs(f_n1) < precision)

            {
                return t_n;
            }
            else
            {
                return NAN;
            }
        }

    } while (fabs(f_n1) > precision || deltat > min_deltat);

    return t_n1;
}

/**
 * @brief 子弹函数
 * 
 * @param t 飞行时间
 * @param target_x 水平位移
 * @param target_vx 水平速度
 * @param bullet_speed 子弹弹速
 * @return float
 */
static float bullet_flight_function(float t, float target_x, float target_vx, float bullet_speed)
{
    return ((1.0f / AIR_K1) * log(AIR_K1 * bullet_speed * t + 1.0f) - target_x - target_vx * t);
}
/**
 * @brief 子弹微分函数
 * 
 * @param t 飞行时间
 * @param target_vx 水平速度
 * @param bullet_speed 子弹弹速
 * @return float
 */
static float bullet_flight_diff_function(float t, float target_vx, float bullet_speed)
{
    return ((bullet_speed / (AIR_K1 * bullet_speed * t + 1.0f)) - target_vx);
}

/**
 * @brief 弹道补偿 比例迭代器进行迭代
 * 
 * @param bullet_flight_time 子弹飞行时间
 * @param armor_position_x 装甲板空间x距离
 * @param armor_position_z 装甲板空间y轴距离
 * @param bullet_speed 弹速
 * @param precision 迭代精度
 * @param max_iterate_count 最大迭代次数
 * @return z轴补偿瞄准位置 
 */
static float calc_gimbal_aim_z_compensation(float bullet_flight_time, float armor_position_x, float armor_position_z, float bullet_speed, float precision, float max_iterate_count)
{
    //计算落点高度
    float bullet_drop_z = armor_position_z;
    //瞄准高度
    float aim_z = armor_position_z;
    //仰角
    float pitch = 0;
    // 计算值与真实值之间的误差
    float calc_and_actual_error = 0;
    for (int i = 0; i < max_iterate_count; i++)
    {
        //计算仰角
        pitch = atan2(aim_z, armor_position_x);
        //计算子弹落点高度
        bullet_drop_z = bullet_speed * sin(pitch) * bullet_flight_time - 0.5f * G * pow(bullet_flight_time, 2);
        //计算误差
        calc_and_actual_error = armor_position_z - bullet_drop_z;
        //对瞄准高度进行补偿
        aim_z += calc_and_actual_error * ITERATE_SCALE_FACTOR;
        //判断误差是否符合精度要求
        if (fabs(calc_and_actual_error) < precision)
        {
            break;
        } 
    }  
    return aim_z;
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
