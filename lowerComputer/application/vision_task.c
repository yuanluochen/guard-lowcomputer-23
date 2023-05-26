/**
 * @file vision_task.c
 * @author yuanluochen
 * @brief �����Ӿ����ݰ��������Ӿ��۲����ݣ�Ԥ��װ�װ�λ�ã��Լ����㵯���켣�����е�������
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

//����ת��
typedef receive_packet_t target_data_t;

//�Ӿ������ʼ��
static void vision_task_init(vision_control_t* init);
//�Ӿ��������ݸ���
static void vision_task_feedback_update(vision_control_t* update);
// ������λ������,���㵯���Ŀռ���㣬������ռ���Խ�
static void vision_data_process(vision_control_t* vision_data);
//����yaw��pitch������
static void vision_analysis_date(vision_control_t* vision_set);
//���÷������ݰ�
static void set_vision_send_packet(vision_control_t* set_send_packet);
//�жϵз�������װ�װ���ɫ������0 ��з�Ϊ��ɫ������1 ��з�Ϊ��ɫ
static void judge_enemy_robot_armor_color(vision_control_t* judge_detect_color);
//ʵʱ���㵯��
static void calc_current_bullet_speed(vision_control_t* calc_cur_bullet_speed);

//��ʼ����������Ĳ���
static void solve_trajectory_param_init(solve_trajectory_t* solve_trajectory, fp32 k1, fp32 init_flight_time, fp32 z_static, fp32 distance_static);
//��ֵ���������һЩ�ɱ����
static void assign_solve_trajectory_param(solve_trajectory_t* solve_trajectory, fp32 current_pitch, fp32 current_yaw, fp32 current_bullet_speed);
//ѡ�����Ż���Ŀ��
static void select_optimal_target(solve_trajectory_t* solve_trajectory, target_data_t* vision_data, target_position_t* optimal_target_position);
//��ֵ��̨��׼λ��
static void calc_robot_gimbal_aim_vector(vector_t* robot_gimbal_aim_vector, target_position_t* target_position, fp32 vx, fp32 vy, fp32 vz, fp32 predict_time);
// �����ӵ����
float calc_bullet_drop(solve_trajectory_t* solve_trajectory, float x, float bullet_speed, float pitch);
// ��άƽ�浯��ģ�ͣ�����pitch��ĸ߶�
float calc_target_position_pitch_angle(solve_trajectory_t* solve_trajectory, fp32 x, fp32 z);

//��ȡ�������ݰ�ָ��
static vision_receive_t* get_vision_receive_point(void);

//�Ӿ�����ṹ��
vision_control_t vision_control = { 0 };
//�Ӿ����սṹ��
vision_receive_t vision_receive = { 0 };

//δ���յ��Ӿ����ݱ�־λ����λΪ1 ��δ����
bool_t not_rx_vision_data_flag = 1;

void vision_task(void const* pvParameters)
{
    // ��ʱ�ȴ����ȴ���λ���������ݳɹ�
    vTaskDelay(VISION_TASK_INIT_TIME);
    // �Ӿ������ʼ��
    vision_task_init(&vision_control);
    // �ȴ���̨�����ʼ�����
    while(shoot_control_vision_task() && gimbal_control_vision_task())
    {
        //ϵͳ��ʱ
        vTaskDelay(VISION_CONTROL_TIME_MS);
    }

    while(1)
    {
        // ��������
        vision_task_feedback_update(&vision_control);
        // ������λ������,���㵯���Ŀռ���㣬������ռ���Խ�
        vision_data_process(&vision_control);
        // ������λ������,����yaw��pitch������,�Լ��ж��Ƿ���
        vision_analysis_date(&vision_control);

        // ���÷������ݰ�
        set_vision_send_packet(&vision_control);
        // �������ݰ�
        send_packet(&vision_control);

        // ϵͳ��ʱ
        vTaskDelay(VISION_CONTROL_TIME_MS);
    }
}

static void vision_task_init(vision_control_t* init)
{
    // ��ȡ�����Ǿ��Խ�ָ��                                                                                                                                                                                                                                                                                                                                                           init->vision_angle_point = get_INS_angle_point();
    init->vision_angle_point = get_INS_angle_point();
    // ��ȡ�������ݰ�ָ��
    init->vision_receive_point = get_vision_receive_point();
    // ��ȡ������״ָ̬��
    init->robot_state_point = get_game_robot_status_point();
    // ��ȡ�����������ָ��
    init->shoot_data_point = get_shoot_data_point();
    //��ʼ������ģʽΪֹͣϮ��
    init->shoot_vision_control.shoot_command = SHOOT_STOP_ATTACK;
    //��ʼ��һЩ�����ĵ�������
    solve_trajectory_param_init(&init->solve_trajectory, AIR_K1, INIT_FILIGHT_TIME, Z_STATIC, DISTANCE_STATIC);
    //��������
    vision_task_feedback_update(init);
}

static void vision_task_feedback_update(vision_control_t* update)
{
    // ��ȡ��̨λ������
    update->imu_absolution_angle.yaw = *(update->vision_angle_point + INS_YAW_ADDRESS_OFFSET);
    update->imu_absolution_angle.pitch = *(update->vision_angle_point + INS_PITCH_ADDRESS_OFFSET);
    update->imu_absolution_angle.roll = *(update->vision_angle_point + INS_ROLL_ADDRESS_OFFSET);
    // �жϵз�װ�װ���ɫ
    judge_enemy_robot_armor_color(update);
    // ��ֵ��ǰ����
    calc_current_bullet_speed(update);
}

static void vision_data_process(vision_control_t* vision_data)
{
    //����������
    target_data_t robot_data = {0};
    memcpy(&robot_data, &vision_data->vision_receive_point->receive_packet, sizeof(target_data_t));
    // //��ʼ������ṹ��
    // GimbalControlInit(vision_data->imu_absolution_angle.pitch, vision_data->imu_absolution_angle.yaw, robot_data.yaw, robot_data.v_yaw, robot_data.r1, robot_data.r2, robot_data.dz, 0, BULLET_SPEED, 0.076);
    // //��������
    // GimbalControlTransform(robot_data.x, robot_data.y, robot_data.z, robot_data.vx, robot_data.vy, robot_data.vz, TIME_BIAS, &vision_data->vision_absolution_angle.pitch, &vision_data->vision_absolution_angle.yaw, &vision_data->robot_gimbal_aim_vector.x, &vision_data->robot_gimbal_aim_vector.y, &vision_data->robot_gimbal_aim_vector.z);

    //��ֵ��������Ŀɱ����
    assign_solve_trajectory_param(&vision_data->solve_trajectory, vision_data->imu_absolution_angle.pitch, vision_data->imu_absolution_angle.yaw, vision_data->current_bullet_speed);
    //ѡ������װ�װ�
    select_optimal_target(&vision_data->solve_trajectory, &robot_data, &vision_data->target_position);
    //�����������׼λ��
    calc_robot_gimbal_aim_vector(&vision_data->robot_gimbal_aim_vector, &vision_data->target_position, robot_data.vx, robot_data.vy, robot_data.vz, vision_data->solve_trajectory.predict_time);
    //���������pitch����yaw��Ƕ�
    vision_data->vision_absolution_angle.pitch = calc_target_position_pitch_angle(&vision_data->solve_trajectory, sqrt(pow(vision_data->robot_gimbal_aim_vector.x, 2) + pow(vision_data->robot_gimbal_aim_vector.y, 2)) + vision_data->solve_trajectory.distance_static, vision_data->robot_gimbal_aim_vector.z - vision_data->solve_trajectory.z_static);
    vision_data->vision_absolution_angle.yaw = atan2(vision_data->robot_gimbal_aim_vector.y, vision_data->robot_gimbal_aim_vector.x);

}


static void vision_analysis_date(vision_control_t *vision_set)
{
    static fp32 vision_gimbal_yaw = 0;   // yaw����Խ�
    static fp32 vision_gimbal_pitch = 0; // pitch����Խ�
    // δ���յ���λ����ʱ��
    static int32_t unrx_time = MAX_UNRX_TIME;

    // �жϵ�ǰ��̨ģʽΪ����ģʽ
    if (judge_gimbal_mode_is_auto_mode())
    {
        // ����ģʽ�����ýǶ�Ϊ��λ�����ýǶ�

        // �жϽ����ڴ����Ƿ����δ��ȡ������
        if (vision_set->vision_receive_point->receive_state == UNLOADED) //����δ��ȡ������
        {
            // ���յ����ݱ�־λΪ0
            not_rx_vision_data_flag = 0;

            unrx_time = 0;
            // ��������Ѿ���ȡ
            vision_set->vision_receive_point->receive_state = LOADED;

            // ��ȡ��λ���Ӿ�����
            vision_gimbal_pitch = vision_set->vision_absolution_angle.pitch;
            vision_gimbal_yaw = vision_set->vision_absolution_angle.yaw;

            // �жϷ���
            vision_shoot_judge(vision_set, (vision_gimbal_yaw - vision_set->imu_absolution_angle.yaw), (vision_gimbal_pitch - vision_set->imu_absolution_angle.pitch));
        }
        else
        {
            unrx_time++;
        }

        // �ж���λ���Ӿ�ֹͣ����ָ��
        if (unrx_time >= MAX_UNRX_TIME)
        {
            // ��������
            unrx_time = 0;
            // ֹͣ����
            vision_set->shoot_vision_control.shoot_command = SHOOT_STOP_ATTACK;
            not_rx_vision_data_flag = 1;
        }
    }
    else
    {
        //������ģʽ������ģʽΪֹͣ����
        vision_set->shoot_vision_control.shoot_command = SHOOT_STOP_ATTACK;
    }

    // ��ֵ����ֵ
    // �ж��Ƿ����ֵ����ֵ
    if (vision_gimbal_pitch == 0 && vision_gimbal_yaw == 0)
    {

        // δ��ֵ����Ϊ��ǰֵ
        vision_set->gimbal_vision_control.gimbal_pitch = vision_set->imu_absolution_angle.pitch;
        vision_set->gimbal_vision_control.gimbal_yaw = vision_set->imu_absolution_angle.yaw;
    }
    else
    {
        // �Ѹ�ֵ��������ֵ
        vision_set->gimbal_vision_control.gimbal_pitch = vision_gimbal_pitch;
        vision_set->gimbal_vision_control.gimbal_yaw = vision_gimbal_yaw;
    }
}



/**
 * @brief �����Ӿ�ԭʼ�������ݣ�����ԭʼ���ݣ��ж��Ƿ�Ҫ���з��䣬�ж�yaw��pitch�ĽǶȣ������һ����Χ�ڣ������ֵ���ӣ����ӵ�һ����ֵ���жϷ��䣬���yaw��pitch��Ƕȴ��ڸ÷�Χ�����������
 * 
 * @param shoot_judge �Ӿ��ṹ��
 * @param vision_begin_add_yaw_angle ��λ���Ӿ�yuw��ԭʼ���ӽǶ�
 * @param vision_begin_add_pitch_angle ��λ���Ӿ�pitch��ԭʼ���ӽǶ�
 */
void vision_shoot_judge(vision_control_t* shoot_judge, fp32 vision_begin_add_yaw_angle, fp32 vision_begin_add_pitch_angle)
{
    // �жϻ������
    static int attack_count = 0;
    // �ж�ֹͣ����Ĵ��� 
    static int stop_attack_count = 0;

    
    // ��λ�����ͽǶȵ�һ����λ�ڼ���ֵ����
    if (fabs(vision_begin_add_pitch_angle) <= ALLOW_ATTACK_ERROR && fabs(vision_begin_add_yaw_angle) <= ALLOW_ATTACK_ERROR)
    {
        // ֹͣ�������ֵ����
        stop_attack_count = 0;

        // �жϼ���ֵ�Ƿ񵽴��жϻ���ļ���ֵ
        if (attack_count >= JUDGE_ATTACK_COUNT)
        {
            // ����ɻ���Ĵ���
            // ���û���
            shoot_judge->shoot_vision_control.shoot_command = SHOOT_ATTACK;
        }
        else
        {
            // δ����ɻ���Ĵ���
            // ����ֵ����
            attack_count++;
        }
    }
    // ��λ������Ƕȴ��ڸ÷�Χ����ֵ����
    else if (fabs(vision_begin_add_pitch_angle) > ALLOW_ATTACK_ERROR || fabs(vision_begin_add_yaw_angle) > ALLOW_ATTACK_ERROR)
    {
        

        if (stop_attack_count >= JUDGE_STOP_ATTACK_COUNT)
        {
            //�ﵽֹͣ����ļ���
            // �жϻ������ֵ����
            attack_count = 0;
            //����ֹͣ����
            shoot_judge->shoot_vision_control.shoot_command = SHOOT_STOP_ATTACK;
        }
        else
        {
            // δ����ֹͣ����Ĵ���
            // ����ֵ����
            stop_attack_count ++;
        }
    }
}

static void set_vision_send_packet(vision_control_t* set_send_packet)
{
    //�жϵз���������ɫ

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
    //�жϻ�����id�Ƿ����ROBOT_RED_AND_BULE_DIVIDE_VALUE���ֵ
    if (judge_detect_color->robot_state_point->robot_id > ROBOT_RED_AND_BLUE_DIVIDE_VALUE)
    {
        //�Լ�Ϊ��ɫ������1������з�Ϊ��ɫ
        judge_detect_color->detect_armor_color = RED;
    }
    else
    {
        judge_detect_color->detect_armor_color = BLUE;
    }
}

/**
 * @brief �����������¶�Ӱ�죬���ٿ��ܻ���ֱ���ź��ϲ����ϴ�ı仯������ʵʱ���㵯�� 
 * 
 * @param calc_cur_bullet_speed �Ӿ����ƽṹ��
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
    //���CRC16����β
    append_CRC16_check_sum((uint8_t*)&send->send_packet, sizeof(send->send_packet));
    //��������
    CDC_Transmit_FS((uint8_t*)&send->send_packet, sizeof(send->send_packet));
}

void receive_decode(uint8_t* buf, uint32_t len)
{
    if (buf == NULL || len < 2)
    {
        return;
    }
    //CRCУ��
    if (verify_CRC16_check_sum(buf, len))
    {
        receive_packet_t temp_packet = {0};
        // �������յ������ݵ���ʱ�ڴ���
        memcpy(&temp_packet, buf, sizeof(receive_packet_t));
        if (temp_packet.header == HIGH_TO_LOWER_HEAD)
        {
            if (!(temp_packet.vx == 0 && temp_packet.vy == 0 && temp_packet.vz == 0))
            {
                // ������ȷ������ʱ���ݿ������������ݰ���
                memcpy(&vision_receive.receive_packet, &temp_packet, sizeof(receive_packet_t));
                // ������������״̬��־Ϊδ��ȡ
                vision_receive.receive_state = UNLOADED;
            }
       }
    }
}

/**
 * @brief ��ʼ����������Ĳ���
 * 
 * @param solve_trajectory ��������ṹ��
 * @param k1 ��������
 * @param init_flight_time ��ʼ����ʱ�����ֵ
 * @param z_static yaw������ǹ��ˮƽ��Ĵ�ֱ����
 * @param distance_static /ǹ��ǰ�ƾ��� 
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
 * @brief ��ֵ���������һЩ�ɱ����
 *
 * @param solve_trajectory ��������ṹ��
 * @param current_pitch ��ǰ��̨��pitch
 * @param current_yaw ��ǰ��̨��yaw
 * @param current_bullet_speed ��ǰ����
 */
static void assign_solve_trajectory_param(solve_trajectory_t* solve_trajectory, fp32 current_pitch, fp32 current_yaw, fp32 current_bullet_speed)
{
    solve_trajectory->current_yaw = current_yaw;
    solve_trajectory->current_pitch = current_pitch;
    solve_trajectory->current_bullet_speed = current_bullet_speed;
}

/**
 * @brief ѡ�����Ż���Ŀ��
 * 
 * @param solve_trajectory ��������ṹ�� 
 * @param vision_data �����Ӿ�����
 * @param optimal_target_position ����Ŀ��λ�� 
 */
static void select_optimal_target(solve_trajectory_t* solve_trajectory, target_data_t* vision_data, target_position_t* optimal_target_position)
{
    //����Ԥ��ʱ�� = ��һ�ε��ӵ�����ʱ�� + ����ƫ��ʱ��, ʱ����ܲ���ȷ�������Խ���
    solve_trajectory->predict_time = solve_trajectory->flight_time + TIME_MS_TO_S(TIME_BIAS);
    //�����ӵ�����Ŀ��ʱ��yaw�Ƕ�
    solve_trajectory->target_yaw = vision_data->yaw + vision_data->v_yaw * solve_trajectory->predict_time;

    //��ֵװ�װ�����
    solve_trajectory->armor_num = vision_data->armors_num;
    
    //����װ�װ�������λ�ñ����Ŀռ�
    if (solve_trajectory->all_target_position_point == NULL)
    {
       solve_trajectory->all_target_position_point = malloc(solve_trajectory->armor_num * sizeof(target_position_t));
    }

    //ѡ��Ŀ���������
    uint8_t select_targrt_num = 0;
    
    //��������װ�װ��λ��
    for (int i = 0; i < solve_trajectory->armor_num; i++)
    {
        //�����Ŀ�װ�װ������������ľ��벻ͬ������һ�������Գƣ����Խ��м���װ�װ�λ��ʱ����0 2���õ�ǰ�뾶����1 3������һ�ΰ뾶
        fp32 r = (i % 2 == 0) ? vision_data->r1 : vision_data->r2;
        solve_trajectory->all_target_position_point[i].yaw = solve_trajectory->target_yaw + i * (ALL_CIRCLE / solve_trajectory->armor_num);
        solve_trajectory->all_target_position_point[i].x = vision_data->x - r * cos(solve_trajectory->all_target_position_point[i].yaw);
        solve_trajectory->all_target_position_point[i].y = vision_data->y - r * sin(solve_trajectory->all_target_position_point[i].yaw);
        solve_trajectory->all_target_position_point[i].z = (i % 2 == 0) ? vision_data->z : vision_data->z + vision_data->dz;
    }

    // ѡ�������������yaw��ֵ��С��Ŀ��,ð������ѡ����СĿ��
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

    //��ѡ���װ�װ����ݣ�����������Ŀ����ȥ
    memcpy(optimal_target_position, &solve_trajectory->all_target_position_point[select_targrt_num], sizeof(target_position_t));
    //�ͷſ��ٵ��ڴ�
    free(solve_trajectory->all_target_position_point);
    //ָ���ÿ�
    solve_trajectory->all_target_position_point = NULL;
}

/**
 * @brief ����װ�װ���׼λ��
 * 
 * @param robot_gimbal_aim_vector ��������̨��׼����
 * @param target_position Ŀ��λ��
 * @param vx �����������ٶ�
 * @param vy �����������ٶ�
 * @param vz �����������ٶ�
 * @param predict_time Ԥ��ʱ��
 */
static void calc_robot_gimbal_aim_vector(vector_t* robot_gimbal_aim_vector, target_position_t* target_position, fp32 vx, fp32 vy, fp32 vz, fp32 predict_time)
{
    //����Ŀ����۲����Ĵ���ͬһϵ���ٶ���ͬ
    robot_gimbal_aim_vector->x = target_position->x + vx * predict_time;
    robot_gimbal_aim_vector->y = target_position->y + vy * predict_time;
    robot_gimbal_aim_vector->z = target_position->z + vz * predict_time;
}

/**
 * @brief �����ӵ����
 * 
 * @param solve_trajectory ��������ṹ��
 * @param x ˮƽ����
 * @param bullet_speed ����
 * @param pitch ����
 * @return �ӵ����
 */
float calc_bullet_drop(solve_trajectory_t* solve_trajectory, float x, float bullet_speed, float pitch)
{
    solve_trajectory->flight_time = (float)((exp(solve_trajectory->k1 * x) - 1) / (solve_trajectory->k1 * bullet_speed * cos(pitch)));
    //�����ӵ����߶�
    fp32 bullet_drop_z = (float)(bullet_speed * sin(pitch) * solve_trajectory->flight_time - 0.5f * GRAVITY * pow(solve_trajectory->flight_time, 2));
    return bullet_drop_z;
}

/**
 * @brief ��άƽ�浯��ģ�ͣ�����pitch��ĸ߶�
 * 
 * @param solve_tragectory ��������ṹ��
 * @param x ˮƽ����
 * @param z ��ֱ����
 * @param bullet_speed ����
 * @return ����pitch����ֵ
 */
float calc_target_position_pitch_angle(solve_trajectory_t* solve_trajectory, fp32 x, fp32 z)
{
    // �������߶�
    float bullet_drop_z = 0;
    // ��׼�߶�
    float aim_z = z;

    // ����
    float pitch = 0;
    // ����ֵ����ʵֵ֮������
    float calc_and_actual_error = 0;
    // ����������
    for (int i = 0; i < MAX_ITERATE_COUNT; i++)
    {
        // ��������
        pitch = atan2(aim_z, x);
        // �����ӵ����߶�
        bullet_drop_z = calc_bullet_drop(solve_trajectory, x, solve_trajectory->current_bullet_speed, pitch);
        // �������
        calc_and_actual_error = z - bullet_drop_z;
        // ����׼�߶Ƚ��в���
        aim_z += calc_and_actual_error * ITERATE_SCALE_FACTOR;
        // �ж�����Ƿ���Ͼ���Ҫ��
        if (fabs(calc_and_actual_error) < PRECISION)
        {
            break;
        }
    }
    //����Ϊ����ϵ��pitchΪ����Ϊ���������ø�
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

// ��ȡ��λ����̨����
gimbal_vision_control_t *get_vision_gimbal_point(void)
{
    return &vision_control.gimbal_vision_control;
}

// ��ȡ��λ����������
shoot_vision_control_t *get_vision_shoot_point(void)
{
    return &vision_control.shoot_vision_control;
}
