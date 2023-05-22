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

//�Ӿ������ʼ��
static void vision_task_init(vision_control_t* init);
//�Ӿ��������ݸ���
static void vision_task_feedback_update(vision_control_t* update);
// ������λ������,���㵯���Ŀռ���㣬������ռ���Խ�
static void vision_data_process(vision_control_t* vision_data);
//����ϵ��̨��׼��������
static void calc_gimbal_aim_target_vector(vision_control_t* calc_aim_vector);
//����yaw��pitch������
static void vision_analysis_date(vision_control_t* vision_set);
//���÷������ݰ�
static void set_vision_send_packet(vision_control_t* set_send_packet);
//�жϵз�������װ�װ���ɫ������0 ��з�Ϊ��ɫ������1 ��з�Ϊ��ɫ
static uint8_t judge_enemy_robot_armor_color(ext_game_robot_state_t* person_robor_state);

//��ȡ�������ݰ�ָ��
static vision_receive_t* get_vision_receive_point(void);

//����������x�����תΪװ�װ�����x�����
static float robot_center_x_to_armor_center_x(fp32 xc_k, fp32 vx, fp32 yaw_k, fp32 vyaw, fp32 r, fp32 t);

//����������y�����תΪװ�װ�����y�����
static float robot_center_y_to_armor_center_y(fp32 yc_k, fp32 vy, fp32 yaw_k, fp32 vyaw, fp32 r, fp32 t);

//����������z�����תΪװ�װ�����z�����
static float robot_center_z_to_armor_center_z(fp32 zc_k, fp32 vz, fp32 t);
//��������
static float bullet_flight_function(fp32 t, fp32 xc_k, fp32 yc_k, fp32 yaw_k,
                           fp32 vx, fp32 vy, fp32 vyaw,
                           fp32 r, fp32 bullet_speed, fp32 k1);


//����΢�ַ���
static float bullet_flight_diff_function(fp32 t, fp32 xc_k, fp32 yc_k, fp32 yaw_k,
                           fp32 vx, fp32 vy, fp32 vyaw,
                           fp32 r, fp32 bullet_speed, fp32 k1);

//ţ�ٵ���������ӵ�����ʱ��
static float newton_iterate_to_calc_bullet_flight_time(float t_0, float precision, float min_deltat, int max_iterate_count,
                                                       receive_packet_t* robot_data, float bullet_speed);

/**
 * @brief �������� �������������е���
 *
 * @param bullet_flight_time �ӵ�����ʱ��
 * @param armor_position_distance װ�װ�ռ�x����
 * @param armor_position_vertical װ�װ�ռ�y�����
 * @param bullet_speed ����
 * @param precision ��������
 * @param max_iterate_count ����������
 * @return z�Ჹ����׼λ��
 */
static float calc_gimbal_aim_z_compensation(float bullet_flight_time, float armor_position_distance, float armor_position_vertical, float bullet_speed, float precision, float max_iterate_count);


//�Ӿ�����ṹ��
vision_control_t vision_control = { 0 };
//�Ӿ����սṹ��
vision_receive_t vision_receive = { 0 };

//δ���յ��Ӿ����ݱ�־λ����λΪ1 ��δ����
bool_t not_rx_vision_data_flag = 1;

fp32 k1 = AIR_K1;
uint8_t res = 4;
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

    while (1)
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
    // ��ȡ��Ԫ��ָ��
    init->vision_quat_point = get_INS_quat_point();
    // ��ȡ�������ݰ�ָ��
    init->vision_receive_point = get_vision_receive_point();
    // ��ȡ������״ָ̬��
    init->robot_state_point = get_game_robot_status_point();
    //��ʼ������ģʽΪֹͣϮ��
    init->shoot_vision_control.shoot_command = SHOOT_STOP_ATTACK;
    //��������
    vision_task_feedback_update(init);
}

static void vision_task_feedback_update(vision_control_t* update)
{
    // ��ȡ��̨λ������
    update->imu_absolution_angle.yaw = *(update->vision_angle_point + INS_YAW_ADDRESS_OFFSET);
    update->imu_absolution_angle.pitch = *(update->vision_angle_point + INS_PITCH_ADDRESS_OFFSET);
    update->imu_absolution_angle.roll = *(update->vision_angle_point + INS_ROLL_ADDRESS_OFFSET);
    // ��ȡ��Ԫ��
    memcpy(update->quat, update->vision_quat_point, sizeof(update->vision_quat_point));
}

static void vision_data_process(vision_control_t* vision_data)
{
    // �����������ָ̨��Ŀռ�����
    calc_gimbal_aim_target_vector(vision_data);

    //����ŷ���ǣ���������ת��ʽ yaw -> pitch -> roll
    vision_data->vision_absolution_angle.yaw = atan2(vision_data->robot_gimbal_aim_vector.y, vision_data->robot_gimbal_aim_vector.x);
    vision_data->vision_absolution_angle.pitch = atan2(vision_data->robot_gimbal_aim_vector.z, sqrt(pow(vision_data->robot_gimbal_aim_vector.x, 2) + pow(vision_data->robot_gimbal_aim_vector.y, 2)));   
}



static void calc_gimbal_aim_target_vector(vision_control_t* calc_aim_vector)
{
    //��ȡ��ǰ�Ӿ�����
    receive_packet_t robot_data = { 0 };
    memcpy(&robot_data, &calc_aim_vector->vision_receive_point->receive_packet, sizeof(robot_data));
    //�����ӵ�����ʱ��
    fp32 flight_time = newton_iterate_to_calc_bullet_flight_time(T_0, PRECISION, MIN_DELTAT, MAX_ITERATE_COUNT, &robot_data, BULLET_SPEED);
    //����װ�װ�λ��
    calc_aim_vector->target_armor_vector.x = robot_center_x_to_armor_center_x(robot_data.x, robot_data.vx, robot_data.yaw, robot_data.v_yaw, robot_data.r1, flight_time);
    calc_aim_vector->target_armor_vector.y = robot_center_x_to_armor_center_x(robot_data.y, robot_data.vy, robot_data.yaw, robot_data.v_yaw, robot_data.r1, flight_time);
    calc_aim_vector->target_armor_vector.z = robot_center_z_to_armor_center_z(robot_data.z, robot_data.vz, flight_time);
    //�߶Ȳ���
    calc_aim_vector->robot_gimbal_aim_vector.z = calc_gimbal_aim_z_compensation(flight_time, sqrt(pow(calc_aim_vector->target_armor_vector.x, 2) + pow(calc_aim_vector->target_armor_vector.y, 2)), calc_aim_vector->target_armor_vector.z, BULLET_SPEED, PRECISION, MAX_ITERATE_COUNT);
    //��ֵ��ֵ
    calc_aim_vector->robot_gimbal_aim_vector.x = calc_aim_vector->target_armor_vector.x;
    calc_aim_vector->robot_gimbal_aim_vector.y = calc_aim_vector->target_armor_vector.y;

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
    set_send_packet->send_packet.detect_color = judge_enemy_robot_armor_color(set_send_packet->robot_state_point);
    set_send_packet->send_packet.reserved = res;
    set_send_packet->send_packet.pitch = set_send_packet->imu_absolution_angle.pitch;
    set_send_packet->send_packet.yaw = set_send_packet->imu_absolution_angle.yaw;
    set_send_packet->send_packet.aim_x = set_send_packet->robot_gimbal_aim_vector.x;
    set_send_packet->send_packet.aim_y = set_send_packet->robot_gimbal_aim_vector.y;
    set_send_packet->send_packet.aim_z = set_send_packet->robot_gimbal_aim_vector.z;
}

static uint8_t judge_enemy_robot_armor_color(ext_game_robot_state_t* person_robor_state)
{
    //�жϻ�����id�Ƿ����ROBOT_RED_AND_BULE_DIVIDE_VALUE���ֵ
    if (person_robor_state->robot_id > ROBOT_RED_AND_BLUE_DIVIDE_VALUE)
    {
        //�Լ�Ϊ��ɫ������1������з�Ϊ��ɫ
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
        memcpy(&temp_packet, buf, sizeof(temp_packet));
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

static vision_receive_t* get_vision_receive_point(void)
{
    return &vision_receive;
}


/**
 * @brief �������ϵת��������ϵ
 * 
 * @param vector �������ϵ�µĿռ�����
 * @param q ��Ԫ��
 */
void relativeFrame_to_earthFrame(vector_t* vector, const float* q)
{
    //���������ʱ����
    vector_t relativeFrame_temp;
    memcpy(&relativeFrame_temp, vector, sizeof(vector));
    //�������ת��������
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
 * @brief ��������ϵת�������ϵ 
 *  
 * @param vector ��������ϵ�µĿռ�����
 * @param q ��Ԫ��
 */
void earthFrame_to_relativeFrame(vector_t* vector, const float* q)
{
    //��������ϵ�µ���ʱ���� 
    vector_t earthFrame_temp;
    memcpy(&earthFrame_temp, vector, sizeof(vector));
    //��������ϵת�������ϵ
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

//ţ�ٵ���������ӵ�����ʱ��
static float newton_iterate_to_calc_bullet_flight_time(float t_0, float precision, float min_deltat, int max_iterate_count,
                                                       receive_packet_t* robot_data, float bullet_speed)
{
    //��ǰ��
    float t_n = t_0;
    // ������Ľ�
    float t_n1 = t_0;
    // ������ĺ���ֵ
    float f_n1 = 0;
    // ����ֵ
    float f_n = 0;
    // ΢�ֺ���ֵ
    float diff_f_n = 0;
    // ǰ�����ε����Ĳ�ֵ
    float deltat = 0;
    // ��������
    int iterate_count = 0;
    do
    {
        // ���µ�����ֵ
        t_n = t_n1;

        //���º���ֵ
        f_n = bullet_flight_function(t_n, robot_data->x, robot_data->y, robot_data->yaw, robot_data->vx, robot_data->vy, robot_data->v_yaw, robot_data->r1, bullet_speed, k1);
        diff_f_n = bullet_flight_diff_function(t_n, robot_data->x, robot_data->y, robot_data->yaw, robot_data->vx, robot_data->vy, robot_data->v_yaw, robot_data->r1, bullet_speed, k1);

        //�ж�΢��ֵ�Ƿ�Ϸ�
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
        //����
        t_n1 = t_n - (f_n / diff_f_n);

        f_n1 = bullet_flight_function(t_n, robot_data->x, robot_data->y, robot_data->yaw, robot_data->vx, robot_data->vy, robot_data->v_yaw, robot_data->r1, bullet_speed, k1);

        deltat = fabs(t_n1 - t_n);

    } while (fabs(f_n1) > precision || deltat > min_deltat || iterate_count++ > max_iterate_count);

    return t_n1;
}

static float robot_center_x_to_armor_center_x(fp32 xc_k, fp32 vx, fp32 yaw_k, fp32 vyaw, fp32 r, fp32 t)
{
    return xc_k + t * vx - r * cos(yaw_k + t * vyaw);
}

static float robot_center_y_to_armor_center_y(fp32 yc_k, fp32 vy, fp32 yaw_k, fp32 vyaw, fp32 r, fp32 t)
{
    return yc_k + t * vy - r * sin(yaw_k + t * vyaw);
}

static float robot_center_z_to_armor_center_z(fp32 zc_k, fp32 vz, fp32 t)
{
    return zc_k + t * vz;
}

//����΢�ֺ���
static float bullet_flight_diff_function(fp32 t, fp32 xc_k, fp32 yc_k, fp32 yaw_k,
                           fp32 vx, fp32 vy, fp32 vyaw,
                           fp32 r, fp32 bullet_speed, fp32 k1)
{
    fp32 temp_1 = robot_center_y_to_armor_center_y(yc_k, vy, yaw_k, vyaw, r, t);
    fp32 temp_2 = robot_center_x_to_armor_center_x(xc_k, vx, yaw_k, vyaw, r, t);
    return bullet_speed / (k1 * bullet_speed * t + 1.0f) - (2 * (vx + r * vyaw * sin(yaw_k + t * vyaw)) * temp_2 + 2 * (vy - r * vyaw * cos(yaw_k + t * vyaw)) * temp_2) / (2 * sqrt(pow(temp_1, 2) + pow(temp_2, 2)));
}

//��������
static float bullet_flight_function(fp32 t, fp32 xc_k, fp32 yc_k, fp32 yaw_k,
                           fp32 vx, fp32 vy, fp32 vyaw,
                           fp32 r, fp32 bullet_speed, fp32 k1)
{
    fp32 temp_1 = robot_center_y_to_armor_center_y(yc_k, vy, yaw_k, vyaw, r, t);
    fp32 temp_2 = robot_center_x_to_armor_center_x(xc_k, vx, yaw_k, vyaw, r, t);
    return (1.0f / k1) * log(k1 * bullet_speed * t + 1.0f) - sqrt(pow(temp_1, 2) + pow(temp_2, 2));
}

/**
 * @brief �������� �������������е���
 *
 * @param bullet_flight_time �ӵ�����ʱ��
 * @param armor_position_distance װ�װ�ռ����
 * @param armor_position_vertical װ�װ�ռ�z�����
 * @param bullet_speed ����
 * @param precision ��������
 * @param max_iterate_count ����������
 * @return z�Ჹ����׼λ��
 */
static float calc_gimbal_aim_z_compensation(float bullet_flight_time, float armor_position_distance, float armor_position_vertical, float bullet_speed, float precision, float max_iterate_count)
{
    // �������߶�
    float bullet_drop_z = armor_position_vertical;
    // ��׼�߶�
    float aim_z = armor_position_vertical;
    // ����
    float pitch = 0;
    // ����ֵ����ʵֵ֮������
    float calc_and_actual_error = 0;
    // ����������
    for (int i = 0; i < max_iterate_count; i++)
    {
        // ��������
        pitch = atan2(aim_z, armor_position_distance);
        // �����ӵ����߶�
        bullet_drop_z = bullet_speed * sin(pitch) * bullet_flight_time - 0.5f * G * pow(bullet_flight_time, 2);
        // �������
        calc_and_actual_error = armor_position_vertical - bullet_drop_z;
        // ����׼�߶Ƚ��в���
        aim_z += calc_and_actual_error * ITERATE_SCALE_FACTOR;
        // �ж�����Ƿ���Ͼ���Ҫ��
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
