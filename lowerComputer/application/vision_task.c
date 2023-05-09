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

//imuתǹ�ڵ�ƽ�ƾ���
#define TRANSLATION_IMU_TO_GUNPOINT {0, 0, -fabs(IMU_TO_GUMPOINT_DISTANCE)}

//�Ӿ������ʼ��
static void vision_task_init(vision_control_t* init);
//�Ӿ��������ݸ���
static void vision_task_feedback_update(vision_control_t* update);
// ������λ������,���㵯���Ŀռ���㣬������ռ���Խ�
static void vision_data_process(vision_control_t* vision_data);
//����ϵ�£���������������תװ�װ���������
static void robot_center_vector_to_armor_center_vetcor(fp32 robot_center_x, fp32 robot_center_y, fp32 robot_center_z, 
                                                       fp32 armor_to_robot_center_r, fp32 armor_to_robot_center_theta, 
                                                       vector_t* armor_center_vector);
// //����ϵ�£�װ�װ�����Ŀռ�ԭ������imuΪ�ռ�ԭ��ת��Ϊ��ǹ��Ϊ�ռ�ԭ��
// static void imu_origin_to_gunpoint_origin_in_earth_frame(vector_t* armor_centor_vector, const fp32 quat[4]);
// //�������ϵ�£�װ�װ�����Ŀռ�ԭ������imuΪ�ռ�ԭ��ת��Ϊ��ǹ��Ϊ�ռ�ԭ��
// static void imu_origin_to_gunpoint_origin_in_relative_frame(vector_t* armor_centor_vector);
//����ϵ��̨��׼��������
static void calc_gimbal_aim_target_vector(vector_t* armor_target_vector, vector_t* aim_vector, fp32 observe_vx, fp32 observe_vy, fp32 observe_vz, fp32 bullet_speed);
//����yaw��pitch������
static void vision_analysis_date(vision_control_t* vision_set);
//���÷������ݰ�
static void set_vision_send_packet(vision_control_t* set_send_packet);

//��ȡ�������ݰ�ָ��
static vision_receive_t* get_vision_receive_point(void);
/**
 * @brief ţ�ٵ���������ӵ�����ʱ��
 * 
 * @param t_0 ��ʼ����ʱ��
 * @param precision ��������
 * @param min_deltat ��С������ֵ
 * @param max_iterate_count ���������� 
 * @param target_x Ŀ���x��ˮƽ����
 * @param target_vx Ŀ��x��۲��ٶ�
 * @param bullet_speed ����
 * @return ��������ֵ���ӵ�����ʱ�� 
 */
static float newton_iterate_to_calc_bullet_flight_time(float t_0, float precision, float min_deltat, int max_iterate_count,
                                           float target_x, float target_vx, float bullet_speed);
/**
 * @brief �ӵ�����
 * 
 * @param t ����ʱ��
 * @param target_x ˮƽλ��
 * @param target_vx ˮƽ�ٶ�
 * @param bullet_speed �ӵ�����
 * @return float
 */
static float bullet_flight_function(float t, float target_x, float target_vx, float bullet_speed);

/**
 * @brief �ӵ�΢�ֺ���
 * 
 * @param t ����ʱ��
 * @param target_vx ˮƽ�ٶ�
 * @param bullet_speed �ӵ�����
 * @return float
 */
static float bullet_flight_diff_function(float t, float target_vx, float bullet_speed);
/**
 * @brief �������� �������������е���
 * 
 * @param bullet_flight_time �ӵ�����ʱ��
 * @param armor_position_x װ�װ�ռ�x����
 * @param armor_position_z װ�װ�ռ�y�����
 * @param bullet_speed ����
 * @param precision ��������
 * @param max_iterate_count ����������
 * @return z�Ჹ����׼λ�� 
 */
static float calc_gimbal_aim_z_compensation(float bullet_flight_time, float armor_position_x, float armor_position_z, float bullet_speed, float precision, float max_iterate_count);


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
    //��ȡ��ǰ��ʱ����
    receive_packet_t robot_temp;
    memcpy(&robot_temp, &vision_data->vision_receive_point->receive_packet, sizeof(robot_temp));
    
    // ����ϵ�»��������ĵĿռ�����תװ�װ�ռ�����
    robot_center_vector_to_armor_center_vetcor(robot_temp.x, robot_temp.y, robot_temp.z, robot_temp.r1, robot_temp.yaw, &vision_data->target_armor_vector);

    //���㵯���ռ����
    calc_gimbal_aim_target_vector(&vision_data->target_armor_vector, &vision_data->robot_gimbal_aim_vector, robot_temp.vx, robot_temp.vy, robot_temp.vz, BULLET_SPEED);


    //����ŷ����
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
//     //װ�װ�ռ������ɹ���ϵתΪ��imuΪԭ��Ļ����������ϵ
//     earthFrame_to_relativeFrame(armor_centor_vector, quat);
//     //�������ϵ�¿ռ�ԭ���imuת��ǹ��
//     imu_origin_to_gunpoint_origin_in_relative_frame(armor_centor_vector);
//     //װ�װ�ռ��������������תΪ��ǹ��Ϊԭ��Ĺ�������ϵ
//     relativeFrame_to_earthFrame(armor_centor_vector, quat);    
// }

// static void imu_origin_to_gunpoint_origin_in_relative_frame(vector_t* armor_centor_vector)
// {
//     //ƽ�ƾ���
//     fp32 translation_imu_to_gunpoint[3] = TRANSLATION_IMU_TO_GUNPOINT;

//     //����ƽ��
//     armor_centor_vector->x += translation_imu_to_gunpoint[0];
//     armor_centor_vector->y += translation_imu_to_gunpoint[1];
//     armor_centor_vector->z += translation_imu_to_gunpoint[2];
// }


static void calc_gimbal_aim_target_vector(vector_t* armor_target_vector, vector_t* aim_vector, fp32 observe_vx, fp32 observe_vy, fp32 observe_vz, fp32 bullet_speed)
{
    //�����ӵ�����ʱ��
    fp32 bullet_flight_time = newton_iterate_to_calc_bullet_flight_time(T_0, PRECISION, MIN_DELTAT, MAX_ITERATE_COUNT, armor_target_vector->x, observe_vx, BULLET_SPEED);

    //������׼λ��
    aim_vector->x = armor_target_vector->x + observe_vx * bullet_flight_time;
    aim_vector->y = armor_target_vector->y + observe_vy * bullet_flight_time;
    aim_vector->z = armor_target_vector->z + observe_vz * bullet_flight_time;

    //���㵯������
    aim_vector->z = calc_gimbal_aim_z_compensation(bullet_flight_time, armor_target_vector->x, armor_target_vector->z, bullet_speed, PRECISION, MAX_ITERATE_COUNT);

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

        // �ж��Ƿ���յ���λ������
        if (vision_set->vision_receive_point->rx_flag) // ʶ��Ŀ��
        {
            // ���յ����ݱ�־λΪ0
            not_rx_vision_data_flag = 0;

            unrx_time = 0;
            // ���յ���λ������
            // ���ձ�־λ ����
            vision_set->vision_receive_point->rx_flag = 0;

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
            //������ȷ������ʱ���ݿ������������ݰ���
            memcpy(&vision_receive.receive_packet, &temp_packet, sizeof(receive_packet_t));
            //��������λ��1
            vision_receive.rx_flag = 1;
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
/**
 * @brief ţ�ٵ���������ӵ�����ʱ��
 * 
 * @param t_0 ��ʼ����ʱ��
 * @param precision ��������
 * @param min_deltat ��С������ֵ
 * @param max_iterate_count ���������� 
 * @param target_x Ŀ���x��ˮƽ����
 * @param target_vx Ŀ��x��۲��ٶ�
 * @param bullet_speed ����
 * @return ��������ֵ���ӵ�����ʱ�� 
 */
static float newton_iterate_to_calc_bullet_flight_time(float t_0, float precision, float min_deltat, int max_iterate_count,
                                                       float target_x, float target_vx, float bullet_speed)
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
        f_n = bullet_flight_function(t_n, target_x, target_vx, bullet_speed);
        diff_f_n = bullet_flight_diff_function(t_n, target_vx, bullet_speed);

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

        f_n1 = bullet_flight_function(t_n1, target_x, target_vx, bullet_speed);

        deltat = fabs(t_n1 - t_n);

        // �ж��Ƿ񳬹�����������
        if (iterate_count++ > max_iterate_count)
        {
            // �жϵ��������Ƿ���Ͼ���Ҫ��
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
 * @brief �ӵ�����
 * 
 * @param t ����ʱ��
 * @param target_x ˮƽλ��
 * @param target_vx ˮƽ�ٶ�
 * @param bullet_speed �ӵ�����
 * @return float
 */
static float bullet_flight_function(float t, float target_x, float target_vx, float bullet_speed)
{
    return ((1.0f / AIR_K1) * log(AIR_K1 * bullet_speed * t + 1.0f) - target_x - target_vx * t);
}
/**
 * @brief �ӵ�΢�ֺ���
 * 
 * @param t ����ʱ��
 * @param target_vx ˮƽ�ٶ�
 * @param bullet_speed �ӵ�����
 * @return float
 */
static float bullet_flight_diff_function(float t, float target_vx, float bullet_speed)
{
    return ((bullet_speed / (AIR_K1 * bullet_speed * t + 1.0f)) - target_vx);
}

/**
 * @brief �������� �������������е���
 * 
 * @param bullet_flight_time �ӵ�����ʱ��
 * @param armor_position_x װ�װ�ռ�x����
 * @param armor_position_z װ�װ�ռ�y�����
 * @param bullet_speed ����
 * @param precision ��������
 * @param max_iterate_count ����������
 * @return z�Ჹ����׼λ�� 
 */
static float calc_gimbal_aim_z_compensation(float bullet_flight_time, float armor_position_x, float armor_position_z, float bullet_speed, float precision, float max_iterate_count)
{
    //�������߶�
    float bullet_drop_z = armor_position_z;
    //��׼�߶�
    float aim_z = armor_position_z;
    //����
    float pitch = 0;
    // ����ֵ����ʵֵ֮������
    float calc_and_actual_error = 0;
    for (int i = 0; i < max_iterate_count; i++)
    {
        //��������
        pitch = atan2(aim_z, armor_position_x);
        //�����ӵ����߶�
        bullet_drop_z = bullet_speed * sin(pitch) * bullet_flight_time - 0.5f * G * pow(bullet_flight_time, 2);
        //�������
        calc_and_actual_error = armor_position_z - bullet_drop_z;
        //����׼�߶Ƚ��в���
        aim_z += calc_and_actual_error * ITERATE_SCALE_FACTOR;
        //�ж�����Ƿ���Ͼ���Ҫ��
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
gimbal_vision_control_t* get_vision_gimbal_point(void)
{
    return &vision_control.gimbal_vision_control;
}

// ��ȡ��λ����������
shoot_vision_control_t* get_vision_shoot_point(void)
{
    return &vision_control.shoot_vision_control;
}
