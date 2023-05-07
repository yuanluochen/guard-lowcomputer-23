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



//�Ӿ���������ṹ���ʼ��
static void vision_send_task_init(vision_send_t* init);
//�Ӿ������������ݸ���
static void vision_send_task_feedback_update(vision_send_t* update);
//���÷������ݰ�
static void set_vision_send_packet(vision_send_t* set_send_packet);
//�������ݰ�
static void send_packet(vision_send_t* send);

//�Ӿ������ʼ��
static void vision_task_init(vision_control_t* init);
//�Ӿ��������ݸ���
static void vision_task_feedback_update(vision_control_t* update);
//�����Ӿ����ݰ�
static void vision_data_process(vision_control_t* vision_data);
//����yaw��pitch������
static void vision_analysis_date(vision_control_t* vision_set);

//����ϵ�£���������������תװ�װ�����
// static void robot_center_vector_to_armor_vector()

//��ȡ�������ݰ�ָ��
static vision_receive_t* get_vision_receive_point(void);

//�Ӿ�����ṹ��
vision_control_t vision_control = { 0 };
//�Ӿ����ͽṹ��
vision_send_t vision_send = { 0 };
//�Ӿ����սṹ��
vision_receive_t vision_receive = { 0 };


//δ���յ��Ӿ����ݱ�־λ����λΪ1 ��δ����
bool_t not_rx_vision_data_flag = 1;


void vision_send_task(void const *pvParameters)
{
    // ��ʱ�ȴ����ȴ������������������������
    vTaskDelay(VISION_SEND_TASK_INIT_TIME);
    //�Ӿ���������ṹ���ʼ��
    vision_send_task_init(&vision_send);
    while(1)
    {
        // ���ݸ���
        vision_send_task_feedback_update(&vision_send);
        // ���÷������ݰ�
        set_vision_send_packet(&vision_send);
        //�������ݰ�
        send_packet(&vision_send);
        //ϵͳ��ʱ
        vTaskDelay(VISION_SEND_CONTROL_TIME_MS);
    }
}

static void vision_send_task_init(vision_send_t* init)
{
    //��ȡ�����Ǿ��Խ�ָ��
    init->INS_angle_point = get_INS_angle_point();
    // ���ݸ���
    vision_send_task_feedback_update(init);
}

static void vision_send_task_feedback_update(vision_send_t* update)
{
    //��������ŷ����
    update->eular_angle.yaw = *(update->INS_angle_point + INS_YAW_ADDRESS_OFFSET); 
    update->eular_angle.pitch = *(update->INS_angle_point + INS_PITCH_ADDRESS_OFFSET); 
    update->eular_angle.roll = *(update->INS_angle_point + INS_ROLL_ADDRESS_OFFSET); 
    //������׼λ��
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

void vision_task(void const* pvParameters)
{
    // ��ʱ�ȴ����ȴ���λ���������ݳɹ�
    vTaskDelay(VISION_TASK_INIT_TIME);
    // �Ӿ������ʼ��
    vision_task_init(&vision_control);
    while (1)
    {
        // �ȴ�Ԥװ����ϣ��Լ���̨���̳�ʼ����ϣ��ж������Ƿ����
        if (shoot_control_vision_task() /*&& gimbal_control_vision_task()*/)
        {
            // ��������
            vision_task_feedback_update(&vision_control);
            // ������λ������
            vision_data_process(&vision_control);
            // ������λ������,����yaw��pitch������,�Լ��ж��Ƿ���
            vision_analysis_date(&vision_control);
        }
        else
        {
            //��������
        }
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
    
    //��ȡĿ���������ϵ�µĿռ������
    update->target_robot_vector.x = update->vision_receive_point->receive_packet.x;
    update->target_robot_vector.y = update->vision_receive_point->receive_packet.y;
    update->target_robot_vector.z = update->vision_receive_point->receive_packet.z;
}

static void vision_data_process(vision_control_t* vision_data)
{
    //��ȡ��ǰ��ʱ����
    receive_packet_t robot_temp;
    memcpy(&robot_temp, &vision_data->vision_receive_point->receive_packet, sizeof(robot_temp));
    //����ϵ�»��������ĵĿռ�����תװ�װ�ռ�����
    vision_data->target_armor_vector.x = robot_temp.x - robot_temp.r1 * cosf(robot_temp.yaw);
    vision_data->target_armor_vector.y = robot_temp.y - robot_temp.r1 * sinf(robot_temp.yaw);
    vision_data->target_armor_vector.z = robot_temp.z;
    //����ŷ����
    vision_data->vision_absolution_angle.yaw = atan2(vision_data->target_armor_vector.y, vision_data->target_armor_vector.x);
    // vision_data->vision_absolution_angle.yaw = 0;
    // vision_data->vision_absolution_angle.pitch = atan2(vision_data->target_armor_vector.z, sqrt(vision_data->target_armor_vector.x * vision_data->target_armor_vector.x + vision_data->target_armor_vector.y * vision_data->target_armor_vector.y));   
    vision_data->vision_absolution_angle.pitch = atan2(vision_data->target_armor_vector.z, vision_data->target_armor_vector.x);   
    // vision_data->vision_absolution_angle.pitch = atan2(vision_data->target_armor_vector.z, vision_data->target_armor_vector.x);   
    // vision_data->vision_absolution_angle.pitch = 0;        
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


static vision_receive_t* get_vision_receive_point(void)
{
    return &vision_receive;
}

/**
 * @brief �������ϵת��������ϵ 
 * 
 * @param vecRF �������ϵ�µĿռ�����
 * @param vecEF ��������ϵ�µĿռ�����
 * @param q ��Ԫ��
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
 * @brief ��������ϵת�������ϵ 
 *  
 * @param vecEF ��������ϵ�µĿռ�����
 * @param vecRF �������ϵ�µĿռ�����
 * @param q ��Ԫ��
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
