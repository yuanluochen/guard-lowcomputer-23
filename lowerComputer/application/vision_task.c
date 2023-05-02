/**
 * @file vision_task.c
 * @author yuanluochen
 * @brief ��������yaw��pitch��roll����ԽǶȸ���λ���Ӿ����������Ӿ��������ݣ�����hal������ԭ���ȫ˫������ͨ��֧�ֲ����ر�ã�
 *        Ϊ����������⽫�������ݷ������봮�ڷ��ͷ��룬���ӳ����ڷ���ʱ��
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


//�Ӿ���������ṹ���ʼ��
static void vision_send_task_init(vision_send_t* init);
//�Ӿ������ʼ��
static void vision_task_init(vision_control_t* init);
//�Ӿ������������ݸ���
static void vision_send_task_feedback_update(vision_send_t* update);
//��������ģʽ
static void vision_set_mode(vision_send_t* vision_mode);
//�Ӿ��������ݸ���
static void vision_task_feedback_update(vision_control_t* update);
//���ݱ���
static void vision_tx_encode(vision_send_t* vision_tx_data_encode);
//����yaw��pitch������
static void vision_analysis_date(vision_control_t* vision_set);
/**
 * @brief �����Ӿ�ԭʼ�������ݣ�����ԭʼ���ݣ��ж��Ƿ�Ҫ���з���
 * 
 * @param shoot_judge �Ӿ��ṹ��
 * @param vision_begin_add_yaw_angle ��λ���Ӿ�yuw��ԭʼ���ӽǶ�
 * @param vision_begin_add_pitch_angle ��λ���Ӿ�pitch��ԭʼ���ӽǶ�
 */
static void vision_shoot_judge(vision_control_t* shoot_judge, fp32 vision_begin_add_yaw_angle, fp32 vision_begin_add_pitch_angle);


//�Ӿ�����ṹ��
vision_control_t vision_control = { 0 };
//�Ӿ���������ṹ��
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
        // ��������ģʽ
        vision_set_mode(&vision_send);
        // ���ô��ڷ�������,����
        vision_tx_encode(&vision_send);

        //��������
        CDC_Transmit_FS(vision_send.send_message, SUM_SEND_MESSAGE);
        //ϵͳ��ʱ
        vTaskDelay(VISION_SEND_CONTROL_TIME_MS);
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
        if (shoot_control_vision_task() && gimbal_control_vision_task())
        {
            // ��������
            vision_task_feedback_update(&vision_control);
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

static void vision_send_task_init(vision_send_t* init)
{
    //��ȡ�����Ǿ��Խ�ָ��                                                                                                                                                                                                                                                                                                                                                           init->vision_angle_point = get_INS_angle_point();
    init->vision_quat_point = get_INS_quat_point();
    //��ʼ���������ݽṹ���ͷ֡β֡
    init->send_msg_struct.head = LOWER_TO_HIGH_HEAD;
    init->send_msg_struct.end = END_DATA;
    // ���ݸ���
    vision_send_task_feedback_update(init);
}

static void vision_task_init(vision_control_t* init)
{
    // ��ȡ�����Ǿ��Խ�ָ��                                                                                                                                                                                                                                                                                                                                                           init->vision_angle_point = get_INS_angle_point();
    init->vision_angle_point = get_INS_angle_point();
	
    // ��ȡ��λ���Ӿ�ָ��
    init->vision_rxfifo = get_vision_rxfifo_point();
    //��ʼ������ģʽΪֹͣϮ��
    init->shoot_vision_control.shoot_command = SHOOT_STOP_ATTACK;

    vision_task_feedback_update(init);
 
}

static void vision_send_task_feedback_update(vision_send_t* update)
{
    //��ȡ��Ԫ����ֵ
    memcpy(update->send_msg_struct.quat, update->vision_quat_point, 4 * sizeof(fp32));
}

static void vision_task_feedback_update(vision_control_t* update)
{
    // ��ȡԭʼ���ݲ�ת��Ϊ�Ƕ���
    update->absolution_angle.yaw = *(update->vision_angle_point + INS_YAW_ADDRESS_OFFSET) * RADIAN_TO_ANGLE;
    update->absolution_angle.pitch = *(update->vision_angle_point + INS_PITCH_ADDRESS_OFFSET) * RADIAN_TO_ANGLE;
    update->absolution_angle.roll = *(update->vision_angle_point + INS_ROLL_ADDRESS_OFFSET) * RADIAN_TO_ANGLE;

}


static void vision_set_mode(vision_send_t* vision_mode)
{
    //��������ģʽΪװ�װ�ģʽ
    vision_mode->send_msg_struct.mode_change = ARMOURED_PLATE_MODE;
}

static void vision_tx_encode(vision_send_t* vision_tx_data_encode)
{
    //ͷ
    memcpy((void*)(vision_tx_data_encode->send_message + (int)HEAD_ADDRESS_OFFSET), (void*)&vision_tx_data_encode->send_msg_struct.head, DATA_QUAT_REAL_ADDRESS_OFFSET - HEAD_ADDRESS_OFFSET);
    //���ݶ� ��Ԫ��
    memcpy((void*)(vision_tx_data_encode->send_message + (int)DATA_QUAT_REAL_ADDRESS_OFFSET), (void*)vision_tx_data_encode->send_msg_struct.quat, MODE_SWITCH_ADDRESS_OFFSET - DATA_QUAT_REAL_ADDRESS_OFFSET);
    //���ݶ�ģʽת��
    memcpy((void*)(vision_tx_data_encode->send_message + (int)MODE_SWITCH_ADDRESS_OFFSET), (void*)&vision_tx_data_encode->send_msg_struct.mode_change, CHECK_BIT_ADDRESS_OFFSET - MODE_SWITCH_ADDRESS_OFFSET);
    //У��λ
    memcpy((void*)(vision_tx_data_encode->send_message + (int)CHECK_BIT_ADDRESS_OFFSET), (void*)&vision_tx_data_encode->send_msg_struct.check, END_ADDRESS_OFFSET - CHECK_BIT_ADDRESS_OFFSET);
    //β
    memcpy((void*)(vision_tx_data_encode->send_message + (int)END_ADDRESS_OFFSET), (void*)&vision_tx_data_encode->send_msg_struct.end, SUM_SEND_MESSAGE - END_ADDRESS_OFFSET);
}

static void vision_analysis_date(vision_control_t *vision_set)
{

    // ��λ���Ӿ��汾
    static fp32 vision_gimbal_yaw = 0;   // yaw����Խ�
    static fp32 vision_gimbal_pitch = 0; // pitch����Խ�
    // δ���յ���λ����ʱ��
    static int32_t unrx_time = MAX_UNRX_TIME;

    // �жϵ�ǰ��̨ģʽΪ����ģʽ
    if (judge_gimbal_mode_is_auto_mode())
    {
        // ������ģʽ�����ýǶ�Ϊ��λ�����ýǶ�

        // �ж��Ƿ���յ���λ������
        if (vision_set->vision_rxfifo->rx_flag) // ʶ��Ŀ��
        {
            // ���յ����ݱ�־λΪ0
            not_rx_vision_data_flag = 0;

            unrx_time = 0;
            // ���յ���λ������
            // ���ձ�־λ ����
            vision_set->vision_rxfifo->rx_flag = 0;

            // ��ȡ��λ���Ӿ�����
            vision_gimbal_pitch = vision_set->vision_rxfifo->pitch_fifo;
            vision_gimbal_yaw = vision_set->vision_rxfifo->yaw_fifo;

            // �жϷ���
            vision_shoot_judge(vision_set, (vision_gimbal_yaw - vision_set->absolution_angle.yaw), (vision_gimbal_pitch - vision_set->absolution_angle.pitch));
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
        vision_set->gimbal_vision_control.gimbal_pitch = vision_set->absolution_angle.pitch;
        vision_set->gimbal_vision_control.gimbal_yaw = vision_set->absolution_angle.yaw;
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
static void vision_shoot_judge(vision_control_t* shoot_judge, fp32 vision_begin_add_yaw_angle, fp32 vision_begin_add_pitch_angle)
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


void highcomputer_rx_decode(uint8_t* rx_Buf, uint32_t* rx_buf_Len)
{
    // �������ݽ���
    //ͷ
    memcpy((void *)vision_receive.receive_msg_struct.head, (void *)(rx_Buf + (int)HEAD_ADDRESS_OFFSET), (DATA_QUAT_REAL_ADDRESS_OFFSET - HEAD_ADDRESS_OFFSET));
    // ���ݶ�
    memcpy((void *)&vision_receive.receive_msg_struct.quat, (void *)(rx_Buf + (int)DATA_QUAT_REAL_ADDRESS_OFFSET), (MODE_SWITCH_ADDRESS_OFFSET - DATA_QUAT_REAL_ADDRESS_OFFSET));
    // ģʽת��λ
    memcpy((void *)&vision_receive.receive_msg_struct.mode_change, (void *)(rx_Buf + (int)MODE_SWITCH_ADDRESS_OFFSET), (CHECK_BIT_ADDRESS_OFFSET - MODE_SWITCH_ADDRESS_OFFSET));
    // У��λ
    memcpy((void *)&vision_receive.receive_msg_struct.check, (void *)(rx_Buf + (int)CHECK_BIT_ADDRESS_OFFSET), (END_ADDRESS_OFFSET - CHECK_BIT_ADDRESS_OFFSET));
    // β
    memcpy((void *)&vision_receive.receive_msg_struct.end, (void *)(rx_Buf + (int)END_ADDRESS_OFFSET), (SUM_SEND_MESSAGE - END_ADDRESS_OFFSET));
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
