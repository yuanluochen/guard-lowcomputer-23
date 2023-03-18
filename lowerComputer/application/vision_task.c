/**
 * @file vision_task.c
 * @author yuanluochen
 * @brief ��������yaw��pitch��roll����ԽǶȸ���λ���Ӿ����������Ӿ��������� 
 * @version 0.1
 * @date 2023-03-11
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#include "vision_task.h"

#include "FreeRTOS.h"
#include "task.h"



//�Ӿ�����ṹ���ʼ��
static void vision_task_init(vision_t* init);
//�Ӿ��������ݸ���
static void vision_task_feedback_update(vision_t* update);
//���ݱ���
static void vision_tx_encode(uint8_t* buf, float yaw, float pitch, float roll, uint8_t mode_switch);
//����yaw��pitch������
static void vision_set_add_value(vision_t* vision_set);
//�Ӿ�����ṹ��
vision_t vision = { 0 };

void vision_task(void const *pvParameters)
{
    // ��ʱ�ȴ����ȴ������������������������
    vTaskDelay(VISION_TASK_INIT_TIME);
    //�Ӿ�����ṹ���ʼ��
    vision_task_init(&vision);
    while(1)
    {

        //���ݸ���
        vision_task_feedback_update(&vision);

        // ����yaw��pitch������
        vision_set_add_value(&vision);

        //���ݷ���
        //���ô��ڷ�������,����
        vision_tx_encode(vision.send_message, vision.absolution_angle.yaw * RADIAN_TO_ANGle, 
                                              vision.absolution_angle.pitch * RADIAN_TO_ANGle, 
                                              vision.absolution_angle.roll * RADIAN_TO_ANGle, 
                                              1);
        //���ڷ���
        HAL_UART_Transmit(&huart1, vision.send_message, SERIAL_SEND_MESSAGE_SIZE, VISION_USART_TIME_OUT);

        //ϵͳ��ʱ
        vTaskDelay(VISION_CONTROL_TIME_MS);
    }
    
}


static void vision_task_init(vision_t* init)
{
    //��ȡ�����Ǿ��Խ�ָ��
    init->vision_angle_point = get_INS_angle_point();
    //��ȡ��λ���Ӿ�ָ��
    init->vision_rxfifo = get_vision_rxfifo_point();

    //��ʼ��һάkalman filter
    kalmanCreate(&init->vision_kalman_filter.gimbal_pitch_kalman, GIMBAL_PITCH_MOTOR_KALMAN_Q, GIMBAL_PITCH_MOTOR_KALMAN_R);
    kalmanCreate(&init->vision_kalman_filter.gimbal_yaw_kalman, GIMBAL_YAW_MOTOR_KALMAN_Q, GIMBAL_PITCH_MOTOR_KALMAN_R);


    //���ݸ���
    vision_task_feedback_update(init);
}

static void vision_task_feedback_update(vision_t* update)
{
    update->absolution_angle.yaw = *(update->vision_angle_point + INS_YAW_ADDRESS_OFFSET);
    update->absolution_angle.pitch = -*(update->vision_angle_point + INS_PITCH_ADDRESS_OFFSET);
    update->absolution_angle.roll = *(update->vision_angle_point + INS_ROLL_ADDRESS_OFFSET);
}

static void vision_tx_encode(uint8_t* buf, float yaw, float pitch, float roll, uint8_t mode_switch)
{
    //������ʼ
    date32_to_date8_t head1_temp = { 0 };
    date32_to_date8_t head2_temp = { 0 };
    //yaw����ֵת��
    date32_to_date8_t yaw_temp = { 0 };
    //pitch����ֵת��
    date32_to_date8_t pitch_temp = { 0 };
    //roll������ת��
    date32_to_date8_t roll_temp = { 0 };
    //ģʽת��
    date32_to_date8_t switch_temp = { 0 };
    //��ֹλ
    date32_to_date8_t end_temp = { 0 };

    //��ֵ����
    head1_temp.uint32_val = HEAD1_DATA;
    head2_temp.uint32_val = HEAD2_DATA;
    yaw_temp.uint32_val = yaw * DOUBLE_;
    pitch_temp.uint32_val = pitch * DOUBLE_;
    roll_temp.uint32_val = roll * DOUBLE_;  
    switch_temp.uint32_val = mode_switch;
    end_temp.uint32_val = 0x0A;//"/n"

    for (int i = 3; i >= 0; i--)
    {
        int j = -(i - 3);//����λ��ת��
        buf[HEAD1_ADDRESS_OFFSET * UINT8_T_DATA_SIZE + j] = head1_temp.uin8_value[i];
        buf[HEAD2_ADDRESS_OFFSET * UINT8_T_DATA_SIZE + j] = head2_temp.uin8_value[i];
        buf[YAW_ADDRESS_OFFSET * UINT8_T_DATA_SIZE + j] = yaw_temp.uin8_value[i];
        buf[PITCH_ADDRESS_OFFSET * UINT8_T_DATA_SIZE + j] = pitch_temp.uin8_value[i];
        buf[ROLL_ADDRESS_OFFSET * UINT8_T_DATA_SIZE + j] = roll_temp.uin8_value[i];
        buf[SWITCH_ADDRESS_OFFSET * UINT8_T_DATA_SIZE + j] = switch_temp.uin8_value[i]; 
        buf[END_ADDRESS_OFFSET * UINT8_T_DATA_SIZE + j] = end_temp.uin8_value[i];
    }
}

static void vision_set_add_value(vision_t* vision_set)
{
    //��λ���Ӿ�ԭʼ����
    static fp32 vision_gimbal_yaw_add = 0;   // yaw������
    static fp32 vision_gimbal_pitch_add = 0; // pitch������
    //��λ��
    // �ж��Ƿ���յ���λ������
    if (vision_set->vision_rxfifo->rx_flag)
    {
        //���յ���λ������

        //����λ����
        vision_set->vision_rxfifo->rx_flag = 0;

        //��ȡ��λ���Ӿ�����
        vision_gimbal_pitch_add = vision_set->vision_rxfifo->pitch_fifo;
        vision_gimbal_yaw_add = vision_set->vision_rxfifo->yaw_fifo;

        //������λ���Ӿ�����,���Ӿ����ݽ���kalman filter
        KalmanFilter(&vision_set->vision_kalman_filter.gimbal_yaw_kalman, vision_gimbal_yaw_add);
        KalmanFilter(&vision_set->vision_kalman_filter.gimbal_pitch_kalman, vision_gimbal_pitch_add);

        //��ȡ��λ��kalamn filter��������ֵ
        vision_set->gimbal_motor_command.gimbal_yaw_add = vision_set->vision_kalman_filter.gimbal_yaw_kalman.X_now;
        vision_set->gimbal_motor_command.gimbal_pitch_add = vision_set->vision_kalman_filter.gimbal_pitch_kalman.X_now;


    }
    else
    {
        //δ���յ���λ������
        //����yaw��pitch������Ϊ0
        vision.gimbal_motor_command.gimbal_yaw_add = 0;
        vision.gimbal_motor_command.gimbal_pitch_add = 0;
    }

}

vision_t* get_vision_point()
{
    return &vision;
}
