/**
 * @file vision_task.c
 * @author yuanluochen
 * @brief ��������yaw��pitch��roll����ԽǶȸ���λ���Ӿ� 
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
static void vision_init(vision_t* init);
//�Ӿ��������ݸ���
static void vision_task_feedback_update(vision_t* update);
//���ݱ���
static void vision_tx_encode(uint8_t* buf, float yaw, float pitch, float roll, uint8_t mode_switch);
//�Ӿ�����ṹ��
vision_t vision = { 0 };

void vision_task(void const *pvParameters)
{
    // ��ʱ�ȴ����ȴ������������������������
    vTaskDelay(VISION_TASK_INIT_TIME);
    //�Ӿ�����ṹ���ʼ��
    vision_init(&vision);
    while(1)
    {
        //���ݸ���
        vision_task_feedback_update(&vision);
        //���ô��ڷ�������
        vision_tx_encode(vision.send_message, vision.absolution_angle.yaw * RADIAN_TO_ANGle, vision.absolution_angle.pitch * RADIAN_TO_ANGle, vision.absolution_angle.roll * RADIAN_TO_ANGle, 1);
        //���ڷ���
        HAL_UART_Transmit(&huart1, vision.send_message, SERIAL_SEND_MESSAGE_SIZE, VISION_USART_TIME_OUT);

    }
    
}


static void vision_init(vision_t* init)
{
    //��ȡ�����Ǿ��Խ�ָ��
    init->vision_angle_point = get_INS_angle_point();
    //���ݸ���
    vision_task_feedback_update(init);
}

static void vision_task_feedback_update(vision_t* update)
{
    update->absolution_angle.yaw = *(update->vision_angle_point + INS_YAW_ADDRESS_OFFSET);
    update->absolution_angle.pitch = -*(update->vision_angle_point + INS_PITCH_ADDRESS_OFFSET);
    update->absolution_angle.roll = *(update->vision_angle_point + INS_ROLL_ADDRESS_OFFSET);
}

//����
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

