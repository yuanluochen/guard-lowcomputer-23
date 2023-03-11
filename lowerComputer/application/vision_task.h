/**
 * @file vision_task.h
 * @author yuanluochen
 * @brief �Ӿ�����
 * @version 0.1
 * @date 2023-03-11
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#ifndef VISION_TASK_H
#define VISION_TASK_H

#include "usart.h"
#include "INS_task.h"
#include "arm_math.h"

//��ʱ�ȴ�
#define VISION_TASK_INIT_TIME 401
//���ڷ������ݴ�С
#define SERIAL_SEND_MESSAGE_SIZE 28
//�ӱ�
#define DOUBLE_ 10000
//��λ�����ڽṹ��
#define VISION_USART huart1
//��������ʱ��
#define VISION_USART_TIME_OUT 1000
//������ת�Ƕ���
#define RADIAN_TO_ANGle (360 / (2 * PI))

#define HEAD1_DATA 0x34
#define HEAD2_DATA 0X43

//��ʼλ1ƫ����
#define HEAD1_ADDRESS_OFFSET 0
//��ʼλ2ƫ����
#define HEAD2_ADDRESS_OFFSET 1
//yaw��ƫ����
#define YAW_ADDRESS_OFFSET 2
//pitch��ƫ����
#define PITCH_ADDRESS_OFFSET 3
//roll��ƫ����
#define ROLL_ADDRESS_OFFSET 4
//ģʽת��ƫ����
#define SWITCH_ADDRESS_OFFSET 5
//��β����ƫ����
#define END_ADDRESS_OFFSET 6

//���ݷ���uint8_t������Ĵ�С
#define UINT8_T_DATA_SIZE 4

//32λ����ת8λ����
typedef union 
{
    float fp32_val;
    uint32_t uint32_val;
    uint8_t uin8_value[UINT8_T_DATA_SIZE];
    
}date32_to_date8_t;

typedef struct 
{
    fp32 yaw;
    fp32 pitch;
    fp32 roll;
}eular_angle_t;


//�Ӿ�����ṹ��
typedef struct 
{
    //������Խ�ָ��
    const fp32* vision_angle_point;
    //���Խ�
    eular_angle_t absolution_angle; 
    //���ô��ڷ�������
    uint8 send_message[SERIAL_SEND_MESSAGE_SIZE];
}vision_t;



//�Ӿ�ͨ������
void vision_task(void const *pvParameters);



#endif // !VISION_TASK_H
