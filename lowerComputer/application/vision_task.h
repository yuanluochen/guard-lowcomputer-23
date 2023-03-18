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
#include "kalman.h"
#include "rm_usart.h"
#include "pid.h"

//��ʱ�ȴ�
#define VISION_TASK_INIT_TIME 401
//ϵͳ��ʱʱ��
#define VISION_CONTROL_TIME_MS 2

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

//һ��kalman filter����
#define GIMBAL_YAW_MOTOR_KALMAN_Q 400
#define GIMBAL_YAW_MOTOR_KALMAN_R 400

#define GIMBAL_PITCH_MOTOR_KALMAN_Q 200
#define GIMBAL_PITCH_MOTOR_KALMAN_R 400


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

//�Ӿ�kalman filter�ṹ��
typedef struct 
{
    //һάkalman
    kalman gimbal_pitch_kalman;  //pitch����kalman filter�ṹ��    
    kalman gimbal_yaw_kalman;    //��̨yaw����kalman filter�ṹ��
    kalman disdance_kalman;      //����ṹ��    

    //����kalman filter,��Ҫ������̨�������
    kalman_filter_init_t gimbal_pitch_second_order_kalman;
    kalman_filter_init_t gimbal_yaw_second_order_kalman;
}vision_kalman_filter_t;

//�ڱ���̨����˶�����,���˲���������ֵ
typedef struct 
{
    //������̨yaw������
    fp32 gimbal_yaw_add;
    //������̨pitch������
    fp32 gimbal_pitch_add;
}gimbal_motor_command_t;

//�ڱ���̨pid�ṹ��
typedef struct 
{
    //��̨pitch��pid
    pid_type_def gimbal_pitch_pid;
    //��̨yaw��pid
    pid_type_def gimbal_yaw_pid;
}vision_pid_t;


//�Ӿ�����ṹ��
typedef struct 
{
    //������Խ�ָ��
    const fp32* vision_angle_point;
    //���Խ�
    eular_angle_t absolution_angle; 
    //���ô��ڷ�������
    uint8 send_message[SERIAL_SEND_MESSAGE_SIZE];

    //��λ���Ӿ�ָ��
    vision_rxfifo_t* vision_rxfifo;

    //kalman filter �ṹ�壬���ڴ����Ӿ���λ������������
    vision_kalman_filter_t vision_kalman_filter;

    //pid �ṹ�壬���������λ���Ӿ���Ӧ
    vision_pid_t vision_pid;
    
    //��̨����˶�����
    gimbal_motor_command_t gimbal_motor_command;    
    
}vision_t;


//�Ӿ�ͨ������
void vision_task(void const *pvParameters);
//��ȡ��λ������
vision_t* get_vision_point(void);


#endif // !VISION_TASK_H
