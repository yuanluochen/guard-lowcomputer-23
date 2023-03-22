/**
 * @file vision_task.h
 * @author yuanluochen
 * @brief ��������yaw��pitch��roll����ԽǶȸ���λ���Ӿ����������Ӿ��������ݣ�����hal������ԭ���ȫ˫������ͨ��֧�ֲ����ر�ã�
 *        Ϊ����������⽫�������ݷ������봮�ڷ��ͷ��룬���ӳ����ڷ���ʱ��
 * @version 0.1
 * @date 2023-03-11
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#ifndef VISION_TASK_H
#define VISION_TASK_H

#include "usart.h"
#include "dma.h"
#include "INS_task.h"
#include "arm_math.h"
#include "kalman.h"
#include "rm_usart.h"
#include "pid.h"

//�ⲿ����
extern DMA_HandleTypeDef hdma_usart1_tx;

/**
 * @brief kalman filter ����
 *        0 Ϊ����
 *        1 Ϊһ�� 
 * 
 */
#define KALMAN_FILTER_TYPE 0

//����ʱ���
#define DT 2
//�����С
#define MATRIX_SIZE 4

//pitch��λ�Ʒ���
#define PITCH_DP 1
//pitch���ٶȷ���
#define PITCH_DV 1
//pitch����ٶȷ���
#define PITCH_DA 1

//yaw��λ�Ʒ���
#define YAW_DP 1
//yaw���ٶȷ���
#define YAW_DV 1
//yaw����ٶȷ���
#define YAW_DA 1

//�˲���ĽǶ�ƫ����
#define GIMBAL_ANGLE_ADDRESS_OFFSET 0


//һ��kalman filter����
#define GIMBAL_YAW_MOTOR_KALMAN_Q 400
#define GIMBAL_YAW_MOTOR_KALMAN_R 400

#define GIMBAL_PITCH_MOTOR_KALMAN_Q 200
#define GIMBAL_PITCH_MOTOR_KALMAN_R 400


//�������Ƕ����
#define ALLOW_ATTACK_ERROR 5

//�����жϼ�������
#define JUDGE_ATTACK_COUNT 20
//����ֹͣ�жϼ�������
#define JUDGE_STOP_ATTACK_COUNT 20

//��ʱ�ȴ�
#define VISION_SEND_TASK_INIT_TIME 401
//ϵͳ��ʱʱ��
#define VISION_SEND_CONTROL_TIME_MS 10

//��ʱ�ȴ�
#define VISION_TASK_INIT_TIME 450
//ϵͳ��ʱʱ��
#define VISION_CONTROL_TIME_MS 2

//���ڷ������ݴ�С
#define SERIAL_SEND_MESSAGE_SIZE 28
//�ӱ�
#define DOUBLE_ 10000
//��λ�����ڽṹ��
#define VISION_USART huart1
//��λ������dma�ṹ��
#define VISION_TX_DMA hdma_usart1_tx
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

//����λ����������Ϊ�����������pi����
#define SEND_MESSAGE_ERROR PI


//��λ��ģʽ,����װ�װ�ģʽ,��������ģʽ,ǰ��վģʽ
typedef enum
{
    ARMOURED_PLATE = 1, //װ�װ�ģʽ
    ENERGY_ORGAN = 2,   //��������
    OUTPOST = 3,        //ǰ��վ
}vision_mode_e;

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
    kalman_filter_t gimbal_pitch_second_order_kalman;
    kalman_filter_t gimbal_yaw_second_order_kalman;

    //����kalman filter ��ʼ���ṹ�壬���ڸ����������ľ���ֵ
    kalman_filter_init_t gimbal_pitch_second_order_kalman_init;
    kalman_filter_init_t gimbal_yaw_second_order_kalman_init;
}vision_kalman_filter_t;

//�ڱ���̨����˶�����,���˲���������ֵ
typedef struct 
{
    //������̨yaw������
    fp32 gimbal_yaw_add;
    //������̨pitch������
    fp32 gimbal_pitch_add;
}gimbal_vision_control_t;

//�ڱ���������
typedef enum
{
    SHOOT_ATTACK,       // Ϯ��
    SHOOT_READY_ATTACK, // ׼��Ϯ��
    SHOOT_STOP_ATTACK,  // ֹͣϮ��
} shoot_command_e;

//�ڱ��������˶���������
typedef struct 
{
    //�Զ���������
    shoot_command_e shoot_command;
}shoot_vision_control_t;

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
    // ��λ���Ӿ�ָ��
    vision_rxfifo_t *vision_rxfifo;
    
    // ������Խ�ָ��
    const fp32* vision_angle_point;
    
    // ���Խ�
    eular_angle_t absolution_angle;

    // kalman filter �ṹ�壬���ڴ����Ӿ���λ������������
    vision_kalman_filter_t vision_kalman_filter;

    // ��̨����˶�����
    gimbal_vision_control_t gimbal_vision_control;    

    //���������������
    shoot_vision_control_t shoot_vision_control;

} vision_control_t;

//�Ӿ���������ṹ��
typedef struct
{
    
    // ������Խ�ָ��
    const fp32 *vision_angle_point;
    // ���Խ�
    eular_angle_t absolution_angle;
    // ���͵ľ��Խ����ݣ�Ϊ�������Ŷ�ԭʼ��������pi����
    eular_angle_t send_absolution_angle;
    // ���ô��ڷ�������
    uint8 send_message[SERIAL_SEND_MESSAGE_SIZE];
    // ���ô�������(��Ϊ��ַ)
    UART_HandleTypeDef *send_message_usart;
    // ���ô��ڷ���dma
    DMA_HandleTypeDef *send_message_dma;

}vision_send_t;


//�Ӿ�ͨ������
void vision_send_task(void const *pvParameters);

//�Ӿ����ݴ�������
void vision_task(void const* pvParameters);

// ��ȡ��λ����̨����
gimbal_vision_control_t* get_vision_gimbal_point(void);

// ��ȡ��λ����������
shoot_vision_control_t* get_vision_shoot_point(void);

#endif // !VISION_TASK_H
