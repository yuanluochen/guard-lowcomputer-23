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
#include "rm_usart.h"

//�������Ƕ����
#define ALLOW_ATTACK_ERROR 7

//�����жϼ�������
#define JUDGE_ATTACK_COUNT 2
//����ֹͣ�жϼ�������
#define JUDGE_STOP_ATTACK_COUNT 100

//��ʱ�ȴ�
#define VISION_SEND_TASK_INIT_TIME 401
//ϵͳ��ʱʱ��
#define VISION_SEND_CONTROL_TIME_MS 2

//��ʱ�ȴ�
#define VISION_TASK_INIT_TIME 450
//ϵͳ��ʱʱ��
#define VISION_CONTROL_TIME_MS 2

//�ӱ�
#define DOUBLE_ 10000
//������ת�Ƕ���
#define RADIAN_TO_ANGLE (360 / (2 * PI))


//���ݷ���uint8_t������Ĵ�С
#define UINT8_T_DATA_SIZE 4

//����λ����������Ϊ�����������pi����
#define SEND_MESSAGE_ERROR PI

//���δ���ܵ���λ�����ݵ�ʱ��
#define MAX_UNRX_TIME 400

//����β֡����
#define END_DATA 0x0A

//������ʼ֡����
typedef enum
{
    //��λ�����͵���λ��
    LOWER_TO_HIGH_HEAD = 0x34,
    //��λ�����͵���λ��
    HIGH_TO_LOWER_HEAD = 0X44,
}data_head_type_e;


//��λ��ģʽ,����װ�װ�ģʽ,��������ģʽ,ǰ��վģʽ
typedef enum
{
    ARMOURED_PLATE_MODE = 1, //װ�װ�ģʽ
    ENERGY_ORGAN_MODE = 2,   //��������
    OUTPOST_MODE = 3,        //ǰ��վ
}highcomputer_mode_e;

//��������ƫ����
enum
{
    HEAD_ADDRESS_OFFSET = 0,                                                   // ��ʼ֡
    DATA_QUAT_REAL_ADDRESS_OFFSET,                                             // ���ݶ� ��Ԫ��ʵ��
    DATA_QUAT_X_ADDRESS_OFFSET = DATA_QUAT_REAL_ADDRESS_OFFSET + sizeof(fp32), // ���ݶ� ��Ԫ��x���鲿
    DATA_QUAT_Y_ADDRESS_OFFSET = DATA_QUAT_X_ADDRESS_OFFSET + sizeof(fp32),    // ���ݶ� ��Ԫ��y���鲿
    DATA_QUAT_Z_ADDRESS_OFFSET = DATA_QUAT_Y_ADDRESS_OFFSET + sizeof(fp32),    // ���ݶ� ��Ԫ��z���鲿
    MODE_SWITCH_ADDRESS_OFFSET = DATA_QUAT_Z_ADDRESS_OFFSET + sizeof(fp32),    // ģʽ�л���
    CHECK_BIT_ADDRESS_OFFSET,                                                  // У���
    END_ADDRESS_OFFSET,                                                        // β֡
    SUM_SEND_MESSAGE,                                                          // ��ֵ
};



//����λ��ͨ������
typedef struct 
{
    //��ʼ֡
    uint8_t head;
    //���ݶ�-��Ԫ����
    fp32 quat[4];
    //ģʽ�л���
    uint8_t mode_change;
    //У���
    uint8_t check;
    //β
    uint8_t end;
}communication_data_t;


typedef struct 
{
    fp32 yaw;
    fp32 pitch;
    fp32 roll;
}eular_angle_t;

//�ڱ���̨����˶�����,���˲���������ֵ
typedef struct 
{
    //������̨yaw����ֵ
    fp32 gimbal_yaw;
    //������̨pitch����ֵ
    fp32 gimbal_pitch;
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

//�Ӿ�����ṹ��
typedef struct
{
    // ��λ���Ӿ�ָ��
    vision_rxfifo_t *vision_rxfifo;
    
    // ������Խ�ָ��
    const fp32* vision_angle_point;
    
    // ���Խ�
    eular_angle_t absolution_angle;

    // ��̨����˶�����
    gimbal_vision_control_t gimbal_vision_control;    

    //���������������
    shoot_vision_control_t shoot_vision_control;

} vision_control_t;

//�Ӿ���������ṹ��
typedef struct
{
    // ��Ԫ��ָ��
    const fp32* vision_quat_point;
    //�������ݽṹ��
    communication_data_t send_msg_struct;
    // ���ô��ڷ�������
    uint8 send_message[SUM_SEND_MESSAGE];
} vision_send_t;

//�Ӿ���������ṹ��
typedef struct 
{
    //�������ݽṹ��
    communication_data_t receive_msg_struct;
    //���յ����ݱ�־λ
    bool_t rx_flag;
}vision_receive_t;


//�Ӿ�ͨ������
void vision_send_task(void const *pvParameters);

//�Ӿ����ݴ�������
void vision_task(void const* pvParameters);

//��λ��ԭʼ���ݽ���
void highcomputer_rx_decode(uint8_t* rx_Buf, uint32_t* rx_buf_Len);

// ��ȡ��λ����̨����
gimbal_vision_control_t* get_vision_gimbal_point(void);

// ��ȡ��λ����������
shoot_vision_control_t* get_vision_shoot_point(void);

/**
 * @brief �ж�δ���յ���λ������
 * 
 * @return ����1 δ���յ��� ����0 ���յ� 
 */
bool_t judge_not_rx_vision_data(void);

#endif // !VISION_TASK_H
