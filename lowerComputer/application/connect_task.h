/**
 * @file connect_task.h
 * @author yuanluochen 
 * @brief �ڱ�ͨ�����񣬽�����λ�����ݣ�������λ�����ݽ��д���
 * @version 0.1
 * @date 2023-02-21
 * 
 * @copyright Copyright (c) 2023
 * 
 */
#ifndef CONNECT_TASK_H
#define CONNECT_TASK_H

#include "rm_usart.h"
#include "kalman.h"

//��ʼ����ʱʱ��
#define CONNECT_TASK_INIT_TIME 150
//ϵͳ��ʱ
#define CONNECT_CONTROL_TIME_MS 2

//����ģʽ
typedef enum
{
    fail,
}connect_mode_t;

//�����˶����ݽṹ��
typedef struct 
{
    fp32 vx_set;
    fp32 vy_set;
    fp32 angel_set;
}chassis_move_cmd_t;

//��̨�˶����ݽṹ��
typedef struct 
{
    fp32 pitch_add_angle;
    fp32 yaw_add_angle;

    //kalman filer �ṹ��
    kalman pitch_kalman_filter;//pitch����kalman filter�ṹ��
    

}gimbal_movd_cmd_t;



//�ڱ��Զ�ģʽ�ṹ��
typedef struct 
{
    //�����˶�����
    chassis_move_cmd_t chassis_cmd;
    //��̨�˶�����
    gimbal_movd_cmd_t gimbal_cmd;

    //��λ��ָ��
    const vision_rxfifo_t* master_computer;


}connect_control_t;


void connect_task(void const* pvParameters);


#endif // !CONNECT_TASK_H
