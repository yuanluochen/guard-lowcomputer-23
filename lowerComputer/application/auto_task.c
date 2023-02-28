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
#include "connect_task.h"
#include "FreeRTOS.h"
#include "task.h"

/**
 * @brief �ڱ��Զ����ƽṹ���ʼ��
 * 
 * @param auto_init �Զ����ƽṹ��ָ��
 */
static void connect_task_init(connect_control_t* auto_init);

/**
 * @brief ��λ�����ݸ��£�����λ�����ݽ������ݸ��£���Ҫ���˲������Ż��Ż���λ������
 * 
 */

//�ڱ��Զ����ƽṹ��
connect_control_t connect_control;

void connect_task(void const* pvParameters)
{
    //��ʱ�ȴ���̨���̳�ʼ�����
    vTaskDelay(CONNECT_TASK_INIT_TIME);
    // ��ʼ���Զ����ƽṹ��
    connect_task_init(&connect_control);
    while(1)
    {

        //ϵͳ��ʱ
        vTaskDelay(AUTO_CONTROL_TIME_MS); 
    }
}

static void connect_task_init(connect_control_t* auto_init)
{
    //��ȡ��ȡ��λ������ָ��
    auto_init->master_computer = get_vision_rxfifo_point(); 
}