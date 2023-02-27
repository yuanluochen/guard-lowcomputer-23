/**
 * @file connect_task.h
 * @author yuanluochen 
 * @brief 哨兵通信任务，接收上位机数据，并对上位机数据进行处理
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
 * @brief 哨兵自动控制结构体初始化
 * 
 * @param connect_init 自动控制结构体指针
 */
static void connect_task_init(connect_control_t* connect_init);



//哨兵自动控制结构体
connect_control_t connect_control;

void connect_task(void const* pvParameters)
{
    //延时等待云台底盘初始化完毕
    vTaskDelay(CONNECT_TASK_INIT_TIME);
    // 初始化自动控制结构体
    connect_task_init(&connect_control);
    while(1)
    {


        //系统延时
        vTaskDelay(CONNECT_CONTROL_TIME_MS); 
    }
}

static void connect_task_init(connect_control_t* connect_init)
{
    //读取获取上位机数据指针
    connect_init->master_computer = get_vision_rxfifo_point(); 
    
}
