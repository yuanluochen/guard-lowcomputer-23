#ifndef SHOOT_TASK_H
#define SHOOT_TASK_H


#include "struct_typedef.h"

#include "CAN_receive.h"
#include "gimbal_task.h"
#include "remote_control.h"
#include "user_lib.h"
#include "vision_task.h"

//���ң��������Ħ����ͨ��
#define SHOOT_MODE_CHANNEL 1
//���ң�������Ʋ�����ͨ��
#define SHOOT_TRIGGER_CHANNEL 1
//���ң��������������Ʒ�ʽ��ͨ��
#define SHOOT_CONTROL_CHANNEL 0

//����������ʱʱ�� 1ms
#define SHOOT_TASK_DELAY_TIME 1

// ��������ʱ��ת�� ��ת����
#define SHOOT_TASK_S_TO_MS(x) ((int32_t)((x * 1000.0f) / (SHOOT_TASK_DELAY_TIME)))

//�����������ʱ�䣬����Ϊ��λ 20 s
#define SHOOT_TASK_MAX_INIT_TIME 10

//Ħ���ֵ��ת��
#define FRIC_MOTOR_RUN_SPEED 2.9
//Ħ���ֵ��ֹͣת��
#define FRIC_MOTOR_STOP_SPEED 0

//�����̵��ת��
#define TRIGGER_MOTOR_RUN_SPEED 10.0
//�����̵��ͣת
#define TRIGGER_MOTOR_STOP_SPEED 0

#define GIMBAL_ModeChannel  1

#define SHOOT_CONTROL_TIME  0.02

#define RC_S_LONG_TIME 2000

#define PRESS_LONG_TIME 400

#define SHOOT_ON_KEYBOARD KEY_PRESSED_OFFSET_Q
#define SHOOT_OFF_KEYBOARD KEY_PRESSED_OFFSET_E


#define MAX_SPEED 15.0f //-12.0f
#define MID_SPEED 12.0f //-12.0f
#define MIN_SPEED 10.0f //-12.0f
#define Ready_Trigger_Speed 6.0f

#define Motor_RMP_TO_SPEED 0.00290888208665721596153948461415f
#define Motor_ECD_TO_ANGLE 0.000021305288720633905968306772076277f
#define FULL_COUNT 18

#define TRIGGER_ANGLE_PID_KP 900///2450.0f
#define TRIGGER_ANGLE_PID_KI 0.0f
#define TRIGGER_ANGLE_PID_KD 100.0f

#define TRIGGER_READY_PID_MAX_OUT 8000.0f
#define TRIGGER_READY_PID_MAX_IOUT 2500.0f

#define TRIGGER_BULLET_PID_MAX_OUT 15000.0f
#define TRIGGER_BULLET_PID_MAX_IOUT 5000.0f

#define Half_ecd_range 395  //395  7796
#define ecd_range 8191

#define S3505_MOTOR_SPEED_PID_KP 8700.f
#define S3505_MOTOR_SPEED_PID_KI  0.0f
#define S3505_MOTOR_SPEED_PID_KD  10.f
#define S3505_MOTOR_SPEED_PID_MAX_OUT 11000.0f
#define S3505_MOTOR_SPEED_PID_MAX_IOUT 1000.0f

#define PI_Four 0.78539816339744830961566084581988f
#define PI_Three 1.0466666666666f
#define PI_Ten 0.314f

#define TRIGGER_SPEED 3.0f
#define SWITCH_TRIGGER_ON 0

typedef enum
{
    SHOOT_STOP = 0,
    SHOOT_READY,
    SHOOT_BULLET,
    SHOOT_BULLET_ONE,
    SHOOT_DONE,
    SHOOT_INIT, //��ʼ��ģʽ
} shoot_mode_e;

//�������ģʽ
typedef enum
{
    SHOOT_MOTOR_RUN,  // �������
    SHOOT_MOTOR_STOP, // ���ֹͣ
} shoot_motor_control_mode_e;


typedef enum
{
    SHOOT_AUTO_CONTROL,    //�Զ�����ģʽ
    SHOOT_RC_CONTROL,      //ң��������ģʽ
    SHOOT_INIT_CONTROL,    //��ʼ������ģʽ
    SHOOT_STOP_CONTROL,    //ֹͣ����ģʽ
}shoot_control_mode_e;

typedef struct
{
    //PID�ṹ��

    PidTypeDef motor_pid;

    const motor_measure_t *shoot_motor_measure;
    fp32 speed;
    fp32 speed_set;
    fp32 angle;
    int8_t ecd_count;
    fp32 set_angle;
    int16_t given_current;

    bool_t press_l;
    bool_t press_r;
    bool_t last_press_l;
    bool_t last_press_r;
    uint16_t press_l_time;
    uint16_t press_r_time;
    uint16_t rc_s_time;

    uint32_t run_time;
    uint32_t cmd_time;
    int16_t move_flag;
    int16_t move_flag_ONE;
    bool_t key;
    int16_t BulletShootCnt;
    int16_t last_butter_count;
    fp32 shoot_CAN_Set_Current;
    fp32 blocking_angle_set;
    fp32 blocking_angle_current;
    int8_t blocking_ecd_count;
} Shoot_Motor_t;

typedef struct
{
    const motor_measure_t *fric_motor_measure;
    fp32 accel;
    fp32 speed;
    fp32 speed_set;
    int16_t give_current;
    uint16_t rc_key_time;
} fric_Motor_t;

//΢������״̬
typedef enum
{
    PRESS = GPIO_PIN_RESET,   // ����
    RELEASE = GPIO_PIN_SET, // �ɿ�
} micro_switch_state_e;

//��ʼ��״̬
typedef enum
{
    SHOOT_INIT_FINISH,   // ��ʼ�����
    SHOOT_INIT_UNFINISH, // ��ʼ��δ���
} shoot_init_state_e;

typedef struct
{
    const RC_ctrl_t *shoot_rc;            // ң����
    const shoot_vision_control_t *shoot_vision_control; // �Ӿ�����ָ��

    shoot_mode_e fric_mode;               // ����ģʽ
    shoot_mode_e last_fric_mode;          // ��һ�εķ���ģʽ
    fric_Motor_t motor_fric[2];           // ����Ħ����
    fp32 fric_CAN_Set_Current[2];         // can�������
    PidTypeDef motor_speed_pid[4];        // ����ٶ�pid


    first_order_filter_type_t fric1_cmd_slow_set_speed; // Ħ����һ�׵�ͨ
    first_order_filter_type_t fric2_cmd_slow_set_speed; // Ħ����һ�׵�ͨ

    fp32 angle[2];
    int16_t ecd_count[2];
    int16_t given_current[2];
    fp32 set_angle[2];
    fp32 speed[2];
    fp32 speed_set[2];
    fp32 current_set[2];
    bool_t move_flag;

    fp32 min_speed;
    fp32 max_speed;
    int flag[2];
    int laster_add;
    
} fric_move_t;

extern void shoot_init(void);
extern void shoot_control_loop(void);



/**
 * @brief ���������
 * 
 * @param pvParameters 
 */
void shoot_task(void const *pvParameters);

//��������Ӿ�����
bool_t shoot_control_vision_task(void);


#endif
