/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       shoot_task.c/h
  * @brief      �������.
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-21-2022     LYH              1. ���
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  */

#include "shoot_task.h"
#include "main.h"
#include "cmsis_os.h"
#include "bsp_laser.h"
#include "bsp_fric.h"
#include "arm_math.h"

#include "user_lib.h"
#include "referee.h"
#include "CAN_receive.h"
#include "gimbal_behaviour.h"
#include "detect_task.h"
#include "pid.h"
#include "stm32.h"
/*----------------------------------�궨��---------------------------*/
#define shoot_laser_on() laser_on()                                              // ���⿪���궨��
#define shoot_laser_off() laser_off()                                            // ����رպ궨��
#define BUTTEN_TRIG_PIN HAL_GPIO_ReadPin(BUTTON_TRIG_GPIO_Port, BUTTON_TRIG_Pin) // ΢������IO
#define trigger_motor(speed) trigger_control.trigger_speed_set = speed           // �����������
/*----------------------------------�ڲ�����---------------------------*/
/**
 * @brief   ���״̬��
 */
static void Shoot_Set_Mode(void);
/**
 * @brief   ������ݸ���
 */
static void Shoot_Feedback_Update(void);
/** @brief   Ħ���ֿ���ѭ��
 */
static void fric_control_loop(fric_move_t *fric_move_control_loop);
/**
 * @brief �����̿���ѭ��
 * 
 */
static void trigger_control_loop(Shoot_Motor_t* trigger_move_control_loop);

/**
 * @brief ���÷������ģʽ
 * 
 */
static void shoot_set_control_mode(fric_move_t* fric_set_control);

/**
 * @brief ���ģʽ�л����ݹ��� 
 * 
 */
static void shoot_mode_change_transit(void);

/**
 * @brief  �����ʼ��
 */
void shoot_init(void);



/*----------------------------------�ڲ�����---------------------------*/
fp32 fric;
/*----------------------------------�ṹ��------------------------------*/
Shoot_Motor_t trigger_motor;                                  // ����������
fric_move_t fric_move;                                        // �������
/*----------------------------------�ⲿ����---------------------------*/
extern ExtY_stm32 stm32_Y_shoot;
extern ext_power_heat_data_t power_heat_data_t;//�����˵�ǰ�Ĺ���״̬����Ҫ�ж�ǹ������
/*---------------------------------------------------------------------*/
// ����ģʽ
shoot_mode_e shoot_mode = SHOOT_STOP;                             // �˴����ģʽ
shoot_mode_e last_shoot_mode = SHOOT_STOP;                        // �ϴ����ģʽ
shoot_control_mode_e shoot_control_mode = SHOOT_STOP_CONTROL;     // �������ģʽ
shoot_init_state_e shoot_init_state = SHOOT_INIT_UNFINISH;        // �����ʼ��ö����
shoot_motor_control_mode_e fric_motor_mode = SHOOT_MOTOR_STOP;    // Ħ���ֵ��
shoot_motor_control_mode_e trigger_motor_mode = SHOOT_MOTOR_STOP; // �����̵��
/**
 * @brief          ������񣬼�� GIMBAL_CONTROL_TIME 1ms
 * @param[in]      pvParameters: ��
 * @retval         none
 */
void shoot_task(void const *pvParameters)
{
    vTaskDelay(GIMBAL_TASK_INIT_TIME);
    shoot_init();
    while (1)
    {
        //���÷���ģʽ
        Shoot_Set_Mode();
        // ģʽ�л����ݹ���,��ҪPID���
        shoot_mode_change_transit();
        //�������ݸ���
        Shoot_Feedback_Update();
        //�������ѭ��
        shoot_control_loop();
        //���Ϳ��Ƶ���
        if (!(toe_is_error(TRIGGER_MOTOR_TOE) && !toe_is_error(FRIC_LEFT_MOTOR_TOE) && !toe_is_error(FRIC_RIGHT_MOTOR_TOE)))
        {
            if (toe_is_error(DBUS_TOE))
            {
                // ң��������ֹͣ����
                CAN_cmd_shoot(0, 0, 0, 0);
            }
            else
            {
                // ���Ϳ���ָ��
                CAN_cmd_shoot(fric_move.fric_CAN_Set_Current[0], fric_move.fric_CAN_Set_Current[1], trigger_motor.given_current, 0);
            }
        }
        vTaskDelay(SHOOT_TASK_DELAY_TIME);
    }
}
/**
 * @brief          �����ʼ������ʼ��PID��ң����ָ�룬���ָ��
 * @param[in]      void
 * @retval         ���ؿ�
 */
void shoot_init(void)
{
    fric_move.laster_add = 0;
    trigger_motor.move_flag = 1;
    // ��ʼ��PID
    stm32_shoot_pid_init();
    //Ħ���ֵ��PID���
    stm32_step_shoot_pid_clear();
    static const fp32 Trigger_speed_pid[3] = {900, 0, 100};
    PID_Init(&trigger_motor.motor_pid, PID_POSITION, Trigger_speed_pid, TRIGGER_READY_PID_MAX_OUT, TRIGGER_READY_PID_MAX_IOUT);
    // ����ָ���ȡ
    fric_move.shoot_rc = get_remote_control_point();
    // ��ȡ�Ӿ�����ָ��
    fric_move.shoot_vision_control = get_vision_shoot_point();
    trigger_motor.shoot_motor_measure = get_trigger_motor_measure_point();
    trigger_motor.blocking_angle_set = 0;
    fric_move.motor_fric[0].fric_motor_measure = get_shoot_motor_measure_point(0); // ��Ħ����
    fric_move.motor_fric[1].fric_motor_measure = get_shoot_motor_measure_point(1); // ��Ħ����
    // �˲���ʼ��
    const static fp32 fric_1_order_filter[1] = {0.1666666667f};
    const static fp32 fric_2_order_filter[1] = {0.1666666667f};
    first_order_filter_init(&fric_move.fric1_cmd_slow_set_speed, SHOOT_CONTROL_TIME, fric_1_order_filter);
    first_order_filter_init(&fric_move.fric2_cmd_slow_set_speed, SHOOT_CONTROL_TIME, fric_2_order_filter);
    Shoot_Feedback_Update();
}

static void shoot_mode_change_transit(void)
{
    if (last_shoot_mode != shoot_mode)
    {
        //ģʽ�����л�,pid���
        stm32_step_shoot_pid_clear();
    }
}

/**
 * @brief          ������ݸ���
 * @param[in]      void
 * @retval         void
 */
static void Shoot_Feedback_Update(void)
{
    uint8_t i;
    // �˲���������>������
    static fp32 speed_fliter_1 = 0.0f;
    static fp32 speed_fliter_2 = 0.0f;
    static fp32 speed_fliter_3 = 0.0f;
    static const fp32 fliter_num[3] = {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f};
    speed_fliter_1 = speed_fliter_2;
    speed_fliter_2 = speed_fliter_3;
    speed_fliter_3 = speed_fliter_2 * fliter_num[0] + speed_fliter_1 * fliter_num[1] + (trigger_motor.shoot_motor_measure->speed_rpm * Motor_RMP_TO_SPEED) * fliter_num[2];
    trigger_motor.speed = speed_fliter_3;
    // ���Ȧ�����ã� ��Ϊ�������תһȦ�� �������ת 36Ȧ������������ݴ������������ݣ����ڿ��������Ƕ�
    if (trigger_motor.shoot_motor_measure->ecd - trigger_motor.shoot_motor_measure->last_ecd > Half_ecd_range)
    {
        trigger_motor.ecd_count--;
    }
    else if (trigger_motor.shoot_motor_measure->ecd - trigger_motor.shoot_motor_measure->last_ecd < -Half_ecd_range)
    {
        trigger_motor.ecd_count++;
    }
    if (trigger_motor.ecd_count == FULL_COUNT)
    {
        trigger_motor.ecd_count = -(FULL_COUNT - 1);
    }
    else if (trigger_motor.ecd_count == -FULL_COUNT)
    {
        trigger_motor.ecd_count = FULL_COUNT - 1;
    }
    // ���������Ƕ�
    trigger_motor.angle = (trigger_motor.ecd_count * ecd_range + trigger_motor.shoot_motor_measure->ecd) * Motor_ECD_TO_ANGLE;
    // Ħ���֡���>�ٶȴ���
    for (i = 0; i < 2; i++)
    {
        fric_move.motor_fric[i].speed = 0.000415809748903494517209f * fric_move.motor_fric[i].fric_motor_measure->speed_rpm;
    }
}

static void shoot_set_control_mode(fric_move_t *fric_set_control)
{

    // ����ģʽ

    // �жϳ�ʼ���Ƿ����
    if (shoot_control_mode == SHOOT_INIT_CONTROL)
    {
        static uint32_t init_time = 0;
        // �жϲ����Ƿ񲦵��µ�
        if (switch_is_down(fric_set_control->shoot_rc->rc.s[SHOOT_CONTROL_CHANNEL]))
        {
            // �����µ�ֹͣ��ʼ��
            init_time = 0;
        }
        else
        {
            // �ж��Ƿ��ʼ�����
            if (shoot_init_state == SHOOT_INIT_UNFINISH)
            {
                // ��ʼ��δ���

                // �жϳ�ʼ��ʱ���Ƿ����
                if (init_time >= SHOOT_TASK_S_TO_MS(SHOOT_TASK_MAX_INIT_TIME))
                {
                    // ��ʼ��ʱ����������г�ʼ������������ģʽ
                    init_time = 0;
                }
                else
                {
                    // �ж�΢�������Ƿ��
                    if (BUTTEN_TRIG_PIN == PRESS)
                    {
                        // ����
                        // ���ó�ʼ�����
                        shoot_init_state = SHOOT_INIT_FINISH;
                        init_time = 0;
                        // ��������ģʽ
                    }
                    else
                    {
                        // ��ʼ��ģʽ����ԭ״����ʼ��ʱ������
                        init_time++;
                        return;
                    }
                }
            }
            else
            {
                // ��������ģʽ
                init_time = 0;
            }
        }
    }
    // ����ң�����������÷������ģʽ
    if (switch_is_up(fric_set_control->shoot_rc->rc.s[SHOOT_CONTROL_CHANNEL]))
    {
        // �Զ�����ģʽ
        shoot_control_mode = SHOOT_AUTO_CONTROL;
    }
    else if (switch_is_mid(fric_set_control->shoot_rc->rc.s[SHOOT_CONTROL_CHANNEL]))
    {
        // ң��������ģʽ
        shoot_control_mode = SHOOT_RC_CONTROL;
    }
    else if (switch_is_down(fric_set_control->shoot_rc->rc.s[SHOOT_CONTROL_CHANNEL]))
    {
        // ֹͣ
        shoot_control_mode = SHOOT_STOP_CONTROL;
    }
    else if (toe_is_error(DBUS_TOE))
    {
        // ң����������
        shoot_control_mode = SHOOT_STOP_CONTROL;
    }
    else
    {
        shoot_control_mode = SHOOT_STOP_CONTROL;
    }

    // ��̨��ĳЩģʽ�·���ֹͣ
    if (gimbal_cmd_to_shoot_stop())
    {
        shoot_control_mode = SHOOT_STOP_CONTROL;
    }

    // �жϽ����ʼ��ģʽ
    static shoot_control_mode_e last_shoot_control_mode = SHOOT_STOP_CONTROL;
    if (shoot_control_mode != SHOOT_STOP_CONTROL && last_shoot_control_mode == SHOOT_STOP_CONTROL)
    {
        // �����ʼ��ģʽ
        shoot_control_mode = SHOOT_INIT_CONTROL;
    }
    last_shoot_control_mode = shoot_control_mode;
}

/**
 * @brief          ���ģʽ����
 * @param[in]      void
 * @retval         ������
 */
static void Shoot_Set_Mode(void)
{
    //���÷������ģʽ
    shoot_set_control_mode(&fric_move);

    //�����ϴ����ģʽ
    last_shoot_mode = shoot_mode;

    //���µ�ǰ���ģʽ

    //�жϵ�ǰǹ�������Ƿ񼴽��������ֵ
    if (GUARD_MAX_MUZZLE_HEAT - power_heat_data_t.shooter_id1_17mm_cooling_heat >= GUARD_MAX_ALLOW_MUZZLE_HEAT_ERR0R &&
        GUARD_MAX_MUZZLE_HEAT - power_heat_data_t.shooter_id2_17mm_cooling_heat >= GUARD_MAX_ALLOW_MUZZLE_HEAT_ERR0R)
    {
        // δ�����ﵽ���ֵ������������������

        // ���ݿ���ģʽ���÷���ģʽ
        if (shoot_control_mode == SHOOT_AUTO_CONTROL)
        {
            if (fric_move.shoot_vision_control->shoot_command == SHOOT_ATTACK)
            {
                // ���÷���ģʽ����Ħ���֣�������
                shoot_mode = SHOOT_BULLET;
            }
            else
            {
                // ����״̬Ħ����һֱ����
                // ����׼������ģʽ����Ħ����
                shoot_mode = SHOOT_READY;
            }
        }
        else if (shoot_control_mode == SHOOT_RC_CONTROL)
        {
            // ��ʱ�ڱ�Ϊң��������ģʽ������ֶ�
            if (switch_is_up(fric_move.shoot_rc->rc.s[SHOOT_MODE_CHANNEL]))
            {
                shoot_mode = SHOOT_READY;
                // ���Ʒ���
                if (abs(fric_move.shoot_rc->rc.ch[4]) >= 100)
                {
                    shoot_mode = SHOOT_BULLET;
                }
            }
            else
            {
                shoot_mode = SHOOT_STOP;
            }
        }
        else if (shoot_control_mode == SHOOT_INIT_CONTROL)
        {
            // ��ʱ�ڱ���ʼ������ģʽ
            shoot_mode = SHOOT_INIT;
        }
        else if (shoot_control_mode == SHOOT_STOP_CONTROL)
        {
            shoot_mode = SHOOT_STOP;
        }
        else
        {
            shoot_mode = SHOOT_STOP;
        }
    }
    else
    {
        //�����������ֵ��ֹͣ����
        if (shoot_control_mode == SHOOT_AUTO_CONTROL)
        {
            //���Ϊ�Զ�����ģʽ��ֹͣ����
            shoot_mode = SHOOT_READY;
        }
        else if (shoot_control_mode == SHOOT_RC_CONTROL)
        {
            //���Ϊң��������ģʽ����ֹͣһ��
            shoot_mode = SHOOT_STOP;
        }
        else
        {
            shoot_mode = SHOOT_STOP;
        }
    }
}
/**
 * @brief          ������ѭ��
 * @param[in]      void
 * @retval         ������
 */
void shoot_control_loop(void)
{
    if (shoot_mode == SHOOT_BULLET)
    {
        //����Ħ���ֵ���Ͳ����̵��ת��
        fric_motor_mode = SHOOT_MOTOR_RUN;
        trigger_motor_mode = SHOOT_MOTOR_RUN;
        
    }
    else if (shoot_mode == SHOOT_READY)
    {
        // ����Ħ���ֵ��ת���������̵����ת
        fric_motor_mode = SHOOT_MOTOR_RUN;
        trigger_motor_mode = SHOOT_MOTOR_STOP;
    }
    else if (shoot_mode == SHOOT_STOP)
    {
        //����Ħ���ֵ���Ͳ����̵��ͣת
        fric_motor_mode = SHOOT_MOTOR_STOP;
        trigger_motor_mode = SHOOT_MOTOR_STOP;
    }
    else if (shoot_mode == SHOOT_INIT)
    {
       
        //�ж��Ƿ��ʼ�����
        if(shoot_init_state == SHOOT_INIT_UNFINISH)
        {
            //��ʼ��δ���
            //΢�������Ƿ�����
            if (BUTTEN_TRIG_PIN == RELEASE)
            {
                shoot_init_state = SHOOT_INIT_UNFINISH;
                //δ����,���ò�����ת��,Ħ����ͣת
                fric_motor_mode = SHOOT_MOTOR_STOP;
                trigger_motor_mode = SHOOT_MOTOR_RUN;
            }
            else
            {
                //����
                shoot_init_state = SHOOT_INIT_FINISH;
                //���ͣת
                fric_motor_mode = SHOOT_MOTOR_STOP;
                trigger_motor_mode = SHOOT_MOTOR_STOP;
            }
        }
        else
        {
            shoot_init_state = SHOOT_INIT_FINISH;
            // ���ͣת
            fric_motor_mode = SHOOT_MOTOR_STOP;
            trigger_motor_mode = SHOOT_MOTOR_STOP;
        }
    }


    //���ݵ��ģʽ���õ��ת���ٶ�
    switch (fric_motor_mode)
    {
    case SHOOT_MOTOR_RUN:
        //���õ��ת��
        fric = FRIC_MOTOR_RUN_SPEED;
        break;
    case SHOOT_MOTOR_STOP:
        //���õ��ͣת
        fric = FRIC_MOTOR_STOP_SPEED;
        break;
    }
    
    switch (trigger_motor_mode)
    {
    case SHOOT_MOTOR_RUN:
        trigger_motor.speed_set = TRIGGER_MOTOR_RUN_SPEED;
        break;
    case SHOOT_MOTOR_STOP:
        trigger_motor.speed_set = TRIGGER_MOTOR_STOP_SPEED;
        break;
    }

    // pid����
    fric_control_loop(&fric_move);        // Ħ���ֿ���
    trigger_control_loop(&trigger_motor); // �����̿���
}

/**
 * @brief  Ħ���ֿ���ѭ��
 */
static void fric_control_loop(fric_move_t *fric_move_control_loop)
{
    // �ٶ�����
    fric_move_control_loop->speed_set[0] = fric;
    fric_move_control_loop->speed_set[1] = -fric;
    // pid ����
    stm32_step_shoot_0(fric_move_control_loop->speed_set[0], fric_move_control_loop->motor_fric[0].speed);
    stm32_step_shoot_1(fric_move_control_loop->speed_set[1], fric_move_control_loop->motor_fric[1].speed);
    fric_move.fric_CAN_Set_Current[0] = stm32_Y_shoot.out_shoot_0;
    fric_move.fric_CAN_Set_Current[1] = stm32_Y_shoot.out_shoot_1;
}


static void trigger_control_loop(Shoot_Motor_t* trigger_move_control_loop)
{
    trigger_move_control_loop->motor_pid.max_out = TRIGGER_BULLET_PID_MAX_OUT;
    trigger_move_control_loop->motor_pid.max_iout = TRIGGER_BULLET_PID_MAX_IOUT;
    PID_Calc(&trigger_move_control_loop->motor_pid, trigger_move_control_loop->speed, trigger_move_control_loop->speed_set); // ������
    trigger_move_control_loop->given_current = (int16_t)(trigger_move_control_loop->motor_pid.out);
}


//��������Ӿ�
bool_t shoot_control_vision_task(void)
{
    if (shoot_mode == SHOOT_INIT || shoot_control_mode == SHOOT_INIT || toe_is_error(DBUS_TOE))
    {
        //�����ʼ��ģʽ��ң�������źţ�ֹͣ����
        return 0;
    }   
    else
    {
        return 1;
    }
}
