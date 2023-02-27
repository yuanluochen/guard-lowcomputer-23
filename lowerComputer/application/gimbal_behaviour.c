/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       gimbal_task.c/h
  * @brief          
  *             �����̨��������������̨ʹ�������ǽ�����ĽǶȣ��䷶Χ�ڣ�-pi,pi��
  *             �ʶ�����Ŀ��ǶȾ�Ϊ��Χ���������ԽǶȼ���ĺ�������̨��Ҫ��Ϊ2��
  *             ״̬�������ǿ���״̬�����ð��������ǽ������̬�ǽ��п��ƣ�����������
  *             ״̬��ͨ����������ı���ֵ���Ƶ�У׼�����⻹��У׼״̬��ֹͣ״̬�ȡ�
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *
  @verbatim
  ==============================================================================
        
    ���Ҫ���һ���µ���Ϊģʽ
    1.���ȣ���gimbal_behaviour.h�ļ��У� ���һ������Ϊ������ gimbal_behaviour_e
    erum
    {  
        ...
        ...
        GIMBAL_XXX_XXX, // ����ӵ�
    }gimbal_behaviour_e,

    2. ʵ��һ���µĺ��� gimbal_xxx_xxx_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);
        "yaw, pitch" ��������̨�˶�����������
        ��һ������: 'yaw' ͨ������yaw���ƶ�,ͨ���ǽǶ�����,��ֵ����ʱ���˶�,��ֵ��˳ʱ��
        �ڶ�������: 'pitch' ͨ������pitch���ƶ�,ͨ���ǽǶ�����,��ֵ����ʱ���˶�,��ֵ��˳ʱ��
        ������µĺ���, ���ܸ� "yaw"��"pitch"��ֵ��Ҫ�Ĳ���
    3.  ��"gimbal_behavour_set"��������У�����µ��߼��жϣ���gimbal_behaviour��ֵ��GIMBAL_XXX_XXX
        ��gimbal_behaviour_mode_set����������"else if(gimbal_behaviour == GIMBAL_XXX_XXX)" ,Ȼ��ѡ��һ����̨����ģʽ
        3��:
        GIMBAL_MOTOR_RAW : ʹ��'yaw' and 'pitch' ��Ϊ��������趨ֵ,ֱ�ӷ��͵�CAN������.
        GIMBAL_MOTOR_ENCONDE : 'yaw' and 'pitch' �ǽǶ�����,  ���Ʊ�����ԽǶ�.
        GIMBAL_MOTOR_GYRO : 'yaw' and 'pitch' �ǽǶ�����,  ���������Ǿ��ԽǶ�.
    4.  ��"gimbal_behaviour_control_set" ������������
        else if(gimbal_behaviour == GIMBAL_XXX_XXX)
        {
            gimbal_xxx_xxx_control(&rc_add_yaw, &rc_add_pit, gimbal_control_set);
        }
  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#include "gimbal_behaviour.h"
#include "arm_math.h"
#include "bsp_buzzer.h"
#include "detect_task.h"
#include "chassis_behaviour.h"
#include "user_lib.h"

#define int_abs(x) ((x) > 0 ? (x) : (-x))

/**
  * @brief          ң�����������жϣ���Ϊң�����Ĳ�������λ��ʱ�򣬲�һ��Ϊ0��
  */
#define rc_deadband_limit(input, output, dealine)        \
    {                                                    \
        if ((input) > (dealine) || (input) < -(dealine)) \
        {                                                \
            (output) = (input);                          \
        }                                                \
        else                                             \
        {                                                \
            (output) = 0;                                \
        }                                                \
    }


/**
  * @brief          ͨ���жϽ��ٶ����ж���̨�Ƿ񵽴Ｋ��λ��
  */
#define gimbal_cali_gyro_judge(gyro, cmd_time, angle_set, angle, ecd_set, ecd, step) \
    {                                                                                \
        if ((gyro) < GIMBAL_CALI_GYRO_LIMIT)                                         \
        {                                                                            \
            (cmd_time)++;                                                            \
            if ((cmd_time) > GIMBAL_CALI_STEP_TIME)                                  \
            {                                                                        \
                (cmd_time) = 0;                                                      \
                (angle_set) = (angle);                                               \
                (ecd_set) = (ecd);                                                   \
                (step)++;                                                            \
            }                                                                        \
        }                                                                            \
    }
/*----------------------------------�ڲ�����---------------------------*/
/**
  * @brief          pitch���˲�.
  */
void Fiter(fp32 pitch);   
/**
  * @brief          ��̨��Ϊ״̬������.
  * @param[in]      gimbal_mode_set: ��̨����ָ��
  * @retval         none
  */
static void gimbal_behavour_set(gimbal_control_t *gimbal_mode_set);
/**
  * @brief          ����̨��Ϊģʽ��GIMBAL_ZERO_FORCE, ��������ᱻ����,��̨����ģʽ��rawģʽ.ԭʼģʽ��ζ��
  *                 �趨ֵ��ֱ�ӷ��͵�CAN������,�������������������Ϊ0.
  */
static void gimbal_zero_force_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);

/**
  * @brief          ��̨��ʼ�����ƣ�����������ǽǶȿ��ƣ���̨��̧��pitch�ᣬ����תyaw��
  */
static void gimbal_init_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);

/**
  * @brief          ��̨У׼���ƣ������raw���ƣ���̨��̧��pitch������pitch������תyaw�����תyaw����¼��ʱ�ĽǶȺͱ���ֵ
  */
static void gimbal_cali_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);

/**
  * @brief          ��̨���ƣ�����ǽǶȿ��ƣ�
  * @param[out]     yaw: yaw��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
  * @param[out]     pitch:pitch��Ƕȿ��ƣ�Ϊ�Ƕȵ����� ��λ rad
  * @param[in]      gimbal_control_set:��̨����ָ��
  */
static void gimbal_RC_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);
/**
  * @brief          ��̨����ң������������ƣ��������ԽǶȿ��ƣ�
  * @author         RM
  */
static void gimbal_motionless_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set);

#if GIMBAL_AUTO_MODE
/**
 * @brief                     ��̨�����Զ�ģʽ����̨��̬���Ӿ���λ�����ƣ�����Ǿ��ԽǶȿ���
 * 
 * @param yaw                 yaw��Ƕ�����
 * @param pitch               pitch��Ƕ�����
 * @param gimbal_control_set  ��̨����ָ��
 * @author                    yuanluochen
 */
static void gimbal_auto_control(fp32* yaw, fp32* pitch, gimbal_control_t* gimbal_control_set);
#endif

/**
 * @brief                     ��̨���п���ģʽ����ʱ��̨���ھ��ԽǶȿ��ƻ���ԽǶȿ���
 *
 * @param yaw                 yaw��Ƕ�����
 * @param pitch               pitch��Ƕ�����
 * @param gimbal_control_set  ��̨����ָ��
 * @author                    yuanluochen
 */
static void gimbal_run_control(fp32* yaw, fp32* pitch, gimbal_control_t* gimbal_control_set);


/*----------------------------------�ṹ��---------------------------*/
//��̨��Ϊ״̬��
static gimbal_behaviour_e gimbal_behaviour = GIMBAL_ZERO_FORCE;
/*----------------------------------�ڲ�����---------------------------*/
int yaw_flag=0;  
fp32 Pitch_Set[8]={0};
/*----------------------------------�ⲿ����---------------------------*/
extern chassis_behaviour_e chassis_behaviour_mode;

/**
  * @brief          gimbal_set_mode����������gimbal_task.c,��̨��Ϊ״̬���Լ����״̬������
  * @param[out]     gimbal_mode_set: ��̨����ָ��
  * @retval         none
  */

void gimbal_behaviour_mode_set(gimbal_control_t *gimbal_mode_set)
{
    if (gimbal_mode_set == NULL)
    {
        return;
    }
    //set gimbal_behaviour variable
    //��̨��Ϊ״̬������
    gimbal_behavour_set(gimbal_mode_set);

    //accoring to gimbal_behaviour, set motor control mode
    //������̨��Ϊ״̬�����õ��״̬��

#if 1

    switch (gimbal_behaviour)
    {
    //����ģʽ��,����Ϊ���ԭʼֵ���ƣ������õ����������̬
    case GIMBAL_ZERO_FORCE:
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_RAW;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_RAW;
        break;
        
    //��ԽǶȿ��ƺ�ң�����ͳ�ʼ��ģʽ�Լ��Զ�ģʽ������һ�ֿ���ģʽ
    case GIMBAL_RC:
    case GIMBAL_RELATIVE_ANGLE: 
    case GIMBAL_INIT:
#if GIMBAL_AUTO_MODE
    case GIMBAL_AUTO:
#endif
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_GYRO;      // yaw��ͨ�������ǵľ��Խǿ���
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_GYRO;    // pitch��ͨ�������ǵľ��Խǿ���
        break;

    case GIMBAL_MOTIONLESS:
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
        break;

    }

#else //ԭʼ�汾����
    if (gimbal_behaviour == GIMBAL_ZERO_FORCE)
    {
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_RAW;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_RAW;
    }
    else if (gimbal_behaviour == GIMBAL_ABSOLUTE_ANGLE)
    {
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_GYRO;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_GYRO;
    }
    else if (gimbal_behaviour == GIMBAL_RELATIVE_ANGLE)
    {
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_GYRO;
    }
    else if (gimbal_behaviour == GIMBAL_MOTIONLESS)
    {
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
    }
#if GIMBAL_AUTO_MODE
    else if (gimbal_behaviour == GIMBAL_AUTO)//����ģʽ�£���̨�������Ϊ���ԽǶȿ���
    {
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_GYRO;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_GYRO;
    }
#endif
#endif
}

/**
 * @brief                          (�޸İ�)��̨��Ϊģʽ���ƣ���̨pitch�������ԽǶȿ��ƣ���̨yaw����þ��ԽǶȿ���,ԭ��Ϊpitch���Խ���������
 * 
 * @param add_yaw                  yaw ��ĽǶ����� Ϊָ��
 * @param add_pitch                pitch ��ĽǶ����� Ϊָ�� 
 * @param gimbal_control_set       ��̨�ṹ��ָ�� 
 */
void gimbal_behaviour_control_set(fp32 *add_yaw, fp32 *add_pitch, gimbal_control_t *gimbal_control_set)
{
#if 1
//�޸İ�Ĵ���,pitch��ԽǶȿ��ƣ� yaw���ԽǶȿ���
    if (add_yaw == NULL || add_pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }
    //�жϵ���ģʽ�����ݵ���ģʽѡ����̿��Ʒ�ʽ
    switch(gimbal_behaviour)
    {
    case GIMBAL_ZERO_FORCE://����ģʽ��̨����
        gimbal_zero_force_control(add_yaw, add_pitch, gimbal_control_set);
        break;
    case GIMBAL_INIT:      //��ʼ��ģʽ����̨��ʼ��
        gimbal_init_control(add_yaw, add_pitch, gimbal_control_set);
        break;
    case GIMBAL_CALI:      //У׼ģʽ����̨У׼
        gimbal_cali_control(add_yaw, add_pitch, gimbal_control_set);
        break;
    case GIMBAL_RC: //��ԽǶȿ��ƺ;��ԽǶȿ��ƶ�����ͳһ�Ŀ��Ʒ�ʽ����pitch����ԽǶȿ��ƣ�yaw����ԽǶȿ���
    case GIMBAL_RELATIVE_ANGLE:
        gimbal_run_control(add_yaw, add_pitch, gimbal_control_set); 
        break;

    case GIMBAL_MOTIONLESS: //���ź��µĿ���,������
        gimbal_motionless_control(add_yaw, add_pitch, gimbal_control_set);
        break;

#if GIMBAL_AUTO_MODE
    case GIMBAL_AUTO:       //�Զ�ģʽ����λ������
        gimbal_auto_control(add_yaw, add_pitch, gimbal_control_set);
        break;
#endif
    
    }

#else //ԭ�����
    if (add_yaw == NULL || add_pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }

    if (gimbal_behaviour == GIMBAL_ZERO_FORCE)
    {
        gimbal_zero_force_control(add_yaw, add_pitch, gimbal_control_set);
    }
    else if (gimbal_behaviour == GIMBAL_INIT)
    {
        gimbal_init_control(add_yaw, add_pitch, gimbal_control_set);
    }
    else if (gimbal_behaviour == GIMBAL_CALI)
    {
        gimbal_cali_control(add_yaw, add_pitch, gimbal_control_set);
    }
    else if (gimbal_behaviour == GIMBAL_ABSOLUTE_ANGLE)
    {
        gimbal_RC_control(add_yaw, add_pitch, gimbal_control_set);
    }
    else if (gimbal_behaviour == GIMBAL_RELATIVE_ANGLE)
    {
        // gimbal_RC_control(add_yaw, add_pitch, gimbal_control_set);
        gimbal_relative_angle_control(add_yaw, add_pitch, gimbal_control_set);
    }
    else if (gimbal_behaviour == GIMBAL_MOTIONLESS)
    {
        gimbal_motionless_control(add_yaw, add_pitch, gimbal_control_set);
    }
#if GIMBAL_AUTO_MODE
    else if (gimbal_behaviour == GIMBAL_AUTO_MODE)
    {
        gimbal_auto_control(add_yaw, add_pitch, gimbal_control_set);            
    }
#endif
#endif
}

/**
  * @brief          ��̨��ĳЩ��Ϊ�£���Ҫ���̲���
  * @param[in]      none
  * @retval         1: no move 0:normal
  */

bool_t gimbal_cmd_to_chassis_stop(void)
{
    if (gimbal_behaviour == GIMBAL_INIT || gimbal_behaviour == GIMBAL_CALI || gimbal_behaviour == GIMBAL_MOTIONLESS || gimbal_behaviour == GIMBAL_ZERO_FORCE)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

/**
  * @brief          ��̨��ĳЩ��Ϊ�£���Ҫ���ֹͣ
  * @param[in]      none
  * @retval         1: no move 0:normal
  */

bool_t gimbal_cmd_to_shoot_stop(void)
{
    if (gimbal_behaviour == GIMBAL_INIT || gimbal_behaviour == GIMBAL_CALI || gimbal_behaviour == GIMBAL_ZERO_FORCE)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

/**
  * @brief          ��̨��Ϊ״̬������.
  * @param[in]      gimbal_mode_set: ��̨����ָ��
  * @retval         none
  */
static void gimbal_behavour_set(gimbal_control_t *gimbal_mode_set)
{
    if (gimbal_mode_set == NULL)
    {
        return;
    }
#if GIMBAL_AUTO_MODE
    if (switch_is_down(gimbal_mode_set->gimbal_rc_ctrl->rc.s[GIMBAL_MODE_CHANNEL]) && chassis_behaviour_mode != CHASSIS_NO_MOVE)
    {
        //ң��������ģʽ
        gimbal_behaviour = GIMBAL_RC;
    }
    if (switch_is_mid(gimbal_mode_set->gimbal_rc_ctrl->rc.s[GIMBAL_MODE_CHANNEL]) && chassis_behaviour_mode != CHASSIS_NO_MOVE)
    {
        //�л���ң��������ģʽ
        gimbal_behaviour = GIMBAL_RC;
    }
    else if (switch_is_up(gimbal_mode_set->gimbal_rc_ctrl->rc.s[GIMBAL_MODE_CHANNEL]) && chassis_behaviour_mode != CHASSIS_NO_MOVE)
    {
        //�л�����̨�Զ�ģʽ
        gimbal_behaviour = GIMBAL_AUTO;
    }
    else if (!toe_is_error(DBUS_TOE) && chassis_behaviour_mode == CHASSIS_NO_MOVE)
    {
        gimbal_behaviour = GIMBAL_RELATIVE_ANGLE;
    }
    if (toe_is_error(DBUS_TOE))
    {
        gimbal_behaviour = GIMBAL_ZERO_FORCE;
    }

    // //�жϽ����ʼ��״̬��,����������״̬��������ģʽתΪ����ģʽ
    // static gimbal_behaviour_e last_gimbal_behaviour = GIMBAL_ZERO_FORCE;
    // if (last_gimbal_behaviour == GIMBAL_ZERO_FORCE && gimbal_behaviour != GIMBAL_ZERO_FORCE)
    // {
    //     gimbal_behaviour = GIMBAL_INIT;
    // }
    // last_gimbal_behaviour = gimbal_behaviour;

#else
    //���ؿ��� ��̨״̬
    if (switch_is_down(gimbal_mode_set->gimbal_rc_ctrl->rc.s[0])&&chassis_behaviour_mode!=CHASSIS_NO_MOVE)
    {
        gimbal_behaviour = GIMBAL_ZERO_FORCE;
    }
    else if (switch_is_mid(gimbal_mode_set->gimbal_rc_ctrl->rc.s[0])&&chassis_behaviour_mode!=CHASSIS_NO_MOVE)
    {
        gimbal_behaviour = GIMBAL_ABSOLUTE_ANGLE;
    }
    else if (switch_is_up(gimbal_mode_set->gimbal_rc_ctrl->rc.s[0])&&chassis_behaviour_mode!=CHASSIS_NO_MOVE)
    {
        gimbal_behaviour = GIMBAL_ABSOLUTE_ANGLE;
    }
	else if ( !toe_is_error(DBUS_TOE)&&chassis_behaviour_mode == CHASSIS_NO_MOVE)
    {
        gimbal_behaviour = GIMBAL_RELATIVE_ANGLE;
    }
    if( toe_is_error(DBUS_TOE))
    {
        gimbal_behaviour = GIMBAL_ZERO_FORCE;
    }
		
#endif
}
/**
  * @brief          ����̨��Ϊģʽ��GIMBAL_ZERO_FORCE, ��������ᱻ����,��̨����ģʽ��rawģʽ.ԭʼģʽ��ζ��
  *                 �趨ֵ��ֱ�ӷ��͵�CAN������,�������������������Ϊ0.
  */
static void gimbal_zero_force_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }

    *yaw = 0.0f;
    *pitch = 0.0f;
}
/**
 * @brief                     ��̨���п���ģʽ����ʱ��̨��ң�������ƣ�pitch��Ϊ���ԽǶȿ��ƣ�yaw��Ϊ���ԽǶȿ���
 *
 * @param yaw                 yaw��Ƕ�����
 * @param pitch               pitch��Ƕ�����
 * @param gimbal_control_set  ��̨����ָ��
 * @author                    yuanluochen
 */
static void gimbal_run_control(fp32* yaw, fp32* pitch, gimbal_control_t* gimbal_control_set)
{
    gimbal_RC_control(yaw, pitch, gimbal_control_set);
}

/**
  * @brief          ��̨���ƣ�����ǽǶȿ��ƣ�
  */

static void gimbal_RC_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
{
    static fp32 rc_add_yaw, rc_add_pit;
    static fp32 rc_add_yaw_RC, rc_add_pit_RC;
    fp32 rc_add_z = 0;
    static int16_t yaw_channel = 0, pitch_channel = 0;
    static int16_t yaw_channel_RC;
    static int16_t pitch_channel_RC;
    fp32 yaw_turn = 0;
    fp32 yaw_turn1 = 0;
    fp32 pitch_turn = 0;
    if (yaw_flag == 0)
    {
        if (gimbal_control_set->gimbal_rc_ctrl->key.v & KEY_PRESSED_OFFSET_X)
        {
            yaw_turn = 3.1415926f;
            yaw_flag = 1;
        }
    }
    if ((gimbal_control_set->gimbal_rc_ctrl->key.v & KEY_PRESSED_OFFSET_X) == 0)
    {
        yaw_flag = 0;
    }
    if (gimbal_control_set->gimbal_rc_ctrl->key.v & KEY_PRESSED_OFFSET_Q)
    {
        yaw_turn1 = 0.0025f;
    }
    else if (gimbal_control_set->gimbal_rc_ctrl->key.v & KEY_PRESSED_OFFSET_E)
    {
        yaw_turn1 = -0.0025f;
    }
    // ң��������
    rc_deadband_limit(gimbal_control_set->gimbal_rc_ctrl->rc.ch[YAW_CHANNEL], yaw_channel_RC, 15);
    rc_deadband_limit(gimbal_control_set->gimbal_rc_ctrl->rc.ch[PITCH_CHANNEL], pitch_channel_RC, 15);

    rc_add_yaw_RC = yaw_channel_RC * YAW_RC_SEN;
    rc_add_pit_RC = pitch_channel_RC * PITCH_RC_SEN;
    // һ�׵�ͨ�˲�����б����Ϊ����
    first_order_filter_cali(&gimbal_control_set->gimbal_cmd_slow_set_vx_RC, rc_add_yaw_RC);
    first_order_filter_cali(&gimbal_control_set->gimbal_cmd_slow_set_vy_RC, rc_add_pit_RC);

    // ���̿���
    rc_deadband_limit(gimbal_control_set->gimbal_rc_ctrl->mouse.x, yaw_channel, 5);
    rc_deadband_limit(gimbal_control_set->gimbal_rc_ctrl->mouse.y, pitch_channel, 5);

    rc_add_yaw = -yaw_channel * Yaw_Mouse_Sen;
    rc_add_pit = -pitch_channel * Pitch_Mouse_Sen;
    // �����˲�
    Fiter(rc_add_pit);
    rc_add_pit = (rc_add_pit * 8 + Pitch_Set[1] - Pitch_Set[7]) / 8;
    // һ�׵�ͨ�˲�����б����Ϊ����
    first_order_filter_cali(&gimbal_control_set->gimbal_cmd_slow_set_vx, rc_add_yaw);
    first_order_filter_cali(&gimbal_control_set->gimbal_cmd_slow_set_vy, rc_add_pit);

    if (gimbal_control_set->gimbal_cmd_slow_set_vx.out > 2.f)
        gimbal_control_set->gimbal_cmd_slow_set_vx.out = 2.f;
    else if (gimbal_control_set->gimbal_cmd_slow_set_vx.out < -2.f)
        gimbal_control_set->gimbal_cmd_slow_set_vx.out = -2.f;

    *yaw = gimbal_control_set->gimbal_cmd_slow_set_vx.out + yaw_turn + yaw_turn1 + gimbal_control_set->gimbal_cmd_slow_set_vx_RC.out;
    *pitch = (gimbal_control_set->gimbal_cmd_slow_set_vy.out + pitch_turn + gimbal_control_set->gimbal_cmd_slow_set_vz.out + gimbal_control_set->gimbal_cmd_slow_set_vy_RC.out);
}



void Fiter(fp32 pitch)
{
	Pitch_Set[7]=Pitch_Set[6];
	Pitch_Set[6]=Pitch_Set[5];
	Pitch_Set[5]=Pitch_Set[4];
	Pitch_Set[4]=Pitch_Set[3];
	Pitch_Set[3]=Pitch_Set[2];
	Pitch_Set[2]=Pitch_Set[1];
	Pitch_Set[1]=Pitch_Set[0];
	Pitch_Set[0]=pitch;
}

/**
  * @brief          ��̨����ң������������ƣ��������ԽǶȿ��ƣ�
  */
static void gimbal_motionless_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }
    *yaw = 0.0f;
    *pitch = 0.0f;
}

#if GIMBAL_AUTO_MODE
/**
 * @brief                     ��̨�����Զ�ģʽ����̨��̬���Ӿ���λ�����ƣ�����Ǿ��ԽǶȿ���
 * 
 * @param yaw                 yaw ��Ƕ�����         
 * @param pitch               pitch ��Ƕ�����
 * @param gimbal_control_set  ��ָ̨��
 */
static void gimbal_auto_control(fp32* yaw, fp32* pitch, gimbal_control_t* gimbal_control_set)
{
    //yaw�ᣬ pitch����λ��ԭʼ��ֵ
    fp32 yaw_value = 0;
    fp32 pitch_value = 0;
    //yaw�ᣬ pitch����λ��ԭʼ����
    
    //�ж��Ƿ���յ���λ�����͵�����
    if (gimbal_control_set->gimbal_vision_control->rx_flag)
    {
        //���յ���λ������
        //��ȡ��λ��yaw�� pitch��Ƕ���������
        yaw_value = gimbal_control_set->gimbal_vision_control->yaw_fifo;
        pitch_value = gimbal_control_set->gimbal_vision_control->pitch_fifo;

        //kalman filter ����
        KalmanFilter(&gimbal_control_set->gimbal_yaw_motor.gimbal_motor_kalman_filter, yaw_value);
        KalmanFilter(&gimbal_control_set->gimbal_pitch_motor.gimbal_motor_kalman_filter, pitch_value);

        //��ֵ�ƶ�
        *yaw = gimbal_control_set->gimbal_yaw_motor.gimbal_motor_kalman_filter.X_now * MOTOR_ECD_TO_RAD;
        *pitch = gimbal_control_set->gimbal_pitch_motor.gimbal_motor_kalman_filter.X_now * MOTOR_ECD_TO_RAD;
    }
    else
    {
        //δ���յ���λ������,���ƶ�
        *yaw = 0;
        *pitch = 0;
    }
} 
#endif
/**
  * @brief          ��̨��ʼ�����ƣ�����������ǽǶȿ��ƣ���̨��̧��pitch�ᣬ����תyaw��
  */
static void gimbal_init_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }

    //��ʼ��״̬����������
    if (fabs(INIT_PITCH_SET - gimbal_control_set->gimbal_pitch_motor.absolute_angle) > GIMBAL_INIT_ANGLE_ERROR)
    {
        *pitch = (INIT_PITCH_SET - gimbal_control_set->gimbal_pitch_motor.absolute_angle) * GIMBAL_INIT_PITCH_SPEED;
        *yaw = 0.0f;
    }
    else
    {
        *pitch = (INIT_PITCH_SET - gimbal_control_set->gimbal_pitch_motor.absolute_angle) * GIMBAL_INIT_PITCH_SPEED;
        *yaw = (INIT_YAW_SET - gimbal_control_set->gimbal_yaw_motor.relative_angle) * GIMBAL_INIT_YAW_SPEED;
    }
}

/**
  * @brief          ��̨У׼���ƣ������raw���ƣ���̨��̧��pitch������pitch������תyaw�����תyaw����¼��ʱ�ĽǶȺͱ���ֵ
  */
static void gimbal_cali_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }
    static uint16_t cali_time = 0;

    if (gimbal_control_set->gimbal_cali.step == GIMBAL_CALI_PITCH_MAX_STEP)
    {

        *pitch = GIMBAL_CALI_MOTOR_SET;
        *yaw = 0;

        //�ж����������ݣ� ����¼�����С�Ƕ�����
        gimbal_cali_gyro_judge(gimbal_control_set->gimbal_pitch_motor.motor_gyro, cali_time, gimbal_control_set->gimbal_cali.max_pitch,
                               gimbal_control_set->gimbal_pitch_motor.absolute_angle, gimbal_control_set->gimbal_cali.max_pitch_ecd,
                               gimbal_control_set->gimbal_pitch_motor.gimbal_motor_measure->ecd, gimbal_control_set->gimbal_cali.step);
    }
    else if (gimbal_control_set->gimbal_cali.step == GIMBAL_CALI_PITCH_MIN_STEP)
    {
        *pitch = -GIMBAL_CALI_MOTOR_SET;
        *yaw = 0;

        gimbal_cali_gyro_judge(gimbal_control_set->gimbal_pitch_motor.motor_gyro, cali_time, gimbal_control_set->gimbal_cali.min_pitch,
                               gimbal_control_set->gimbal_pitch_motor.absolute_angle, gimbal_control_set->gimbal_cali.min_pitch_ecd,
                               gimbal_control_set->gimbal_pitch_motor.gimbal_motor_measure->ecd, gimbal_control_set->gimbal_cali.step);
    }
    else if (gimbal_control_set->gimbal_cali.step == GIMBAL_CALI_YAW_MAX_STEP)
    {
        *pitch = 0;
        *yaw = GIMBAL_CALI_MOTOR_SET;

        gimbal_cali_gyro_judge(gimbal_control_set->gimbal_yaw_motor.motor_gyro, cali_time, gimbal_control_set->gimbal_cali.max_yaw,
                               gimbal_control_set->gimbal_yaw_motor.absolute_angle, gimbal_control_set->gimbal_cali.max_yaw_ecd,
                               gimbal_control_set->gimbal_yaw_motor.gimbal_motor_measure->ecd, gimbal_control_set->gimbal_cali.step);
    }

    else if (gimbal_control_set->gimbal_cali.step == GIMBAL_CALI_YAW_MIN_STEP)
    {
        *pitch = 0;
        *yaw = -GIMBAL_CALI_MOTOR_SET;

        gimbal_cali_gyro_judge(gimbal_control_set->gimbal_yaw_motor.motor_gyro, cali_time, gimbal_control_set->gimbal_cali.min_yaw,
                               gimbal_control_set->gimbal_yaw_motor.absolute_angle, gimbal_control_set->gimbal_cali.min_yaw_ecd,
                               gimbal_control_set->gimbal_yaw_motor.gimbal_motor_measure->ecd, gimbal_control_set->gimbal_cali.step);
    }
    else if (gimbal_control_set->gimbal_cali.step == GIMBAL_CALI_END_STEP)
    {
        cali_time = 0;
    }
}
