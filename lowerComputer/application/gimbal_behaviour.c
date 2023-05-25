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

/**
 * @brief                     ��̨�����Զ�ģʽ����̨��̬���Ӿ���λ�����ƣ�����Ǿ��ԽǶȿ���
 * 
 * @param yaw                 yaw��Ƕ�����
 * @param pitch               pitch��Ƕ�����
 * @param gimbal_control_set  ��̨����ָ��
 * @author                    yuanluochen
 */
static void gimbal_auto_control(fp32* yaw, fp32* pitch, gimbal_control_t* gimbal_control_set);

/*----------------------------------�ṹ��---------------------------*/
//��̨��Ϊ״̬��
static gimbal_behaviour_e gimbal_behaviour = GIMBAL_ZERO_FORCE;
/*----------------------------------�ڲ�����---------------------------*/
int yaw_flag=0;  
fp32 Pitch_Set[8]={0};
/*----------------------------------�ⲿ����---------------------------*/
extern chassis_behaviour_e chassis_behaviour_mode;
//��̨��ʼ����ϱ�־λ
bool_t gimbal_init_finish_flag = 0;
/**
 * @brief ��־��������̨������ģʽ��������ģʽ��־λ
 * 
 *        ��λΪ1�� �����˴�����ģʽ��������ģʽ
 * 
 */
bool_t other_mode_transform_auto_mode_flag = 0;

/**
 * @brief ������̨�Զ�ɨ����ֵ 
 * 
 */
bool_t save_auto_scan_center_value_flag = 0;



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
    switch (gimbal_behaviour)
    {
        // ����ģʽ��,����Ϊ���ԭʼֵ���ƣ������õ����������̬
    case GIMBAL_ZERO_FORCE:
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_RAW;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_RAW;
        break;

    // ��ԽǶȿ��ƺ�ң�����ͳ�ʼ��ģʽ�Լ��Զ�ģʽ������һ�ֿ���ģʽ
    case GIMBAL_RC:
    case GIMBAL_INIT:
    case GIMBAL_AUTO:
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_GYRO;   // yaw��ͨ�������ǵľ��Խǿ���
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_GYRO; // pitch��ͨ�������ǵľ��Խǿ���
        break;

    case GIMBAL_MOTIONLESS:
        gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
        gimbal_mode_set->gimbal_pitch_motor.gimbal_motor_mode = GIMBAL_MOTOR_ENCONDE;
        break;
    }
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
    if (add_yaw == NULL || add_pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }
    // �жϵ���ģʽ�����ݵ���ģʽѡ����̿��Ʒ�ʽ
    switch (gimbal_behaviour)
    {
    case GIMBAL_ZERO_FORCE: // ����ģʽ��̨����
        gimbal_zero_force_control(add_yaw, add_pitch, gimbal_control_set);
        break;

    case GIMBAL_INIT: // ��ʼ��ģʽ����̨��ʼ��
        gimbal_init_control(add_yaw, add_pitch, gimbal_control_set);
        break;

    case GIMBAL_RC: // ң��������ģʽ,���Խǿ���
        gimbal_RC_control(add_yaw, add_pitch, gimbal_control_set);
        break;

    case GIMBAL_MOTIONLESS: // ���ź��µĿ���,������
        gimbal_motionless_control(add_yaw, add_pitch, gimbal_control_set);
        break;

    case GIMBAL_AUTO: // �Զ�ģʽ����λ������
        gimbal_auto_control(add_yaw, add_pitch, gimbal_control_set);
        break;
    }
}

/**
 * @brief          ��̨��ĳЩ��Ϊ�£���Ҫ���̲���
 * @param[in]      none
 * @retval         1: no move 0:normal
 */

bool_t gimbal_cmd_to_chassis_stop(void)
{
    if (gimbal_behaviour == GIMBAL_INIT || gimbal_behaviour == GIMBAL_MOTIONLESS || gimbal_behaviour == GIMBAL_ZERO_FORCE)
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
    if (gimbal_behaviour == GIMBAL_INIT || gimbal_behaviour == GIMBAL_ZERO_FORCE)
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
    //�ж�imu�Ƿ�У׼���
    if (judge_imu_offset_calc_finish())
    {
        //У׼�����̨��������ģʽ
        // �ж��Ƿ�Ϊ��ʼ��ģʽ
        if (gimbal_behaviour == GIMBAL_INIT)
        {
            // ��ʼ��ʱ��
            static int init_time = 0;
            if (switch_is_down(gimbal_mode_set->gimbal_rc_ctrl->rc.s[GIMBAL_MODE_CHANNEL]))
            {
                //ֹͣ��ʼ��
                init_time = 0;
            }
            else
            {
                init_time++; // ��ʼ��ʱ������
                // �Ƿ��ʼ�����
                if ((fabs(gimbal_mode_set->gimbal_pitch_motor.absolute_angle - INIT_PITCH_SET) > GIMBAL_INIT_ANGLE_ERROR) ||
                    (fabs(gimbal_mode_set->gimbal_yaw_motor.gimbal_motor_measure->ecd - gimbal_mode_set->gimbal_yaw_motor.frist_ecd) * MOTOR_ECD_TO_RAD > GIMBAL_INIT_ANGLE_ERROR))
                {
                    // ��ʼ��δ��ɣ��жϳ�ʼ��ʱ��
                    if (init_time >= GIMBAL_INIT_TIME)
                    {
                        // �������κ���Ϊ��ֱ���жϽ�������ģʽ,��ʱ����
                        init_time = 0;
                    }
                    else
                    {
                        // �˳�ģʽѡ������Ϊ��ʼ��ģʽ
                        return;
                    }
                }
                else
                {
                    // ��ʼ�����,��ʱ����,���¼�ʱ
                    init_time = 0;
                    // ��־��ʼ�����
                    gimbal_init_finish_flag = 1;
                }
                // ��ֵ������һ��
                if (save_auto_scan_center_value_flag == 0)
                {
                    save_auto_scan_center_value_flag = 1;
                    // ��ʼ����ɱ���ɨ�����ĵ�
                    gimbal_mode_set->gimbal_auto_scan.yaw_center_value = gimbal_mode_set->gimbal_yaw_motor.absolute_angle_set;
                    gimbal_mode_set->gimbal_auto_scan.pitch_center_value = gimbal_mode_set->gimbal_pitch_motor.absolute_angle_set;
                }
            }
       }

        if (switch_is_down(gimbal_mode_set->gimbal_rc_ctrl->rc.s[GIMBAL_MODE_CHANNEL]))
        {
            // ң��������ģʽ
            gimbal_behaviour = GIMBAL_ZERO_FORCE;
        }
        if (switch_is_mid(gimbal_mode_set->gimbal_rc_ctrl->rc.s[GIMBAL_MODE_CHANNEL]))
        {
            // �л���ң��������ģʽ
            gimbal_behaviour = GIMBAL_RC;
        }
        else if (switch_is_up(gimbal_mode_set->gimbal_rc_ctrl->rc.s[GIMBAL_MODE_CHANNEL]))
        {
            // �л�����̨�Զ�ģʽ
            gimbal_behaviour = GIMBAL_AUTO;
        }
        // ң����������
        if (toe_is_error(DBUS_TOE))
        {
            gimbal_behaviour = GIMBAL_ZERO_FORCE;
        }

        // �жϽ����ʼ��ģʽ
        static gimbal_behaviour_e last_gimbal_behaviour = GIMBAL_ZERO_FORCE;
        // �ж���̨������ģʽתΪ����ģʽ,�����ʼ��״̬
        if (last_gimbal_behaviour == GIMBAL_ZERO_FORCE && gimbal_behaviour != GIMBAL_ZERO_FORCE)
        {
            gimbal_behaviour = GIMBAL_INIT;
            // ��־��ʼ��δ���
            gimbal_init_finish_flag = 0;
        }

        // �ж��Ƿ�����̨������ģʽ��������ģʽ
        if (last_gimbal_behaviour != GIMBAL_AUTO && gimbal_behaviour == GIMBAL_AUTO)
        {
            other_mode_transform_auto_mode_flag = 1;
            // ���浱ǰ��̨λ��
            gimbal_mode_set->gimbal_auto_scan.yaw_center_value = gimbal_mode_set->gimbal_yaw_motor.absolute_angle;
            gimbal_mode_set->gimbal_auto_scan.pitch_center_value = gimbal_mode_set->gimbal_pitch_motor.absolute_angle;
        }
        // ������ʷ����
        last_gimbal_behaviour = gimbal_behaviour;
    }
    else
    {
        //imuУ׼δ��ϣ���̨����
        gimbal_behaviour = GIMBAL_ZERO_FORCE;
    }
    
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
 * @brief          ��̨���ƣ�����ǽǶȿ��ƣ�ң������������
 */

static void gimbal_RC_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
{
    static int16_t yaw_channel_RC;
    static int16_t pitch_channel_RC;
    
    // ң��������
    rc_deadband_limit(gimbal_control_set->gimbal_rc_ctrl->rc.ch[YAW_CHANNEL], yaw_channel_RC, RC_DEADBAND);
    rc_deadband_limit(gimbal_control_set->gimbal_rc_ctrl->rc.ch[PITCH_CHANNEL], pitch_channel_RC, RC_DEADBAND);

    *yaw = yaw_channel_RC * YAW_RC_SEN;
    *pitch = -pitch_channel_RC * PITCH_RC_SEN;
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

/**
 * @brief                     ��̨�����Զ�ģʽ����̨��̬���Ӿ���λ�����ƣ�����Ǿ��ԽǶȿ���
 *
 * @param yaw                 yaw ��Ƕ�����
 * @param pitch               pitch ��Ƕ�����
 * @param gimbal_control_set  ��ָ̨��
 */
static void gimbal_auto_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
{
    // yaw pitch ���趨ֵ�뵱ǰֵ�Ĳ�ֵ
    fp32 pitch_error = 0; 
    fp32 yaw_error = 0;

    // pitch��yaw���趨�Ƕ�
    fp32 pitch_set_angle = 0;
    fp32 yaw_set_angle = 0;

    // �����ȥ�趨�Ƕ��뵱ǰ�Ƕ�֮��Ĳ�ֵ
    yaw_error = gimbal_control_set->gimbal_yaw_motor.absolute_angle_set - gimbal_control_set->gimbal_yaw_motor.absolute_angle;
    pitch_error = gimbal_control_set->gimbal_pitch_motor.absolute_angle_set - gimbal_control_set->gimbal_pitch_motor.absolute_angle;

    // �ж������Ƿ񳤾�δ����
    if (judge_not_rx_vision_data())
    {
        // ����δ����

        // �Զ�ɨ�����ø���ֵ
        static fp32 auto_scan_AC_set_yaw = 0;
        static fp32 auto_scan_AC_set_pitch = 0;
        // ��������ʱ��
        gimbal_control_set->gimbal_auto_scan.scan_run_time = TIME_MS_TO_S(HAL_GetTick()) - gimbal_control_set->gimbal_auto_scan.scan_begin_time;
        //��̨�Զ�ɨ��,���ø���ֵ
        scan_control_set(&auto_scan_AC_set_yaw, gimbal_control_set->gimbal_auto_scan.yaw_range, gimbal_control_set->gimbal_auto_scan.scan_yaw_period, gimbal_control_set->gimbal_auto_scan.scan_run_time);
        scan_control_set(&auto_scan_AC_set_pitch, gimbal_control_set->gimbal_auto_scan.pitch_range, gimbal_control_set->gimbal_auto_scan.scan_pitch_period, gimbal_control_set->gimbal_auto_scan.scan_run_time);
        // ��ֵ����ֵ  = ����ֵ + ���ϸ�������
        pitch_set_angle = auto_scan_AC_set_pitch;
        yaw_set_angle = auto_scan_AC_set_yaw + gimbal_control_set->gimbal_auto_scan.yaw_center_value;
        // yaw_set_angle = gimbal_control_set->gimbal_auto_scan.yaw_center_value;
        // pitch_set_angle = gimbal_control_set->gimbal_auto_scan.pitch_center_value;

    }
    else
    {
        //  ��ȡ��λ���Ӿ�����
        pitch_set_angle = gimbal_control_set->gimbal_vision_point->gimbal_pitch;
        yaw_set_angle = gimbal_control_set->gimbal_vision_point->gimbal_yaw;
        // ��̨�Զ�ɨ����³�ʼʱ��
        gimbal_control_set->gimbal_auto_scan.scan_begin_time = TIME_MS_TO_S(HAL_GetTick());
   }
    // ��ֵ����
    *yaw = yaw_set_angle - gimbal_control_set->gimbal_yaw_motor.absolute_angle - yaw_error;
    *pitch = pitch_set_angle - gimbal_control_set->gimbal_pitch_motor.absolute_angle - pitch_error;
}
/**
 * @brief          ��̨��ʼ�����ƣ�����������ǽǶȿ��ƣ���̨��̧��pitch�ᣬ����תyaw��
 */
static void gimbal_init_control(fp32 *yaw, fp32 *pitch, gimbal_control_t *gimbal_control_set)
{
    if (yaw == NULL || pitch == NULL || gimbal_control_set == NULL)
    {
        return;
    }
    // ��ʼ��״̬����������
    if (fabs(INIT_PITCH_SET - gimbal_control_set->gimbal_pitch_motor.absolute_angle) > GIMBAL_INIT_ANGLE_ERROR) // pitch�����
    {
        *pitch = (INIT_PITCH_SET - gimbal_control_set->gimbal_pitch_motor.absolute_angle) * GIMBAL_INIT_PITCH_SPEED;
        *yaw = 0.0f;
    }
    else // yaw��ع��ʼֵ
    {
        static fp32 yaw_error = 0;
        yaw_error = gimbal_control_set->gimbal_yaw_motor.absolute_angle_set - gimbal_control_set->gimbal_yaw_motor.absolute_angle;
        // pitch�ᱣ�ֲ���yaw��ع���ֵ
        *pitch = (INIT_PITCH_SET - gimbal_control_set->gimbal_pitch_motor.absolute_angle) * GIMBAL_INIT_PITCH_SPEED;
        // yaw����ԽǼ������yaw��������
        *yaw = (gimbal_control_set->gimbal_yaw_motor.frist_ecd - gimbal_control_set->gimbal_yaw_motor.gimbal_motor_measure->ecd) * MOTOR_ECD_TO_RAD - yaw_error;
    }
}



void scan_control_set(fp32* gimbal_set, fp32 range, fp32 period, fp32 run_time)
{
    // ���㵥�����еĲ���
    fp32 step = 4.0f * range / period;

    // �ж���̨���ø����Ƕ��Ƿ񳬹����ֵ,�������ֵ
    if (*gimbal_set >= range)
    {
        *gimbal_set = range;
    }
    else if (*gimbal_set <= -range)
    {
        *gimbal_set = -range;
    }

    // ��������ʱ�䣬������ʱ�䴦��һ��������
    fp32 calc_time = run_time - period * ((int16_t)(run_time / period));
    // �жϵ�ǰʱ��������λ�ã����ݵ�ǰλ�ã��ж���ֵ���㷽��
    if (calc_time < period / 2.0f)
    {
        // �ϰ�����,��ֵΪ���ϵ�������,stepΪ��ֵ
        *gimbal_set = step * calc_time - range;
    }
    else if (calc_time >= period / 2.0f)
    {
        // �°����ڣ�����Ϊ���µݼ�������stepΪ��ֵ
        *gimbal_set = -(step * calc_time) + 3 * range;
    }
}

bool_t judge_gimbal_mode_is_auto_mode(void)
{
    // �жϵ�ǰ��̨ģʽ�Ƿ�Ϊ�Զ�ģʽ
    return (gimbal_behaviour == GIMBAL_AUTO) ? 1 : 0;
}

bool_t judge_other_mode_transform_auto_mode(void)
{
    // ��ȡ��ֵ���ж��¼��Ƿ���
    bool_t temp = other_mode_transform_auto_mode_flag;
    // ��ֵ����
    other_mode_transform_auto_mode_flag = 0;
    return temp;
}

bool_t gimbal_control_vision_task(void)
{
    return gimbal_init_finish_flag;
}
