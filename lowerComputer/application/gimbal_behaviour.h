/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       gimbal_task.c/h
  * @brief      gimbal control task, because use the euler angle calculated by
  *             gyro sensor, range (-pi,pi), angle set-point must be in this 
  *             range.gimbal has two control mode, gyro mode and enconde mode
  *             gyro mode: use euler angle to control, encond mode: use enconde
  *             angle to control. and has some special mode:cali mode, motionless
  *             mode.
  *             �����̨��������������̨ʹ�������ǽ�����ĽǶȣ��䷶Χ�ڣ�-pi,pi��
  *             �ʶ�����Ŀ��ǶȾ�Ϊ��Χ���������ԽǶȼ���ĺ�������̨��Ҫ��Ϊ2��
  *             ״̬�������ǿ���״̬�����ð��������ǽ������̬�ǽ��п��ƣ�����������
  *             ״̬��ͨ����������ı���ֵ���Ƶ�У׼�����⻹��У׼״̬��ֹͣ״̬�ȡ�
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. done
  *  V1.1.0     Nov-11-2019     RM              1. add some annotation
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
#ifndef GIMBAL_BEHAVIOUR_H
#define GIMBAL_BEHAVIOUR_H
#include "struct_typedef.h"

#include "gimbal_task.h"

#define AUTO_FILTER_ERROR 0.1f



//pitch������
#define PITCH_CENTER_VAL 0

//�Ӿ�δ�����жϼ�����ֵ
#define VISION_DATA_MAX_NOT_UPDATE_DATA_TIME 500

//ʱ��msתs
#define TIME_MS_TO_S(ms) (fp32)(ms / 1000.0f)



typedef enum
{
  GIMBAL_ZERO_FORCE = 0, 
  GIMBAL_INIT,           
  GIMBAL_CALI,           
  GIMBAL_AUTO,             //�Զ�ģʽ
  GIMBAL_RC,               //ң��������ģʽ
  GIMBAL_ABSOLUTE_ANGLE, 
  GIMBAL_RELATIVE_ANGLE, 
  GIMBAL_MOTIONLESS,     
} gimbal_behaviour_e;

typedef enum
{
    OTHER_MODE_TO_AUTO_MODE,
    OTHER_MODE,
    AUTO_MODE,
} auto_mode_judge_e;

//��̨ҡ�ڷ���
typedef enum
{
    POSITIVE, //���� 
    NEGATIVE, //����
}gimbal_swing_direction_e;



/**
  * @brief          ��gimbal_set_mode����������gimbal_task.c,��̨��Ϊ״̬���Լ����״̬������
  * @param[out]     gimbal_mode_set: ��̨����ָ��
  * @retval         none
  */

extern void gimbal_behaviour_mode_set(gimbal_control_t *gimbal_mode_set);

/**
  * @brief          ��̨��Ϊ���ƣ����ݲ�ͬ��Ϊ���ò�ͬ���ƺ���
  * @param[out]     add_yaw:���õ�yaw�Ƕ�����ֵ����λ rad
  * @param[out]     add_pitch:���õ�pitch�Ƕ�����ֵ����λ rad
  * @param[in]      gimbal_mode_set:��̨����ָ��
  * @retval         none
  */
extern void gimbal_behaviour_control_set(fp32 *add_yaw, fp32 *add_pitch, gimbal_control_t *gimbal_control_set);

/**
  * @brief          ��̨��ĳЩ��Ϊ�£���Ҫ���̲���
  * @param[in]      none
  * @retval         1: no move 0:normal
  */

extern bool_t gimbal_cmd_to_chassis_stop(void);

/**
  * @brief          ��̨��ĳЩ��Ϊ�£���Ҫ���ֹͣ
  * @param[in]      none
  * @retval         1: no move 0:normal
  */
extern bool_t gimbal_cmd_to_shoot_stop(void);

/**
 * @brief �ж���̨ģʽΪ�Զ�ģʽ
 * 
 * @return ����1 Ϊ�Զ�ģʽ������0 �����Զ�ģʽ 
 */
bool_t judge_gimbal_mode_is_auto_mode(void);

/**
 * @brief  �ж���̨������ģʽ�����Զ�ģʽ�����ú���ִ��ʱ����־λ������
 * 
 * @return ����1 �¼����� ����0 �¼�δ���� 
 */
bool_t judge_other_mode_transform_auto_mode(void);
/**
 * @brief ��̨���Ƶ�������ִ��
 * 
 * @return bool_t 
 */
bool_t gimbal_control_vision_task(void);
#endif
