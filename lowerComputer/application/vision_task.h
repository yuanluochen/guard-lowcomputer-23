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

//�������Ƕ����
#define ALLOW_ATTACK_ERROR 0.02

//�����жϼ�������
#define JUDGE_ATTACK_COUNT 2
//����ֹͣ�жϼ�������
#define JUDGE_STOP_ATTACK_COUNT 10

//��ʱ�ȴ�
#define VISION_SEND_TASK_INIT_TIME 401
//ϵͳ��ʱʱ��
#define VISION_SEND_CONTROL_TIME_MS 1

//��ʱ�ȴ�
#define VISION_TASK_INIT_TIME 450
//ϵͳ��ʱʱ��
#define VISION_CONTROL_TIME_MS 1

//������ת�Ƕ���
#define RADIAN_TO_ANGLE (360 / (2 * PI))

//���δ���ܵ���λ�����ݵ�ʱ��
#define MAX_UNRX_TIME 400

//IMU �� ǹ��֮�����ֱ����
#define IMU_TO_GUMPOINT_DISTANCE 0.1

//����
#define BULLET_SPEED 22.5f

//��������ϵ�� K1 = (0.5 * density * C * S) / m
#define AIR_K1 0.25f
//��ʼ�ӵ����е�����ֵ
#define T_0 0.0f
//��������
#define PRECISION 0.000001f
//��С������ֵ
#define MIN_DELTAT 0.001f
//����������
#define MAX_ITERATE_COUNT 20
//�Ӿ�����ʱ��
#define VISION_CALC_TIME 0.03f

//��������������ϵ��
#define ITERATE_SCALE_FACTOR 0.3f
//�������ٶ�
#define G 9.8f



//������ʼ֡����
typedef enum
{
    //��λ�����͵���λ��
    LOWER_TO_HIGH_HEAD = 0x5A,
    //��λ�����͵���λ��
    HIGH_TO_LOWER_HEAD = 0XA5,
}data_head_type_e;

//�ڱ���������
typedef enum
{
    SHOOT_ATTACK,       // Ϯ��
    SHOOT_READY_ATTACK, // ׼��Ϯ��
    SHOOT_STOP_ATTACK,  // ֹͣϮ��
} shoot_command_e;

//ŷ���ǽṹ��
typedef struct
{
    fp32 yaw;
    fp32 pitch;
    fp32 roll;
} eular_angle_t;

//�����ṹ�� 
typedef struct
{
    fp32 x;
    fp32 y;
    fp32 z;
} vector_t;

    // �������ݰ�(����ģʽ�µĽṹ�壬��ֹ�����ݶ������������ݴ�λ)
typedef struct __attribute__((packed))
{
    uint8_t header;
    uint8_t robot_color : 1;
    uint8_t task_mode : 2;
    uint8_t reserved : 5;
    float pitch;
    float yaw;
    float aim_x;
    float aim_y;
    float aim_z;
    uint16_t checksum;
} send_packet_t;

// �������ݰ�(����ģʽ�µĽṹ�壬��ֹ�����ݶ������������ݴ�λ)
typedef struct __attribute__((packed))
{
    uint8_t header;
    bool_t tracking;
    float x;
    float y;
    float z;
    float yaw;
    float vx;
    float vy;
    float vz;
    float v_yaw;
    float r1;
    float r2;
    float z_2;
    uint16_t checksum;
} receive_packet_t;

// �Ӿ����սṹ��
typedef struct
{
    // ���ձ�־λ
    bool_t rx_flag;
    // �������ݰ�
    receive_packet_t receive_packet;
} vision_receive_t;

// �ڱ���̨����˶�����,���˲���������ֵ
typedef struct
{
    // ������̨yaw����ֵ
    fp32 gimbal_yaw;
    // ������̨pitch����ֵ
    fp32 gimbal_pitch;
} gimbal_vision_control_t;

// �ڱ��������˶���������
typedef struct
{
    // �Զ���������
    shoot_command_e shoot_command;
} shoot_vision_control_t;


// �Ӿ�����ṹ��
typedef struct
{
    // ���Խ�ָ��
    const fp32* vision_angle_point;
    // ��Ԫ��ָ��
    const fp32* vision_quat_point; 

    // ����imu���Խ�
    eular_angle_t imu_absolution_angle;
    // �Ӿ�����ľ��Խ�
    eular_angle_t vision_absolution_angle;
    // ��Ԫ��
    fp32 quat[4];

    // Ŀ��װ�װ��������ϵ�¿ռ������
    vector_t target_armor_vector;
    // ��������̨��׼λ������
    vector_t robot_gimbal_aim_vector;

    //���յ����ݰ�ָ��
    vision_receive_t* vision_receive_point; 
    //�������ݰ�
    send_packet_t send_packet;

    // ��̨����˶�����
    gimbal_vision_control_t gimbal_vision_control;

    // ���������������
    shoot_vision_control_t shoot_vision_control;

} vision_control_t;

// �Ӿ����ݴ�������
void vision_task(void const *pvParameters);

// ��ȡ��λ����̨����
gimbal_vision_control_t *get_vision_gimbal_point(void);

// ��ȡ��λ����������
shoot_vision_control_t *get_vision_shoot_point(void);

/**
 * @brief �ж�δ���յ���λ������
 *
 * @return ����1 δ���յ��� ����0 ���յ�
 */
bool_t judge_not_rx_vision_data(void);

/**
 * @brief �����Ӿ�ԭʼ�������ݣ�����ԭʼ���ݣ��ж��Ƿ�Ҫ���з���
 * 
 * @param shoot_judge �Ӿ��ṹ��
 * @param vision_begin_add_yaw_angle ��λ���Ӿ�yuw��ԭʼ���ӽǶ�
 * @param vision_begin_add_pitch_angle ��λ���Ӿ�pitch��ԭʼ���ӽǶ�
 */
void vision_shoot_judge(vision_control_t* shoot_judge, fp32 vision_begin_add_yaw_angle, fp32 vision_begin_add_pitch_angle);

/**
 * @brief �������ݽ���
 * 
 * @param buf ���յ�������
 * @param len ���յ������ݳ���
 */
void receive_decode(uint8_t* buf, uint32_t len);

/**
 * @brief �����ݰ�ͨ��usb���͵�nuc
 * 
 * @param send �������ݰ�
 */
void send_packet(vision_control_t* send);


//��������ϵ�µĿռ�����ת��Ϊ�������ϵ�µĿռ�����
void earthFrame_to_relativeFrame(vector_t* vector, const float* q);
//�������ϵ�µĿռ�����ת��Ϊ��������ϵ�µĿռ�����
void relativeFrame_to_earthFrame(vector_t* vector, const float* q);

#endif // !VISION_TASK_H
