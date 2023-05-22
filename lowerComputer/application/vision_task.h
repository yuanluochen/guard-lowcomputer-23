/**
 * @file vision_task.h
 * @author yuanluochen
 * @brief �����Ӿ����ݰ��������Ӿ��۲����ݣ�Ԥ��װ�װ�λ�ã��Լ����㵯���켣�����е�������
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
#include "referee.h"

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

//�����˺���id�ֽ�ֵ�����ڸ�ֵ�����������Ϊ��ɫ��С�����ֵ����������Ϊ��ɫ
#define ROBOT_RED_AND_BLUE_DIVIDE_VALUE 100

//���δ���ܵ���λ�����ݵ�ʱ��
#define MAX_UNRX_TIME 100

//IMU �� ǹ��֮�����ֱ����
#define IMU_TO_GUMPOINT_VERTICAK 0.05f
//IMU �� ǹ��֮�����ֱ����
#define IMU_TO_GUNPOINT_DISTANCE 0.20f

//����
#define BULLET_SPEED 24.5f

//��������ϵ�� K1 = (0.5 * density * C * S) / m
#define AIR_K1 0.20f
//��ʼ�ӵ����е�����ֵ
#define T_0 0.0f
//��������
#define PRECISION 0.000001f
//��С������ֵ
#define MIN_DELTAT 0.001f
//����������
#define MAX_ITERATE_COUNT 20
//�Ӿ�����ʱ��
#define VISION_CALC_TIME 0.003f

//��������������ϵ��
#define ITERATE_SCALE_FACTOR 0.3f
//�������ٶ�
#define G 9.8f




//��������״̬
typedef enum
{
    //δ��ȡ
    UNLOADED,
    //�Ѷ�ȡ
    LOADED,
}receive_state_e;

//������ʼ֡����
typedef enum
{
    //��λ�����͵���λ��
    LOWER_TO_HIGH_HEAD = 0x5A,
    //��λ�����͵���λ��
    HIGH_TO_LOWER_HEAD = 0XA5,
}data_head_type_e;

//װ�װ���ɫ
typedef enum
{
    RED = 0,
    BLUE = 1,
}robot_armor_color_e;

//id
typedef enum
{
    OUTPOST = 0, // ǰ��վ
    GUADE = 6,   // �ڱ�
    BASE = 7,    // ����
} armor_id_e;

// //װ�װ���
// typedef enum
// {
//     BALANCE = 2,
//     OUTPOST = 3,
//     NORMAL = 4,
// } armor_num_e;

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
    uint8_t detect_color : 1; // 0-red 1-blue
    uint8_t reserved : 7;
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
    bool_t tracking : 1;
    uint8_t id : 3;         // 0-outpost 6-guard 7-base
    uint8_t armors_num : 3; // 2-balance 3-outpost 4-normal
    uint8_t reserved : 1;
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
    float dz;
    uint16_t checksum;
} receive_packet_t;

// �Ӿ����սṹ��
typedef struct
{
    // ���ձ�־λ
    uint8_t receive_state : 1;
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

    //������״ָ̬��
    ext_game_robot_state_t* robot_state_point;

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
