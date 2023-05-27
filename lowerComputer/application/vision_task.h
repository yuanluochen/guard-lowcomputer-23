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
#define ALLOW_ATTACK_ERROR 0.04

//�����жϼ�������
#define JUDGE_ATTACK_COUNT 2
//����ֹͣ�жϼ�������
#define JUDGE_STOP_ATTACK_COUNT 5

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
#define BULLET_SPEED 25.0f

//��������ϵ�� K1 = (0.5 * density * C * S) / m
#define AIR_K1 0.09f
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
#define GRAVITY 9.78f

//����ʱ��ƫ�Ƽ���λ������ʱ�䵥λms
#define TIME_BIAS 15

//msתs
#ifndef TIME_MS_TO_S
#define TIME_MS_TO_S(ms) (fp32)(ms / 1000.0f)

#endif // !TIME_MS_TO_S(x)

//ȫԲ����
#define ALL_CIRCLE (2 * PI)

//yaw������ǹ�ڵ���ֱ�߶�
#define Z_STATIC 0.15f
//ǹ��ǰ�ƾ���
#define DISTANCE_STATIC 0.25f
//��ʼ����ʱ��
#define INIT_FILIGHT_TIME 0.5f

//����ǹ��id
#define SHOOT_ID 1
//�ӵ�����
#define BULLET_TYPE 1




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
    ID_OUTPOST = 0, // ǰ��վ
    ID_GUADE = 6,   // �ڱ�
    ID_BASE = 7,    // ����
} armor_id_e;

//װ�װ���ֵ
typedef enum
{
    ARMOR_NUM_BALANCE = 2,
    ARMOR_NUM_OUTPOST = 3,
    ARMOR_NUM_NORMAL = 4
}armor_num_e;


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
    bool_t reset_tracker : 1;
    uint8_t reserved : 6;
    float roll;
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
    uint8_t armors_num : 3; // װ�װ����� 2-balance 3-outpost 4-normal
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

//����ת��
typedef receive_packet_t target_data_t;

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

//�ڱ����̿�������
typedef struct 
{
    //�ҷ������˾���з������˵ľ���
    fp32 distance;
}chassis_vision_control_t;



// Ŀ��λ�ýṹ��
typedef struct
{
    float x;
    float y;
    float z;
    float yaw;
} target_position_t;

//��������ṹ��
typedef struct
{
    // ��ǰ����
    fp32 current_bullet_speed;
    // ��ǰpitch
    fp32 current_pitch;
    // ��ǰyaw
    fp32 current_yaw;
    // ����ϵ��
    fp32 k1;
    //�ӵ�����ʱ��
    fp32 flight_time;
    //Ԥ��ʱ��
    fp32 predict_time;

    // Ŀ��yaw
    fp32 target_yaw;

    // װ�װ�����
    uint8_t armor_num;

    //yaw������ǹ��ˮƽ��Ĵ�ֱ����
    fp32 z_static;
    //ǹ��ǰ�ƾ���
    fp32 distance_static;

    //����װ�װ�λ��ָ��
    target_position_t* all_target_position_point;

} solve_trajectory_t;



// �Ӿ�����ṹ��
typedef struct
{
    // ���Խ�ָ��
    const fp32* vision_angle_point;
    // ��ǰ����
    fp32 current_bullet_speed;
    // ���װ�װ����ɫ(�з�װ�װ����ɫ)
    uint8_t detect_armor_color;

    //�з�����������
    target_data_t target_data;
    //��������
    solve_trajectory_t solve_trajectory;
    //Ŀ��λ��
    target_position_t target_position;

    // ����imu���Խ�
    eular_angle_t imu_absolution_angle;
    // �Ӿ�����ľ��Խ�
    eular_angle_t vision_absolution_angle;

    //������״ָ̬��
    const ext_game_robot_state_t* robot_state_point;
    //�����������ָ��
    const ext_shoot_data_t* shoot_data_point;

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
    //�����˶�����
    chassis_vision_control_t chassis_vision_control;

} vision_control_t;

// �Ӿ����ݴ�������
void vision_task(void const *pvParameters);

// ��ȡ��λ����̨����
const gimbal_vision_control_t *get_vision_gimbal_point(void);

// ��ȡ��λ����������
const shoot_vision_control_t *get_vision_shoot_point(void);

// ��ȡ��λ�����̿�������
const chassis_vision_control_t* get_vision_chassis_point(void);
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




#endif // !VISION_TASK_H
