#ifndef __SOLVETRAJECTORY_H__
#define __SOLVETRAJECTORY_H__
#ifndef PI
#define PI 3.1415926535f
#endif
#define GRAVITY 9.78f
typedef unsigned char uint8_t;
struct SolveTrajectory
{
    float current_v;      //��ǰ����
    float _k;             //����ϵ��
    float current_pitch;  //��ǰpitch
    float current_yaw;    //��ǰyaw

    float tar_yaw;        //Ŀ��yaw
    float v_yaw;
    float tar_r1;        //Ŀ�����ĵ�ǰ��װ�װ�ľ���
    float tar_r2;        //Ŀ�����ĵ�����װ�װ�ľ���
    float z2;            //Ŀ�����ĵ�����װ�װ�ľ���
    uint8_t armor_type;   //װ�װ�����
};

struct tar_pos
{
    float x;
    float y;
    float z;
    float yaw;
};

extern void GimbalControlInit(float pitch, float yaw, float tar_yaw , float v_yaw, float r1, float r2, float z2, uint8_t armor_type, float v, float k);
extern float GimbalControlBulletModel(float x, float v, float angle);
extern float GimbalControlGetPitch(float x, float y, float v);

extern void GimbalControlTransform(float xw, float yw, float zw,
                                float vxw, float vyw, float vzw,
                                int bias_time, float *pitch, float *yaw,
                                float *aim_x, float *aim_y, float *aim_z);

#endif /*__SOLVETRAJECTORY_H__*/
