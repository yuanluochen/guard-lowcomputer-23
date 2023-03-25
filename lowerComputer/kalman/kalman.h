#ifndef _KALMAN_H
#define _KALMAN_H

#include "stdlib.h"

typedef struct {
    float X_last; //上一时刻的最优结果
    float X_mid;  //当前时刻的预测结果
    float X_now;  //当前时刻的最优结果
    float P_mid;  //当前时刻预测结果的协方差
    float P_now;  //当前时刻最优结果的协方差
    float P_last; //上一时刻最优结果的协方差
    float kg;     //kalman增益
    float A;      //系统参数
    float Q;
    float R;
    float H;
}kalman;



#include <stdio.h>
#include "stdbool.h"
#include "string.h"
#include "stdint.h"
#include "stm32f4xx.h"
#include "arm_math.h"

/*************一阶卡尔曼**************/




/*************二阶卡尔曼**************/
#define mat arm_matrix_instance_f32    //float
#define mat_64 arm_matrix_instance_f64 //double
#define mat_init arm_mat_init_f32
#define mat_add arm_mat_add_f32
#define mat_sub arm_mat_sub_f32
#define mat_mult arm_mat_mult_f32
#define mat_trans arm_mat_trans_f32 //浮点矩阵转置
#define mat_inv arm_mat_inverse_f32
#define mat_inv_f64 arm_mat_inverse_f64

// //kalman filter数据矩阵
// typedef struct 
// {
//     float32_t X[2];
//     float32_t P[4];
//     float32_t F[4];
//     float32_t FT[4];
//     float32_t R[1];
//     float32_t H[2];
//     float32_t HT[2];
//     float32_t I[4];
//     float32_t Q[4];
//     float32_t Z[2];
// }kalman_filter_data_t;


// //kalman filter 计算结构体
// typedef struct 
// {
//     mat x; //状态矩阵
//     mat P; //状态协方差矩阵
//     mat u; //外部运动
//     mat F; //状态转移矩阵
//     mat H; //测量矩阵
//     mat R; //测量协方差矩阵
//     mat I; //单位矩阵
//     mat Q; //过程噪声协方差矩阵
//     mat K; //kalman 增益
//     mat Z; //观测矩阵
//     mat HT;
//     mat FT;
// }kalman_filter_calc_t;


#define Angle_limit 200         //角度小于50开启预测
#define PredictAngle_limit 250//预测值限幅

#define Kf_Angle 0
#define Kf_Speed 1


typedef struct
{
  float raw_value;
  float filtered_value[2];
  mat xhat, xhatminus, z, A, H, AT, HT, Q, R, P, Pminus, K;
} kalman_filter_t;

typedef struct
{
  float raw_value;
  float filtered_value[2];
  float xhat_data[2], xhatminus_data[2], z_data[1], Pminus_data[4], K_data[2];
  float P_data[4];
  float AT_data[4], HT_data[4];
  float A_data[4];
  float H_data[2];
  float Q_data[4];
  float R_data[4];
} kalman_filter_init_t;

typedef struct
{
  float Vision_Angle; //视觉--角度
  float Vision_Speed; //视觉--速度
  float *Kf_result;   //卡尔曼输出值
  uint16_t Kf_Delay;  //卡尔曼延时计时

  struct
  {
    float Predicted_Factor;   //预测比例因子
    float Predicted_SpeedMin; //预测值最小速度
    float Predicted_SpeedMax; //预测值最大速度
    float kf_delay_open;      //卡尔曼延时开启时间
  } Parameter;
} Kalman_Data_t;

void kalman_filter_init(kalman_filter_t *F, kalman_filter_init_t *I);
float *kalman_filter_calc(kalman_filter_t *F, float angle);
/*************二阶卡尔曼 END**************/

#endif

void kalmanCreate(kalman *p,float T_Q,float T_R);
float KalmanFilter(kalman* p,float dat);


