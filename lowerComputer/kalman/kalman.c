
#include "kalman.h"

/**
  * @name   kalmanCreate
  * @brief  创建一个卡尔曼滤波器
  * @param  p:  滤波器
  *         T_Q:系统噪声协方差  //Q无限代表只用测量值
  *         T_R:测量噪声协方差
  *         
  * @retval none
  */
  // R固定，Q越大，代表越信任侧量值，Q无穷代表只用测量值
  //		       	反之，Q越小代表越信任模型预测值，Q为零则是只用模型预测
void kalmanCreate(kalman *p,float T_Q,float T_R)  
{
    //kalman* p = ( kalman*)malloc(sizeof( kalman));
    p->X_last = (float)0;
    p->P_last = 0;
    p->Q = T_Q;
    p->R = T_R;
    p->A = 1;
    p->H = 1;
    p->X_mid = p->X_last;
    //return p;
}

/**
  * @name   KalmanFilter
  * @brief  卡尔曼滤波器
  * @param  p:  滤波器
  *         dat:待滤波数据
  * @retval 滤波后的数据
  */

float KalmanFilter(kalman* p,float dat)
{
    p->X_mid =p->A*p->X_last;                     //x(k|k-1) = AX(k-1|k-1)+BU(k)
    p->P_mid = p->A*p->P_last+p->Q;               //p(k|k-1) = Ap(k-1|k-1)A'+Q
    p->kg = p->P_mid/(p->P_mid+p->R);             //kg(k) = p(k|k-1)H'/(Hp(k|k-1)'+R)
    p->X_now = p->X_mid+p->kg*(dat-p->X_mid);     //x(k|k) = X(k|k-1)+kg(k)(Z(k)-HX(k|k-1))
    p->P_now = (1-p->kg)*p->P_mid;                //p(k|k) = (I-kg(k)H)P(k|k-1)
    p->P_last = p->P_now;                         //状态更新
    p->X_last = p->X_now;
    return p->X_now;
}


/**
 * 二阶卡尔曼滤波器
 */

float matrix_value1;
float matrix_value2;

void kalman_filter_init(kalman_filter_t *F, kalman_filter_init_t *I)
{
    mat_init(&F->xhat, 2, 1, I->xhat_data);
    mat_init(&F->xhatminus, 2, 1, I->xhatminus_data);
    mat_init(&F->z, 1, 1, I->z_data);
    mat_init(&F->A, 2, 2, I->A_data);
    mat_init(&F->H, 1, 2, I->H_data);
    mat_init(&F->Q, 2, 2, I->Q_data);
    mat_init(&F->R, 1, 1, I->R_data);
    mat_init(&F->P, 2, 2, I->P_data);
    mat_init(&F->Pminus, 2, 2, I->Pminus_data);
    mat_init(&F->K, 2, 1, I->K_data);
    mat_init(&F->AT, 2, 2, I->AT_data);
    mat_trans(&F->A, &F->AT);
    mat_init(&F->HT, 2, 1, I->HT_data);
    mat_trans(&F->H, &F->HT);
    
    //  matrix_value2 = F->A.pData[1];
}

// xhatminus==x(k|k-1)  xhat==X(k-1|k-1)
// Pminus==p(k|k-1)     P==p(k-1|k-1)    AT==A'
// HT==H'   K==kg(k)    I=1
//

/**
  *@param 卡尔曼参数结构体
  *@param 角度
*/
float *kalman_filter_calc(kalman_filter_t *F, float angle)
{
    float TEMP_data22[4] = {
        0, 0,
        0, 0
    };
    float TEMP_data21[2] = {
        0,
        0
    };
    float TEMP_data11[1] = {
        0
    };
    float TEMP_data11_2[1] = {
        0
    };
    float TEMP_data12[2] = {
        0,
        0
    };

    // 单位矩阵I
    float I_data[4] = {
        1, 0,
        0, 1
    };



    mat TEMP22, TEMP21, TEMP11, TEMP12, TEMP11_2;
    //单位矩阵
    mat I;

    mat_init(&TEMP22, 2, 2, TEMP_data22);     //
    mat_init(&TEMP21, 2, 1, TEMP_data21); //
    mat_init(&TEMP12, 1, 2, TEMP_data12); //
    mat_init(&TEMP11, 1, 1, TEMP_data11);
    mat_init(&TEMP11_2, 1, 1, TEMP_data11_2);
    //单位矩阵初始化
    mat_init(&I, 2, 2, I_data);

    F->z.pData[0] = angle; // z(k)
    //   F->xhat.pData[1] = 10;0000000000000000000

    // 1. xhat'(k)= A xhat(k-1)
    mat_mult(&F->A, &F->xhat, &F->xhatminus); //  x(k|k-1) = A*X(k-1|k-1)+B*U(k)+W(K)

    // 2. P'(k) = A P(k-1) AT + Q
    mat_mult(&F->A, &F->P, &F->Pminus);  //   p(k|k-1) = A*p(k-1|k-1)*A'+Q
    mat_mult(&F->Pminus, &F->AT, &TEMP22); //  p(k|k-1) = A*p(k-1|k-1)*A'+Q
    mat_add(&TEMP22, &F->Q, &F->Pminus);   //    p(k|k-1) = A*p(k-1|k-1)*A'+Q

    // 3. K(k) = P'(k) HT / (H P'(k) HT + R)
    mat_mult(&F->H, &F->Pminus, &TEMP12); //  kg(k) = p(k|k-1)*H'/(H*p(k|k-1)*H'+R)
    mat_mult(&TEMP12, &F->HT, &TEMP11);     //      kg(k) = p(k|k-1)*H'/(H*p(k|k-1)*H'+R)
    mat_add(&TEMP11, &F->R, &TEMP11_2);       //        kg(k) = p(k|k-1)*H'/(H*p(k|k-1)*H'+R)
    mat_inv(&TEMP11_2, &TEMP11);               //
    mat_mult(&F->Pminus, &F->HT, &TEMP21); //
    mat_mult(&TEMP21, &TEMP11, &F->K);       //

    // 4. xhat(k) = xhat'(k) + K(k) (z(k) - H xhat'(k))
    mat_mult(&F->H, &F->xhatminus, &TEMP11);   //      x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))
    mat_sub(&F->z, &TEMP11, &TEMP11);         //            x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))
    mat_mult(&F->K, &TEMP11, &TEMP21);        //           x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))
    mat_add(&F->xhatminus, &TEMP21, &F->xhat); //    x(k|k) = X(k|k-1)+kg(k)*(Z(k)-H*X(k|k-1))

    // 5. P(k) = (1-K(k)H)P'(k)
    mat_mult(&F->K, &F->H, &F->P); //            p(k|k) = (I-kg(k)*H)*P(k|k-1)
    mat_sub(&I, &F->P, &TEMP22);  //
    mat_mult(&TEMP22, &F->Pminus, &F->P);

    matrix_value1 = F->xhat.pData[0];
    matrix_value2 = F->xhat.pData[1];

    F->filtered_value[0] = F->xhat.pData[0];
    F->filtered_value[1] = F->xhat.pData[1];
    return F->filtered_value;
}

// void kalman_filter_init(kalman_filter_calc_t* kalman_filter, kalman_filter_data_t* kalman_filter_init_data)
// {   
//     //单位矩阵
//     float32_t I_data[4] = {
//         1, 0,
//         0, 1
//     };
//     memcpy(kalman_filter_init_data->I, I_data, 4 * sizeof(float32_t));

//     //初始化状态矩阵
//     mat_init(&kalman_filter->x, 2, 1, kalman_filter_init_data->X);
//     //初始化状态协方差矩阵
//     mat_init(&kalman_filter->P, 2, 2, kalman_filter_init_data->P);
//     //初始化状态转移矩阵
//     mat_init(&kalman_filter->F, 2, 2, kalman_filter_init_data->F);
//     //初始化测量噪声协方差矩阵
//     mat_init(&kalman_filter->R, 1, 1, kalman_filter_init_data->R);
//     //初始化状态转移
//     mat_init(&kalman_filter->H, 1, 2, kalman_filter_init_data->H);
//     //初始化单位矩阵
//     mat_init(&kalman_filter->I, 2, 2, kalman_filter_init_data->I);
//     //过程协方差矩阵
//     mat_init(&kalman_filter->Q, 2, 2, kalman_filter_init_data->Q);
//     //观测矩阵
//     mat_init(&kalman_filter->Z, )

//     mat_init(&kalman_filter->HT, 2, 1, kalman_filter_init_data->HT);
//     mat_trans(&kalman_filter->H, &kalman_filter->HT);

//     mat_init(&kalman_filter->FT, 2, 2, kalman_filter_init_data->FT);
//     mat_trans(&kalman_filter->F, &kalman_filter->FT);
// }

// void kalman_filter(kalman_filter_calc_t* kalman_filter, float32_t angle)
// {
//     float32_t temp22[4] = {
//        0 
//     };

//     float32_t temp11[1] = {
//         0
//     };
    
//     float32_t temp21[2] = {
//         0
//     };

//     float32_t temp12[2] = {
//         0
//     };

//     //赋值z矩阵
//     kalman_filter->

//     mat mat_temp11;
//     mat mat_temp22;
//     mat mat_temp21;
//     mat mat_temp12;


//     mat_init(&mat_temp11, 1, 1, temp11);
//     mat_init(&mat_temp21, 2, 1, temp21);
//     mat_init(&mat_temp12, 1, 2, temp12);
//     mat_init(&mat_temp22, 2, 2, temp11);


// }
