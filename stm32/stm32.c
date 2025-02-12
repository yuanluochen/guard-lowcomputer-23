#include "stm32.h"
#include "stm32_private.h"
#include <string.h>

#define PI 3.1415926f
/* Block states (default storage) */
DW_stm32 stm32_DW_yaw;
DW_stm32 stm32_DW_pitch;
DW_stm32 stm32_DW_shoot_0;
DW_stm32 stm32_DW_shoot_1;
/* External inputs (root inport signals with default storage) */
ExtU_stm32 stm32_U_shoot;
ExtU_stm32 stm32_U_yaw;
ExtU_stm32 stm32_U_pitch;
/* External outputs (root outports fed by signals with default storage) */
ExtY_stm32 stm32_Y_shoot;
ExtY_stm32 stm32_Y_yaw;
ExtY_stm32 stm32_Y_pitch;

void stm32_pid_init_yaw(void) // yaw
{
    stm32_U_yaw.P_P = 1500;
    stm32_U_yaw.P_I = 80;
    stm32_U_yaw.P_D = 1;
    stm32_U_yaw.P_N = 10;
    stm32_U_yaw.S_P = 150;
    stm32_U_yaw.S_I = 10;
    stm32_U_yaw.S_D = 10;
    stm32_U_yaw.S_N = 10;
}

void stm32_pid_init_pitch(void) // pitch
{
    stm32_U_pitch.P_P = 1300;
    stm32_U_pitch.P_I = 100;
    stm32_U_pitch.P_D = 1;
    stm32_U_pitch.P_N = 10;
    stm32_U_pitch.S_P = 100;
    stm32_U_pitch.S_I = 20;
    stm32_U_pitch.S_D = 10;
    stm32_U_pitch.S_N = 10;
}

void stm32_shoot_pid_init(void)
{
    stm32_U_shoot.KP = 10000;
    stm32_U_shoot.KI = 80;
    stm32_U_shoot.KD = 10;
    stm32_U_shoot.N = 70; 
}


stm32_PID_t stm32_pid_yaw;   // yaw
stm32_PID_t stm32_pid_pitch; // pitch


void stm32_step_yaw(fp32 angle_set, fp32 angle_feedback, fp32 speed_feedback) // yaw
{

    stm32_U_yaw.angle_set = angle_set;
    stm32_U_yaw.angle_feedback = angle_feedback;
    stm32_U_yaw.speed_feedback = speed_feedback;
    stm32_pid_yaw.rtb_FilterDifferentiatorTF = stm32_U_yaw.P_N * 0.0005f;
    stm32_pid_yaw.rtb_Sum1 = 1.0f / (stm32_pid_yaw.rtb_FilterDifferentiatorTF + 1.0f);
    stm32_pid_yaw.TmpSignalConversionAtFilterDifferentiatorTFInport2_idx_1 =
        (stm32_pid_yaw.rtb_FilterDifferentiatorTF - 1.0f) * stm32_pid_yaw.rtb_Sum1;
    stm32_pid_yaw.rtb_FilterDifferentiatorTF = stm32_U_yaw.S_N * 0.0005f;
    stm32_pid_yaw.rtb_Reciprocal = 1.0f / (stm32_pid_yaw.rtb_FilterDifferentiatorTF + 1.0f);
    stm32_pid_yaw.TmpSignalConversionAtFilterDifferentiatorTFInport2_c_idx_1 =
        (stm32_pid_yaw.rtb_FilterDifferentiatorTF - 1.0f) * stm32_pid_yaw.rtb_Reciprocal;
    stm32_pid_yaw.rtb_FilterDifferentiatorTF = stm32_U_yaw.angle_set - stm32_U_yaw.angle_feedback;
    if (stm32_pid_yaw.rtb_FilterDifferentiatorTF > 1.5f * PI)
    {
        stm32_pid_yaw.rtb_FilterDifferentiatorTF -= 2 * PI;
    }
    else if (stm32_pid_yaw.rtb_FilterDifferentiatorTF < -1.5f * PI)
    {
        stm32_pid_yaw.rtb_FilterDifferentiatorTF += 2 * PI;
    }
    stm32_pid_yaw.rtb_IProdOut = stm32_pid_yaw.rtb_FilterDifferentiatorTF * stm32_U_yaw.P_I;
    stm32_pid_yaw.Integrator = 0.0005f * stm32_pid_yaw.rtb_IProdOut + stm32_DW_yaw.Integrator_DSTATE;
    stm32_pid_yaw.TmpSignalConversionAtFilterDifferentiatorTFInport2_idx_1 =
        stm32_pid_yaw.rtb_FilterDifferentiatorTF * stm32_U_yaw.P_D -
        stm32_pid_yaw.TmpSignalConversionAtFilterDifferentiatorTFInport2_idx_1 *
            stm32_DW_yaw.FilterDifferentiatorTF_states;
    stm32_pid_yaw.rtb_Sum1 = ((stm32_pid_yaw.TmpSignalConversionAtFilterDifferentiatorTFInport2_idx_1 +
                               -stm32_DW_yaw.FilterDifferentiatorTF_states) *
                                  stm32_pid_yaw.rtb_Sum1 * stm32_U_yaw.P_N +
                              (stm32_pid_yaw.rtb_FilterDifferentiatorTF * stm32_U_yaw.P_P + stm32_pid_yaw.Integrator)) -
                             stm32_U_yaw.speed_feedback;
    stm32_pid_yaw.rtb_FilterDifferentiatorTF = stm32_pid_yaw.rtb_Sum1 * stm32_U_yaw.S_D -
                                               stm32_pid_yaw.TmpSignalConversionAtFilterDifferentiatorTFInport2_c_idx_1 *
                                                   stm32_DW_yaw.FilterDifferentiatorTF_states_o;
    stm32_pid_yaw.TmpSignalConversionAtFilterDifferentiatorTFInport2_c_idx_1 = stm32_pid_yaw.rtb_Sum1 *
                                                                               stm32_U_yaw.S_I;
    stm32_pid_yaw.Integrator_d = 0.0005f *
                                     stm32_pid_yaw.TmpSignalConversionAtFilterDifferentiatorTFInport2_c_idx_1 +
                                 stm32_DW_yaw.Integrator_DSTATE_p;
    stm32_Y_yaw.Out1 = (stm32_pid_yaw.rtb_FilterDifferentiatorTF +
                        -stm32_DW_yaw.FilterDifferentiatorTF_states_o) *
                           stm32_pid_yaw.rtb_Reciprocal *
                           stm32_U_yaw.S_N +
                       (stm32_pid_yaw.rtb_Sum1 * stm32_U_yaw.S_P + stm32_pid_yaw.Integrator_d);

    if (stm32_Y_yaw.Out1 >= 30000)
        stm32_Y_yaw.Out1 = 30000;
    else if (stm32_Y_yaw.Out1 <= -30000)
        stm32_Y_yaw.Out1 = -30000;
    stm32_DW_yaw.Integrator_DSTATE = 0.0005f * stm32_pid_yaw.rtb_IProdOut + stm32_pid_yaw.Integrator;
    stm32_DW_yaw.FilterDifferentiatorTF_states =
        stm32_pid_yaw.TmpSignalConversionAtFilterDifferentiatorTFInport2_idx_1;
    stm32_DW_yaw.FilterDifferentiatorTF_states_o = stm32_pid_yaw.rtb_FilterDifferentiatorTF;
    stm32_DW_yaw.Integrator_DSTATE_p = 0.0005f *
                                           stm32_pid_yaw.TmpSignalConversionAtFilterDifferentiatorTFInport2_c_idx_1 +
                                       stm32_pid_yaw.Integrator_d;
}

void stm32_step_pitch(fp32 angle_set, fp32 angle_feedback, fp32 speed_feedback) // pitch
{

    stm32_U_pitch.angle_set = angle_set;
    stm32_U_pitch.angle_feedback = angle_feedback;
    stm32_U_pitch.speed_feedback = speed_feedback;
    stm32_pid_pitch.rtb_FilterDifferentiatorTF = stm32_U_pitch.P_N * 0.0005f;
    stm32_pid_pitch.rtb_Sum1 = 1.0f / (stm32_pid_pitch.rtb_FilterDifferentiatorTF + 1.0f);
    stm32_pid_pitch.TmpSignalConversionAtFilterDifferentiatorTFInport2_idx_1 =
        (stm32_pid_pitch.rtb_FilterDifferentiatorTF - 1.0f) * stm32_pid_pitch.rtb_Sum1;
    stm32_pid_pitch.rtb_FilterDifferentiatorTF = stm32_U_pitch.S_N * 0.0005f;
    stm32_pid_pitch.rtb_Reciprocal = 1.0f / (stm32_pid_pitch.rtb_FilterDifferentiatorTF + 1.0f);
    stm32_pid_pitch.TmpSignalConversionAtFilterDifferentiatorTFInport2_c_idx_1 =
        (stm32_pid_pitch.rtb_FilterDifferentiatorTF - 1.0f) * stm32_pid_pitch.rtb_Reciprocal;
    stm32_pid_pitch.rtb_FilterDifferentiatorTF = stm32_U_pitch.angle_set - stm32_U_pitch.angle_feedback;

    stm32_pid_pitch.rtb_IProdOut = stm32_pid_pitch.rtb_FilterDifferentiatorTF * stm32_U_pitch.P_I;
    stm32_pid_pitch.Integrator = 0.0005f * stm32_pid_pitch.rtb_IProdOut + stm32_DW_pitch.Integrator_DSTATE;
    stm32_pid_pitch.TmpSignalConversionAtFilterDifferentiatorTFInport2_idx_1 =
        stm32_pid_pitch.rtb_FilterDifferentiatorTF * stm32_U_pitch.P_D -
        stm32_pid_pitch.TmpSignalConversionAtFilterDifferentiatorTFInport2_idx_1 *
            stm32_DW_pitch.FilterDifferentiatorTF_states;
    stm32_pid_pitch.rtb_Sum1 = ((stm32_pid_pitch.TmpSignalConversionAtFilterDifferentiatorTFInport2_idx_1 +
                                 -stm32_DW_pitch.FilterDifferentiatorTF_states) *
                                    stm32_pid_pitch.rtb_Sum1 * stm32_U_pitch.P_N +
                                (stm32_pid_pitch.rtb_FilterDifferentiatorTF * stm32_U_pitch.P_P + stm32_pid_pitch.Integrator)) -
                               stm32_U_pitch.speed_feedback;
    stm32_pid_pitch.rtb_FilterDifferentiatorTF = stm32_pid_pitch.rtb_Sum1 * stm32_U_pitch.S_D -
                                                 stm32_pid_pitch.TmpSignalConversionAtFilterDifferentiatorTFInport2_c_idx_1 *
                                                     stm32_DW_pitch.FilterDifferentiatorTF_states_o;
    stm32_pid_pitch.TmpSignalConversionAtFilterDifferentiatorTFInport2_c_idx_1 = stm32_pid_pitch.rtb_Sum1 *
                                                                                 stm32_U_pitch.S_I;
    stm32_pid_pitch.Integrator_d = 0.0005f *
                                       stm32_pid_pitch.TmpSignalConversionAtFilterDifferentiatorTFInport2_c_idx_1 +
                                   stm32_DW_pitch.Integrator_DSTATE_p;
    stm32_Y_pitch.Out1 = (stm32_pid_pitch.rtb_FilterDifferentiatorTF +
                          -stm32_DW_pitch.FilterDifferentiatorTF_states_o) *
                             stm32_pid_pitch.rtb_Reciprocal *
                             stm32_U_pitch.S_N +
                         (stm32_pid_pitch.rtb_Sum1 * stm32_U_pitch.S_P + stm32_pid_pitch.Integrator_d);

    if (stm32_Y_pitch.Out1 >= 30000)
        stm32_Y_pitch.Out1 = 30000;
    else if (stm32_Y_pitch.Out1 <= -30000)
        stm32_Y_pitch.Out1 = -30000;
    stm32_DW_pitch.Integrator_DSTATE = 0.0005f * stm32_pid_pitch.rtb_IProdOut + stm32_pid_pitch.Integrator;
    stm32_DW_pitch.FilterDifferentiatorTF_states =
        stm32_pid_pitch.TmpSignalConversionAtFilterDifferentiatorTFInport2_idx_1;
    stm32_DW_pitch.FilterDifferentiatorTF_states_o = stm32_pid_pitch.rtb_FilterDifferentiatorTF;
    stm32_DW_pitch.Integrator_DSTATE_p = 0.0005f *
                                             stm32_pid_pitch.TmpSignalConversionAtFilterDifferentiatorTFInport2_c_idx_1 +
                                         stm32_pid_pitch.Integrator_d;
}

void stm32_step_shoot_0(fp32 speedset, fp32 speedback)
{
    real_T rtb_Reciprocal;
    real_T rtb_Sum_p;
    real_T rtb_IProdOut;
    real_T Integrator;
    real_T TmpSignalConversionAtFilterDifferentiatorTFInport2_idx_1;
    rtb_Sum_p = stm32_U_shoot.N * 0.0005f;
    rtb_Reciprocal = 1.0f / (rtb_Sum_p + 1.0f);
    TmpSignalConversionAtFilterDifferentiatorTFInport2_idx_1 = (rtb_Sum_p - 1.0f) *
                                                               rtb_Reciprocal;
    rtb_Sum_p = speedset - speedback;
    TmpSignalConversionAtFilterDifferentiatorTFInport2_idx_1 = rtb_Sum_p *
                                                                   stm32_U_shoot.KD -
                                                               TmpSignalConversionAtFilterDifferentiatorTFInport2_idx_1 *
                                                                   stm32_DW_shoot_0.FilterDifferentiatorTF_states;
    rtb_IProdOut = rtb_Sum_p * stm32_U_shoot.KI;
    Integrator = 0.0005f * rtb_IProdOut + stm32_DW_shoot_0.Integrator_DSTATE;
    stm32_Y_shoot.out_shoot_0 = (TmpSignalConversionAtFilterDifferentiatorTFInport2_idx_1 +
                                 -stm32_DW_shoot_0.FilterDifferentiatorTF_states) *
                                    rtb_Reciprocal *
                                    stm32_U_shoot.N +
                                (rtb_Sum_p * stm32_U_shoot.KP + Integrator);
    stm32_DW_shoot_0.FilterDifferentiatorTF_states =
        TmpSignalConversionAtFilterDifferentiatorTFInport2_idx_1;
    stm32_DW_shoot_0.Integrator_DSTATE = 0.0005f * rtb_IProdOut + Integrator;
}

void stm32_step_shoot_1(fp32 speedset, fp32 speedback)
{
    real_T rtb_Reciprocal;
    real_T rtb_Sum_p;
    real_T rtb_IProdOut;
    real_T Integrator;
    real_T TmpSignalConversionAtFilterDifferentiatorTFInport2_idx_1;
    rtb_Sum_p = stm32_U_shoot.N * 0.0005f;
    rtb_Reciprocal = 1.0f / (rtb_Sum_p + 1.0f);
    TmpSignalConversionAtFilterDifferentiatorTFInport2_idx_1 = (rtb_Sum_p - 1.0f) *
                                                               rtb_Reciprocal;
    rtb_Sum_p = speedset - speedback;
    TmpSignalConversionAtFilterDifferentiatorTFInport2_idx_1 = rtb_Sum_p *
                                                                   stm32_U_shoot.KD -
                                                               TmpSignalConversionAtFilterDifferentiatorTFInport2_idx_1 *
                                                                   stm32_DW_shoot_1.FilterDifferentiatorTF_states;
    rtb_IProdOut = rtb_Sum_p * stm32_U_shoot.KI;
    Integrator = 0.0005f * rtb_IProdOut + stm32_DW_shoot_1.Integrator_DSTATE;
    stm32_Y_shoot.out_shoot_1 = (TmpSignalConversionAtFilterDifferentiatorTFInport2_idx_1 +
                                 -stm32_DW_shoot_1.FilterDifferentiatorTF_states) *
                                    rtb_Reciprocal *
                                    stm32_U_shoot.N +
                                (rtb_Sum_p * stm32_U_shoot.KP + Integrator);
    stm32_DW_shoot_1.FilterDifferentiatorTF_states =
        TmpSignalConversionAtFilterDifferentiatorTFInport2_idx_1;
    stm32_DW_shoot_1.Integrator_DSTATE = 0.0005f * rtb_IProdOut + Integrator;
}

void stm32_step_shoot_pid_clear(void)
{
    // 清空发射摩擦轮电机PID数据
    memset((void *)&stm32_DW_shoot_0, 0, sizeof(stm32_DW_shoot_0));
    memset((void *)&stm32_DW_shoot_1, 0, sizeof(stm32_DW_shoot_1));
}

void stm32_step_gimbal_pid_clear(void)
{
    //清空云台电机pid
    memset((void *)&stm32_DW_yaw, 0, sizeof(stm32_DW_yaw));
    memset((void *)&stm32_DW_pitch, 0, sizeof(stm32_DW_pitch));
}

/* File trailer for Real-Time Workshop generated code.
 *
 * [EOF] stm32.c
 */
