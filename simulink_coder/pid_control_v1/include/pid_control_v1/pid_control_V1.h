/*
 * pid_control_V1.h
 *
 * Trial License - for use to evaluate programs for possible purchase as
 * an end-user only.
 *
 * Code generation for model "pid_control_V1".
 *
 * Model version              : 12.139
 * Simulink Coder version : 25.2 (R2025b) 28-Jul-2025
 * C++ source code generated on : Mon Apr  6 11:59:06 2026
 *
 * Target selection: ert.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Intel->x86-64 (Linux 64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef pid_control_V1_h_
#define pid_control_V1_h_
#include <cstring>
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "slros2_initialize.h"
#include "pid_control_V1_types.h"

extern "C"
{

#include "rt_nonfinite.h"

}

#include <string.h>

extern "C"
{

#include "rtGetInf.h"

}

extern "C"
{

#include "rtGetNaN.h"

}

#include <stddef.h>
#include "zero_crossing_types.h"

/* Macros for accessing real-time model data structure */
#ifndef rtmGetContStateDisabled
#define rtmGetContStateDisabled(rtm)   ((rtm)->contStateDisabled)
#endif

#ifndef rtmSetContStateDisabled
#define rtmSetContStateDisabled(rtm, val) ((rtm)->contStateDisabled = (val))
#endif

#ifndef rtmGetContStates
#define rtmGetContStates(rtm)          ((rtm)->contStates)
#endif

#ifndef rtmSetContStates
#define rtmSetContStates(rtm, val)     ((rtm)->contStates = (val))
#endif

#ifndef rtmGetContTimeOutputInconsistentWithStateAtMajorStepFlag
#define rtmGetContTimeOutputInconsistentWithStateAtMajorStepFlag(rtm) ((rtm)->CTOutputIncnstWithState)
#endif

#ifndef rtmSetContTimeOutputInconsistentWithStateAtMajorStepFlag
#define rtmSetContTimeOutputInconsistentWithStateAtMajorStepFlag(rtm, val) ((rtm)->CTOutputIncnstWithState = (val))
#endif

#ifndef rtmGetDerivCacheNeedsReset
#define rtmGetDerivCacheNeedsReset(rtm) ((rtm)->derivCacheNeedsReset)
#endif

#ifndef rtmSetDerivCacheNeedsReset
#define rtmSetDerivCacheNeedsReset(rtm, val) ((rtm)->derivCacheNeedsReset = (val))
#endif

#ifndef rtmGetIntgData
#define rtmGetIntgData(rtm)            ((rtm)->intgData)
#endif

#ifndef rtmSetIntgData
#define rtmSetIntgData(rtm, val)       ((rtm)->intgData = (val))
#endif

#ifndef rtmGetOdeF
#define rtmGetOdeF(rtm)                ((rtm)->odeF)
#endif

#ifndef rtmSetOdeF
#define rtmSetOdeF(rtm, val)           ((rtm)->odeF = (val))
#endif

#ifndef rtmGetOdeY
#define rtmGetOdeY(rtm)                ((rtm)->odeY)
#endif

#ifndef rtmSetOdeY
#define rtmSetOdeY(rtm, val)           ((rtm)->odeY = (val))
#endif

#ifndef rtmGetPeriodicContStateIndices
#define rtmGetPeriodicContStateIndices(rtm) ((rtm)->periodicContStateIndices)
#endif

#ifndef rtmSetPeriodicContStateIndices
#define rtmSetPeriodicContStateIndices(rtm, val) ((rtm)->periodicContStateIndices = (val))
#endif

#ifndef rtmGetPeriodicContStateRanges
#define rtmGetPeriodicContStateRanges(rtm) ((rtm)->periodicContStateRanges)
#endif

#ifndef rtmSetPeriodicContStateRanges
#define rtmSetPeriodicContStateRanges(rtm, val) ((rtm)->periodicContStateRanges = (val))
#endif

#ifndef rtmGetZCCacheNeedsReset
#define rtmGetZCCacheNeedsReset(rtm)   ((rtm)->zCCacheNeedsReset)
#endif

#ifndef rtmSetZCCacheNeedsReset
#define rtmSetZCCacheNeedsReset(rtm, val) ((rtm)->zCCacheNeedsReset = (val))
#endif

#ifndef rtmGetdX
#define rtmGetdX(rtm)                  ((rtm)->derivs)
#endif

#ifndef rtmSetdX
#define rtmSetdX(rtm, val)             ((rtm)->derivs = (val))
#endif

#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

#ifndef rtmGetStopRequested
#define rtmGetStopRequested(rtm)       ((rtm)->Timing.stopRequestedFlag)
#endif

#ifndef rtmSetStopRequested
#define rtmSetStopRequested(rtm, val)  ((rtm)->Timing.stopRequestedFlag = (val))
#endif

#ifndef rtmGetStopRequestedPtr
#define rtmGetStopRequestedPtr(rtm)    (&((rtm)->Timing.stopRequestedFlag))
#endif

#ifndef rtmGetT
#define rtmGetT(rtm)                   (rtmGetTPtr((rtm))[0])
#endif

#ifndef rtmGetTPtr
#define rtmGetTPtr(rtm)                ((rtm)->Timing.t)
#endif

#ifndef rtmGetTStart
#define rtmGetTStart(rtm)              ((rtm)->Timing.tStart)
#endif

/* Block signals for system '<S285>/Enabled Subsystem' */
struct B_EnabledSubsystem_pid_contro_T {
  SL_Bus_std_msgs_Bool In1;            /* '<S328>/In1' */
};

/* Block signals for system '<S286>/Enabled Subsystem' */
struct B_EnabledSubsystem_pid_cont_d_T {
  SL_Bus_std_msgs_Float64 In1;         /* '<S329>/In1' */
};

/* Block signals (default storage) */
struct B_pid_control_V1_T {
  uint32_T state[625];
  uint32_T state_m[625];
  SL_Bus_gazebo_msgs_SetEntityStateRequest BusAssignment;/* '<Root>/Bus Assignment' */
  real_T allRandData[27];
  real_T IC[12];                       /* '<S12>/IC' */
  real_T x[12];                        /* '<S12>/Integrator' */
  real_T R[9];
  real_T RotationAnglestoDirectionCo[9];
                        /* '<S12>/Rotation Angles to Direction Cosine Matrix' */
  real_T dv[9];
  real_T ap_AxesMisalignment[9];
  real_T gp_AxesMisalignment[9];
  real_T rmat[9];
  real_T valIn[9];
  real_T TmpSignalConversionAtSFunct[5];/* '<S12>/MATLAB Function - MODEL' */
  real_T q[4];                         /* '<Root>/MATLAB Function3' */
  char_T b_zeroDelimTopic[25];
  real_T Memory1[3];                   /* '<S12>/Memory1' */
  real_T FA_b[3];
  real_T FE1_b[3];
  real_T F_b[3];
  real_T IMU1_o1[3];                   /* '<Root>/IMU1' */
  real_T IMU1_o2[3];                   /* '<Root>/IMU1' */
  real_T ap_ConstantBias[3];
  real_T ap_BiasInstability[3];
  real_T ap_TemperatureBias[3];
  real_T ap_TemperatureScaleFactor[3];
  real_T gp_ConstantBias[3];
  real_T gp_NoiseDensity[3];
  real_T gp_BiasInstability[3];
  real_T gp_RandomWalk[3];
  real_T gp_TemperatureBias[3];
  real_T gp_TemperatureScaleFactor[3];
  real_T gp_AccelerationBias[3];
  real_T mp_ConstantBias[3];
  real_T mp_NoiseDensity[3];
  real_T mp_BiasInstability[3];
  real_T mp_RandomWalk[3];
  real_T mp_TemperatureBias[3];
  real_T val[3];
  real_T B[3];
  real_T whiteNoiseDrift[3];
  real_T accelerationDrift[3];
  real_T varargin_3[3];
  real_T obj[3];
  real_T ap_ConstantBias_c[3];
  real_T ap_NoiseDensity[3];
  real_T ap_BiasInstability_k[3];
  real_T ap_RandomWalk[3];
  real_T ap_TemperatureBias_c[3];
  real_T gp_TemperatureScaleFactor_b[3];
  real_T val_p[3];
  real_T y[3];
  real_T varargin_2[3];
  real_T obj_c[3];
  char_T b_zeroDelimTopic_f[22];
  char_T b_zeroDelimTopic_g[22];
  char_T b_zeroDelimTopic_g1[17];
  char_T b_zeroDelimTopic_m[17];
  sJ4ih70VmKcvCeguWN0mNVF deadline;
  sJ4ih70VmKcvCeguWN0mNVF deadline_n;
  sJ4ih70VmKcvCeguWN0mNVF deadline_p;
  sJ4ih70VmKcvCeguWN0mNVF deadline_l;
  sJ4ih70VmKcvCeguWN0mNVF deadline_j;
  sJ4ih70VmKcvCeguWN0mNVF deadline_d;
  sJ4ih70VmKcvCeguWN0mNVF deadline_g;
  sJ4ih70VmKcvCeguWN0mNVF deadline_ld;
  real_T w1_c[2];                      /* '<S304>/w1' */
  real_T w_d[2];                       /* '<S303>/w' */
  real_T w_e0[2];                      /* '<S302>/w' */
  real_T UnaryMinus[2];                /* '<S302>/Unary Minus' */
  real_T w_o[2];                       /* '<S301>/w' */
  real_T sigma_w[2];                   /* '<S301>/sigma_w' */
  real_T frac[2];
  real_T dv1[2];
  real_T b_Denominator[2];
  real_T b_Denominator_d[2];
  real_T coeffs_Denominator[2];
  real_T dv2[2];
  real_T dv3[2];
  real_T obj_d[2];
  real_T dv4[2];
  real_T b_Denominator_l[2];
  real_T dv5[2];
  real_T obj_o[2];
  boolean_T flag[12];
  char_T obj1_Value[12];
  boolean_T flag_b[11];
  real_T Switch3;                      /* '<Root>/Switch3' */
  real_T Gain;                         /* '<Root>/Gain' */
  real_T FilterCoefficient;            /* '<S107>/Filter Coefficient' */
  real_T Saturation;                   /* '<S111>/Saturation' */
  real_T Switch1;                      /* '<Root>/Switch1' */
  real_T FilterCoefficient_c;          /* '<S55>/Filter Coefficient' */
  real_T Saturation_k;                 /* '<S59>/Saturation' */
  real_T Saturation_i;                 /* '<Root>/Saturation' */
  real_T RateLimiter;                  /* '<Root>/Rate Limiter' */
  real_T FilterCoefficient_m;          /* '<S159>/Filter Coefficient' */
  real_T Saturation_f;                 /* '<S163>/Saturation' */
  real_T FilterCoefficient_p;          /* '<S211>/Filter Coefficient' */
  real_T Saturation_m;                 /* '<S215>/Saturation' */
  real_T FilterCoefficient_cv;         /* '<S265>/Filter Coefficient' */
  real_T Saturation_o;                 /* '<S269>/Saturation' */
  real_T Memory[3];                    /* '<S12>/Memory' */
  real_T Switch;                       /* '<S42>/Switch' */
  real_T SumI4;                        /* '<S96>/SumI4' */
  real_T SumI4_i;                      /* '<S148>/SumI4' */
  real_T IntegralGain;                 /* '<S205>/Integral Gain' */
  real_T Switch_j;                     /* '<S252>/Switch' */
  real_T Product[4];                   /* '<S300>/Product' */
  real_T Switch_p[3];                  /* '<S12>/Switch' */
  real_T Switch1_p[3];                 /* '<S12>/Switch1' */
  real_T data;
  real_T data_n;
  real_T Power;                        /* '<S12>/Product2' */
  real_T Gain3;                        /* '<S12>/Gain3' */
  real_T EnergykWh;                    /* '<S12>/Gain1' */
  real_T powerdemand;                  /* '<S12>/Divide' */
  real_T loadtorque;                   /* '<S12>/Divide1' */
  real_T Output;                       /* '<S280>/Output' */
  real_T Sum[3];                       /* '<S12>/Sum' */
  real_T Sum1[3];                      /* '<S12>/Sum1' */
  real_T XDOT[40];                     /* '<S12>/MATLAB Function - MODEL' */
  real_T Va_out;                       /* '<S12>/MATLAB Function - MODEL' */
  real_T CL_total;                     /* '<S12>/MATLAB Function - MODEL' */
  real_T h_out;                        /* '<S12>/MATLAB Function - MODEL' */
  real_T mu_Lw_out;                    /* '<S12>/MATLAB Function - MODEL' */
  real_T mu_Dw_out;                    /* '<S12>/MATLAB Function - MODEL' */
  real_T w[2];                         /* '<S306>/w' */
  real_T w_a[2];                       /* '<S306>/w ' */
  real_T LwgV1[2];                     /* '<S306>/Lwg//V 1' */
  real_T w_g[2];                       /* '<S305>/w' */
  real_T w_e[2];                       /* '<S305>/w ' */
  real_T w1[2];                        /* '<S305>/w 1' */
  real_T w_n[2];                       /* '<S304>/w' */
  real_T roll_sp;                      /* '<Root>/MATLAB Function2' */
  real_T yaw_rate_sp;                  /* '<Root>/MATLAB Function2' */
  uint8_T stringOut[128];              /* '<Root>/MATLAB Function1' */
  uint8_T stringOut_l[128];            /* '<Root>/MATLAB Function' */
  real_T cy;
  real_T sy;
  real_T cp;
  real_T sp;
  real_T cr;
  real_T sr;
  real_T phi_lim;
  real_T factor_bloqueo;
  real_T u2;
  real_T beta;
  real_T q_aero;
  real_T hw;
  real_T hh;
  real_T Q;
  real_T CL_w_IGE;
  real_T CL_h_IGE;
  real_T CD_iw_IGE;
  real_T CD_ih_IGE;
  real_T Dtot;
  real_T Ltot;
  real_T CQ;
  real_T Cl;
  real_T Vd1;
  real_T Vd2;
  real_T c_phi;
  real_T c_the;
  real_T s_the;
  real_T c_psi;
  real_T s_psi;
  real_T FA_b_tmp;
  real_T FA_b_tmp_n;
  real_T FE_b;
  real_T Mcg_b_idx_0;
  real_T FE2_b_idx_0;
  real_T FE2_b_idx_2;
  real_T Fg_b_idx_2;
  real_T FE_b_idx_0;
  real_T FA_b_b;
  real_T R_tmp;
  real_T R_tmp_l;
  real_T Ltot_tmp;
  real_T Switch2;                      /* '<Root>/Switch2' */
  real_T ab2;
  real_T ac2;
  real_T ad2;
  real_T bc2;
  real_T bd2;
  real_T cd2;
  real_T aasq;
  real_T n;
  real_T val_h;
  real_T b_x;
  real_T varargin_3_b;
  real_T x_idx_5;
  real_T y_d;
  real_T b_x_e;
  real_T varargin_2_b;
  real_T x_idx_5_j;
  SL_Bus_std_msgs_Float64 SourceBlock_o2_k;/* '<S287>/SourceBlock' */
  SL_Bus_std_msgs_Float64 SourceBlock_o2_p;/* '<S286>/SourceBlock' */
  SL_Bus_std_msgs_Float64 SourceBlock_o2_d;/* '<S13>/SourceBlock' */
  SL_Bus_std_msgs_Float64 SourceBlock_o2_o;/* '<S14>/SourceBlock' */
  SL_Bus_std_msgs_Float64 SourceBlock_o2;/* '<S15>/SourceBlock' */
  uint32_T bpIndex[2];
  uint32_T u32[2];
  uint32_T b_u[2];
  int32_T i;
  int32_T i_f;
  int32_T i_a;
  int32_T trueCount;
  int32_T tmp_size_idx_1;
  int32_T i1;
  int32_T ret;
  int32_T trueCount_j;
  int32_T partialTrueCount;
  int32_T i2;
  int32_T ret_j;
  int32_T trueCount_o;
  int32_T partialTrueCount_n;
  int32_T i3;
  int32_T k;
  int32_T b_j;
  uint32_T lengthOut;                  /* '<Root>/MATLAB Function1' */
  uint32_T lengthOut_e;                /* '<Root>/MATLAB Function' */
  uint32_T i_i;
  uint32_T u_idx_0;
  uint32_T u_idx_1;
  uint32_T mti;
  uint32_T y_o;
  int8_T b_data[3];
  int8_T tmp_data[3];
  int8_T b_data_n[3];
  int8_T tmp_data_m[3];
  int8_T b_data_c[3];
  int8_T tmp_data_md[3];
  int8_T rtPrevAction;
  int8_T rtAction;
  boolean_T Compare;                   /* '<S281>/Compare' */
  boolean_T AND3;                      /* '<S42>/AND3' */
  boolean_T Memory_a;                  /* '<S42>/Memory' */
  boolean_T AND3_c;                    /* '<S252>/AND3' */
  boolean_T Memory_h;                  /* '<S252>/Memory' */
  boolean_T SourceBlock_o1;            /* '<S15>/SourceBlock' */
  boolean_T SourceBlock_o1_e;          /* '<S14>/SourceBlock' */
  boolean_T SourceBlock_o1_o;          /* '<S13>/SourceBlock' */
  boolean_T SourceBlock_o1_h;          /* '<S288>/SourceBlock' */
  boolean_T SourceBlock_o1_d;          /* '<S287>/SourceBlock' */
  boolean_T SourceBlock_o1_c;          /* '<S286>/SourceBlock' */
  boolean_T SourceBlock_o1_k;          /* '<S285>/SourceBlock' */
  boolean_T serverAvailableOnTime;
  boolean_T b;
  boolean_T b1;
  boolean_T flag_m;
  SL_Bus_gazebo_msgs_SetEntityStateResponse r;
  SL_Bus_std_msgs_Bool SourceBlock_o2_dd;/* '<S285>/SourceBlock' */
  SL_Bus_std_msgs_Bool SourceBlock_o2_j;/* '<S288>/SourceBlock' */
  B_EnabledSubsystem_pid_cont_d_T EnabledSubsystem_a;/* '<S15>/Enabled Subsystem' */
  B_EnabledSubsystem_pid_cont_d_T EnabledSubsystem_ps;/* '<S14>/Enabled Subsystem' */
  B_EnabledSubsystem_pid_cont_d_T EnabledSubsystem_b;/* '<S13>/Enabled Subsystem' */
  B_EnabledSubsystem_pid_contro_T EnabledSubsystem_pt;/* '<S288>/Enabled Subsystem' */
  B_EnabledSubsystem_pid_cont_d_T EnabledSubsystem_p;/* '<S287>/Enabled Subsystem' */
  B_EnabledSubsystem_pid_cont_d_T EnabledSubsystem_k;/* '<S286>/Enabled Subsystem' */
  B_EnabledSubsystem_pid_contro_T EnabledSubsystem;/* '<S285>/Enabled Subsystem' */
};

/* Block states (default storage) for system '<Root>' */
struct DW_pid_control_V1_T {
  fusion_simulink_imuSensor_pid_T obj; /* '<Root>/IMU1' */
  ros_slros2_internal_block_Ser_T obj_j;/* '<S2>/ServiceCaller' */
  ros_slros2_internal_block_Sub_T obj_k;/* '<S15>/SourceBlock' */
  ros_slros2_internal_block_Sub_T obj_i;/* '<S14>/SourceBlock' */
  ros_slros2_internal_block_Sub_T obj_m;/* '<S13>/SourceBlock' */
  ros_slros2_internal_block_Sub_T obj_h;/* '<S288>/SourceBlock' */
  ros_slros2_internal_block_Sub_T obj_p;/* '<S287>/SourceBlock' */
  ros_slros2_internal_block_Sub_T obj_hy;/* '<S286>/SourceBlock' */
  ros_slros2_internal_block_Sub_T obj_h4;/* '<S285>/SourceBlock' */
  real_T UnitDelay3_DSTATE;            /* '<Root>/Unit Delay3' */
  real_T UnitDelay2_DSTATE;            /* '<Root>/Unit Delay2' */
  real_T UnitDelay_DSTATE;             /* '<Root>/Unit Delay' */
  real_T UnitDelay5_DSTATE;            /* '<Root>/Unit Delay5' */
  real_T UnitDelay6_DSTATE;            /* '<Root>/Unit Delay6' */
  real_T UnitDelay1_DSTATE;            /* '<Root>/Unit Delay1' */
  real_T Memory2_PreviousInput[12];    /* '<S12>/Memory2' */
  real_T PrevY;                        /* '<Root>/Rate Limiter' */
  real_T LastMajorTime;                /* '<Root>/Rate Limiter' */
  real_T Memory_PreviousInput[3];      /* '<S12>/Memory' */
  real_T Memory1_PreviousInput[3];     /* '<S12>/Memory1' */
  real_T NextOutput[4];                /* '<S300>/White Noise' */
  real_T NextOutput_k;                 /* '<S280>/White Noise' */
  struct {
    void *LoggedData;
  } ToWorkspace_PWORK;                 /* '<Root>/To Workspace' */

  struct {
    void *LoggedData;
  } ToWorkspace_PWORK_g;               /* '<S12>/To Workspace' */

  struct {
    void *LoggedData;
  } ToWorkspace1_PWORK;                /* '<S12>/To Workspace1' */

  struct {
    void *LoggedData;
  } ToWorkspace10_PWORK;               /* '<S12>/To Workspace10' */

  struct {
    void *LoggedData;
  } ToWorkspace11_PWORK;               /* '<S12>/To Workspace11' */

  struct {
    void *LoggedData;
  } ToWorkspace12_PWORK;               /* '<S12>/To Workspace12' */

  struct {
    void *LoggedData;
  } ToWorkspace13_PWORK;               /* '<S12>/To Workspace13' */

  struct {
    void *LoggedData;
  } ToWorkspace14_PWORK;               /* '<S12>/To Workspace14' */

  struct {
    void *LoggedData;
  } ToWorkspace15_PWORK;               /* '<S12>/To Workspace15' */

  struct {
    void *LoggedData;
  } ToWorkspace16_PWORK;               /* '<S12>/To Workspace16' */

  struct {
    void *LoggedData;
  } ToWorkspace17_PWORK;               /* '<S12>/To Workspace17' */

  struct {
    void *LoggedData;
  } ToWorkspace18_PWORK;               /* '<S12>/To Workspace18' */

  struct {
    void *LoggedData;
  } ToWorkspace19_PWORK;               /* '<S12>/To Workspace19' */

  struct {
    void *LoggedData;
  } ToWorkspace2_PWORK;                /* '<S12>/To Workspace2' */

  struct {
    void *LoggedData;
  } ToWorkspace20_PWORK;               /* '<S12>/To Workspace20' */

  struct {
    void *LoggedData;
  } ToWorkspace21_PWORK;               /* '<S12>/To Workspace21' */

  struct {
    void *LoggedData;
  } ToWorkspace22_PWORK;               /* '<S12>/To Workspace22' */

  struct {
    void *LoggedData;
  } ToWorkspace23_PWORK;               /* '<S12>/To Workspace23' */

  struct {
    void *LoggedData;
  } ToWorkspace24_PWORK;               /* '<S12>/To Workspace24' */

  struct {
    void *LoggedData;
  } ToWorkspace25_PWORK;               /* '<S12>/To Workspace25' */

  struct {
    void *LoggedData;
  } ToWorkspace26_PWORK;               /* '<S12>/To Workspace26' */

  struct {
    void *LoggedData;
  } ToWorkspace27_PWORK;               /* '<S12>/To Workspace27' */

  struct {
    void *LoggedData;
  } ToWorkspace28_PWORK;               /* '<S12>/To Workspace28' */

  struct {
    void *LoggedData;
  } ToWorkspace29_PWORK;               /* '<S12>/To Workspace29' */

  struct {
    void *LoggedData;
  } ToWorkspace3_PWORK;                /* '<S12>/To Workspace3' */

  struct {
    void *LoggedData;
  } ToWorkspace30_PWORK;               /* '<S12>/To Workspace30' */

  struct {
    void *LoggedData;
  } ToWorkspace31_PWORK;               /* '<S12>/To Workspace31' */

  struct {
    void *LoggedData;
  } ToWorkspace32_PWORK;               /* '<S12>/To Workspace32' */

  struct {
    void *LoggedData;
  } ToWorkspace33_PWORK;               /* '<S12>/To Workspace33' */

  struct {
    void *LoggedData;
  } ToWorkspace4_PWORK;                /* '<S12>/To Workspace4' */

  struct {
    void *LoggedData;
  } ToWorkspace5_PWORK;                /* '<S12>/To Workspace5' */

  struct {
    void *LoggedData;
  } ToWorkspace6_PWORK;                /* '<S12>/To Workspace6' */

  struct {
    void *LoggedData;
  } ToWorkspace7_PWORK;                /* '<S12>/To Workspace7' */

  struct {
    void *LoggedData;
  } ToWorkspace8_PWORK;                /* '<S12>/To Workspace8' */

  struct {
    void *LoggedData;
  } ToWorkspace9_PWORK;                /* '<S12>/To Workspace9' */

  uint32_T PreLookUpIndexSearchprobofexcee;
                        /* '<S307>/PreLook-Up Index Search  (prob of exceed)' */
  uint32_T PreLookUpIndexSearchaltitude_DW;
                              /* '<S307>/PreLook-Up Index Search  (altitude)' */
  uint32_T RandSeed[4];                /* '<S300>/White Noise' */
  uint32_T RandSeed_a;                 /* '<S280>/White Noise' */
  robotics_slcore_internal_bloc_T obj_c;
                             /* '<Root>/Coordinate Transformation Conversion' */
  int8_T ifHeightMaxlowaltitudeelseifHei;
  /* '<S296>/if Height < Max low altitude  elseif Height > Min isotropic altitude ' */
  int8_T ifHeightMaxlowaltitudeelseifH_a;
  /* '<S295>/if Height < Max low altitude  elseif Height > Min isotropic altitude ' */
  boolean_T IC_FirstOutputTime;        /* '<S12>/IC' */
  boolean_T Integrator_DWORK1;         /* '<S12>/Integrator' */
  boolean_T PrevLimited;               /* '<Root>/Rate Limiter' */
  boolean_T Memory_PreviousInput_o;    /* '<S42>/Memory' */
  boolean_T Memory_PreviousInput_a;    /* '<S252>/Memory' */
  boolean_T objisempty;                /* '<S15>/SourceBlock' */
  boolean_T objisempty_f;              /* '<S14>/SourceBlock' */
  boolean_T objisempty_g;              /* '<S13>/SourceBlock' */
  boolean_T objisempty_a;              /* '<S288>/SourceBlock' */
  boolean_T objisempty_e;              /* '<S287>/SourceBlock' */
  boolean_T objisempty_l;              /* '<S286>/SourceBlock' */
  boolean_T objisempty_c;              /* '<S285>/SourceBlock' */
  boolean_T objisempty_d;              /* '<Root>/IMU1' */
  boolean_T objisempty_dc;   /* '<Root>/Coordinate Transformation Conversion' */
  boolean_T objisempty_ft;             /* '<S2>/ServiceCaller' */
  boolean_T Hwgws_MODE;                /* '<S291>/Hwgw(s)' */
  boolean_T Hvgws_MODE;                /* '<S291>/Hvgw(s)' */
  boolean_T Hugws_MODE;                /* '<S291>/Hugw(s)' */
  boolean_T Hrgw_MODE;                 /* '<S290>/Hrgw' */
  boolean_T Hqgw_MODE;                 /* '<S290>/Hqgw' */
  boolean_T Hpgw_MODE;                 /* '<S290>/Hpgw' */
};

/* Continuous states (default storage) */
struct X_pid_control_V1_T {
  real_T Integrator_CSTATE[12];        /* '<S12>/Integrator' */
  real_T Integrator_CSTATE_n;          /* '<S104>/Integrator' */
  real_T Filter_CSTATE;                /* '<S99>/Filter' */
  real_T Integrator_CSTATE_m;          /* '<S52>/Integrator' */
  real_T Filter_CSTATE_g;              /* '<S47>/Filter' */
  real_T Integrator_CSTATE_p;          /* '<S156>/Integrator' */
  real_T Filter_CSTATE_m;              /* '<S151>/Filter' */
  real_T Integrator_CSTATE_d;          /* '<S208>/Integrator' */
  real_T Filter_CSTATE_f;              /* '<S203>/Filter' */
  real_T Integrator_CSTATE_f;          /* '<S262>/Integrator' */
  real_T Filter_CSTATE_l;              /* '<S257>/Filter' */
  real_T Integrator1_CSTATE;           /* '<S12>/Integrator1' */
  real_T TransferFcn_CSTATE[2];        /* '<S12>/Transfer Fcn' */
  real_T TransferFcn1_CSTATE;          /* '<S12>/Transfer Fcn1' */
  real_T wg_p1_CSTATE[2];              /* '<S306>/wg_p1' */
  real_T wg_p2_CSTATE[2];              /* '<S306>/wg_p2' */
  real_T vg_p1_CSTATE[2];              /* '<S305>/vg_p1' */
  real_T vgw_p2_CSTATE[2];             /* '<S305>/vgw_p2' */
  real_T ug_p_CSTATE[2];               /* '<S304>/ug_p' */
  real_T rgw_p_CSTATE[2];              /* '<S303>/rgw_p' */
  real_T qgw_p_CSTATE[2];              /* '<S302>/qgw_p' */
  real_T pgw_p_CSTATE[2];              /* '<S301>/pgw_p' */
};

/* State derivatives (default storage) */
struct XDot_pid_control_V1_T {
  real_T Integrator_CSTATE[12];        /* '<S12>/Integrator' */
  real_T Integrator_CSTATE_n;          /* '<S104>/Integrator' */
  real_T Filter_CSTATE;                /* '<S99>/Filter' */
  real_T Integrator_CSTATE_m;          /* '<S52>/Integrator' */
  real_T Filter_CSTATE_g;              /* '<S47>/Filter' */
  real_T Integrator_CSTATE_p;          /* '<S156>/Integrator' */
  real_T Filter_CSTATE_m;              /* '<S151>/Filter' */
  real_T Integrator_CSTATE_d;          /* '<S208>/Integrator' */
  real_T Filter_CSTATE_f;              /* '<S203>/Filter' */
  real_T Integrator_CSTATE_f;          /* '<S262>/Integrator' */
  real_T Filter_CSTATE_l;              /* '<S257>/Filter' */
  real_T Integrator1_CSTATE;           /* '<S12>/Integrator1' */
  real_T TransferFcn_CSTATE[2];        /* '<S12>/Transfer Fcn' */
  real_T TransferFcn1_CSTATE;          /* '<S12>/Transfer Fcn1' */
  real_T wg_p1_CSTATE[2];              /* '<S306>/wg_p1' */
  real_T wg_p2_CSTATE[2];              /* '<S306>/wg_p2' */
  real_T vg_p1_CSTATE[2];              /* '<S305>/vg_p1' */
  real_T vgw_p2_CSTATE[2];             /* '<S305>/vgw_p2' */
  real_T ug_p_CSTATE[2];               /* '<S304>/ug_p' */
  real_T rgw_p_CSTATE[2];              /* '<S303>/rgw_p' */
  real_T qgw_p_CSTATE[2];              /* '<S302>/qgw_p' */
  real_T pgw_p_CSTATE[2];              /* '<S301>/pgw_p' */
};

/* State disabled  */
struct XDis_pid_control_V1_T {
  boolean_T Integrator_CSTATE[12];     /* '<S12>/Integrator' */
  boolean_T Integrator_CSTATE_n;       /* '<S104>/Integrator' */
  boolean_T Filter_CSTATE;             /* '<S99>/Filter' */
  boolean_T Integrator_CSTATE_m;       /* '<S52>/Integrator' */
  boolean_T Filter_CSTATE_g;           /* '<S47>/Filter' */
  boolean_T Integrator_CSTATE_p;       /* '<S156>/Integrator' */
  boolean_T Filter_CSTATE_m;           /* '<S151>/Filter' */
  boolean_T Integrator_CSTATE_d;       /* '<S208>/Integrator' */
  boolean_T Filter_CSTATE_f;           /* '<S203>/Filter' */
  boolean_T Integrator_CSTATE_f;       /* '<S262>/Integrator' */
  boolean_T Filter_CSTATE_l;           /* '<S257>/Filter' */
  boolean_T Integrator1_CSTATE;        /* '<S12>/Integrator1' */
  boolean_T TransferFcn_CSTATE[2];     /* '<S12>/Transfer Fcn' */
  boolean_T TransferFcn1_CSTATE;       /* '<S12>/Transfer Fcn1' */
  boolean_T wg_p1_CSTATE[2];           /* '<S306>/wg_p1' */
  boolean_T wg_p2_CSTATE[2];           /* '<S306>/wg_p2' */
  boolean_T vg_p1_CSTATE[2];           /* '<S305>/vg_p1' */
  boolean_T vgw_p2_CSTATE[2];          /* '<S305>/vgw_p2' */
  boolean_T ug_p_CSTATE[2];            /* '<S304>/ug_p' */
  boolean_T rgw_p_CSTATE[2];           /* '<S303>/rgw_p' */
  boolean_T qgw_p_CSTATE[2];           /* '<S302>/qgw_p' */
  boolean_T pgw_p_CSTATE[2];           /* '<S301>/pgw_p' */
};

/* Zero-crossing (trigger) state */
struct PrevZCX_pid_control_V1_T {
  ZCSigState Integrator_Reset_ZCE;     /* '<S12>/Integrator' */
};

/* Invariant block signals (default storage) */
struct ConstB_pid_control_V1_T {
  real_T UnitConversion;               /* '<S289>/Unit Conversion' */
  real_T UnitConversion_k;             /* '<S299>/Unit Conversion' */
  real_T sigma_wg;                     /* '<S308>/sigma_wg ' */
  real_T UnitConversion_n;             /* '<S293>/Unit Conversion' */
  real_T UnitConversion_c;             /* '<S327>/Unit Conversion' */
  real_T PreLookUpIndexSearchprobofe;
                        /* '<S307>/PreLook-Up Index Search  (prob of exceed)' */
  real_T Sqrt[4];                      /* '<S300>/Sqrt' */
  real_T Sqrt1;                        /* '<S300>/Sqrt1' */
  real_T Divide[4];                    /* '<S300>/Divide' */
  real_T motorspeed;                   /* '<S12>/Gain2' */
  real_T Sum;                          /* '<S317>/Sum' */
  real_T Sum_a;                        /* '<S309>/Sum' */
  real_T sqrt_a;                       /* '<S306>/sqrt' */
  real_T w4;                           /* '<S301>/w4' */
  real_T u16;                          /* '<S301>/u^1//6' */
  uint32_T PreLookUpIndexSearchprobo_g;
                        /* '<S307>/PreLook-Up Index Search  (prob of exceed)' */
};

#ifndef ODE4_INTG
#define ODE4_INTG

/* ODE4 Integration Data */
struct ODE4_IntgData {
  real_T *y;                           /* output */
  real_T *f[4];                        /* derivatives */
};

#endif

/* Constant parameters (default storage) */
struct ConstP_pid_control_V1_T {
  /* Expression: [ 27.5550, -2.4169, -16.0849 ]
   * Referenced by: '<Root>/IMU1'
   */
  real_T IMU1_MagneticFieldNED[3];

  /* Pooled Parameter (Expression: [ 0, 0, 0 ])
   * Referenced by: '<Root>/IMU1'
   */
  real_T pooled5[3];

  /* Pooled Parameter (Expression: [ 100, 0, 0; 0, 100, 0; 0, 0, 100 ])
   * Referenced by: '<Root>/IMU1'
   */
  real_T pooled6[9];

  /* Expression: [0.0004, 0.0004, 0.0004]
   * Referenced by: '<Root>/IMU1'
   */
  real_T IMU1_AccelParamsNoiseDensity[3];

  /* Pooled Parameter (Expression: [0.0001, 0.0001, 0.0001])
   * Referenced by: '<Root>/IMU1'
   */
  real_T pooled7[3];

  /* Pooled Parameter (Expression: fractalcoef().Denominator)
   * Referenced by: '<Root>/IMU1'
   */
  real_T pooled9[2];

  /* Expression: [0.00001, 0.00001, 0.00001]
   * Referenced by: '<Root>/IMU1'
   */
  real_T IMU1_GyroParamsBiasInstability[3];

  /* Pooled Parameter (Expression: x_nom)
   * Referenced by:
   *   '<S12>/IC'
   *   '<S12>/Memory2'
   */
  real_T pooled15[12];

  /* Expression: h_vec
   * Referenced by: '<S307>/PreLook-Up Index Search  (altitude)'
   */
  real_T PreLookUpIndexSearchaltitude_Br[12];

  /* Expression: sigma_vec'
   * Referenced by: '<S307>/Medium//High Altitude Intensity'
   */
  real_T MediumHighAltitudeIntensity_Tab[84];

  /* Computed Parameter: MediumHighAltitudeIntensity_max
   * Referenced by: '<S307>/Medium//High Altitude Intensity'
   */
  uint32_T MediumHighAltitudeIntensity_max[2];
};

/* Real-time Model Data Structure */
struct tag_RTM_pid_control_V1_T {
  const char_T *errorStatus;
  RTWSolverInfo solverInfo;
  X_pid_control_V1_T *contStates;
  int_T *periodicContStateIndices;
  real_T *periodicContStateRanges;
  real_T *derivs;
  XDis_pid_control_V1_T *contStateDisabled;
  boolean_T zCCacheNeedsReset;
  boolean_T derivCacheNeedsReset;
  boolean_T CTOutputIncnstWithState;
  real_T odeY[42];
  real_T odeF[4][42];
  ODE4_IntgData intgData;

  /*
   * Sizes:
   * The following substructure contains sizes information
   * for many of the model attributes such as inputs, outputs,
   * dwork, sample times, etc.
   */
  struct {
    int_T numContStates;
    int_T numPeriodicContStates;
    int_T numSampTimes;
  } Sizes;

  /*
   * Timing:
   * The following substructure contains information regarding
   * the timing information for the model.
   */
  struct {
    uint32_T clockTick0;
    uint32_T clockTickH0;
    time_T stepSize0;
    uint32_T clockTick1;
    uint32_T clockTickH1;
    boolean_T firstInitCondFlag;
    time_T tStart;
    SimTimeStep simTimeStep;
    boolean_T stopRequestedFlag;
    time_T *t;
    time_T tArray[2];
  } Timing;
};

extern const ConstB_pid_control_V1_T pid_control_V1_ConstB;/* constant block i/o */

/* Constant parameters (default storage) */
extern const ConstP_pid_control_V1_T pid_control_V1_ConstP;

/* Class declaration for model pid_control_V1 */
class pid_control_V1
{
  /* public data and function members */
 public:
  /* Real-Time Model get method */
  RT_MODEL_pid_control_V1_T * getRTM();
  void ModelPrevZCStateInit();

  /* model start function */
  void start();

  /* Initial conditions function */
  void initialize();

  /* model step function */
  void step();

  /* model terminate function */
  void terminate();

  /* Constructor */
  pid_control_V1();

  /* Destructor */
  ~pid_control_V1();

  /* private data and function members */
 private:
  /* Block signals */
  B_pid_control_V1_T pid_control_V1_B;

  /* Block states */
  DW_pid_control_V1_T pid_control_V1_DW;

  /* Block continuous states */
  X_pid_control_V1_T pid_control_V1_X;

  /* Block Continuous state disabled vector */
  XDis_pid_control_V1_T pid_control_V1_XDis;

  /* Triggered events */
  PrevZCX_pid_control_V1_T pid_control_V1_PrevZCX;

  /* private member function(s) for subsystem '<S285>/Enabled Subsystem'*/
  static void pid_contr_EnabledSubsystem_Init(B_EnabledSubsystem_pid_contro_T
    *localB);
  static void pid_control_V1_EnabledSubsystem(boolean_T rtu_Enable, const
    SL_Bus_std_msgs_Bool *rtu_In1, B_EnabledSubsystem_pid_contro_T *localB);

  /* private member function(s) for subsystem '<S286>/Enabled Subsystem'*/
  static void pid_con_EnabledSubsystem_i_Init(B_EnabledSubsystem_pid_cont_d_T
    *localB);
  static void pid_control__EnabledSubsystem_k(boolean_T rtu_Enable, const
    SL_Bus_std_msgs_Float64 *rtu_In1, B_EnabledSubsystem_pid_cont_d_T *localB);

  /* private member function(s) for subsystem '<Root>'*/
  void IMUSensorParameters_updateSyste(real_T obj_MeasurementRange, real_T
    obj_Resolution, const real_T obj_ConstantBias[3], const real_T
    obj_AxesMisalignment[9], const real_T obj_NoiseDensity[3], const real_T
    obj_BiasInstability[3], const real_T obj_RandomWalk[3], real_T
    obj_BiasInstabilityCoefficients, const real_T
    obj_BiasInstabilityCoefficien_0[2], const char_T obj_NoiseType_Value[12],
    const real_T obj_TemperatureBias[3], const real_T
    obj_TemperatureScaleFactor[3], d_fusion_internal_Acceleromet_T *sobj);
  void IMUSensorParameters_updateSys_o(real_T obj_MeasurementRange, real_T
    obj_Resolution, const real_T obj_ConstantBias[3], const real_T
    obj_AxesMisalignment[9], const real_T obj_NoiseDensity[3], const real_T
    obj_BiasInstability[3], const real_T obj_RandomWalk[3], real_T
    obj_BiasInstabilityCoefficients, const real_T
    obj_BiasInstabilityCoefficien_0[2], const char_T obj_NoiseType_Value[12],
    const real_T obj_TemperatureBias[3], const real_T
    obj_TemperatureScaleFactor[3], const real_T obj_AccelerationBias[3],
    e_fusion_internal_GyroscopeSi_T *sobj);
  void IMUSensorParameters_updateSy_on(real_T obj_MeasurementRange, real_T
    obj_Resolution, const real_T obj_ConstantBias[3], const real_T
    obj_AxesMisalignment[9], const real_T obj_NoiseDensity[3], const real_T
    obj_BiasInstability[3], const real_T obj_RandomWalk[3], real_T
    obj_BiasInstabilityCoefficients, const real_T
    obj_BiasInstabilityCoefficien_0[2], const char_T obj_NoiseType_Value[12],
    const real_T obj_TemperatureBias[3], const real_T
    obj_TemperatureScaleFactor[3], e_fusion_internal_Magnetomete_T *sobj);
  void pid_control_imuSensor_setupImpl(fusion_simulink_imuSensor_pid_T *obj);
  void pid_c_Subscriber_setupImpl_onhg(const ros_slros2_internal_block_Sub_T
    *obj);
  void pid_Subscriber_setupImpl_onhgd0(const ros_slros2_internal_block_Sub_T
    *obj);
  void pid__Subscriber_setupImpl_onhgd(const ros_slros2_internal_block_Sub_T
    *obj);
  void pid_con_ServiceCaller_setupImpl(const ros_slros2_internal_block_Ser_T
    *obj);
  void pid_co_Subscriber_setupImpl_onh(const ros_slros2_internal_block_Sub_T
    *obj);
  void pid_contro_Subscriber_setupImpl(const ros_slros2_internal_block_Sub_T
    *obj);
  void pid_cont_Subscriber_setupImpl_o(const ros_slros2_internal_block_Sub_T
    *obj);
  void pid_con_Subscriber_setupImpl_on(const ros_slros2_internal_block_Sub_T
    *obj);
  void pid_con_IMUSensorBase_resetImpl(fusion_simulink_imuSensor_pid_T *obj);
  boolean_T pid_control_V1_isequal_o(const real_T varargin_1[3], const real_T
    varargin_2[3]);
  void imuSensor_set_MagneticFieldNED(fusion_simulink_imuSensor_pid_T *obj,
    const real_T val[3]);
  boolean_T pid_control_V1_isequal_on(const real_T varargin_1[9], const real_T
    varargin_2[9]);
  boolean_T pid_control_V1_isequal(const real_T varargin_1[2], const real_T
    varargin_2[2]);
  boolean_T pid_control_V1_vectorAny(const boolean_T x_data[], const int32_T
    x_size[2]);
  void pid_contr_genrand_uint32_vector(uint32_T mt[625], uint32_T u[2]);
  real_T pid_control_V1_genrandu(uint32_T mt[625]);
  void pid_control_V1_filter(real_T b, real_T a[2], const real_T x[3], const
    real_T zi[3], real_T y[3], real_T zf[3]);
  void pid_control_V_SystemCore_step_o(d_fusion_internal_Acceleromet_T *obj,
    const real_T varargin_1[3], const real_T varargin_2[9], const real_T
    varargin_3[9], real_T varargout_1[3]);
  void pid_control__SystemCore_step_on(e_fusion_internal_GyroscopeSi_T *obj,
    const real_T varargin_1[3], const real_T varargin_2[3], const real_T
    varargin_3[9], const real_T varargin_4[9], real_T varargout_1[3]);
  void pid_control_V1_SystemCore_step(fusion_simulink_imuSensor_pid_T *obj,
    const real_T varargin_1[3], const real_T varargin_2[3], const real_T
    varargin_3[4], real_T varargout_1[3], real_T varargout_2[3], real_T
    varargout_3[3]);

  /* Global mass matrix */

  /* Continuous states update member function*/
  void rt_ertODEUpdateContinuousStates(RTWSolverInfo *si );

  /* Derivatives member function */
  void pid_control_V1_derivatives();

  /* Real-Time Model */
  RT_MODEL_pid_control_V1_T pid_control_V1_M;
};

extern volatile boolean_T stopRequested;
extern volatile boolean_T runModel;

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<Root>/Display' : Unused code path elimination
 * Block '<S148>/Kb' : Eliminated nontunable gain of 1
 * Block '<Root>/Rate Transition' : Eliminated since input and output rates are identical
 * Block '<Root>/Rate Transition1' : Eliminated since input and output rates are identical
 * Block '<Root>/Rate Transition2' : Eliminated since input and output rates are identical
 * Block '<Root>/Rate Transition3' : Eliminated since input and output rates are identical
 * Block '<Root>/Reshape' : Reshape block reduction
 * Block '<Root>/Reshape1' : Reshape block reduction
 * Block '<Root>/Reshape2' : Reshape block reduction
 * Block '<S282>/Cast' : Eliminate redundant data type conversion
 * Block '<S282>/Cast To Double' : Eliminate redundant data type conversion
 * Block '<S282>/Cast To Double1' : Eliminate redundant data type conversion
 * Block '<S282>/Cast To Double2' : Eliminate redundant data type conversion
 * Block '<S282>/Cast To Double3' : Eliminate redundant data type conversion
 * Block '<S282>/Cast To Double4' : Eliminate redundant data type conversion
 * Block '<S313>/Reshape' : Reshape block reduction
 * Block '<S313>/Reshape1' : Reshape block reduction
 * Block '<S315>/Reshape' : Reshape block reduction
 * Block '<S321>/Reshape' : Reshape block reduction
 * Block '<S321>/Reshape1' : Reshape block reduction
 * Block '<S323>/Reshape' : Reshape block reduction
 */

/*-
 * The generated code includes comments that allow you to trace directly
 * back to the appropriate location in the model.  The basic format
 * is <system>/block_name, where system is the system number (uniquely
 * assigned by Simulink) and block_name is the name of the block.
 *
 * Use the MATLAB hilite_system command to trace the generated code back
 * to the model.  For example,
 *
 * hilite_system('<S3>')    - opens system 3
 * hilite_system('<S3>/Kp') - opens and selects block Kp which resides in S3
 *
 * Here is the system hierarchy for this model
 *
 * '<Root>' : 'pid_control_V1'
 * '<S1>'   : 'pid_control_V1/Blank Message'
 * '<S2>'   : 'pid_control_V1/Call Service'
 * '<S3>'   : 'pid_control_V1/MATLAB Function'
 * '<S4>'   : 'pid_control_V1/MATLAB Function1'
 * '<S5>'   : 'pid_control_V1/MATLAB Function2'
 * '<S6>'   : 'pid_control_V1/MATLAB Function3'
 * '<S7>'   : 'pid_control_V1/PID ALERON'
 * '<S8>'   : 'pid_control_V1/PID ALTURA'
 * '<S9>'   : 'pid_control_V1/PID PITCH//ELEVATOR'
 * '<S10>'  : 'pid_control_V1/PID TIIMON'
 * '<S11>'  : 'pid_control_V1/PID VELOCIDAD'
 * '<S12>'  : 'pid_control_V1/SUBSYSTEM_MODEL'
 * '<S13>'  : 'pid_control_V1/Subscribe-ALTURA1'
 * '<S14>'  : 'pid_control_V1/Subscribe-ROLL'
 * '<S15>'  : 'pid_control_V1/Subscribe-YAW'
 * '<S16>'  : 'pid_control_V1/PID ALERON/Anti-windup'
 * '<S17>'  : 'pid_control_V1/PID ALERON/D Gain'
 * '<S18>'  : 'pid_control_V1/PID ALERON/External Derivative'
 * '<S19>'  : 'pid_control_V1/PID ALERON/Filter'
 * '<S20>'  : 'pid_control_V1/PID ALERON/Filter ICs'
 * '<S21>'  : 'pid_control_V1/PID ALERON/I Gain'
 * '<S22>'  : 'pid_control_V1/PID ALERON/Ideal P Gain'
 * '<S23>'  : 'pid_control_V1/PID ALERON/Ideal P Gain Fdbk'
 * '<S24>'  : 'pid_control_V1/PID ALERON/Integrator'
 * '<S25>'  : 'pid_control_V1/PID ALERON/Integrator ICs'
 * '<S26>'  : 'pid_control_V1/PID ALERON/N Copy'
 * '<S27>'  : 'pid_control_V1/PID ALERON/N Gain'
 * '<S28>'  : 'pid_control_V1/PID ALERON/P Copy'
 * '<S29>'  : 'pid_control_V1/PID ALERON/Parallel P Gain'
 * '<S30>'  : 'pid_control_V1/PID ALERON/Reset Signal'
 * '<S31>'  : 'pid_control_V1/PID ALERON/Saturation'
 * '<S32>'  : 'pid_control_V1/PID ALERON/Saturation Fdbk'
 * '<S33>'  : 'pid_control_V1/PID ALERON/Sum'
 * '<S34>'  : 'pid_control_V1/PID ALERON/Sum Fdbk'
 * '<S35>'  : 'pid_control_V1/PID ALERON/Tracking Mode'
 * '<S36>'  : 'pid_control_V1/PID ALERON/Tracking Mode Sum'
 * '<S37>'  : 'pid_control_V1/PID ALERON/Tsamp - Integral'
 * '<S38>'  : 'pid_control_V1/PID ALERON/Tsamp - Ngain'
 * '<S39>'  : 'pid_control_V1/PID ALERON/postSat Signal'
 * '<S40>'  : 'pid_control_V1/PID ALERON/preInt Signal'
 * '<S41>'  : 'pid_control_V1/PID ALERON/preSat Signal'
 * '<S42>'  : 'pid_control_V1/PID ALERON/Anti-windup/Cont. Clamping Parallel'
 * '<S43>'  : 'pid_control_V1/PID ALERON/Anti-windup/Cont. Clamping Parallel/Dead Zone'
 * '<S44>'  : 'pid_control_V1/PID ALERON/Anti-windup/Cont. Clamping Parallel/Dead Zone/Enabled'
 * '<S45>'  : 'pid_control_V1/PID ALERON/D Gain/Internal Parameters'
 * '<S46>'  : 'pid_control_V1/PID ALERON/External Derivative/Error'
 * '<S47>'  : 'pid_control_V1/PID ALERON/Filter/Cont. Filter'
 * '<S48>'  : 'pid_control_V1/PID ALERON/Filter ICs/Internal IC - Filter'
 * '<S49>'  : 'pid_control_V1/PID ALERON/I Gain/Internal Parameters'
 * '<S50>'  : 'pid_control_V1/PID ALERON/Ideal P Gain/Passthrough'
 * '<S51>'  : 'pid_control_V1/PID ALERON/Ideal P Gain Fdbk/Disabled'
 * '<S52>'  : 'pid_control_V1/PID ALERON/Integrator/Continuous'
 * '<S53>'  : 'pid_control_V1/PID ALERON/Integrator ICs/Internal IC'
 * '<S54>'  : 'pid_control_V1/PID ALERON/N Copy/Disabled'
 * '<S55>'  : 'pid_control_V1/PID ALERON/N Gain/Internal Parameters'
 * '<S56>'  : 'pid_control_V1/PID ALERON/P Copy/Disabled'
 * '<S57>'  : 'pid_control_V1/PID ALERON/Parallel P Gain/Internal Parameters'
 * '<S58>'  : 'pid_control_V1/PID ALERON/Reset Signal/Disabled'
 * '<S59>'  : 'pid_control_V1/PID ALERON/Saturation/Enabled'
 * '<S60>'  : 'pid_control_V1/PID ALERON/Saturation Fdbk/Disabled'
 * '<S61>'  : 'pid_control_V1/PID ALERON/Sum/Sum_PID'
 * '<S62>'  : 'pid_control_V1/PID ALERON/Sum Fdbk/Disabled'
 * '<S63>'  : 'pid_control_V1/PID ALERON/Tracking Mode/Disabled'
 * '<S64>'  : 'pid_control_V1/PID ALERON/Tracking Mode Sum/Passthrough'
 * '<S65>'  : 'pid_control_V1/PID ALERON/Tsamp - Integral/TsSignalSpecification'
 * '<S66>'  : 'pid_control_V1/PID ALERON/Tsamp - Ngain/Passthrough'
 * '<S67>'  : 'pid_control_V1/PID ALERON/postSat Signal/Forward_Path'
 * '<S68>'  : 'pid_control_V1/PID ALERON/preInt Signal/Internal PreInt'
 * '<S69>'  : 'pid_control_V1/PID ALERON/preSat Signal/Forward_Path'
 * '<S70>'  : 'pid_control_V1/PID ALTURA/Anti-windup'
 * '<S71>'  : 'pid_control_V1/PID ALTURA/D Gain'
 * '<S72>'  : 'pid_control_V1/PID ALTURA/External Derivative'
 * '<S73>'  : 'pid_control_V1/PID ALTURA/Filter'
 * '<S74>'  : 'pid_control_V1/PID ALTURA/Filter ICs'
 * '<S75>'  : 'pid_control_V1/PID ALTURA/I Gain'
 * '<S76>'  : 'pid_control_V1/PID ALTURA/Ideal P Gain'
 * '<S77>'  : 'pid_control_V1/PID ALTURA/Ideal P Gain Fdbk'
 * '<S78>'  : 'pid_control_V1/PID ALTURA/Integrator'
 * '<S79>'  : 'pid_control_V1/PID ALTURA/Integrator ICs'
 * '<S80>'  : 'pid_control_V1/PID ALTURA/N Copy'
 * '<S81>'  : 'pid_control_V1/PID ALTURA/N Gain'
 * '<S82>'  : 'pid_control_V1/PID ALTURA/P Copy'
 * '<S83>'  : 'pid_control_V1/PID ALTURA/Parallel P Gain'
 * '<S84>'  : 'pid_control_V1/PID ALTURA/Reset Signal'
 * '<S85>'  : 'pid_control_V1/PID ALTURA/Saturation'
 * '<S86>'  : 'pid_control_V1/PID ALTURA/Saturation Fdbk'
 * '<S87>'  : 'pid_control_V1/PID ALTURA/Sum'
 * '<S88>'  : 'pid_control_V1/PID ALTURA/Sum Fdbk'
 * '<S89>'  : 'pid_control_V1/PID ALTURA/Tracking Mode'
 * '<S90>'  : 'pid_control_V1/PID ALTURA/Tracking Mode Sum'
 * '<S91>'  : 'pid_control_V1/PID ALTURA/Tsamp - Integral'
 * '<S92>'  : 'pid_control_V1/PID ALTURA/Tsamp - Ngain'
 * '<S93>'  : 'pid_control_V1/PID ALTURA/postSat Signal'
 * '<S94>'  : 'pid_control_V1/PID ALTURA/preInt Signal'
 * '<S95>'  : 'pid_control_V1/PID ALTURA/preSat Signal'
 * '<S96>'  : 'pid_control_V1/PID ALTURA/Anti-windup/Back Calculation'
 * '<S97>'  : 'pid_control_V1/PID ALTURA/D Gain/Internal Parameters'
 * '<S98>'  : 'pid_control_V1/PID ALTURA/External Derivative/Error'
 * '<S99>'  : 'pid_control_V1/PID ALTURA/Filter/Cont. Filter'
 * '<S100>' : 'pid_control_V1/PID ALTURA/Filter ICs/Internal IC - Filter'
 * '<S101>' : 'pid_control_V1/PID ALTURA/I Gain/Internal Parameters'
 * '<S102>' : 'pid_control_V1/PID ALTURA/Ideal P Gain/Passthrough'
 * '<S103>' : 'pid_control_V1/PID ALTURA/Ideal P Gain Fdbk/Disabled'
 * '<S104>' : 'pid_control_V1/PID ALTURA/Integrator/Continuous'
 * '<S105>' : 'pid_control_V1/PID ALTURA/Integrator ICs/Internal IC'
 * '<S106>' : 'pid_control_V1/PID ALTURA/N Copy/Disabled'
 * '<S107>' : 'pid_control_V1/PID ALTURA/N Gain/Internal Parameters'
 * '<S108>' : 'pid_control_V1/PID ALTURA/P Copy/Disabled'
 * '<S109>' : 'pid_control_V1/PID ALTURA/Parallel P Gain/Internal Parameters'
 * '<S110>' : 'pid_control_V1/PID ALTURA/Reset Signal/Disabled'
 * '<S111>' : 'pid_control_V1/PID ALTURA/Saturation/Enabled'
 * '<S112>' : 'pid_control_V1/PID ALTURA/Saturation Fdbk/Disabled'
 * '<S113>' : 'pid_control_V1/PID ALTURA/Sum/Sum_PID'
 * '<S114>' : 'pid_control_V1/PID ALTURA/Sum Fdbk/Disabled'
 * '<S115>' : 'pid_control_V1/PID ALTURA/Tracking Mode/Disabled'
 * '<S116>' : 'pid_control_V1/PID ALTURA/Tracking Mode Sum/Passthrough'
 * '<S117>' : 'pid_control_V1/PID ALTURA/Tsamp - Integral/TsSignalSpecification'
 * '<S118>' : 'pid_control_V1/PID ALTURA/Tsamp - Ngain/Passthrough'
 * '<S119>' : 'pid_control_V1/PID ALTURA/postSat Signal/Forward_Path'
 * '<S120>' : 'pid_control_V1/PID ALTURA/preInt Signal/Internal PreInt'
 * '<S121>' : 'pid_control_V1/PID ALTURA/preSat Signal/Forward_Path'
 * '<S122>' : 'pid_control_V1/PID PITCH//ELEVATOR/Anti-windup'
 * '<S123>' : 'pid_control_V1/PID PITCH//ELEVATOR/D Gain'
 * '<S124>' : 'pid_control_V1/PID PITCH//ELEVATOR/External Derivative'
 * '<S125>' : 'pid_control_V1/PID PITCH//ELEVATOR/Filter'
 * '<S126>' : 'pid_control_V1/PID PITCH//ELEVATOR/Filter ICs'
 * '<S127>' : 'pid_control_V1/PID PITCH//ELEVATOR/I Gain'
 * '<S128>' : 'pid_control_V1/PID PITCH//ELEVATOR/Ideal P Gain'
 * '<S129>' : 'pid_control_V1/PID PITCH//ELEVATOR/Ideal P Gain Fdbk'
 * '<S130>' : 'pid_control_V1/PID PITCH//ELEVATOR/Integrator'
 * '<S131>' : 'pid_control_V1/PID PITCH//ELEVATOR/Integrator ICs'
 * '<S132>' : 'pid_control_V1/PID PITCH//ELEVATOR/N Copy'
 * '<S133>' : 'pid_control_V1/PID PITCH//ELEVATOR/N Gain'
 * '<S134>' : 'pid_control_V1/PID PITCH//ELEVATOR/P Copy'
 * '<S135>' : 'pid_control_V1/PID PITCH//ELEVATOR/Parallel P Gain'
 * '<S136>' : 'pid_control_V1/PID PITCH//ELEVATOR/Reset Signal'
 * '<S137>' : 'pid_control_V1/PID PITCH//ELEVATOR/Saturation'
 * '<S138>' : 'pid_control_V1/PID PITCH//ELEVATOR/Saturation Fdbk'
 * '<S139>' : 'pid_control_V1/PID PITCH//ELEVATOR/Sum'
 * '<S140>' : 'pid_control_V1/PID PITCH//ELEVATOR/Sum Fdbk'
 * '<S141>' : 'pid_control_V1/PID PITCH//ELEVATOR/Tracking Mode'
 * '<S142>' : 'pid_control_V1/PID PITCH//ELEVATOR/Tracking Mode Sum'
 * '<S143>' : 'pid_control_V1/PID PITCH//ELEVATOR/Tsamp - Integral'
 * '<S144>' : 'pid_control_V1/PID PITCH//ELEVATOR/Tsamp - Ngain'
 * '<S145>' : 'pid_control_V1/PID PITCH//ELEVATOR/postSat Signal'
 * '<S146>' : 'pid_control_V1/PID PITCH//ELEVATOR/preInt Signal'
 * '<S147>' : 'pid_control_V1/PID PITCH//ELEVATOR/preSat Signal'
 * '<S148>' : 'pid_control_V1/PID PITCH//ELEVATOR/Anti-windup/Back Calculation'
 * '<S149>' : 'pid_control_V1/PID PITCH//ELEVATOR/D Gain/Internal Parameters'
 * '<S150>' : 'pid_control_V1/PID PITCH//ELEVATOR/External Derivative/Error'
 * '<S151>' : 'pid_control_V1/PID PITCH//ELEVATOR/Filter/Cont. Filter'
 * '<S152>' : 'pid_control_V1/PID PITCH//ELEVATOR/Filter ICs/Internal IC - Filter'
 * '<S153>' : 'pid_control_V1/PID PITCH//ELEVATOR/I Gain/Internal Parameters'
 * '<S154>' : 'pid_control_V1/PID PITCH//ELEVATOR/Ideal P Gain/Passthrough'
 * '<S155>' : 'pid_control_V1/PID PITCH//ELEVATOR/Ideal P Gain Fdbk/Disabled'
 * '<S156>' : 'pid_control_V1/PID PITCH//ELEVATOR/Integrator/Continuous'
 * '<S157>' : 'pid_control_V1/PID PITCH//ELEVATOR/Integrator ICs/Internal IC'
 * '<S158>' : 'pid_control_V1/PID PITCH//ELEVATOR/N Copy/Disabled'
 * '<S159>' : 'pid_control_V1/PID PITCH//ELEVATOR/N Gain/Internal Parameters'
 * '<S160>' : 'pid_control_V1/PID PITCH//ELEVATOR/P Copy/Disabled'
 * '<S161>' : 'pid_control_V1/PID PITCH//ELEVATOR/Parallel P Gain/Internal Parameters'
 * '<S162>' : 'pid_control_V1/PID PITCH//ELEVATOR/Reset Signal/Disabled'
 * '<S163>' : 'pid_control_V1/PID PITCH//ELEVATOR/Saturation/Enabled'
 * '<S164>' : 'pid_control_V1/PID PITCH//ELEVATOR/Saturation Fdbk/Disabled'
 * '<S165>' : 'pid_control_V1/PID PITCH//ELEVATOR/Sum/Sum_PID'
 * '<S166>' : 'pid_control_V1/PID PITCH//ELEVATOR/Sum Fdbk/Disabled'
 * '<S167>' : 'pid_control_V1/PID PITCH//ELEVATOR/Tracking Mode/Disabled'
 * '<S168>' : 'pid_control_V1/PID PITCH//ELEVATOR/Tracking Mode Sum/Passthrough'
 * '<S169>' : 'pid_control_V1/PID PITCH//ELEVATOR/Tsamp - Integral/TsSignalSpecification'
 * '<S170>' : 'pid_control_V1/PID PITCH//ELEVATOR/Tsamp - Ngain/Passthrough'
 * '<S171>' : 'pid_control_V1/PID PITCH//ELEVATOR/postSat Signal/Forward_Path'
 * '<S172>' : 'pid_control_V1/PID PITCH//ELEVATOR/preInt Signal/Internal PreInt'
 * '<S173>' : 'pid_control_V1/PID PITCH//ELEVATOR/preSat Signal/Forward_Path'
 * '<S174>' : 'pid_control_V1/PID TIIMON/Anti-windup'
 * '<S175>' : 'pid_control_V1/PID TIIMON/D Gain'
 * '<S176>' : 'pid_control_V1/PID TIIMON/External Derivative'
 * '<S177>' : 'pid_control_V1/PID TIIMON/Filter'
 * '<S178>' : 'pid_control_V1/PID TIIMON/Filter ICs'
 * '<S179>' : 'pid_control_V1/PID TIIMON/I Gain'
 * '<S180>' : 'pid_control_V1/PID TIIMON/Ideal P Gain'
 * '<S181>' : 'pid_control_V1/PID TIIMON/Ideal P Gain Fdbk'
 * '<S182>' : 'pid_control_V1/PID TIIMON/Integrator'
 * '<S183>' : 'pid_control_V1/PID TIIMON/Integrator ICs'
 * '<S184>' : 'pid_control_V1/PID TIIMON/N Copy'
 * '<S185>' : 'pid_control_V1/PID TIIMON/N Gain'
 * '<S186>' : 'pid_control_V1/PID TIIMON/P Copy'
 * '<S187>' : 'pid_control_V1/PID TIIMON/Parallel P Gain'
 * '<S188>' : 'pid_control_V1/PID TIIMON/Reset Signal'
 * '<S189>' : 'pid_control_V1/PID TIIMON/Saturation'
 * '<S190>' : 'pid_control_V1/PID TIIMON/Saturation Fdbk'
 * '<S191>' : 'pid_control_V1/PID TIIMON/Sum'
 * '<S192>' : 'pid_control_V1/PID TIIMON/Sum Fdbk'
 * '<S193>' : 'pid_control_V1/PID TIIMON/Tracking Mode'
 * '<S194>' : 'pid_control_V1/PID TIIMON/Tracking Mode Sum'
 * '<S195>' : 'pid_control_V1/PID TIIMON/Tsamp - Integral'
 * '<S196>' : 'pid_control_V1/PID TIIMON/Tsamp - Ngain'
 * '<S197>' : 'pid_control_V1/PID TIIMON/postSat Signal'
 * '<S198>' : 'pid_control_V1/PID TIIMON/preInt Signal'
 * '<S199>' : 'pid_control_V1/PID TIIMON/preSat Signal'
 * '<S200>' : 'pid_control_V1/PID TIIMON/Anti-windup/Passthrough'
 * '<S201>' : 'pid_control_V1/PID TIIMON/D Gain/Internal Parameters'
 * '<S202>' : 'pid_control_V1/PID TIIMON/External Derivative/Error'
 * '<S203>' : 'pid_control_V1/PID TIIMON/Filter/Cont. Filter'
 * '<S204>' : 'pid_control_V1/PID TIIMON/Filter ICs/Internal IC - Filter'
 * '<S205>' : 'pid_control_V1/PID TIIMON/I Gain/Internal Parameters'
 * '<S206>' : 'pid_control_V1/PID TIIMON/Ideal P Gain/Passthrough'
 * '<S207>' : 'pid_control_V1/PID TIIMON/Ideal P Gain Fdbk/Disabled'
 * '<S208>' : 'pid_control_V1/PID TIIMON/Integrator/Continuous'
 * '<S209>' : 'pid_control_V1/PID TIIMON/Integrator ICs/Internal IC'
 * '<S210>' : 'pid_control_V1/PID TIIMON/N Copy/Disabled'
 * '<S211>' : 'pid_control_V1/PID TIIMON/N Gain/Internal Parameters'
 * '<S212>' : 'pid_control_V1/PID TIIMON/P Copy/Disabled'
 * '<S213>' : 'pid_control_V1/PID TIIMON/Parallel P Gain/Internal Parameters'
 * '<S214>' : 'pid_control_V1/PID TIIMON/Reset Signal/Disabled'
 * '<S215>' : 'pid_control_V1/PID TIIMON/Saturation/Enabled'
 * '<S216>' : 'pid_control_V1/PID TIIMON/Saturation Fdbk/Disabled'
 * '<S217>' : 'pid_control_V1/PID TIIMON/Sum/Sum_PID'
 * '<S218>' : 'pid_control_V1/PID TIIMON/Sum Fdbk/Disabled'
 * '<S219>' : 'pid_control_V1/PID TIIMON/Tracking Mode/Disabled'
 * '<S220>' : 'pid_control_V1/PID TIIMON/Tracking Mode Sum/Passthrough'
 * '<S221>' : 'pid_control_V1/PID TIIMON/Tsamp - Integral/TsSignalSpecification'
 * '<S222>' : 'pid_control_V1/PID TIIMON/Tsamp - Ngain/Passthrough'
 * '<S223>' : 'pid_control_V1/PID TIIMON/postSat Signal/Forward_Path'
 * '<S224>' : 'pid_control_V1/PID TIIMON/preInt Signal/Internal PreInt'
 * '<S225>' : 'pid_control_V1/PID TIIMON/preSat Signal/Forward_Path'
 * '<S226>' : 'pid_control_V1/PID VELOCIDAD/Anti-windup'
 * '<S227>' : 'pid_control_V1/PID VELOCIDAD/D Gain'
 * '<S228>' : 'pid_control_V1/PID VELOCIDAD/External Derivative'
 * '<S229>' : 'pid_control_V1/PID VELOCIDAD/Filter'
 * '<S230>' : 'pid_control_V1/PID VELOCIDAD/Filter ICs'
 * '<S231>' : 'pid_control_V1/PID VELOCIDAD/I Gain'
 * '<S232>' : 'pid_control_V1/PID VELOCIDAD/Ideal P Gain'
 * '<S233>' : 'pid_control_V1/PID VELOCIDAD/Ideal P Gain Fdbk'
 * '<S234>' : 'pid_control_V1/PID VELOCIDAD/Integrator'
 * '<S235>' : 'pid_control_V1/PID VELOCIDAD/Integrator ICs'
 * '<S236>' : 'pid_control_V1/PID VELOCIDAD/N Copy'
 * '<S237>' : 'pid_control_V1/PID VELOCIDAD/N Gain'
 * '<S238>' : 'pid_control_V1/PID VELOCIDAD/P Copy'
 * '<S239>' : 'pid_control_V1/PID VELOCIDAD/Parallel P Gain'
 * '<S240>' : 'pid_control_V1/PID VELOCIDAD/Reset Signal'
 * '<S241>' : 'pid_control_V1/PID VELOCIDAD/Saturation'
 * '<S242>' : 'pid_control_V1/PID VELOCIDAD/Saturation Fdbk'
 * '<S243>' : 'pid_control_V1/PID VELOCIDAD/Sum'
 * '<S244>' : 'pid_control_V1/PID VELOCIDAD/Sum Fdbk'
 * '<S245>' : 'pid_control_V1/PID VELOCIDAD/Tracking Mode'
 * '<S246>' : 'pid_control_V1/PID VELOCIDAD/Tracking Mode Sum'
 * '<S247>' : 'pid_control_V1/PID VELOCIDAD/Tsamp - Integral'
 * '<S248>' : 'pid_control_V1/PID VELOCIDAD/Tsamp - Ngain'
 * '<S249>' : 'pid_control_V1/PID VELOCIDAD/postSat Signal'
 * '<S250>' : 'pid_control_V1/PID VELOCIDAD/preInt Signal'
 * '<S251>' : 'pid_control_V1/PID VELOCIDAD/preSat Signal'
 * '<S252>' : 'pid_control_V1/PID VELOCIDAD/Anti-windup/Cont. Clamping Parallel'
 * '<S253>' : 'pid_control_V1/PID VELOCIDAD/Anti-windup/Cont. Clamping Parallel/Dead Zone'
 * '<S254>' : 'pid_control_V1/PID VELOCIDAD/Anti-windup/Cont. Clamping Parallel/Dead Zone/Enabled'
 * '<S255>' : 'pid_control_V1/PID VELOCIDAD/D Gain/Internal Parameters'
 * '<S256>' : 'pid_control_V1/PID VELOCIDAD/External Derivative/Error'
 * '<S257>' : 'pid_control_V1/PID VELOCIDAD/Filter/Cont. Filter'
 * '<S258>' : 'pid_control_V1/PID VELOCIDAD/Filter ICs/Internal IC - Filter'
 * '<S259>' : 'pid_control_V1/PID VELOCIDAD/I Gain/Internal Parameters'
 * '<S260>' : 'pid_control_V1/PID VELOCIDAD/Ideal P Gain/Passthrough'
 * '<S261>' : 'pid_control_V1/PID VELOCIDAD/Ideal P Gain Fdbk/Disabled'
 * '<S262>' : 'pid_control_V1/PID VELOCIDAD/Integrator/Continuous'
 * '<S263>' : 'pid_control_V1/PID VELOCIDAD/Integrator ICs/Internal IC'
 * '<S264>' : 'pid_control_V1/PID VELOCIDAD/N Copy/Disabled'
 * '<S265>' : 'pid_control_V1/PID VELOCIDAD/N Gain/Internal Parameters'
 * '<S266>' : 'pid_control_V1/PID VELOCIDAD/P Copy/Disabled'
 * '<S267>' : 'pid_control_V1/PID VELOCIDAD/Parallel P Gain/Internal Parameters'
 * '<S268>' : 'pid_control_V1/PID VELOCIDAD/Reset Signal/Disabled'
 * '<S269>' : 'pid_control_V1/PID VELOCIDAD/Saturation/Enabled'
 * '<S270>' : 'pid_control_V1/PID VELOCIDAD/Saturation Fdbk/Disabled'
 * '<S271>' : 'pid_control_V1/PID VELOCIDAD/Sum/Sum_PID'
 * '<S272>' : 'pid_control_V1/PID VELOCIDAD/Sum Fdbk/Disabled'
 * '<S273>' : 'pid_control_V1/PID VELOCIDAD/Tracking Mode/Disabled'
 * '<S274>' : 'pid_control_V1/PID VELOCIDAD/Tracking Mode Sum/Passthrough'
 * '<S275>' : 'pid_control_V1/PID VELOCIDAD/Tsamp - Integral/TsSignalSpecification'
 * '<S276>' : 'pid_control_V1/PID VELOCIDAD/Tsamp - Ngain/Passthrough'
 * '<S277>' : 'pid_control_V1/PID VELOCIDAD/postSat Signal/Forward_Path'
 * '<S278>' : 'pid_control_V1/PID VELOCIDAD/preInt Signal/Internal PreInt'
 * '<S279>' : 'pid_control_V1/PID VELOCIDAD/preSat Signal/Forward_Path'
 * '<S280>' : 'pid_control_V1/SUBSYSTEM_MODEL/Band-Limited White Noise'
 * '<S281>' : 'pid_control_V1/SUBSYSTEM_MODEL/Compare To Constant'
 * '<S282>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))'
 * '<S283>' : 'pid_control_V1/SUBSYSTEM_MODEL/MATLAB Function - MODEL'
 * '<S284>' : 'pid_control_V1/SUBSYSTEM_MODEL/MATLAB Function-reset'
 * '<S285>' : 'pid_control_V1/SUBSYSTEM_MODEL/Subscribe1_TURBULENCIA1'
 * '<S286>' : 'pid_control_V1/SUBSYSTEM_MODEL/Subscribe_HEAVE'
 * '<S287>' : 'pid_control_V1/SUBSYSTEM_MODEL/Subscribe_RATE'
 * '<S288>' : 'pid_control_V1/SUBSYSTEM_MODEL/Subscribe_TURBULENCIA'
 * '<S289>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Angle Conversion'
 * '<S290>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Filters on angular rates'
 * '<S291>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Filters on velocities'
 * '<S292>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Length Conversion'
 * '<S293>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Length Conversion1'
 * '<S294>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/RMS turbulence  intensities'
 * '<S295>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Select angular rates'
 * '<S296>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Select velocities'
 * '<S297>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Turbulence scale lengths'
 * '<S298>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Velocity Conversion'
 * '<S299>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Velocity Conversion2'
 * '<S300>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/White Noise'
 * '<S301>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Filters on angular rates/Hpgw'
 * '<S302>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Filters on angular rates/Hqgw'
 * '<S303>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Filters on angular rates/Hrgw'
 * '<S304>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Filters on velocities/Hugw(s)'
 * '<S305>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Filters on velocities/Hvgw(s)'
 * '<S306>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Filters on velocities/Hwgw(s)'
 * '<S307>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/RMS turbulence  intensities/High Altitude Intensity'
 * '<S308>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/RMS turbulence  intensities/Low Altitude Intensity'
 * '<S309>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Select angular rates/Interpolate  rates'
 * '<S310>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Select angular rates/Low altitude  rates'
 * '<S311>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Select angular rates/Medium//High  altitude rates'
 * '<S312>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Select angular rates/Merge Subsystems'
 * '<S313>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Select angular rates/Interpolate  rates/wind to body transformation'
 * '<S314>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Select angular rates/Interpolate  rates/wind to body transformation/convert to earth coords'
 * '<S315>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Select angular rates/Low altitude  rates/wind to body transformation'
 * '<S316>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Select angular rates/Low altitude  rates/wind to body transformation/convert to earth coords'
 * '<S317>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Select velocities/Interpolate  velocities'
 * '<S318>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Select velocities/Low altitude  velocities'
 * '<S319>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Select velocities/Medium//High  altitude velocities'
 * '<S320>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Select velocities/Merge Subsystems'
 * '<S321>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Select velocities/Interpolate  velocities/wind to body transformation'
 * '<S322>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Select velocities/Interpolate  velocities/wind to body transformation/convert to earth coords'
 * '<S323>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Select velocities/Low altitude  velocities/wind to body transformation'
 * '<S324>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Select velocities/Low altitude  velocities/wind to body transformation/convert to earth coords'
 * '<S325>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Turbulence scale lengths/Low altitude scale length'
 * '<S326>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Turbulence scale lengths/Medium//High altitude scale length'
 * '<S327>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Turbulence scale lengths/Medium//High altitude scale length/Length Conversion'
 * '<S328>' : 'pid_control_V1/SUBSYSTEM_MODEL/Subscribe1_TURBULENCIA1/Enabled Subsystem'
 * '<S329>' : 'pid_control_V1/SUBSYSTEM_MODEL/Subscribe_HEAVE/Enabled Subsystem'
 * '<S330>' : 'pid_control_V1/SUBSYSTEM_MODEL/Subscribe_RATE/Enabled Subsystem'
 * '<S331>' : 'pid_control_V1/SUBSYSTEM_MODEL/Subscribe_TURBULENCIA/Enabled Subsystem'
 * '<S332>' : 'pid_control_V1/Subscribe-ALTURA1/Enabled Subsystem'
 * '<S333>' : 'pid_control_V1/Subscribe-ROLL/Enabled Subsystem'
 * '<S334>' : 'pid_control_V1/Subscribe-YAW/Enabled Subsystem'
 */
#endif                                 /* pid_control_V1_h_ */
