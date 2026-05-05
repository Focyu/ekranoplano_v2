/*
 * pid_control_V3.h
 *
 * Trial License - for use to evaluate programs for possible purchase as
 * an end-user only.
 *
 * Code generation for model "pid_control_V3".
 *
 * Model version              : 12.149
 * Simulink Coder version : 25.2 (R2025b) 28-Jul-2025
 * C++ source code generated on : Mon May  4 23:01:20 2026
 *
 * Target selection: ert.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Intel->x86-64 (Linux 64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef pid_control_V3_h_
#define pid_control_V3_h_
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "slros2_initialize.h"
#include "pid_control_V3_types.h"

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

/* Block signals for system '<Root>/MATLAB Function' */
struct B_MATLABFunction_pid_control__T {
  uint32_T lengthOut;                  /* '<Root>/MATLAB Function' */
  uint8_T stringOut[128];              /* '<Root>/MATLAB Function' */
};

/* Block signals for system '<S286>/Enabled Subsystem' */
struct B_EnabledSubsystem_pid_contro_T {
  SL_Bus_std_msgs_Bool In1;            /* '<S329>/In1' */
};

/* Block signals for system '<S287>/Enabled Subsystem' */
struct B_EnabledSubsystem_pid_cont_d_T {
  SL_Bus_std_msgs_Float64 In1;         /* '<S330>/In1' */
};

/* Block signals (default storage) */
struct B_pid_control_V3_T {
  SL_Bus_sensor_msgs_Imu BusAssignment1;/* '<Root>/Bus Assignment1' */
  SL_Bus_gazebo_msgs_SetEntityStateRequest BusAssignment;/* '<Root>/Bus Assignment' */
  real_T IC[12];                       /* '<S13>/IC' */
  real_T x[12];                        /* '<S13>/Integrator' */
  uint8_T stringOut[128];              /* '<Root>/MATLAB Function1' */
  real_T R[9];
  real_T RotationAnglestoDirectionCo[9];
                        /* '<S13>/Rotation Angles to Direction Cosine Matrix' */
  real_T dv[9];
  real_T TmpSignalConversionAtSFunct[5];/* '<S13>/MATLAB Function - MODEL' */
  char_T b_zeroDelimTopic[25];
  real_T wbe_b[3];
  real_T FE1_b[3];
  real_T F_b[3];
  real_T Product_m[3];                 /* '<S322>/Product' */
  real_T Dtot[3];
  char_T b_zeroDelimTopic_c[22];
  char_T b_zeroDelimTopic_k[22];
  char_T b_zeroDelimTopic_cx[17];
  char_T b_zeroDelimTopic_b[17];
  char_T b_zeroDelimTopic_p[17];
  sJ4ih70VmKcvCeguWN0mNVF deadline;
  sJ4ih70VmKcvCeguWN0mNVF deadline_c;
  sJ4ih70VmKcvCeguWN0mNVF deadline_f;
  real_T frac[2];
  real_T dv1[2];
  real_T Gaintheta;                    /* '<Root>/Gain-theta' */
  real_T Gainpsi;                      /* '<Root>/Gain-psi' */
  real_T Gainphi;                      /* '<Root>/Gain-phi' */
  real_T Switch3;                      /* '<Root>/Switch3' */
  real_T Gain;                         /* '<Root>/Gain' */
  real_T FilterCoefficient;            /* '<S108>/Filter Coefficient' */
  real_T Saturation;                   /* '<S112>/Saturation' */
  real_T RateLimiter1;                 /* '<Root>/Rate Limiter1' */
  real_T FilterCoefficient_c;          /* '<S56>/Filter Coefficient' */
  real_T Saturation_k;                 /* '<S60>/Saturation' */
  real_T Saturation_i;                 /* '<Root>/Saturation' */
  real_T RateLimiter;                  /* '<Root>/Rate Limiter' */
  real_T FilterCoefficient_m;          /* '<S160>/Filter Coefficient' */
  real_T Saturation_f;                 /* '<S164>/Saturation' */
  real_T Switch2;                      /* '<Root>/Switch2' */
  real_T FilterCoefficient_p;          /* '<S212>/Filter Coefficient' */
  real_T Saturation_m;                 /* '<S216>/Saturation' */
  real_T FilterCoefficient_cv;         /* '<S266>/Filter Coefficient' */
  real_T Saturation_o;                 /* '<S270>/Saturation' */
  real_T Memory[3];                    /* '<S13>/Memory' */
  real_T Memory1[3];                   /* '<S13>/Memory1' */
  real_T Switch;                       /* '<S43>/Switch' */
  real_T SumI4;                        /* '<S97>/SumI4' */
  real_T SumI4_i;                      /* '<S149>/SumI4' */
  real_T SumI4_d;                      /* '<S201>/SumI4' */
  real_T Switch_j;                     /* '<S253>/Switch' */
  real_T Product[4];                   /* '<S301>/Product' */
  real_T Switch_p[3];                  /* '<S13>/Switch' */
  real_T Switch1[3];                   /* '<S13>/Switch1' */
  real_T data;
  real_T data_n;
  real_T Power;                        /* '<S13>/Product2' */
  real_T Gain3;                        /* '<S13>/Gain3' */
  real_T EnergykWh;                    /* '<S13>/Gain1' */
  real_T powerdemand;                  /* '<S13>/Divide' */
  real_T loadtorque;                   /* '<S13>/Divide1' */
  real_T Output;                       /* '<S281>/Output' */
  real_T Sum[3];                       /* '<S13>/Sum' */
  real_T Sum1[3];                      /* '<S13>/Sum1' */
  real_T XDOT[40];                     /* '<S13>/MATLAB Function - MODEL' */
  real_T CL_total;                     /* '<S13>/MATLAB Function - MODEL' */
  real_T mu_Lw_out;                    /* '<S13>/MATLAB Function - MODEL' */
  real_T mu_Dw_out;                    /* '<S13>/MATLAB Function - MODEL' */
  real_T w[2];                         /* '<S307>/w' */
  real_T w_a[2];                       /* '<S307>/w ' */
  real_T LwgV1[2];                     /* '<S307>/Lwg//V 1' */
  real_T w_g[2];                       /* '<S306>/w' */
  real_T w_e[2];                       /* '<S306>/w ' */
  real_T w1[2];                        /* '<S306>/w 1' */
  real_T w_n[2];                       /* '<S305>/w' */
  real_T w1_c[2];                      /* '<S305>/w1' */
  real_T w_d[2];                       /* '<S304>/w' */
  real_T w_e0[2];                      /* '<S303>/w' */
  real_T UnaryMinus[2];                /* '<S303>/Unary Minus' */
  real_T w_o[2];                       /* '<S302>/w' */
  real_T sigma_w[2];                   /* '<S302>/sigma_w' */
  real_T CoordinateTransformationConvers[4];
                             /* '<Root>/Coordinate Transformation Conversion' */
  real_T u1;
  real_T u2;
  real_T w_r;
  real_T beta;
  real_T q_aero;
  real_T hw;
  real_T hh;
  real_T Q;
  real_T CL_w_IGE;
  real_T CL_h_IGE;
  real_T CD_iw_IGE;
  real_T CD_ih_IGE;
  real_T Dtot_g;
  real_T Ltot;
  real_T CQ;
  real_T Cl;
  real_T Vd1;
  real_T L_dec;
  real_T s_phi;
  real_T c_the;
  real_T s_the;
  real_T c_psi;
  real_T s_psi;
  real_T sina;
  real_T sinb;
  real_T sinc;
  real_T cosa;
  real_T cosb;
  real_T cosc;
  real_T Sum1_g;                       /* '<Root>/Sum1' */
  real_T Sum_hl;                       /* '<S166>/Sum' */
  real_T Sum_a;                        /* '<S218>/Sum' */
  real_T FE_b;
  real_T Mcg_b_idx_0;
  real_T FE2_b_idx_0;
  real_T FE2_b_idx_2;
  real_T Fg_b_idx_2;
  real_T FE_b_idx_0;
  real_T FA_b_idx_0;
  real_T FA_b_idx_1;
  real_T FA_b_idx_2;
  real_T R_tmp;
  real_T R_tmp_g;
  real_T Ltot_tmp;
  real_T Switch1_m;                    /* '<Root>/Switch1' */
  SL_Bus_std_msgs_Float64 SourceBlock_o2_k;/* '<S288>/SourceBlock' */
  SL_Bus_std_msgs_Float64 SourceBlock_o2_p;/* '<S287>/SourceBlock' */
  SL_Bus_std_msgs_Float64 SourceBlock_o2;/* '<S16>/SourceBlock' */
  SL_Bus_std_msgs_Float64 SourceBlock_o2_o;/* '<S15>/SourceBlock' */
  uint32_T bpIndex[2];
  uint32_T lengthOut;                  /* '<Root>/MATLAB Function1' */
  boolean_T Compare;                   /* '<S282>/Compare' */
  boolean_T AND3;                      /* '<S43>/AND3' */
  boolean_T Memory_a;                  /* '<S43>/Memory' */
  boolean_T AND3_c;                    /* '<S253>/AND3' */
  boolean_T Memory_h;                  /* '<S253>/Memory' */
  boolean_T SourceBlock_o1;            /* '<S16>/SourceBlock' */
  boolean_T SourceBlock_o1_e;          /* '<S15>/SourceBlock' */
  boolean_T SourceBlock_o1_o;          /* '<S14>/SourceBlock' */
  boolean_T SourceBlock_o1_h;          /* '<S289>/SourceBlock' */
  boolean_T SourceBlock_o1_d;          /* '<S288>/SourceBlock' */
  boolean_T SourceBlock_o1_c;          /* '<S287>/SourceBlock' */
  boolean_T SourceBlock_o1_k;          /* '<S286>/SourceBlock' */
  B_EnabledSubsystem_pid_cont_d_T EnabledSubsystem_a;/* '<S16>/Enabled Subsystem' */
  B_EnabledSubsystem_pid_cont_d_T EnabledSubsystem_ps;/* '<S15>/Enabled Subsystem' */
  B_EnabledSubsystem_pid_cont_d_T EnabledSubsystem_b;/* '<S14>/Enabled Subsystem' */
  B_EnabledSubsystem_pid_contro_T EnabledSubsystem_pt;/* '<S289>/Enabled Subsystem' */
  B_EnabledSubsystem_pid_cont_d_T EnabledSubsystem_p;/* '<S288>/Enabled Subsystem' */
  B_EnabledSubsystem_pid_cont_d_T EnabledSubsystem_k;/* '<S287>/Enabled Subsystem' */
  B_EnabledSubsystem_pid_contro_T EnabledSubsystem;/* '<S286>/Enabled Subsystem' */
  B_MATLABFunction_pid_control__T sf_MATLABFunctionIMU2;/* '<Root>/MATLAB Function-IMU2' */
  B_MATLABFunction_pid_control__T sf_MATLABFunction;/* '<Root>/MATLAB Function' */
};

/* Block states (default storage) for system '<Root>' */
struct DW_pid_control_V3_T {
  ros_slros2_internal_block_Ser_T obj; /* '<S3>/ServiceCaller' */
  ros_slros2_internal_block_Pub_T obj_i;/* '<S12>/SinkBlock' */
  ros_slros2_internal_block_Sub_T obj_k;/* '<S16>/SourceBlock' */
  ros_slros2_internal_block_Sub_T obj_ip;/* '<S15>/SourceBlock' */
  ros_slros2_internal_block_Sub_T obj_m;/* '<S14>/SourceBlock' */
  ros_slros2_internal_block_Sub_T obj_h;/* '<S289>/SourceBlock' */
  ros_slros2_internal_block_Sub_T obj_p;/* '<S288>/SourceBlock' */
  ros_slros2_internal_block_Sub_T obj_hy;/* '<S287>/SourceBlock' */
  ros_slros2_internal_block_Sub_T obj_h4;/* '<S286>/SourceBlock' */
  real_T UnitDelay3_DSTATE;            /* '<Root>/Unit Delay3' */
  real_T UnitDelay1_DSTATE;            /* '<Root>/Unit Delay1' */
  real_T UnitDelay2_DSTATE;            /* '<Root>/Unit Delay2' */
  real_T Memory2_PreviousInput[12];    /* '<S13>/Memory2' */
  real_T PrevY;                        /* '<Root>/Rate Limiter1' */
  real_T PrevY_g;                      /* '<Root>/Rate Limiter' */
  real_T LastMajorTime;                /* '<Root>/Rate Limiter' */
  real_T Memory_PreviousInput[3];      /* '<S13>/Memory' */
  real_T Memory1_PreviousInput[3];     /* '<S13>/Memory1' */
  real_T NextOutput[4];                /* '<S301>/White Noise' */
  real_T NextOutput_k;                 /* '<S281>/White Noise' */
  struct {
    void *LoggedData;
  } ToWorkspace_PWORK;                 /* '<Root>/To Workspace' */

  struct {
    void *LoggedData;
  } ToWorkspace_PWORK_g;               /* '<S13>/To Workspace' */

  struct {
    void *LoggedData;
  } ToWorkspace1_PWORK;                /* '<S13>/To Workspace1' */

  struct {
    void *LoggedData;
  } ToWorkspace10_PWORK;               /* '<S13>/To Workspace10' */

  struct {
    void *LoggedData;
  } ToWorkspace11_PWORK;               /* '<S13>/To Workspace11' */

  struct {
    void *LoggedData;
  } ToWorkspace12_PWORK;               /* '<S13>/To Workspace12' */

  struct {
    void *LoggedData;
  } ToWorkspace13_PWORK;               /* '<S13>/To Workspace13' */

  struct {
    void *LoggedData;
  } ToWorkspace14_PWORK;               /* '<S13>/To Workspace14' */

  struct {
    void *LoggedData;
  } ToWorkspace15_PWORK;               /* '<S13>/To Workspace15' */

  struct {
    void *LoggedData;
  } ToWorkspace16_PWORK;               /* '<S13>/To Workspace16' */

  struct {
    void *LoggedData;
  } ToWorkspace17_PWORK;               /* '<S13>/To Workspace17' */

  struct {
    void *LoggedData;
  } ToWorkspace18_PWORK;               /* '<S13>/To Workspace18' */

  struct {
    void *LoggedData;
  } ToWorkspace19_PWORK;               /* '<S13>/To Workspace19' */

  struct {
    void *LoggedData;
  } ToWorkspace2_PWORK;                /* '<S13>/To Workspace2' */

  struct {
    void *LoggedData;
  } ToWorkspace20_PWORK;               /* '<S13>/To Workspace20' */

  struct {
    void *LoggedData;
  } ToWorkspace21_PWORK;               /* '<S13>/To Workspace21' */

  struct {
    void *LoggedData;
  } ToWorkspace22_PWORK;               /* '<S13>/To Workspace22' */

  struct {
    void *LoggedData;
  } ToWorkspace23_PWORK;               /* '<S13>/To Workspace23' */

  struct {
    void *LoggedData;
  } ToWorkspace24_PWORK;               /* '<S13>/To Workspace24' */

  struct {
    void *LoggedData;
  } ToWorkspace25_PWORK;               /* '<S13>/To Workspace25' */

  struct {
    void *LoggedData;
  } ToWorkspace26_PWORK;               /* '<S13>/To Workspace26' */

  struct {
    void *LoggedData;
  } ToWorkspace27_PWORK;               /* '<S13>/To Workspace27' */

  struct {
    void *LoggedData;
  } ToWorkspace28_PWORK;               /* '<S13>/To Workspace28' */

  struct {
    void *LoggedData;
  } ToWorkspace29_PWORK;               /* '<S13>/To Workspace29' */

  struct {
    void *LoggedData;
  } ToWorkspace3_PWORK;                /* '<S13>/To Workspace3' */

  struct {
    void *LoggedData;
  } ToWorkspace30_PWORK;               /* '<S13>/To Workspace30' */

  struct {
    void *LoggedData;
  } ToWorkspace31_PWORK;               /* '<S13>/To Workspace31' */

  struct {
    void *LoggedData;
  } ToWorkspace32_PWORK;               /* '<S13>/To Workspace32' */

  struct {
    void *LoggedData;
  } ToWorkspace33_PWORK;               /* '<S13>/To Workspace33' */

  struct {
    void *LoggedData;
  } ToWorkspace4_PWORK;                /* '<S13>/To Workspace4' */

  struct {
    void *LoggedData;
  } ToWorkspace5_PWORK;                /* '<S13>/To Workspace5' */

  struct {
    void *LoggedData;
  } ToWorkspace6_PWORK;                /* '<S13>/To Workspace6' */

  struct {
    void *LoggedData;
  } ToWorkspace7_PWORK;                /* '<S13>/To Workspace7' */

  struct {
    void *LoggedData;
  } ToWorkspace8_PWORK;                /* '<S13>/To Workspace8' */

  struct {
    void *LoggedData;
  } ToWorkspace9_PWORK;                /* '<S13>/To Workspace9' */

  uint32_T PreLookUpIndexSearchprobofexcee;
                        /* '<S308>/PreLook-Up Index Search  (prob of exceed)' */
  uint32_T PreLookUpIndexSearchaltitude_DW;
                              /* '<S308>/PreLook-Up Index Search  (altitude)' */
  uint32_T RandSeed[4];                /* '<S301>/White Noise' */
  uint32_T RandSeed_a;                 /* '<S281>/White Noise' */
  robotics_slcore_internal_bloc_T obj_c;
                             /* '<Root>/Coordinate Transformation Conversion' */
  int8_T ifHeightMaxlowaltitudeelseifHei;
  /* '<S297>/if Height < Max low altitude  elseif Height > Min isotropic altitude ' */
  int8_T ifHeightMaxlowaltitudeelseifH_a;
  /* '<S296>/if Height < Max low altitude  elseif Height > Min isotropic altitude ' */
  boolean_T IC_FirstOutputTime;        /* '<S13>/IC' */
  boolean_T Integrator_DWORK1;         /* '<S13>/Integrator' */
  boolean_T PrevLimited;               /* '<Root>/Rate Limiter' */
  boolean_T Memory_PreviousInput_o;    /* '<S43>/Memory' */
  boolean_T Memory_PreviousInput_a;    /* '<S253>/Memory' */
  boolean_T objisempty;                /* '<S16>/SourceBlock' */
  boolean_T objisempty_f;              /* '<S15>/SourceBlock' */
  boolean_T objisempty_g;              /* '<S14>/SourceBlock' */
  boolean_T objisempty_a;              /* '<S289>/SourceBlock' */
  boolean_T objisempty_e;              /* '<S288>/SourceBlock' */
  boolean_T objisempty_l;              /* '<S287>/SourceBlock' */
  boolean_T objisempty_c;              /* '<S286>/SourceBlock' */
  boolean_T objisempty_m;              /* '<S12>/SinkBlock' */
  boolean_T objisempty_d;    /* '<Root>/Coordinate Transformation Conversion' */
  boolean_T objisempty_ft;             /* '<S3>/ServiceCaller' */
  boolean_T Hwgws_MODE;                /* '<S292>/Hwgw(s)' */
  boolean_T Hvgws_MODE;                /* '<S292>/Hvgw(s)' */
  boolean_T Hugws_MODE;                /* '<S292>/Hugw(s)' */
  boolean_T Hrgw_MODE;                 /* '<S291>/Hrgw' */
  boolean_T Hqgw_MODE;                 /* '<S291>/Hqgw' */
  boolean_T Hpgw_MODE;                 /* '<S291>/Hpgw' */
};

/* Continuous states (default storage) */
struct X_pid_control_V3_T {
  real_T Integrator_CSTATE[12];        /* '<S13>/Integrator' */
  real_T Integrator_CSTATE_n;          /* '<S105>/Integrator' */
  real_T Filter_CSTATE;                /* '<S100>/Filter' */
  real_T Integrator_CSTATE_m;          /* '<S53>/Integrator' */
  real_T Filter_CSTATE_g;              /* '<S48>/Filter' */
  real_T Integrator_CSTATE_p;          /* '<S157>/Integrator' */
  real_T Filter_CSTATE_m;              /* '<S152>/Filter' */
  real_T Integrator_CSTATE_d;          /* '<S209>/Integrator' */
  real_T Filter_CSTATE_f;              /* '<S204>/Filter' */
  real_T Integrator_CSTATE_f;          /* '<S263>/Integrator' */
  real_T Filter_CSTATE_l;              /* '<S258>/Filter' */
  real_T Integrator1_CSTATE;           /* '<S13>/Integrator1' */
  real_T TransferFcn_CSTATE[2];        /* '<S13>/Transfer Fcn' */
  real_T TransferFcn1_CSTATE;          /* '<S13>/Transfer Fcn1' */
  real_T wg_p1_CSTATE[2];              /* '<S307>/wg_p1' */
  real_T wg_p2_CSTATE[2];              /* '<S307>/wg_p2' */
  real_T vg_p1_CSTATE[2];              /* '<S306>/vg_p1' */
  real_T vgw_p2_CSTATE[2];             /* '<S306>/vgw_p2' */
  real_T ug_p_CSTATE[2];               /* '<S305>/ug_p' */
  real_T rgw_p_CSTATE[2];              /* '<S304>/rgw_p' */
  real_T qgw_p_CSTATE[2];              /* '<S303>/qgw_p' */
  real_T pgw_p_CSTATE[2];              /* '<S302>/pgw_p' */
};

/* State derivatives (default storage) */
struct XDot_pid_control_V3_T {
  real_T Integrator_CSTATE[12];        /* '<S13>/Integrator' */
  real_T Integrator_CSTATE_n;          /* '<S105>/Integrator' */
  real_T Filter_CSTATE;                /* '<S100>/Filter' */
  real_T Integrator_CSTATE_m;          /* '<S53>/Integrator' */
  real_T Filter_CSTATE_g;              /* '<S48>/Filter' */
  real_T Integrator_CSTATE_p;          /* '<S157>/Integrator' */
  real_T Filter_CSTATE_m;              /* '<S152>/Filter' */
  real_T Integrator_CSTATE_d;          /* '<S209>/Integrator' */
  real_T Filter_CSTATE_f;              /* '<S204>/Filter' */
  real_T Integrator_CSTATE_f;          /* '<S263>/Integrator' */
  real_T Filter_CSTATE_l;              /* '<S258>/Filter' */
  real_T Integrator1_CSTATE;           /* '<S13>/Integrator1' */
  real_T TransferFcn_CSTATE[2];        /* '<S13>/Transfer Fcn' */
  real_T TransferFcn1_CSTATE;          /* '<S13>/Transfer Fcn1' */
  real_T wg_p1_CSTATE[2];              /* '<S307>/wg_p1' */
  real_T wg_p2_CSTATE[2];              /* '<S307>/wg_p2' */
  real_T vg_p1_CSTATE[2];              /* '<S306>/vg_p1' */
  real_T vgw_p2_CSTATE[2];             /* '<S306>/vgw_p2' */
  real_T ug_p_CSTATE[2];               /* '<S305>/ug_p' */
  real_T rgw_p_CSTATE[2];              /* '<S304>/rgw_p' */
  real_T qgw_p_CSTATE[2];              /* '<S303>/qgw_p' */
  real_T pgw_p_CSTATE[2];              /* '<S302>/pgw_p' */
};

/* State disabled  */
struct XDis_pid_control_V3_T {
  boolean_T Integrator_CSTATE[12];     /* '<S13>/Integrator' */
  boolean_T Integrator_CSTATE_n;       /* '<S105>/Integrator' */
  boolean_T Filter_CSTATE;             /* '<S100>/Filter' */
  boolean_T Integrator_CSTATE_m;       /* '<S53>/Integrator' */
  boolean_T Filter_CSTATE_g;           /* '<S48>/Filter' */
  boolean_T Integrator_CSTATE_p;       /* '<S157>/Integrator' */
  boolean_T Filter_CSTATE_m;           /* '<S152>/Filter' */
  boolean_T Integrator_CSTATE_d;       /* '<S209>/Integrator' */
  boolean_T Filter_CSTATE_f;           /* '<S204>/Filter' */
  boolean_T Integrator_CSTATE_f;       /* '<S263>/Integrator' */
  boolean_T Filter_CSTATE_l;           /* '<S258>/Filter' */
  boolean_T Integrator1_CSTATE;        /* '<S13>/Integrator1' */
  boolean_T TransferFcn_CSTATE[2];     /* '<S13>/Transfer Fcn' */
  boolean_T TransferFcn1_CSTATE;       /* '<S13>/Transfer Fcn1' */
  boolean_T wg_p1_CSTATE[2];           /* '<S307>/wg_p1' */
  boolean_T wg_p2_CSTATE[2];           /* '<S307>/wg_p2' */
  boolean_T vg_p1_CSTATE[2];           /* '<S306>/vg_p1' */
  boolean_T vgw_p2_CSTATE[2];          /* '<S306>/vgw_p2' */
  boolean_T ug_p_CSTATE[2];            /* '<S305>/ug_p' */
  boolean_T rgw_p_CSTATE[2];           /* '<S304>/rgw_p' */
  boolean_T qgw_p_CSTATE[2];           /* '<S303>/qgw_p' */
  boolean_T pgw_p_CSTATE[2];           /* '<S302>/pgw_p' */
};

/* Zero-crossing (trigger) state */
struct PrevZCX_pid_control_V3_T {
  ZCSigState Integrator_Reset_ZCE;     /* '<S13>/Integrator' */
};

/* Invariant block signals (default storage) */
struct ConstB_pid_control_V3_T {
  real_T UnitConversion;               /* '<S290>/Unit Conversion' */
  real_T UnitConversion_k;             /* '<S300>/Unit Conversion' */
  real_T sigma_wg;                     /* '<S309>/sigma_wg ' */
  real_T UnitConversion_n;             /* '<S294>/Unit Conversion' */
  real_T UnitConversion_c;             /* '<S328>/Unit Conversion' */
  real_T PreLookUpIndexSearchprobofe;
                        /* '<S308>/PreLook-Up Index Search  (prob of exceed)' */
  real_T Sqrt[4];                      /* '<S301>/Sqrt' */
  real_T Sqrt1;                        /* '<S301>/Sqrt1' */
  real_T Divide[4];                    /* '<S301>/Divide' */
  real_T motorspeed;                   /* '<S13>/Gain2' */
  real_T Sum;                          /* '<S318>/Sum' */
  real_T Sum_a;                        /* '<S310>/Sum' */
  real_T sqrt_a;                       /* '<S307>/sqrt' */
  real_T w4;                           /* '<S302>/w4' */
  real_T u16;                          /* '<S302>/u^1//6' */
  uint32_T PreLookUpIndexSearchprobo_g;
                        /* '<S308>/PreLook-Up Index Search  (prob of exceed)' */
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
struct ConstP_pid_control_V3_T {
  /* Pooled Parameter (Expression: x_nom)
   * Referenced by:
   *   '<S13>/IC'
   *   '<S13>/Memory2'
   */
  real_T pooled10[12];

  /* Expression: h_vec
   * Referenced by: '<S308>/PreLook-Up Index Search  (altitude)'
   */
  real_T PreLookUpIndexSearchaltitude_Br[12];

  /* Expression: sigma_vec'
   * Referenced by: '<S308>/Medium//High Altitude Intensity'
   */
  real_T MediumHighAltitudeIntensity_Tab[84];

  /* Computed Parameter: MediumHighAltitudeIntensity_max
   * Referenced by: '<S308>/Medium//High Altitude Intensity'
   */
  uint32_T MediumHighAltitudeIntensity_max[2];
};

/* Real-time Model Data Structure */
struct tag_RTM_pid_control_V3_T {
  const char_T *errorStatus;
  RTWSolverInfo solverInfo;
  X_pid_control_V3_T *contStates;
  int_T *periodicContStateIndices;
  real_T *periodicContStateRanges;
  real_T *derivs;
  XDis_pid_control_V3_T *contStateDisabled;
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

extern const ConstB_pid_control_V3_T pid_control_V3_ConstB;/* constant block i/o */

/* Constant parameters (default storage) */
extern const ConstP_pid_control_V3_T pid_control_V3_ConstP;

/* Class declaration for model pid_control_V3 */
class pid_control_V3
{
  /* public data and function members */
 public:
  /* Real-Time Model get method */
  RT_MODEL_pid_control_V3_T * getRTM();
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
  pid_control_V3();

  /* Destructor */
  ~pid_control_V3();

  /* private data and function members */
 private:
  /* Block signals */
  B_pid_control_V3_T pid_control_V3_B;

  /* Block states */
  DW_pid_control_V3_T pid_control_V3_DW;

  /* Block continuous states */
  X_pid_control_V3_T pid_control_V3_X;

  /* Block Continuous state disabled vector */
  XDis_pid_control_V3_T pid_control_V3_XDis;

  /* Triggered events */
  PrevZCX_pid_control_V3_T pid_control_V3_PrevZCX;

  /* private member function(s) for subsystem '<Root>/MATLAB Function'*/
  static void pid_control_V3_MATLABFunction(B_MATLABFunction_pid_control__T
    *localB);

  /* private member function(s) for subsystem '<S286>/Enabled Subsystem'*/
  static void pid_contr_EnabledSubsystem_Init(B_EnabledSubsystem_pid_contro_T
    *localB);
  static void pid_control_V3_EnabledSubsystem(boolean_T rtu_Enable, const
    SL_Bus_std_msgs_Bool *rtu_In1, B_EnabledSubsystem_pid_contro_T *localB);

  /* private member function(s) for subsystem '<S287>/Enabled Subsystem'*/
  static void pid_con_EnabledSubsystem_i_Init(B_EnabledSubsystem_pid_cont_d_T
    *localB);
  static void pid_control__EnabledSubsystem_k(boolean_T rtu_Enable, const
    SL_Bus_std_msgs_Float64 *rtu_In1, B_EnabledSubsystem_pid_cont_d_T *localB);

  /* private member function(s) for subsystem '<Root>'*/
  void pid_control_Publisher_setupImpl(const ros_slros2_internal_block_Pub_T
    *obj);
  void pid_c_Subscriber_setupImpl_eg11(const ros_slros2_internal_block_Sub_T
    *obj);
  void pid__Subscriber_setupImpl_eg11g(const ros_slros2_internal_block_Sub_T
    *obj);
  void pid_Subscriber_setupImpl_eg11gb(const ros_slros2_internal_block_Sub_T
    *obj);
  void pid_con_ServiceCaller_setupImpl(const ros_slros2_internal_block_Ser_T
    *obj);
  void pid_co_Subscriber_setupImpl_eg1(const ros_slros2_internal_block_Sub_T
    *obj);
  void pid_contro_Subscriber_setupImpl(const ros_slros2_internal_block_Sub_T
    *obj);
  void pid_cont_Subscriber_setupImpl_e(const ros_slros2_internal_block_Sub_T
    *obj);
  void pid_con_Subscriber_setupImpl_eg(const ros_slros2_internal_block_Sub_T
    *obj);

  /* Global mass matrix */

  /* Continuous states update member function*/
  void rt_ertODEUpdateContinuousStates(RTWSolverInfo *si );

  /* Derivatives member function */
  void pid_control_V3_derivatives();

  /* Real-Time Model */
  RT_MODEL_pid_control_V3_T pid_control_V3_M;
};

extern volatile boolean_T stopRequested;
extern volatile boolean_T runModel;

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<Root>/Display' : Unused code path elimination
 * Block '<Root>/Gain3' : Eliminated nontunable gain of 1
 * Block '<S149>/Kb' : Eliminated nontunable gain of 1
 * Block '<S283>/Cast' : Eliminate redundant data type conversion
 * Block '<S283>/Cast To Double' : Eliminate redundant data type conversion
 * Block '<S283>/Cast To Double1' : Eliminate redundant data type conversion
 * Block '<S283>/Cast To Double2' : Eliminate redundant data type conversion
 * Block '<S283>/Cast To Double3' : Eliminate redundant data type conversion
 * Block '<S283>/Cast To Double4' : Eliminate redundant data type conversion
 * Block '<S314>/Reshape' : Reshape block reduction
 * Block '<S314>/Reshape1' : Reshape block reduction
 * Block '<S316>/Reshape' : Reshape block reduction
 * Block '<S322>/Reshape' : Reshape block reduction
 * Block '<S322>/Reshape1' : Reshape block reduction
 * Block '<S324>/Reshape' : Reshape block reduction
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
 * '<Root>' : 'pid_control_V3'
 * '<S1>'   : 'pid_control_V3/Blank Message'
 * '<S2>'   : 'pid_control_V3/Blank Message2'
 * '<S3>'   : 'pid_control_V3/Call Service'
 * '<S4>'   : 'pid_control_V3/MATLAB Function'
 * '<S5>'   : 'pid_control_V3/MATLAB Function-IMU2'
 * '<S6>'   : 'pid_control_V3/MATLAB Function1'
 * '<S7>'   : 'pid_control_V3/PID ALERON'
 * '<S8>'   : 'pid_control_V3/PID ALTURA'
 * '<S9>'   : 'pid_control_V3/PID ELEVATOR'
 * '<S10>'  : 'pid_control_V3/PID TIIMON'
 * '<S11>'  : 'pid_control_V3/PID VELOCIDAD'
 * '<S12>'  : 'pid_control_V3/Publish'
 * '<S13>'  : 'pid_control_V3/SUBSYSTEM_MODEL'
 * '<S14>'  : 'pid_control_V3/Subscribe-ALTURA1'
 * '<S15>'  : 'pid_control_V3/Subscribe-ROLL'
 * '<S16>'  : 'pid_control_V3/Subscribe-YAW'
 * '<S17>'  : 'pid_control_V3/PID ALERON/Anti-windup'
 * '<S18>'  : 'pid_control_V3/PID ALERON/D Gain'
 * '<S19>'  : 'pid_control_V3/PID ALERON/External Derivative'
 * '<S20>'  : 'pid_control_V3/PID ALERON/Filter'
 * '<S21>'  : 'pid_control_V3/PID ALERON/Filter ICs'
 * '<S22>'  : 'pid_control_V3/PID ALERON/I Gain'
 * '<S23>'  : 'pid_control_V3/PID ALERON/Ideal P Gain'
 * '<S24>'  : 'pid_control_V3/PID ALERON/Ideal P Gain Fdbk'
 * '<S25>'  : 'pid_control_V3/PID ALERON/Integrator'
 * '<S26>'  : 'pid_control_V3/PID ALERON/Integrator ICs'
 * '<S27>'  : 'pid_control_V3/PID ALERON/N Copy'
 * '<S28>'  : 'pid_control_V3/PID ALERON/N Gain'
 * '<S29>'  : 'pid_control_V3/PID ALERON/P Copy'
 * '<S30>'  : 'pid_control_V3/PID ALERON/Parallel P Gain'
 * '<S31>'  : 'pid_control_V3/PID ALERON/Reset Signal'
 * '<S32>'  : 'pid_control_V3/PID ALERON/Saturation'
 * '<S33>'  : 'pid_control_V3/PID ALERON/Saturation Fdbk'
 * '<S34>'  : 'pid_control_V3/PID ALERON/Sum'
 * '<S35>'  : 'pid_control_V3/PID ALERON/Sum Fdbk'
 * '<S36>'  : 'pid_control_V3/PID ALERON/Tracking Mode'
 * '<S37>'  : 'pid_control_V3/PID ALERON/Tracking Mode Sum'
 * '<S38>'  : 'pid_control_V3/PID ALERON/Tsamp - Integral'
 * '<S39>'  : 'pid_control_V3/PID ALERON/Tsamp - Ngain'
 * '<S40>'  : 'pid_control_V3/PID ALERON/postSat Signal'
 * '<S41>'  : 'pid_control_V3/PID ALERON/preInt Signal'
 * '<S42>'  : 'pid_control_V3/PID ALERON/preSat Signal'
 * '<S43>'  : 'pid_control_V3/PID ALERON/Anti-windup/Cont. Clamping Parallel'
 * '<S44>'  : 'pid_control_V3/PID ALERON/Anti-windup/Cont. Clamping Parallel/Dead Zone'
 * '<S45>'  : 'pid_control_V3/PID ALERON/Anti-windup/Cont. Clamping Parallel/Dead Zone/Enabled'
 * '<S46>'  : 'pid_control_V3/PID ALERON/D Gain/Internal Parameters'
 * '<S47>'  : 'pid_control_V3/PID ALERON/External Derivative/Error'
 * '<S48>'  : 'pid_control_V3/PID ALERON/Filter/Cont. Filter'
 * '<S49>'  : 'pid_control_V3/PID ALERON/Filter ICs/Internal IC - Filter'
 * '<S50>'  : 'pid_control_V3/PID ALERON/I Gain/Internal Parameters'
 * '<S51>'  : 'pid_control_V3/PID ALERON/Ideal P Gain/Passthrough'
 * '<S52>'  : 'pid_control_V3/PID ALERON/Ideal P Gain Fdbk/Disabled'
 * '<S53>'  : 'pid_control_V3/PID ALERON/Integrator/Continuous'
 * '<S54>'  : 'pid_control_V3/PID ALERON/Integrator ICs/Internal IC'
 * '<S55>'  : 'pid_control_V3/PID ALERON/N Copy/Disabled'
 * '<S56>'  : 'pid_control_V3/PID ALERON/N Gain/Internal Parameters'
 * '<S57>'  : 'pid_control_V3/PID ALERON/P Copy/Disabled'
 * '<S58>'  : 'pid_control_V3/PID ALERON/Parallel P Gain/Internal Parameters'
 * '<S59>'  : 'pid_control_V3/PID ALERON/Reset Signal/Disabled'
 * '<S60>'  : 'pid_control_V3/PID ALERON/Saturation/Enabled'
 * '<S61>'  : 'pid_control_V3/PID ALERON/Saturation Fdbk/Disabled'
 * '<S62>'  : 'pid_control_V3/PID ALERON/Sum/Sum_PID'
 * '<S63>'  : 'pid_control_V3/PID ALERON/Sum Fdbk/Disabled'
 * '<S64>'  : 'pid_control_V3/PID ALERON/Tracking Mode/Disabled'
 * '<S65>'  : 'pid_control_V3/PID ALERON/Tracking Mode Sum/Passthrough'
 * '<S66>'  : 'pid_control_V3/PID ALERON/Tsamp - Integral/TsSignalSpecification'
 * '<S67>'  : 'pid_control_V3/PID ALERON/Tsamp - Ngain/Passthrough'
 * '<S68>'  : 'pid_control_V3/PID ALERON/postSat Signal/Forward_Path'
 * '<S69>'  : 'pid_control_V3/PID ALERON/preInt Signal/Internal PreInt'
 * '<S70>'  : 'pid_control_V3/PID ALERON/preSat Signal/Forward_Path'
 * '<S71>'  : 'pid_control_V3/PID ALTURA/Anti-windup'
 * '<S72>'  : 'pid_control_V3/PID ALTURA/D Gain'
 * '<S73>'  : 'pid_control_V3/PID ALTURA/External Derivative'
 * '<S74>'  : 'pid_control_V3/PID ALTURA/Filter'
 * '<S75>'  : 'pid_control_V3/PID ALTURA/Filter ICs'
 * '<S76>'  : 'pid_control_V3/PID ALTURA/I Gain'
 * '<S77>'  : 'pid_control_V3/PID ALTURA/Ideal P Gain'
 * '<S78>'  : 'pid_control_V3/PID ALTURA/Ideal P Gain Fdbk'
 * '<S79>'  : 'pid_control_V3/PID ALTURA/Integrator'
 * '<S80>'  : 'pid_control_V3/PID ALTURA/Integrator ICs'
 * '<S81>'  : 'pid_control_V3/PID ALTURA/N Copy'
 * '<S82>'  : 'pid_control_V3/PID ALTURA/N Gain'
 * '<S83>'  : 'pid_control_V3/PID ALTURA/P Copy'
 * '<S84>'  : 'pid_control_V3/PID ALTURA/Parallel P Gain'
 * '<S85>'  : 'pid_control_V3/PID ALTURA/Reset Signal'
 * '<S86>'  : 'pid_control_V3/PID ALTURA/Saturation'
 * '<S87>'  : 'pid_control_V3/PID ALTURA/Saturation Fdbk'
 * '<S88>'  : 'pid_control_V3/PID ALTURA/Sum'
 * '<S89>'  : 'pid_control_V3/PID ALTURA/Sum Fdbk'
 * '<S90>'  : 'pid_control_V3/PID ALTURA/Tracking Mode'
 * '<S91>'  : 'pid_control_V3/PID ALTURA/Tracking Mode Sum'
 * '<S92>'  : 'pid_control_V3/PID ALTURA/Tsamp - Integral'
 * '<S93>'  : 'pid_control_V3/PID ALTURA/Tsamp - Ngain'
 * '<S94>'  : 'pid_control_V3/PID ALTURA/postSat Signal'
 * '<S95>'  : 'pid_control_V3/PID ALTURA/preInt Signal'
 * '<S96>'  : 'pid_control_V3/PID ALTURA/preSat Signal'
 * '<S97>'  : 'pid_control_V3/PID ALTURA/Anti-windup/Back Calculation'
 * '<S98>'  : 'pid_control_V3/PID ALTURA/D Gain/Internal Parameters'
 * '<S99>'  : 'pid_control_V3/PID ALTURA/External Derivative/Error'
 * '<S100>' : 'pid_control_V3/PID ALTURA/Filter/Cont. Filter'
 * '<S101>' : 'pid_control_V3/PID ALTURA/Filter ICs/Internal IC - Filter'
 * '<S102>' : 'pid_control_V3/PID ALTURA/I Gain/Internal Parameters'
 * '<S103>' : 'pid_control_V3/PID ALTURA/Ideal P Gain/Passthrough'
 * '<S104>' : 'pid_control_V3/PID ALTURA/Ideal P Gain Fdbk/Disabled'
 * '<S105>' : 'pid_control_V3/PID ALTURA/Integrator/Continuous'
 * '<S106>' : 'pid_control_V3/PID ALTURA/Integrator ICs/Internal IC'
 * '<S107>' : 'pid_control_V3/PID ALTURA/N Copy/Disabled'
 * '<S108>' : 'pid_control_V3/PID ALTURA/N Gain/Internal Parameters'
 * '<S109>' : 'pid_control_V3/PID ALTURA/P Copy/Disabled'
 * '<S110>' : 'pid_control_V3/PID ALTURA/Parallel P Gain/Internal Parameters'
 * '<S111>' : 'pid_control_V3/PID ALTURA/Reset Signal/Disabled'
 * '<S112>' : 'pid_control_V3/PID ALTURA/Saturation/Enabled'
 * '<S113>' : 'pid_control_V3/PID ALTURA/Saturation Fdbk/Disabled'
 * '<S114>' : 'pid_control_V3/PID ALTURA/Sum/Sum_PID'
 * '<S115>' : 'pid_control_V3/PID ALTURA/Sum Fdbk/Disabled'
 * '<S116>' : 'pid_control_V3/PID ALTURA/Tracking Mode/Disabled'
 * '<S117>' : 'pid_control_V3/PID ALTURA/Tracking Mode Sum/Passthrough'
 * '<S118>' : 'pid_control_V3/PID ALTURA/Tsamp - Integral/TsSignalSpecification'
 * '<S119>' : 'pid_control_V3/PID ALTURA/Tsamp - Ngain/Passthrough'
 * '<S120>' : 'pid_control_V3/PID ALTURA/postSat Signal/Forward_Path'
 * '<S121>' : 'pid_control_V3/PID ALTURA/preInt Signal/Internal PreInt'
 * '<S122>' : 'pid_control_V3/PID ALTURA/preSat Signal/Forward_Path'
 * '<S123>' : 'pid_control_V3/PID ELEVATOR/Anti-windup'
 * '<S124>' : 'pid_control_V3/PID ELEVATOR/D Gain'
 * '<S125>' : 'pid_control_V3/PID ELEVATOR/External Derivative'
 * '<S126>' : 'pid_control_V3/PID ELEVATOR/Filter'
 * '<S127>' : 'pid_control_V3/PID ELEVATOR/Filter ICs'
 * '<S128>' : 'pid_control_V3/PID ELEVATOR/I Gain'
 * '<S129>' : 'pid_control_V3/PID ELEVATOR/Ideal P Gain'
 * '<S130>' : 'pid_control_V3/PID ELEVATOR/Ideal P Gain Fdbk'
 * '<S131>' : 'pid_control_V3/PID ELEVATOR/Integrator'
 * '<S132>' : 'pid_control_V3/PID ELEVATOR/Integrator ICs'
 * '<S133>' : 'pid_control_V3/PID ELEVATOR/N Copy'
 * '<S134>' : 'pid_control_V3/PID ELEVATOR/N Gain'
 * '<S135>' : 'pid_control_V3/PID ELEVATOR/P Copy'
 * '<S136>' : 'pid_control_V3/PID ELEVATOR/Parallel P Gain'
 * '<S137>' : 'pid_control_V3/PID ELEVATOR/Reset Signal'
 * '<S138>' : 'pid_control_V3/PID ELEVATOR/Saturation'
 * '<S139>' : 'pid_control_V3/PID ELEVATOR/Saturation Fdbk'
 * '<S140>' : 'pid_control_V3/PID ELEVATOR/Sum'
 * '<S141>' : 'pid_control_V3/PID ELEVATOR/Sum Fdbk'
 * '<S142>' : 'pid_control_V3/PID ELEVATOR/Tracking Mode'
 * '<S143>' : 'pid_control_V3/PID ELEVATOR/Tracking Mode Sum'
 * '<S144>' : 'pid_control_V3/PID ELEVATOR/Tsamp - Integral'
 * '<S145>' : 'pid_control_V3/PID ELEVATOR/Tsamp - Ngain'
 * '<S146>' : 'pid_control_V3/PID ELEVATOR/postSat Signal'
 * '<S147>' : 'pid_control_V3/PID ELEVATOR/preInt Signal'
 * '<S148>' : 'pid_control_V3/PID ELEVATOR/preSat Signal'
 * '<S149>' : 'pid_control_V3/PID ELEVATOR/Anti-windup/Back Calculation'
 * '<S150>' : 'pid_control_V3/PID ELEVATOR/D Gain/Internal Parameters'
 * '<S151>' : 'pid_control_V3/PID ELEVATOR/External Derivative/Error'
 * '<S152>' : 'pid_control_V3/PID ELEVATOR/Filter/Cont. Filter'
 * '<S153>' : 'pid_control_V3/PID ELEVATOR/Filter ICs/Internal IC - Filter'
 * '<S154>' : 'pid_control_V3/PID ELEVATOR/I Gain/Internal Parameters'
 * '<S155>' : 'pid_control_V3/PID ELEVATOR/Ideal P Gain/Passthrough'
 * '<S156>' : 'pid_control_V3/PID ELEVATOR/Ideal P Gain Fdbk/Disabled'
 * '<S157>' : 'pid_control_V3/PID ELEVATOR/Integrator/Continuous'
 * '<S158>' : 'pid_control_V3/PID ELEVATOR/Integrator ICs/Internal IC'
 * '<S159>' : 'pid_control_V3/PID ELEVATOR/N Copy/Disabled'
 * '<S160>' : 'pid_control_V3/PID ELEVATOR/N Gain/Internal Parameters'
 * '<S161>' : 'pid_control_V3/PID ELEVATOR/P Copy/Disabled'
 * '<S162>' : 'pid_control_V3/PID ELEVATOR/Parallel P Gain/Internal Parameters'
 * '<S163>' : 'pid_control_V3/PID ELEVATOR/Reset Signal/Disabled'
 * '<S164>' : 'pid_control_V3/PID ELEVATOR/Saturation/Enabled'
 * '<S165>' : 'pid_control_V3/PID ELEVATOR/Saturation Fdbk/Disabled'
 * '<S166>' : 'pid_control_V3/PID ELEVATOR/Sum/Sum_PID'
 * '<S167>' : 'pid_control_V3/PID ELEVATOR/Sum Fdbk/Disabled'
 * '<S168>' : 'pid_control_V3/PID ELEVATOR/Tracking Mode/Disabled'
 * '<S169>' : 'pid_control_V3/PID ELEVATOR/Tracking Mode Sum/Passthrough'
 * '<S170>' : 'pid_control_V3/PID ELEVATOR/Tsamp - Integral/TsSignalSpecification'
 * '<S171>' : 'pid_control_V3/PID ELEVATOR/Tsamp - Ngain/Passthrough'
 * '<S172>' : 'pid_control_V3/PID ELEVATOR/postSat Signal/Forward_Path'
 * '<S173>' : 'pid_control_V3/PID ELEVATOR/preInt Signal/Internal PreInt'
 * '<S174>' : 'pid_control_V3/PID ELEVATOR/preSat Signal/Forward_Path'
 * '<S175>' : 'pid_control_V3/PID TIIMON/Anti-windup'
 * '<S176>' : 'pid_control_V3/PID TIIMON/D Gain'
 * '<S177>' : 'pid_control_V3/PID TIIMON/External Derivative'
 * '<S178>' : 'pid_control_V3/PID TIIMON/Filter'
 * '<S179>' : 'pid_control_V3/PID TIIMON/Filter ICs'
 * '<S180>' : 'pid_control_V3/PID TIIMON/I Gain'
 * '<S181>' : 'pid_control_V3/PID TIIMON/Ideal P Gain'
 * '<S182>' : 'pid_control_V3/PID TIIMON/Ideal P Gain Fdbk'
 * '<S183>' : 'pid_control_V3/PID TIIMON/Integrator'
 * '<S184>' : 'pid_control_V3/PID TIIMON/Integrator ICs'
 * '<S185>' : 'pid_control_V3/PID TIIMON/N Copy'
 * '<S186>' : 'pid_control_V3/PID TIIMON/N Gain'
 * '<S187>' : 'pid_control_V3/PID TIIMON/P Copy'
 * '<S188>' : 'pid_control_V3/PID TIIMON/Parallel P Gain'
 * '<S189>' : 'pid_control_V3/PID TIIMON/Reset Signal'
 * '<S190>' : 'pid_control_V3/PID TIIMON/Saturation'
 * '<S191>' : 'pid_control_V3/PID TIIMON/Saturation Fdbk'
 * '<S192>' : 'pid_control_V3/PID TIIMON/Sum'
 * '<S193>' : 'pid_control_V3/PID TIIMON/Sum Fdbk'
 * '<S194>' : 'pid_control_V3/PID TIIMON/Tracking Mode'
 * '<S195>' : 'pid_control_V3/PID TIIMON/Tracking Mode Sum'
 * '<S196>' : 'pid_control_V3/PID TIIMON/Tsamp - Integral'
 * '<S197>' : 'pid_control_V3/PID TIIMON/Tsamp - Ngain'
 * '<S198>' : 'pid_control_V3/PID TIIMON/postSat Signal'
 * '<S199>' : 'pid_control_V3/PID TIIMON/preInt Signal'
 * '<S200>' : 'pid_control_V3/PID TIIMON/preSat Signal'
 * '<S201>' : 'pid_control_V3/PID TIIMON/Anti-windup/Back Calculation'
 * '<S202>' : 'pid_control_V3/PID TIIMON/D Gain/Internal Parameters'
 * '<S203>' : 'pid_control_V3/PID TIIMON/External Derivative/Error'
 * '<S204>' : 'pid_control_V3/PID TIIMON/Filter/Cont. Filter'
 * '<S205>' : 'pid_control_V3/PID TIIMON/Filter ICs/Internal IC - Filter'
 * '<S206>' : 'pid_control_V3/PID TIIMON/I Gain/Internal Parameters'
 * '<S207>' : 'pid_control_V3/PID TIIMON/Ideal P Gain/Passthrough'
 * '<S208>' : 'pid_control_V3/PID TIIMON/Ideal P Gain Fdbk/Disabled'
 * '<S209>' : 'pid_control_V3/PID TIIMON/Integrator/Continuous'
 * '<S210>' : 'pid_control_V3/PID TIIMON/Integrator ICs/Internal IC'
 * '<S211>' : 'pid_control_V3/PID TIIMON/N Copy/Disabled'
 * '<S212>' : 'pid_control_V3/PID TIIMON/N Gain/Internal Parameters'
 * '<S213>' : 'pid_control_V3/PID TIIMON/P Copy/Disabled'
 * '<S214>' : 'pid_control_V3/PID TIIMON/Parallel P Gain/Internal Parameters'
 * '<S215>' : 'pid_control_V3/PID TIIMON/Reset Signal/Disabled'
 * '<S216>' : 'pid_control_V3/PID TIIMON/Saturation/Enabled'
 * '<S217>' : 'pid_control_V3/PID TIIMON/Saturation Fdbk/Disabled'
 * '<S218>' : 'pid_control_V3/PID TIIMON/Sum/Sum_PID'
 * '<S219>' : 'pid_control_V3/PID TIIMON/Sum Fdbk/Disabled'
 * '<S220>' : 'pid_control_V3/PID TIIMON/Tracking Mode/Disabled'
 * '<S221>' : 'pid_control_V3/PID TIIMON/Tracking Mode Sum/Passthrough'
 * '<S222>' : 'pid_control_V3/PID TIIMON/Tsamp - Integral/TsSignalSpecification'
 * '<S223>' : 'pid_control_V3/PID TIIMON/Tsamp - Ngain/Passthrough'
 * '<S224>' : 'pid_control_V3/PID TIIMON/postSat Signal/Forward_Path'
 * '<S225>' : 'pid_control_V3/PID TIIMON/preInt Signal/Internal PreInt'
 * '<S226>' : 'pid_control_V3/PID TIIMON/preSat Signal/Forward_Path'
 * '<S227>' : 'pid_control_V3/PID VELOCIDAD/Anti-windup'
 * '<S228>' : 'pid_control_V3/PID VELOCIDAD/D Gain'
 * '<S229>' : 'pid_control_V3/PID VELOCIDAD/External Derivative'
 * '<S230>' : 'pid_control_V3/PID VELOCIDAD/Filter'
 * '<S231>' : 'pid_control_V3/PID VELOCIDAD/Filter ICs'
 * '<S232>' : 'pid_control_V3/PID VELOCIDAD/I Gain'
 * '<S233>' : 'pid_control_V3/PID VELOCIDAD/Ideal P Gain'
 * '<S234>' : 'pid_control_V3/PID VELOCIDAD/Ideal P Gain Fdbk'
 * '<S235>' : 'pid_control_V3/PID VELOCIDAD/Integrator'
 * '<S236>' : 'pid_control_V3/PID VELOCIDAD/Integrator ICs'
 * '<S237>' : 'pid_control_V3/PID VELOCIDAD/N Copy'
 * '<S238>' : 'pid_control_V3/PID VELOCIDAD/N Gain'
 * '<S239>' : 'pid_control_V3/PID VELOCIDAD/P Copy'
 * '<S240>' : 'pid_control_V3/PID VELOCIDAD/Parallel P Gain'
 * '<S241>' : 'pid_control_V3/PID VELOCIDAD/Reset Signal'
 * '<S242>' : 'pid_control_V3/PID VELOCIDAD/Saturation'
 * '<S243>' : 'pid_control_V3/PID VELOCIDAD/Saturation Fdbk'
 * '<S244>' : 'pid_control_V3/PID VELOCIDAD/Sum'
 * '<S245>' : 'pid_control_V3/PID VELOCIDAD/Sum Fdbk'
 * '<S246>' : 'pid_control_V3/PID VELOCIDAD/Tracking Mode'
 * '<S247>' : 'pid_control_V3/PID VELOCIDAD/Tracking Mode Sum'
 * '<S248>' : 'pid_control_V3/PID VELOCIDAD/Tsamp - Integral'
 * '<S249>' : 'pid_control_V3/PID VELOCIDAD/Tsamp - Ngain'
 * '<S250>' : 'pid_control_V3/PID VELOCIDAD/postSat Signal'
 * '<S251>' : 'pid_control_V3/PID VELOCIDAD/preInt Signal'
 * '<S252>' : 'pid_control_V3/PID VELOCIDAD/preSat Signal'
 * '<S253>' : 'pid_control_V3/PID VELOCIDAD/Anti-windup/Cont. Clamping Parallel'
 * '<S254>' : 'pid_control_V3/PID VELOCIDAD/Anti-windup/Cont. Clamping Parallel/Dead Zone'
 * '<S255>' : 'pid_control_V3/PID VELOCIDAD/Anti-windup/Cont. Clamping Parallel/Dead Zone/Enabled'
 * '<S256>' : 'pid_control_V3/PID VELOCIDAD/D Gain/Internal Parameters'
 * '<S257>' : 'pid_control_V3/PID VELOCIDAD/External Derivative/Error'
 * '<S258>' : 'pid_control_V3/PID VELOCIDAD/Filter/Cont. Filter'
 * '<S259>' : 'pid_control_V3/PID VELOCIDAD/Filter ICs/Internal IC - Filter'
 * '<S260>' : 'pid_control_V3/PID VELOCIDAD/I Gain/Internal Parameters'
 * '<S261>' : 'pid_control_V3/PID VELOCIDAD/Ideal P Gain/Passthrough'
 * '<S262>' : 'pid_control_V3/PID VELOCIDAD/Ideal P Gain Fdbk/Disabled'
 * '<S263>' : 'pid_control_V3/PID VELOCIDAD/Integrator/Continuous'
 * '<S264>' : 'pid_control_V3/PID VELOCIDAD/Integrator ICs/Internal IC'
 * '<S265>' : 'pid_control_V3/PID VELOCIDAD/N Copy/Disabled'
 * '<S266>' : 'pid_control_V3/PID VELOCIDAD/N Gain/Internal Parameters'
 * '<S267>' : 'pid_control_V3/PID VELOCIDAD/P Copy/Disabled'
 * '<S268>' : 'pid_control_V3/PID VELOCIDAD/Parallel P Gain/Internal Parameters'
 * '<S269>' : 'pid_control_V3/PID VELOCIDAD/Reset Signal/Disabled'
 * '<S270>' : 'pid_control_V3/PID VELOCIDAD/Saturation/Enabled'
 * '<S271>' : 'pid_control_V3/PID VELOCIDAD/Saturation Fdbk/Disabled'
 * '<S272>' : 'pid_control_V3/PID VELOCIDAD/Sum/Sum_PID'
 * '<S273>' : 'pid_control_V3/PID VELOCIDAD/Sum Fdbk/Disabled'
 * '<S274>' : 'pid_control_V3/PID VELOCIDAD/Tracking Mode/Disabled'
 * '<S275>' : 'pid_control_V3/PID VELOCIDAD/Tracking Mode Sum/Passthrough'
 * '<S276>' : 'pid_control_V3/PID VELOCIDAD/Tsamp - Integral/TsSignalSpecification'
 * '<S277>' : 'pid_control_V3/PID VELOCIDAD/Tsamp - Ngain/Passthrough'
 * '<S278>' : 'pid_control_V3/PID VELOCIDAD/postSat Signal/Forward_Path'
 * '<S279>' : 'pid_control_V3/PID VELOCIDAD/preInt Signal/Internal PreInt'
 * '<S280>' : 'pid_control_V3/PID VELOCIDAD/preSat Signal/Forward_Path'
 * '<S281>' : 'pid_control_V3/SUBSYSTEM_MODEL/Band-Limited White Noise'
 * '<S282>' : 'pid_control_V3/SUBSYSTEM_MODEL/Compare To Constant'
 * '<S283>' : 'pid_control_V3/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))'
 * '<S284>' : 'pid_control_V3/SUBSYSTEM_MODEL/MATLAB Function - MODEL'
 * '<S285>' : 'pid_control_V3/SUBSYSTEM_MODEL/MATLAB Function-reset'
 * '<S286>' : 'pid_control_V3/SUBSYSTEM_MODEL/Subscribe1_TURBULENCIA1'
 * '<S287>' : 'pid_control_V3/SUBSYSTEM_MODEL/Subscribe_HEAVE'
 * '<S288>' : 'pid_control_V3/SUBSYSTEM_MODEL/Subscribe_RATE'
 * '<S289>' : 'pid_control_V3/SUBSYSTEM_MODEL/Subscribe_TURBULENCIA'
 * '<S290>' : 'pid_control_V3/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Angle Conversion'
 * '<S291>' : 'pid_control_V3/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Filters on angular rates'
 * '<S292>' : 'pid_control_V3/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Filters on velocities'
 * '<S293>' : 'pid_control_V3/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Length Conversion'
 * '<S294>' : 'pid_control_V3/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Length Conversion1'
 * '<S295>' : 'pid_control_V3/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/RMS turbulence  intensities'
 * '<S296>' : 'pid_control_V3/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Select angular rates'
 * '<S297>' : 'pid_control_V3/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Select velocities'
 * '<S298>' : 'pid_control_V3/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Turbulence scale lengths'
 * '<S299>' : 'pid_control_V3/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Velocity Conversion'
 * '<S300>' : 'pid_control_V3/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Velocity Conversion2'
 * '<S301>' : 'pid_control_V3/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/White Noise'
 * '<S302>' : 'pid_control_V3/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Filters on angular rates/Hpgw'
 * '<S303>' : 'pid_control_V3/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Filters on angular rates/Hqgw'
 * '<S304>' : 'pid_control_V3/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Filters on angular rates/Hrgw'
 * '<S305>' : 'pid_control_V3/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Filters on velocities/Hugw(s)'
 * '<S306>' : 'pid_control_V3/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Filters on velocities/Hvgw(s)'
 * '<S307>' : 'pid_control_V3/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Filters on velocities/Hwgw(s)'
 * '<S308>' : 'pid_control_V3/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/RMS turbulence  intensities/High Altitude Intensity'
 * '<S309>' : 'pid_control_V3/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/RMS turbulence  intensities/Low Altitude Intensity'
 * '<S310>' : 'pid_control_V3/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Select angular rates/Interpolate  rates'
 * '<S311>' : 'pid_control_V3/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Select angular rates/Low altitude  rates'
 * '<S312>' : 'pid_control_V3/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Select angular rates/Medium//High  altitude rates'
 * '<S313>' : 'pid_control_V3/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Select angular rates/Merge Subsystems'
 * '<S314>' : 'pid_control_V3/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Select angular rates/Interpolate  rates/wind to body transformation'
 * '<S315>' : 'pid_control_V3/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Select angular rates/Interpolate  rates/wind to body transformation/convert to earth coords'
 * '<S316>' : 'pid_control_V3/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Select angular rates/Low altitude  rates/wind to body transformation'
 * '<S317>' : 'pid_control_V3/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Select angular rates/Low altitude  rates/wind to body transformation/convert to earth coords'
 * '<S318>' : 'pid_control_V3/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Select velocities/Interpolate  velocities'
 * '<S319>' : 'pid_control_V3/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Select velocities/Low altitude  velocities'
 * '<S320>' : 'pid_control_V3/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Select velocities/Medium//High  altitude velocities'
 * '<S321>' : 'pid_control_V3/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Select velocities/Merge Subsystems'
 * '<S322>' : 'pid_control_V3/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Select velocities/Interpolate  velocities/wind to body transformation'
 * '<S323>' : 'pid_control_V3/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Select velocities/Interpolate  velocities/wind to body transformation/convert to earth coords'
 * '<S324>' : 'pid_control_V3/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Select velocities/Low altitude  velocities/wind to body transformation'
 * '<S325>' : 'pid_control_V3/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Select velocities/Low altitude  velocities/wind to body transformation/convert to earth coords'
 * '<S326>' : 'pid_control_V3/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Turbulence scale lengths/Low altitude scale length'
 * '<S327>' : 'pid_control_V3/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Turbulence scale lengths/Medium//High altitude scale length'
 * '<S328>' : 'pid_control_V3/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Turbulence scale lengths/Medium//High altitude scale length/Length Conversion'
 * '<S329>' : 'pid_control_V3/SUBSYSTEM_MODEL/Subscribe1_TURBULENCIA1/Enabled Subsystem'
 * '<S330>' : 'pid_control_V3/SUBSYSTEM_MODEL/Subscribe_HEAVE/Enabled Subsystem'
 * '<S331>' : 'pid_control_V3/SUBSYSTEM_MODEL/Subscribe_RATE/Enabled Subsystem'
 * '<S332>' : 'pid_control_V3/SUBSYSTEM_MODEL/Subscribe_TURBULENCIA/Enabled Subsystem'
 * '<S333>' : 'pid_control_V3/Subscribe-ALTURA1/Enabled Subsystem'
 * '<S334>' : 'pid_control_V3/Subscribe-ROLL/Enabled Subsystem'
 * '<S335>' : 'pid_control_V3/Subscribe-YAW/Enabled Subsystem'
 */
#endif                                 /* pid_control_V3_h_ */
