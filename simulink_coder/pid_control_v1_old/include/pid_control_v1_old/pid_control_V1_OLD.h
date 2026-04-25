/*
 * pid_control_V1_OLD.h
 *
 * Trial License - for use to evaluate programs for possible purchase as
 * an end-user only.
 *
 * Code generation for model "pid_control_V1_OLD".
 *
 * Model version              : 12.136
 * Simulink Coder version : 25.2 (R2025b) 28-Jul-2025
 * C++ source code generated on : Sat Apr 25 10:44:57 2026
 *
 * Target selection: ert.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Intel->x86-64 (Linux 64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef pid_control_V1_OLD_h_
#define pid_control_V1_OLD_h_
#include "rtwtypes.h"
#include "rtw_continuous.h"
#include "rtw_solver.h"
#include "slros2_initialize.h"
#include "pid_control_V1_OLD_types.h"

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

/* Block signals for system '<S336>/Enabled Subsystem' */
struct B_EnabledSubsystem_pid_contro_T {
  SL_Bus_std_msgs_Bool In1;            /* '<S379>/In1' */
};

/* Block signals for system '<S337>/Enabled Subsystem' */
struct B_EnabledSubsystem_pid_cont_d_T {
  SL_Bus_std_msgs_Float64 In1;         /* '<S380>/In1' */
};

/* Block signals (default storage) */
struct B_pid_control_V1_OLD_T {
  SL_Bus_gazebo_msgs_SetEntityStateRequest BusAssignment;/* '<Root>/Bus Assignment' */
  real_T IC[12];                       /* '<S11>/IC' */
  real_T x[12];                        /* '<S11>/Integrator' */
  uint8_T stringOut_l[128];            /* '<Root>/MATLAB Function' */
  real_T R[9];
  real_T RotationAnglestoDirectionCo[9];
                        /* '<S11>/Rotation Angles to Direction Cosine Matrix' */
  real_T dv[9];
  real_T TmpSignalConversionAtSFunct[5];/* '<S11>/MATLAB Function - MODEL' */
  char_T b_zeroDelimTopic[25];
  real_T wbe_b[3];
  real_T FE1_b[3];
  real_T F_b[3];
  real_T Product_m[3];                 /* '<S372>/Product' */
  real_T Dtot[3];
  char_T b_zeroDelimTopic_c[22];
  char_T b_zeroDelimTopic_k[22];
  char_T b_zeroDelimTopic_cx[17];
  char_T b_zeroDelimTopic_b[17];
  sJ4ih70VmKcvCeguWN0mNVF deadline;
  sJ4ih70VmKcvCeguWN0mNVF deadline_p;
  real_T frac[2];
  real_T dv1[2];
  real_T Switch3;                      /* '<Root>/Switch3' */
  real_T Gain;                         /* '<Root>/Gain' */
  real_T FilterCoefficient;            /* '<S106>/Filter Coefficient' */
  real_T Saturation;                   /* '<S110>/Saturation' */
  real_T Switch2;                      /* '<Root>/Switch2' */
  real_T FilterCoefficient_o;          /* '<S316>/Filter Coefficient' */
  real_T RL_phi_ref;                   /* '<Root>/RL_phi_ref' */
  real_T FilterCoefficient_c;          /* '<S54>/Filter Coefficient' */
  real_T Saturation_k;                 /* '<S58>/Saturation' */
  real_T Saturation_i;                 /* '<Root>/Saturation' */
  real_T RateLimiter;                  /* '<Root>/Rate Limiter' */
  real_T FilterCoefficient_m;          /* '<S158>/Filter Coefficient' */
  real_T Saturation_f;                 /* '<S162>/Saturation' */
  real_T ProportionalGain;             /* '<S212>/Proportional Gain' */
  real_T DerivativeGain;               /* '<S200>/Derivative Gain' */
  real_T FilterCoefficient_p;          /* '<S210>/Filter Coefficient' */
  real_T Saturation1;                  /* '<Root>/Saturation1' */
  real_T FilterCoefficient_cv;         /* '<S264>/Filter Coefficient' */
  real_T Saturation_o;                 /* '<S268>/Saturation' */
  real_T Memory[3];                    /* '<S11>/Memory' */
  real_T Memory1[3];                   /* '<S11>/Memory1' */
  real_T Switch;                       /* '<S41>/Switch' */
  real_T SumI4;                        /* '<S95>/SumI4' */
  real_T SumI4_i;                      /* '<S147>/SumI4' */
  real_T IntegralGain;                 /* '<S204>/Integral Gain' */
  real_T Switch_j;                     /* '<S251>/Switch' */
  real_T IntegralGain_n;               /* '<S310>/Integral Gain' */
  real_T Product[4];                   /* '<S351>/Product' */
  real_T Switch_p[3];                  /* '<S11>/Switch' */
  real_T Switch1[3];                   /* '<S11>/Switch1' */
  real_T data;
  real_T data_n;
  real_T Power;                        /* '<S11>/Product2' */
  real_T Gain3;                        /* '<S11>/Gain3' */
  real_T EnergykWh;                    /* '<S11>/Gain1' */
  real_T powerdemand;                  /* '<S11>/Divide' */
  real_T loadtorque;                   /* '<S11>/Divide1' */
  real_T Output;                       /* '<S331>/Output' */
  real_T Sum[3];                       /* '<S11>/Sum' */
  real_T Sum1[3];                      /* '<S11>/Sum1' */
  real_T XDOT[40];                     /* '<S11>/MATLAB Function - MODEL' */
  real_T CL_total;                     /* '<S11>/MATLAB Function - MODEL' */
  real_T h_out;                        /* '<S11>/MATLAB Function - MODEL' */
  real_T mu_Lw_out;                    /* '<S11>/MATLAB Function - MODEL' */
  real_T mu_Dw_out;                    /* '<S11>/MATLAB Function - MODEL' */
  real_T w[2];                         /* '<S357>/w' */
  real_T w_a[2];                       /* '<S357>/w ' */
  real_T LwgV1[2];                     /* '<S357>/Lwg//V 1' */
  real_T w_g[2];                       /* '<S356>/w' */
  real_T w_e[2];                       /* '<S356>/w ' */
  real_T w1[2];                        /* '<S356>/w 1' */
  real_T w_n[2];                       /* '<S355>/w' */
  real_T w1_c[2];                      /* '<S355>/w1' */
  real_T w_d[2];                       /* '<S354>/w' */
  real_T w_e0[2];                      /* '<S353>/w' */
  real_T UnaryMinus[2];                /* '<S353>/Unary Minus' */
  real_T w_o[2];                       /* '<S352>/w' */
  real_T sigma_w[2];                   /* '<S352>/sigma_w' */
  real_T u2;
  real_T w_r;
  real_T Va;
  real_T beta;
  real_T q_aero;
  real_T hw;
  real_T hh;
  real_T Q;
  real_T CL_w_IGE;
  real_T CL_h_IGE;
  real_T CD_iw_IGE;
  real_T CD_ih_IGE;
  real_T Dtot_c;
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
  real_T Sum2_l;                       /* '<Root>/Sum2' */
  real_T Sum_b;                        /* '<S112>/Sum' */
  real_T WhiteNoise_p;                 /* '<S331>/White Noise' */
  real_T SignPreSat;                   /* '<S41>/SignPreSat' */
  real_T Sum5;                         /* '<Root>/Sum5' */
  real_T Sum1_g;                       /* '<Root>/Sum1' */
  real_T Sum_hl;                       /* '<S164>/Sum' */
  real_T SignPreSat_h;                 /* '<S251>/SignPreSat' */
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
  real_T R_tmp_f;
  real_T Ltot_tmp;
  SL_Bus_std_msgs_Float64 SourceBlock_o2_k;/* '<S338>/SourceBlock' */
  SL_Bus_std_msgs_Float64 SourceBlock_o2_p;/* '<S337>/SourceBlock' */
  uint32_T bpIndex[2];
  uint32_T lengthOut;                  /* '<Root>/MATLAB Function1' */
  uint32_T lengthOut_e;                /* '<Root>/MATLAB Function' */
  uint8_T stringOut[128];              /* '<Root>/MATLAB Function1' */
  boolean_T Compare;                   /* '<S332>/Compare' */
  boolean_T AND3;                      /* '<S41>/AND3' */
  boolean_T Memory_a;                  /* '<S41>/Memory' */
  boolean_T AND3_c;                    /* '<S251>/AND3' */
  boolean_T Memory_h;                  /* '<S251>/Memory' */
  boolean_T SourceBlock_o1;            /* '<S13>/SourceBlock' */
  boolean_T SourceBlock_o1_o;          /* '<S12>/SourceBlock' */
  boolean_T SourceBlock_o1_h;          /* '<S339>/SourceBlock' */
  boolean_T SourceBlock_o1_d;          /* '<S338>/SourceBlock' */
  boolean_T SourceBlock_o1_c;          /* '<S337>/SourceBlock' */
  boolean_T SourceBlock_o1_k;          /* '<S336>/SourceBlock' */
  B_EnabledSubsystem_pid_cont_d_T EnabledSubsystem_a;/* '<S13>/Enabled Subsystem' */
  B_EnabledSubsystem_pid_cont_d_T EnabledSubsystem_b;/* '<S12>/Enabled Subsystem' */
  B_EnabledSubsystem_pid_contro_T EnabledSubsystem_pt;/* '<S339>/Enabled Subsystem' */
  B_EnabledSubsystem_pid_cont_d_T EnabledSubsystem_p;/* '<S338>/Enabled Subsystem' */
  B_EnabledSubsystem_pid_cont_d_T EnabledSubsystem_k;/* '<S337>/Enabled Subsystem' */
  B_EnabledSubsystem_pid_contro_T EnabledSubsystem;/* '<S336>/Enabled Subsystem' */
};

/* Block states (default storage) for system '<Root>' */
struct DW_pid_control_V1_OLD_T {
  ros_slros2_internal_block_Ser_T obj; /* '<S2>/ServiceCaller' */
  ros_slros2_internal_block_Sub_T obj_k;/* '<S13>/SourceBlock' */
  ros_slros2_internal_block_Sub_T obj_m;/* '<S12>/SourceBlock' */
  ros_slros2_internal_block_Sub_T obj_h;/* '<S339>/SourceBlock' */
  ros_slros2_internal_block_Sub_T obj_p;/* '<S338>/SourceBlock' */
  ros_slros2_internal_block_Sub_T obj_hy;/* '<S337>/SourceBlock' */
  ros_slros2_internal_block_Sub_T obj_h4;/* '<S336>/SourceBlock' */
  real_T UnitDelay3_DSTATE;            /* '<Root>/Unit Delay3' */
  real_T UnitDelay2_DSTATE;            /* '<Root>/Unit Delay2' */
  real_T UDbeta_DSTATE;                /* '<Root>/UD-beta' */
  real_T Memory2_PreviousInput[12];    /* '<S11>/Memory2' */
  real_T PrevY;                        /* '<Root>/RL_phi_ref' */
  real_T LastMajorTime;                /* '<Root>/RL_phi_ref' */
  real_T PrevY_g;                      /* '<Root>/Rate Limiter' */
  real_T LastMajorTime_j;              /* '<Root>/Rate Limiter' */
  real_T Memory_PreviousInput[3];      /* '<S11>/Memory' */
  real_T Memory1_PreviousInput[3];     /* '<S11>/Memory1' */
  real_T NextOutput[4];                /* '<S351>/White Noise' */
  real_T NextOutput_k;                 /* '<S331>/White Noise' */
  struct {
    void *LoggedData;
  } ToWorkspace_PWORK;                 /* '<Root>/To Workspace' */

  struct {
    void *LoggedData;
  } ToWorkspace_PWORK_g;               /* '<S11>/To Workspace' */

  struct {
    void *LoggedData;
  } ToWorkspace1_PWORK;                /* '<S11>/To Workspace1' */

  struct {
    void *LoggedData;
  } ToWorkspace10_PWORK;               /* '<S11>/To Workspace10' */

  struct {
    void *LoggedData;
  } ToWorkspace11_PWORK;               /* '<S11>/To Workspace11' */

  struct {
    void *LoggedData;
  } ToWorkspace12_PWORK;               /* '<S11>/To Workspace12' */

  struct {
    void *LoggedData;
  } ToWorkspace13_PWORK;               /* '<S11>/To Workspace13' */

  struct {
    void *LoggedData;
  } ToWorkspace14_PWORK;               /* '<S11>/To Workspace14' */

  struct {
    void *LoggedData;
  } ToWorkspace15_PWORK;               /* '<S11>/To Workspace15' */

  struct {
    void *LoggedData;
  } ToWorkspace16_PWORK;               /* '<S11>/To Workspace16' */

  struct {
    void *LoggedData;
  } ToWorkspace17_PWORK;               /* '<S11>/To Workspace17' */

  struct {
    void *LoggedData;
  } ToWorkspace18_PWORK;               /* '<S11>/To Workspace18' */

  struct {
    void *LoggedData;
  } ToWorkspace19_PWORK;               /* '<S11>/To Workspace19' */

  struct {
    void *LoggedData;
  } ToWorkspace2_PWORK;                /* '<S11>/To Workspace2' */

  struct {
    void *LoggedData;
  } ToWorkspace20_PWORK;               /* '<S11>/To Workspace20' */

  struct {
    void *LoggedData;
  } ToWorkspace21_PWORK;               /* '<S11>/To Workspace21' */

  struct {
    void *LoggedData;
  } ToWorkspace22_PWORK;               /* '<S11>/To Workspace22' */

  struct {
    void *LoggedData;
  } ToWorkspace23_PWORK;               /* '<S11>/To Workspace23' */

  struct {
    void *LoggedData;
  } ToWorkspace24_PWORK;               /* '<S11>/To Workspace24' */

  struct {
    void *LoggedData;
  } ToWorkspace25_PWORK;               /* '<S11>/To Workspace25' */

  struct {
    void *LoggedData;
  } ToWorkspace26_PWORK;               /* '<S11>/To Workspace26' */

  struct {
    void *LoggedData;
  } ToWorkspace27_PWORK;               /* '<S11>/To Workspace27' */

  struct {
    void *LoggedData;
  } ToWorkspace28_PWORK;               /* '<S11>/To Workspace28' */

  struct {
    void *LoggedData;
  } ToWorkspace29_PWORK;               /* '<S11>/To Workspace29' */

  struct {
    void *LoggedData;
  } ToWorkspace3_PWORK;                /* '<S11>/To Workspace3' */

  struct {
    void *LoggedData;
  } ToWorkspace30_PWORK;               /* '<S11>/To Workspace30' */

  struct {
    void *LoggedData;
  } ToWorkspace31_PWORK;               /* '<S11>/To Workspace31' */

  struct {
    void *LoggedData;
  } ToWorkspace32_PWORK;               /* '<S11>/To Workspace32' */

  struct {
    void *LoggedData;
  } ToWorkspace33_PWORK;               /* '<S11>/To Workspace33' */

  struct {
    void *LoggedData;
  } ToWorkspace4_PWORK;                /* '<S11>/To Workspace4' */

  struct {
    void *LoggedData;
  } ToWorkspace5_PWORK;                /* '<S11>/To Workspace5' */

  struct {
    void *LoggedData;
  } ToWorkspace6_PWORK;                /* '<S11>/To Workspace6' */

  struct {
    void *LoggedData;
  } ToWorkspace7_PWORK;                /* '<S11>/To Workspace7' */

  struct {
    void *LoggedData;
  } ToWorkspace8_PWORK;                /* '<S11>/To Workspace8' */

  struct {
    void *LoggedData;
  } ToWorkspace9_PWORK;                /* '<S11>/To Workspace9' */

  uint32_T PreLookUpIndexSearchprobofexcee;
                        /* '<S358>/PreLook-Up Index Search  (prob of exceed)' */
  uint32_T PreLookUpIndexSearchaltitude_DW;
                              /* '<S358>/PreLook-Up Index Search  (altitude)' */
  uint32_T RandSeed[4];                /* '<S351>/White Noise' */
  uint32_T RandSeed_a;                 /* '<S331>/White Noise' */
  robotics_slcore_internal_bloc_T obj_c;
                             /* '<Root>/Coordinate Transformation Conversion' */
  int8_T ifHeightMaxlowaltitudeelseifHei;
  /* '<S347>/if Height < Max low altitude  elseif Height > Min isotropic altitude ' */
  int8_T ifHeightMaxlowaltitudeelseifH_a;
  /* '<S346>/if Height < Max low altitude  elseif Height > Min isotropic altitude ' */
  boolean_T IC_FirstOutputTime;        /* '<S11>/IC' */
  boolean_T Integrator_DWORK1;         /* '<S11>/Integrator' */
  boolean_T PrevLimited;               /* '<Root>/RL_phi_ref' */
  boolean_T PrevLimited_a;             /* '<Root>/Rate Limiter' */
  boolean_T Memory_PreviousInput_o;    /* '<S41>/Memory' */
  boolean_T Memory_PreviousInput_a;    /* '<S251>/Memory' */
  boolean_T objisempty;                /* '<S13>/SourceBlock' */
  boolean_T objisempty_g;              /* '<S12>/SourceBlock' */
  boolean_T objisempty_a;              /* '<S339>/SourceBlock' */
  boolean_T objisempty_e;              /* '<S338>/SourceBlock' */
  boolean_T objisempty_l;              /* '<S337>/SourceBlock' */
  boolean_T objisempty_c;              /* '<S336>/SourceBlock' */
  boolean_T objisempty_d;    /* '<Root>/Coordinate Transformation Conversion' */
  boolean_T objisempty_f;              /* '<S2>/ServiceCaller' */
  boolean_T Hwgws_MODE;                /* '<S342>/Hwgw(s)' */
  boolean_T Hvgws_MODE;                /* '<S342>/Hvgw(s)' */
  boolean_T Hugws_MODE;                /* '<S342>/Hugw(s)' */
  boolean_T Hrgw_MODE;                 /* '<S341>/Hrgw' */
  boolean_T Hqgw_MODE;                 /* '<S341>/Hqgw' */
  boolean_T Hpgw_MODE;                 /* '<S341>/Hpgw' */
};

/* Continuous states (default storage) */
struct X_pid_control_V1_OLD_T {
  real_T Integrator_CSTATE[12];        /* '<S11>/Integrator' */
  real_T Integrator_CSTATE_n;          /* '<S103>/Integrator' */
  real_T Filter_CSTATE;                /* '<S98>/Filter' */
  real_T Integrator_CSTATE_p;          /* '<S313>/Integrator' */
  real_T Filter_CSTATE_l;              /* '<S308>/Filter' */
  real_T Integrator_CSTATE_m;          /* '<S51>/Integrator' */
  real_T Filter_CSTATE_g;              /* '<S46>/Filter' */
  real_T Integrator_CSTATE_py;         /* '<S155>/Integrator' */
  real_T Filter_CSTATE_m;              /* '<S150>/Filter' */
  real_T Integrator_CSTATE_d;          /* '<S207>/Integrator' */
  real_T Filter_CSTATE_f;              /* '<S202>/Filter' */
  real_T Integrator_CSTATE_f;          /* '<S261>/Integrator' */
  real_T Filter_CSTATE_lb;             /* '<S256>/Filter' */
  real_T Integrator1_CSTATE;           /* '<S11>/Integrator1' */
  real_T TransferFcn_CSTATE[2];        /* '<S11>/Transfer Fcn' */
  real_T TransferFcn1_CSTATE;          /* '<S11>/Transfer Fcn1' */
  real_T wg_p1_CSTATE[2];              /* '<S357>/wg_p1' */
  real_T wg_p2_CSTATE[2];              /* '<S357>/wg_p2' */
  real_T vg_p1_CSTATE[2];              /* '<S356>/vg_p1' */
  real_T vgw_p2_CSTATE[2];             /* '<S356>/vgw_p2' */
  real_T ug_p_CSTATE[2];               /* '<S355>/ug_p' */
  real_T rgw_p_CSTATE[2];              /* '<S354>/rgw_p' */
  real_T qgw_p_CSTATE[2];              /* '<S353>/qgw_p' */
  real_T pgw_p_CSTATE[2];              /* '<S352>/pgw_p' */
};

/* State derivatives (default storage) */
struct XDot_pid_control_V1_OLD_T {
  real_T Integrator_CSTATE[12];        /* '<S11>/Integrator' */
  real_T Integrator_CSTATE_n;          /* '<S103>/Integrator' */
  real_T Filter_CSTATE;                /* '<S98>/Filter' */
  real_T Integrator_CSTATE_p;          /* '<S313>/Integrator' */
  real_T Filter_CSTATE_l;              /* '<S308>/Filter' */
  real_T Integrator_CSTATE_m;          /* '<S51>/Integrator' */
  real_T Filter_CSTATE_g;              /* '<S46>/Filter' */
  real_T Integrator_CSTATE_py;         /* '<S155>/Integrator' */
  real_T Filter_CSTATE_m;              /* '<S150>/Filter' */
  real_T Integrator_CSTATE_d;          /* '<S207>/Integrator' */
  real_T Filter_CSTATE_f;              /* '<S202>/Filter' */
  real_T Integrator_CSTATE_f;          /* '<S261>/Integrator' */
  real_T Filter_CSTATE_lb;             /* '<S256>/Filter' */
  real_T Integrator1_CSTATE;           /* '<S11>/Integrator1' */
  real_T TransferFcn_CSTATE[2];        /* '<S11>/Transfer Fcn' */
  real_T TransferFcn1_CSTATE;          /* '<S11>/Transfer Fcn1' */
  real_T wg_p1_CSTATE[2];              /* '<S357>/wg_p1' */
  real_T wg_p2_CSTATE[2];              /* '<S357>/wg_p2' */
  real_T vg_p1_CSTATE[2];              /* '<S356>/vg_p1' */
  real_T vgw_p2_CSTATE[2];             /* '<S356>/vgw_p2' */
  real_T ug_p_CSTATE[2];               /* '<S355>/ug_p' */
  real_T rgw_p_CSTATE[2];              /* '<S354>/rgw_p' */
  real_T qgw_p_CSTATE[2];              /* '<S353>/qgw_p' */
  real_T pgw_p_CSTATE[2];              /* '<S352>/pgw_p' */
};

/* State disabled  */
struct XDis_pid_control_V1_OLD_T {
  boolean_T Integrator_CSTATE[12];     /* '<S11>/Integrator' */
  boolean_T Integrator_CSTATE_n;       /* '<S103>/Integrator' */
  boolean_T Filter_CSTATE;             /* '<S98>/Filter' */
  boolean_T Integrator_CSTATE_p;       /* '<S313>/Integrator' */
  boolean_T Filter_CSTATE_l;           /* '<S308>/Filter' */
  boolean_T Integrator_CSTATE_m;       /* '<S51>/Integrator' */
  boolean_T Filter_CSTATE_g;           /* '<S46>/Filter' */
  boolean_T Integrator_CSTATE_py;      /* '<S155>/Integrator' */
  boolean_T Filter_CSTATE_m;           /* '<S150>/Filter' */
  boolean_T Integrator_CSTATE_d;       /* '<S207>/Integrator' */
  boolean_T Filter_CSTATE_f;           /* '<S202>/Filter' */
  boolean_T Integrator_CSTATE_f;       /* '<S261>/Integrator' */
  boolean_T Filter_CSTATE_lb;          /* '<S256>/Filter' */
  boolean_T Integrator1_CSTATE;        /* '<S11>/Integrator1' */
  boolean_T TransferFcn_CSTATE[2];     /* '<S11>/Transfer Fcn' */
  boolean_T TransferFcn1_CSTATE;       /* '<S11>/Transfer Fcn1' */
  boolean_T wg_p1_CSTATE[2];           /* '<S357>/wg_p1' */
  boolean_T wg_p2_CSTATE[2];           /* '<S357>/wg_p2' */
  boolean_T vg_p1_CSTATE[2];           /* '<S356>/vg_p1' */
  boolean_T vgw_p2_CSTATE[2];          /* '<S356>/vgw_p2' */
  boolean_T ug_p_CSTATE[2];            /* '<S355>/ug_p' */
  boolean_T rgw_p_CSTATE[2];           /* '<S354>/rgw_p' */
  boolean_T qgw_p_CSTATE[2];           /* '<S353>/qgw_p' */
  boolean_T pgw_p_CSTATE[2];           /* '<S352>/pgw_p' */
};

/* Zero-crossing (trigger) state */
struct PrevZCX_pid_control_V1_OLD_T {
  ZCSigState Integrator_Reset_ZCE;     /* '<S11>/Integrator' */
};

/* Invariant block signals (default storage) */
struct ConstB_pid_control_V1_OLD_T {
  real_T UnitConversion;               /* '<S340>/Unit Conversion' */
  real_T UnitConversion_k;             /* '<S350>/Unit Conversion' */
  real_T sigma_wg;                     /* '<S359>/sigma_wg ' */
  real_T UnitConversion_n;             /* '<S344>/Unit Conversion' */
  real_T UnitConversion_c;             /* '<S378>/Unit Conversion' */
  real_T PreLookUpIndexSearchprobofe;
                        /* '<S358>/PreLook-Up Index Search  (prob of exceed)' */
  real_T Sqrt[4];                      /* '<S351>/Sqrt' */
  real_T Sqrt1;                        /* '<S351>/Sqrt1' */
  real_T Divide[4];                    /* '<S351>/Divide' */
  real_T motorspeed;                   /* '<S11>/Gain2' */
  real_T Sum;                          /* '<S368>/Sum' */
  real_T Sum_a;                        /* '<S360>/Sum' */
  real_T sqrt_a;                       /* '<S357>/sqrt' */
  real_T w4;                           /* '<S352>/w4' */
  real_T u16;                          /* '<S352>/u^1//6' */
  uint32_T PreLookUpIndexSearchprobo_g;
                        /* '<S358>/PreLook-Up Index Search  (prob of exceed)' */
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
struct ConstP_pid_control_V1_OLD_T {
  /* Pooled Parameter (Expression: x_nom)
   * Referenced by:
   *   '<S11>/IC'
   *   '<S11>/Memory2'
   */
  real_T pooled10[12];

  /* Expression: h_vec
   * Referenced by: '<S358>/PreLook-Up Index Search  (altitude)'
   */
  real_T PreLookUpIndexSearchaltitude_Br[12];

  /* Expression: sigma_vec'
   * Referenced by: '<S358>/Medium//High Altitude Intensity'
   */
  real_T MediumHighAltitudeIntensity_Tab[84];

  /* Computed Parameter: MediumHighAltitudeIntensity_max
   * Referenced by: '<S358>/Medium//High Altitude Intensity'
   */
  uint32_T MediumHighAltitudeIntensity_max[2];
};

/* Real-time Model Data Structure */
struct tag_RTM_pid_control_V1_OLD_T {
  const char_T *errorStatus;
  RTWSolverInfo solverInfo;
  X_pid_control_V1_OLD_T *contStates;
  int_T *periodicContStateIndices;
  real_T *periodicContStateRanges;
  real_T *derivs;
  XDis_pid_control_V1_OLD_T *contStateDisabled;
  boolean_T zCCacheNeedsReset;
  boolean_T derivCacheNeedsReset;
  boolean_T CTOutputIncnstWithState;
  real_T odeY[44];
  real_T odeF[4][44];
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

extern const ConstB_pid_control_V1_OLD_T pid_control_V1_OLD_ConstB;/* constant block i/o */

/* Constant parameters (default storage) */
extern const ConstP_pid_control_V1_OLD_T pid_control_V1_OLD_ConstP;

/* Class declaration for model pid_control_V1_OLD */
class pid_control_V1_OLD
{
  /* public data and function members */
 public:
  /* Real-Time Model get method */
  RT_MODEL_pid_control_V1_OLD_T * getRTM();
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
  pid_control_V1_OLD();

  /* Destructor */
  ~pid_control_V1_OLD();

  /* private data and function members */
 private:
  /* Block signals */
  B_pid_control_V1_OLD_T pid_control_V1_OLD_B;

  /* Block states */
  DW_pid_control_V1_OLD_T pid_control_V1_OLD_DW;

  /* Block continuous states */
  X_pid_control_V1_OLD_T pid_control_V1_OLD_X;

  /* Block Continuous state disabled vector */
  XDis_pid_control_V1_OLD_T pid_control_V1_OLD_XDis;

  /* Triggered events */
  PrevZCX_pid_control_V1_OLD_T pid_control_V1_OLD_PrevZCX;

  /* private member function(s) for subsystem '<S336>/Enabled Subsystem'*/
  static void pid_contr_EnabledSubsystem_Init(B_EnabledSubsystem_pid_contro_T
    *localB);
  static void pid_control_V1_EnabledSubsystem(boolean_T rtu_Enable, const
    SL_Bus_std_msgs_Bool *rtu_In1, B_EnabledSubsystem_pid_contro_T *localB);

  /* private member function(s) for subsystem '<S337>/Enabled Subsystem'*/
  static void pid_con_EnabledSubsystem_i_Init(B_EnabledSubsystem_pid_cont_d_T
    *localB);
  static void pid_control__EnabledSubsystem_k(boolean_T rtu_Enable, const
    SL_Bus_std_msgs_Float64 *rtu_In1, B_EnabledSubsystem_pid_cont_d_T *localB);

  /* private member function(s) for subsystem '<Root>'*/
  void pid_c_Subscriber_setupImpl_cpn1(const ros_slros2_internal_block_Sub_T
    *obj);
  void pid__Subscriber_setupImpl_cpn1s(const ros_slros2_internal_block_Sub_T
    *obj);
  void pid_con_ServiceCaller_setupImpl(const ros_slros2_internal_block_Ser_T
    *obj);
  void pid_co_Subscriber_setupImpl_cpn(const ros_slros2_internal_block_Sub_T
    *obj);
  void pid_contro_Subscriber_setupImpl(const ros_slros2_internal_block_Sub_T
    *obj);
  void pid_cont_Subscriber_setupImpl_c(const ros_slros2_internal_block_Sub_T
    *obj);
  void pid_con_Subscriber_setupImpl_cp(const ros_slros2_internal_block_Sub_T
    *obj);

  /* Global mass matrix */

  /* Continuous states update member function*/
  void rt_ertODEUpdateContinuousStates(RTWSolverInfo *si );

  /* Derivatives member function */
  void pid_control_V1_OLD_derivatives();

  /* Real-Time Model */
  RT_MODEL_pid_control_V1_OLD_T pid_control_V1_OLD_M;
};

extern volatile boolean_T stopRequested;
extern volatile boolean_T runModel;

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<Root>/Display' : Unused code path elimination
 * Block '<S147>/Kb' : Eliminated nontunable gain of 1
 * Block '<S333>/Cast' : Eliminate redundant data type conversion
 * Block '<S333>/Cast To Double' : Eliminate redundant data type conversion
 * Block '<S333>/Cast To Double1' : Eliminate redundant data type conversion
 * Block '<S333>/Cast To Double2' : Eliminate redundant data type conversion
 * Block '<S333>/Cast To Double3' : Eliminate redundant data type conversion
 * Block '<S333>/Cast To Double4' : Eliminate redundant data type conversion
 * Block '<S364>/Reshape' : Reshape block reduction
 * Block '<S364>/Reshape1' : Reshape block reduction
 * Block '<S366>/Reshape' : Reshape block reduction
 * Block '<S372>/Reshape' : Reshape block reduction
 * Block '<S372>/Reshape1' : Reshape block reduction
 * Block '<S374>/Reshape' : Reshape block reduction
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
 * '<Root>' : 'pid_control_V1_OLD'
 * '<S1>'   : 'pid_control_V1_OLD/Blank Message'
 * '<S2>'   : 'pid_control_V1_OLD/Call Service'
 * '<S3>'   : 'pid_control_V1_OLD/MATLAB Function'
 * '<S4>'   : 'pid_control_V1_OLD/MATLAB Function1'
 * '<S5>'   : 'pid_control_V1_OLD/PID ALERON'
 * '<S6>'   : 'pid_control_V1_OLD/PID ALTURA'
 * '<S7>'   : 'pid_control_V1_OLD/PID PITCH//ELEVATOR'
 * '<S8>'   : 'pid_control_V1_OLD/PID TIMON'
 * '<S9>'   : 'pid_control_V1_OLD/PID VELOCIDAD'
 * '<S10>'  : 'pid_control_V1_OLD/PID_HEADING_OUTER'
 * '<S11>'  : 'pid_control_V1_OLD/SUBSYSTEM_MODEL'
 * '<S12>'  : 'pid_control_V1_OLD/Subscribe-ALTURA1'
 * '<S13>'  : 'pid_control_V1_OLD/Subscribe-YAW'
 * '<S14>'  : 'pid_control_V1_OLD/wrapPiErr'
 * '<S15>'  : 'pid_control_V1_OLD/PID ALERON/Anti-windup'
 * '<S16>'  : 'pid_control_V1_OLD/PID ALERON/D Gain'
 * '<S17>'  : 'pid_control_V1_OLD/PID ALERON/External Derivative'
 * '<S18>'  : 'pid_control_V1_OLD/PID ALERON/Filter'
 * '<S19>'  : 'pid_control_V1_OLD/PID ALERON/Filter ICs'
 * '<S20>'  : 'pid_control_V1_OLD/PID ALERON/I Gain'
 * '<S21>'  : 'pid_control_V1_OLD/PID ALERON/Ideal P Gain'
 * '<S22>'  : 'pid_control_V1_OLD/PID ALERON/Ideal P Gain Fdbk'
 * '<S23>'  : 'pid_control_V1_OLD/PID ALERON/Integrator'
 * '<S24>'  : 'pid_control_V1_OLD/PID ALERON/Integrator ICs'
 * '<S25>'  : 'pid_control_V1_OLD/PID ALERON/N Copy'
 * '<S26>'  : 'pid_control_V1_OLD/PID ALERON/N Gain'
 * '<S27>'  : 'pid_control_V1_OLD/PID ALERON/P Copy'
 * '<S28>'  : 'pid_control_V1_OLD/PID ALERON/Parallel P Gain'
 * '<S29>'  : 'pid_control_V1_OLD/PID ALERON/Reset Signal'
 * '<S30>'  : 'pid_control_V1_OLD/PID ALERON/Saturation'
 * '<S31>'  : 'pid_control_V1_OLD/PID ALERON/Saturation Fdbk'
 * '<S32>'  : 'pid_control_V1_OLD/PID ALERON/Sum'
 * '<S33>'  : 'pid_control_V1_OLD/PID ALERON/Sum Fdbk'
 * '<S34>'  : 'pid_control_V1_OLD/PID ALERON/Tracking Mode'
 * '<S35>'  : 'pid_control_V1_OLD/PID ALERON/Tracking Mode Sum'
 * '<S36>'  : 'pid_control_V1_OLD/PID ALERON/Tsamp - Integral'
 * '<S37>'  : 'pid_control_V1_OLD/PID ALERON/Tsamp - Ngain'
 * '<S38>'  : 'pid_control_V1_OLD/PID ALERON/postSat Signal'
 * '<S39>'  : 'pid_control_V1_OLD/PID ALERON/preInt Signal'
 * '<S40>'  : 'pid_control_V1_OLD/PID ALERON/preSat Signal'
 * '<S41>'  : 'pid_control_V1_OLD/PID ALERON/Anti-windup/Cont. Clamping Parallel'
 * '<S42>'  : 'pid_control_V1_OLD/PID ALERON/Anti-windup/Cont. Clamping Parallel/Dead Zone'
 * '<S43>'  : 'pid_control_V1_OLD/PID ALERON/Anti-windup/Cont. Clamping Parallel/Dead Zone/Enabled'
 * '<S44>'  : 'pid_control_V1_OLD/PID ALERON/D Gain/Internal Parameters'
 * '<S45>'  : 'pid_control_V1_OLD/PID ALERON/External Derivative/Error'
 * '<S46>'  : 'pid_control_V1_OLD/PID ALERON/Filter/Cont. Filter'
 * '<S47>'  : 'pid_control_V1_OLD/PID ALERON/Filter ICs/Internal IC - Filter'
 * '<S48>'  : 'pid_control_V1_OLD/PID ALERON/I Gain/Internal Parameters'
 * '<S49>'  : 'pid_control_V1_OLD/PID ALERON/Ideal P Gain/Passthrough'
 * '<S50>'  : 'pid_control_V1_OLD/PID ALERON/Ideal P Gain Fdbk/Disabled'
 * '<S51>'  : 'pid_control_V1_OLD/PID ALERON/Integrator/Continuous'
 * '<S52>'  : 'pid_control_V1_OLD/PID ALERON/Integrator ICs/Internal IC'
 * '<S53>'  : 'pid_control_V1_OLD/PID ALERON/N Copy/Disabled'
 * '<S54>'  : 'pid_control_V1_OLD/PID ALERON/N Gain/Internal Parameters'
 * '<S55>'  : 'pid_control_V1_OLD/PID ALERON/P Copy/Disabled'
 * '<S56>'  : 'pid_control_V1_OLD/PID ALERON/Parallel P Gain/Internal Parameters'
 * '<S57>'  : 'pid_control_V1_OLD/PID ALERON/Reset Signal/Disabled'
 * '<S58>'  : 'pid_control_V1_OLD/PID ALERON/Saturation/Enabled'
 * '<S59>'  : 'pid_control_V1_OLD/PID ALERON/Saturation Fdbk/Disabled'
 * '<S60>'  : 'pid_control_V1_OLD/PID ALERON/Sum/Sum_PID'
 * '<S61>'  : 'pid_control_V1_OLD/PID ALERON/Sum Fdbk/Disabled'
 * '<S62>'  : 'pid_control_V1_OLD/PID ALERON/Tracking Mode/Disabled'
 * '<S63>'  : 'pid_control_V1_OLD/PID ALERON/Tracking Mode Sum/Passthrough'
 * '<S64>'  : 'pid_control_V1_OLD/PID ALERON/Tsamp - Integral/TsSignalSpecification'
 * '<S65>'  : 'pid_control_V1_OLD/PID ALERON/Tsamp - Ngain/Passthrough'
 * '<S66>'  : 'pid_control_V1_OLD/PID ALERON/postSat Signal/Forward_Path'
 * '<S67>'  : 'pid_control_V1_OLD/PID ALERON/preInt Signal/Internal PreInt'
 * '<S68>'  : 'pid_control_V1_OLD/PID ALERON/preSat Signal/Forward_Path'
 * '<S69>'  : 'pid_control_V1_OLD/PID ALTURA/Anti-windup'
 * '<S70>'  : 'pid_control_V1_OLD/PID ALTURA/D Gain'
 * '<S71>'  : 'pid_control_V1_OLD/PID ALTURA/External Derivative'
 * '<S72>'  : 'pid_control_V1_OLD/PID ALTURA/Filter'
 * '<S73>'  : 'pid_control_V1_OLD/PID ALTURA/Filter ICs'
 * '<S74>'  : 'pid_control_V1_OLD/PID ALTURA/I Gain'
 * '<S75>'  : 'pid_control_V1_OLD/PID ALTURA/Ideal P Gain'
 * '<S76>'  : 'pid_control_V1_OLD/PID ALTURA/Ideal P Gain Fdbk'
 * '<S77>'  : 'pid_control_V1_OLD/PID ALTURA/Integrator'
 * '<S78>'  : 'pid_control_V1_OLD/PID ALTURA/Integrator ICs'
 * '<S79>'  : 'pid_control_V1_OLD/PID ALTURA/N Copy'
 * '<S80>'  : 'pid_control_V1_OLD/PID ALTURA/N Gain'
 * '<S81>'  : 'pid_control_V1_OLD/PID ALTURA/P Copy'
 * '<S82>'  : 'pid_control_V1_OLD/PID ALTURA/Parallel P Gain'
 * '<S83>'  : 'pid_control_V1_OLD/PID ALTURA/Reset Signal'
 * '<S84>'  : 'pid_control_V1_OLD/PID ALTURA/Saturation'
 * '<S85>'  : 'pid_control_V1_OLD/PID ALTURA/Saturation Fdbk'
 * '<S86>'  : 'pid_control_V1_OLD/PID ALTURA/Sum'
 * '<S87>'  : 'pid_control_V1_OLD/PID ALTURA/Sum Fdbk'
 * '<S88>'  : 'pid_control_V1_OLD/PID ALTURA/Tracking Mode'
 * '<S89>'  : 'pid_control_V1_OLD/PID ALTURA/Tracking Mode Sum'
 * '<S90>'  : 'pid_control_V1_OLD/PID ALTURA/Tsamp - Integral'
 * '<S91>'  : 'pid_control_V1_OLD/PID ALTURA/Tsamp - Ngain'
 * '<S92>'  : 'pid_control_V1_OLD/PID ALTURA/postSat Signal'
 * '<S93>'  : 'pid_control_V1_OLD/PID ALTURA/preInt Signal'
 * '<S94>'  : 'pid_control_V1_OLD/PID ALTURA/preSat Signal'
 * '<S95>'  : 'pid_control_V1_OLD/PID ALTURA/Anti-windup/Back Calculation'
 * '<S96>'  : 'pid_control_V1_OLD/PID ALTURA/D Gain/Internal Parameters'
 * '<S97>'  : 'pid_control_V1_OLD/PID ALTURA/External Derivative/Error'
 * '<S98>'  : 'pid_control_V1_OLD/PID ALTURA/Filter/Cont. Filter'
 * '<S99>'  : 'pid_control_V1_OLD/PID ALTURA/Filter ICs/Internal IC - Filter'
 * '<S100>' : 'pid_control_V1_OLD/PID ALTURA/I Gain/Internal Parameters'
 * '<S101>' : 'pid_control_V1_OLD/PID ALTURA/Ideal P Gain/Passthrough'
 * '<S102>' : 'pid_control_V1_OLD/PID ALTURA/Ideal P Gain Fdbk/Disabled'
 * '<S103>' : 'pid_control_V1_OLD/PID ALTURA/Integrator/Continuous'
 * '<S104>' : 'pid_control_V1_OLD/PID ALTURA/Integrator ICs/Internal IC'
 * '<S105>' : 'pid_control_V1_OLD/PID ALTURA/N Copy/Disabled'
 * '<S106>' : 'pid_control_V1_OLD/PID ALTURA/N Gain/Internal Parameters'
 * '<S107>' : 'pid_control_V1_OLD/PID ALTURA/P Copy/Disabled'
 * '<S108>' : 'pid_control_V1_OLD/PID ALTURA/Parallel P Gain/Internal Parameters'
 * '<S109>' : 'pid_control_V1_OLD/PID ALTURA/Reset Signal/Disabled'
 * '<S110>' : 'pid_control_V1_OLD/PID ALTURA/Saturation/Enabled'
 * '<S111>' : 'pid_control_V1_OLD/PID ALTURA/Saturation Fdbk/Disabled'
 * '<S112>' : 'pid_control_V1_OLD/PID ALTURA/Sum/Sum_PID'
 * '<S113>' : 'pid_control_V1_OLD/PID ALTURA/Sum Fdbk/Disabled'
 * '<S114>' : 'pid_control_V1_OLD/PID ALTURA/Tracking Mode/Disabled'
 * '<S115>' : 'pid_control_V1_OLD/PID ALTURA/Tracking Mode Sum/Passthrough'
 * '<S116>' : 'pid_control_V1_OLD/PID ALTURA/Tsamp - Integral/TsSignalSpecification'
 * '<S117>' : 'pid_control_V1_OLD/PID ALTURA/Tsamp - Ngain/Passthrough'
 * '<S118>' : 'pid_control_V1_OLD/PID ALTURA/postSat Signal/Forward_Path'
 * '<S119>' : 'pid_control_V1_OLD/PID ALTURA/preInt Signal/Internal PreInt'
 * '<S120>' : 'pid_control_V1_OLD/PID ALTURA/preSat Signal/Forward_Path'
 * '<S121>' : 'pid_control_V1_OLD/PID PITCH//ELEVATOR/Anti-windup'
 * '<S122>' : 'pid_control_V1_OLD/PID PITCH//ELEVATOR/D Gain'
 * '<S123>' : 'pid_control_V1_OLD/PID PITCH//ELEVATOR/External Derivative'
 * '<S124>' : 'pid_control_V1_OLD/PID PITCH//ELEVATOR/Filter'
 * '<S125>' : 'pid_control_V1_OLD/PID PITCH//ELEVATOR/Filter ICs'
 * '<S126>' : 'pid_control_V1_OLD/PID PITCH//ELEVATOR/I Gain'
 * '<S127>' : 'pid_control_V1_OLD/PID PITCH//ELEVATOR/Ideal P Gain'
 * '<S128>' : 'pid_control_V1_OLD/PID PITCH//ELEVATOR/Ideal P Gain Fdbk'
 * '<S129>' : 'pid_control_V1_OLD/PID PITCH//ELEVATOR/Integrator'
 * '<S130>' : 'pid_control_V1_OLD/PID PITCH//ELEVATOR/Integrator ICs'
 * '<S131>' : 'pid_control_V1_OLD/PID PITCH//ELEVATOR/N Copy'
 * '<S132>' : 'pid_control_V1_OLD/PID PITCH//ELEVATOR/N Gain'
 * '<S133>' : 'pid_control_V1_OLD/PID PITCH//ELEVATOR/P Copy'
 * '<S134>' : 'pid_control_V1_OLD/PID PITCH//ELEVATOR/Parallel P Gain'
 * '<S135>' : 'pid_control_V1_OLD/PID PITCH//ELEVATOR/Reset Signal'
 * '<S136>' : 'pid_control_V1_OLD/PID PITCH//ELEVATOR/Saturation'
 * '<S137>' : 'pid_control_V1_OLD/PID PITCH//ELEVATOR/Saturation Fdbk'
 * '<S138>' : 'pid_control_V1_OLD/PID PITCH//ELEVATOR/Sum'
 * '<S139>' : 'pid_control_V1_OLD/PID PITCH//ELEVATOR/Sum Fdbk'
 * '<S140>' : 'pid_control_V1_OLD/PID PITCH//ELEVATOR/Tracking Mode'
 * '<S141>' : 'pid_control_V1_OLD/PID PITCH//ELEVATOR/Tracking Mode Sum'
 * '<S142>' : 'pid_control_V1_OLD/PID PITCH//ELEVATOR/Tsamp - Integral'
 * '<S143>' : 'pid_control_V1_OLD/PID PITCH//ELEVATOR/Tsamp - Ngain'
 * '<S144>' : 'pid_control_V1_OLD/PID PITCH//ELEVATOR/postSat Signal'
 * '<S145>' : 'pid_control_V1_OLD/PID PITCH//ELEVATOR/preInt Signal'
 * '<S146>' : 'pid_control_V1_OLD/PID PITCH//ELEVATOR/preSat Signal'
 * '<S147>' : 'pid_control_V1_OLD/PID PITCH//ELEVATOR/Anti-windup/Back Calculation'
 * '<S148>' : 'pid_control_V1_OLD/PID PITCH//ELEVATOR/D Gain/Internal Parameters'
 * '<S149>' : 'pid_control_V1_OLD/PID PITCH//ELEVATOR/External Derivative/Error'
 * '<S150>' : 'pid_control_V1_OLD/PID PITCH//ELEVATOR/Filter/Cont. Filter'
 * '<S151>' : 'pid_control_V1_OLD/PID PITCH//ELEVATOR/Filter ICs/Internal IC - Filter'
 * '<S152>' : 'pid_control_V1_OLD/PID PITCH//ELEVATOR/I Gain/Internal Parameters'
 * '<S153>' : 'pid_control_V1_OLD/PID PITCH//ELEVATOR/Ideal P Gain/Passthrough'
 * '<S154>' : 'pid_control_V1_OLD/PID PITCH//ELEVATOR/Ideal P Gain Fdbk/Disabled'
 * '<S155>' : 'pid_control_V1_OLD/PID PITCH//ELEVATOR/Integrator/Continuous'
 * '<S156>' : 'pid_control_V1_OLD/PID PITCH//ELEVATOR/Integrator ICs/Internal IC'
 * '<S157>' : 'pid_control_V1_OLD/PID PITCH//ELEVATOR/N Copy/Disabled'
 * '<S158>' : 'pid_control_V1_OLD/PID PITCH//ELEVATOR/N Gain/Internal Parameters'
 * '<S159>' : 'pid_control_V1_OLD/PID PITCH//ELEVATOR/P Copy/Disabled'
 * '<S160>' : 'pid_control_V1_OLD/PID PITCH//ELEVATOR/Parallel P Gain/Internal Parameters'
 * '<S161>' : 'pid_control_V1_OLD/PID PITCH//ELEVATOR/Reset Signal/Disabled'
 * '<S162>' : 'pid_control_V1_OLD/PID PITCH//ELEVATOR/Saturation/Enabled'
 * '<S163>' : 'pid_control_V1_OLD/PID PITCH//ELEVATOR/Saturation Fdbk/Disabled'
 * '<S164>' : 'pid_control_V1_OLD/PID PITCH//ELEVATOR/Sum/Sum_PID'
 * '<S165>' : 'pid_control_V1_OLD/PID PITCH//ELEVATOR/Sum Fdbk/Disabled'
 * '<S166>' : 'pid_control_V1_OLD/PID PITCH//ELEVATOR/Tracking Mode/Disabled'
 * '<S167>' : 'pid_control_V1_OLD/PID PITCH//ELEVATOR/Tracking Mode Sum/Passthrough'
 * '<S168>' : 'pid_control_V1_OLD/PID PITCH//ELEVATOR/Tsamp - Integral/TsSignalSpecification'
 * '<S169>' : 'pid_control_V1_OLD/PID PITCH//ELEVATOR/Tsamp - Ngain/Passthrough'
 * '<S170>' : 'pid_control_V1_OLD/PID PITCH//ELEVATOR/postSat Signal/Forward_Path'
 * '<S171>' : 'pid_control_V1_OLD/PID PITCH//ELEVATOR/preInt Signal/Internal PreInt'
 * '<S172>' : 'pid_control_V1_OLD/PID PITCH//ELEVATOR/preSat Signal/Forward_Path'
 * '<S173>' : 'pid_control_V1_OLD/PID TIMON/Anti-windup'
 * '<S174>' : 'pid_control_V1_OLD/PID TIMON/D Gain'
 * '<S175>' : 'pid_control_V1_OLD/PID TIMON/External Derivative'
 * '<S176>' : 'pid_control_V1_OLD/PID TIMON/Filter'
 * '<S177>' : 'pid_control_V1_OLD/PID TIMON/Filter ICs'
 * '<S178>' : 'pid_control_V1_OLD/PID TIMON/I Gain'
 * '<S179>' : 'pid_control_V1_OLD/PID TIMON/Ideal P Gain'
 * '<S180>' : 'pid_control_V1_OLD/PID TIMON/Ideal P Gain Fdbk'
 * '<S181>' : 'pid_control_V1_OLD/PID TIMON/Integrator'
 * '<S182>' : 'pid_control_V1_OLD/PID TIMON/Integrator ICs'
 * '<S183>' : 'pid_control_V1_OLD/PID TIMON/N Copy'
 * '<S184>' : 'pid_control_V1_OLD/PID TIMON/N Gain'
 * '<S185>' : 'pid_control_V1_OLD/PID TIMON/P Copy'
 * '<S186>' : 'pid_control_V1_OLD/PID TIMON/Parallel P Gain'
 * '<S187>' : 'pid_control_V1_OLD/PID TIMON/Reset Signal'
 * '<S188>' : 'pid_control_V1_OLD/PID TIMON/Saturation'
 * '<S189>' : 'pid_control_V1_OLD/PID TIMON/Saturation Fdbk'
 * '<S190>' : 'pid_control_V1_OLD/PID TIMON/Sum'
 * '<S191>' : 'pid_control_V1_OLD/PID TIMON/Sum Fdbk'
 * '<S192>' : 'pid_control_V1_OLD/PID TIMON/Tracking Mode'
 * '<S193>' : 'pid_control_V1_OLD/PID TIMON/Tracking Mode Sum'
 * '<S194>' : 'pid_control_V1_OLD/PID TIMON/Tsamp - Integral'
 * '<S195>' : 'pid_control_V1_OLD/PID TIMON/Tsamp - Ngain'
 * '<S196>' : 'pid_control_V1_OLD/PID TIMON/postSat Signal'
 * '<S197>' : 'pid_control_V1_OLD/PID TIMON/preInt Signal'
 * '<S198>' : 'pid_control_V1_OLD/PID TIMON/preSat Signal'
 * '<S199>' : 'pid_control_V1_OLD/PID TIMON/Anti-windup/Passthrough'
 * '<S200>' : 'pid_control_V1_OLD/PID TIMON/D Gain/Internal Parameters'
 * '<S201>' : 'pid_control_V1_OLD/PID TIMON/External Derivative/Error'
 * '<S202>' : 'pid_control_V1_OLD/PID TIMON/Filter/Cont. Filter'
 * '<S203>' : 'pid_control_V1_OLD/PID TIMON/Filter ICs/Internal IC - Filter'
 * '<S204>' : 'pid_control_V1_OLD/PID TIMON/I Gain/Internal Parameters'
 * '<S205>' : 'pid_control_V1_OLD/PID TIMON/Ideal P Gain/Passthrough'
 * '<S206>' : 'pid_control_V1_OLD/PID TIMON/Ideal P Gain Fdbk/Disabled'
 * '<S207>' : 'pid_control_V1_OLD/PID TIMON/Integrator/Continuous'
 * '<S208>' : 'pid_control_V1_OLD/PID TIMON/Integrator ICs/Internal IC'
 * '<S209>' : 'pid_control_V1_OLD/PID TIMON/N Copy/Disabled'
 * '<S210>' : 'pid_control_V1_OLD/PID TIMON/N Gain/Internal Parameters'
 * '<S211>' : 'pid_control_V1_OLD/PID TIMON/P Copy/Disabled'
 * '<S212>' : 'pid_control_V1_OLD/PID TIMON/Parallel P Gain/Internal Parameters'
 * '<S213>' : 'pid_control_V1_OLD/PID TIMON/Reset Signal/Disabled'
 * '<S214>' : 'pid_control_V1_OLD/PID TIMON/Saturation/Enabled'
 * '<S215>' : 'pid_control_V1_OLD/PID TIMON/Saturation Fdbk/Disabled'
 * '<S216>' : 'pid_control_V1_OLD/PID TIMON/Sum/Sum_PID'
 * '<S217>' : 'pid_control_V1_OLD/PID TIMON/Sum Fdbk/Disabled'
 * '<S218>' : 'pid_control_V1_OLD/PID TIMON/Tracking Mode/Disabled'
 * '<S219>' : 'pid_control_V1_OLD/PID TIMON/Tracking Mode Sum/Passthrough'
 * '<S220>' : 'pid_control_V1_OLD/PID TIMON/Tsamp - Integral/TsSignalSpecification'
 * '<S221>' : 'pid_control_V1_OLD/PID TIMON/Tsamp - Ngain/Passthrough'
 * '<S222>' : 'pid_control_V1_OLD/PID TIMON/postSat Signal/Forward_Path'
 * '<S223>' : 'pid_control_V1_OLD/PID TIMON/preInt Signal/Internal PreInt'
 * '<S224>' : 'pid_control_V1_OLD/PID TIMON/preSat Signal/Forward_Path'
 * '<S225>' : 'pid_control_V1_OLD/PID VELOCIDAD/Anti-windup'
 * '<S226>' : 'pid_control_V1_OLD/PID VELOCIDAD/D Gain'
 * '<S227>' : 'pid_control_V1_OLD/PID VELOCIDAD/External Derivative'
 * '<S228>' : 'pid_control_V1_OLD/PID VELOCIDAD/Filter'
 * '<S229>' : 'pid_control_V1_OLD/PID VELOCIDAD/Filter ICs'
 * '<S230>' : 'pid_control_V1_OLD/PID VELOCIDAD/I Gain'
 * '<S231>' : 'pid_control_V1_OLD/PID VELOCIDAD/Ideal P Gain'
 * '<S232>' : 'pid_control_V1_OLD/PID VELOCIDAD/Ideal P Gain Fdbk'
 * '<S233>' : 'pid_control_V1_OLD/PID VELOCIDAD/Integrator'
 * '<S234>' : 'pid_control_V1_OLD/PID VELOCIDAD/Integrator ICs'
 * '<S235>' : 'pid_control_V1_OLD/PID VELOCIDAD/N Copy'
 * '<S236>' : 'pid_control_V1_OLD/PID VELOCIDAD/N Gain'
 * '<S237>' : 'pid_control_V1_OLD/PID VELOCIDAD/P Copy'
 * '<S238>' : 'pid_control_V1_OLD/PID VELOCIDAD/Parallel P Gain'
 * '<S239>' : 'pid_control_V1_OLD/PID VELOCIDAD/Reset Signal'
 * '<S240>' : 'pid_control_V1_OLD/PID VELOCIDAD/Saturation'
 * '<S241>' : 'pid_control_V1_OLD/PID VELOCIDAD/Saturation Fdbk'
 * '<S242>' : 'pid_control_V1_OLD/PID VELOCIDAD/Sum'
 * '<S243>' : 'pid_control_V1_OLD/PID VELOCIDAD/Sum Fdbk'
 * '<S244>' : 'pid_control_V1_OLD/PID VELOCIDAD/Tracking Mode'
 * '<S245>' : 'pid_control_V1_OLD/PID VELOCIDAD/Tracking Mode Sum'
 * '<S246>' : 'pid_control_V1_OLD/PID VELOCIDAD/Tsamp - Integral'
 * '<S247>' : 'pid_control_V1_OLD/PID VELOCIDAD/Tsamp - Ngain'
 * '<S248>' : 'pid_control_V1_OLD/PID VELOCIDAD/postSat Signal'
 * '<S249>' : 'pid_control_V1_OLD/PID VELOCIDAD/preInt Signal'
 * '<S250>' : 'pid_control_V1_OLD/PID VELOCIDAD/preSat Signal'
 * '<S251>' : 'pid_control_V1_OLD/PID VELOCIDAD/Anti-windup/Cont. Clamping Parallel'
 * '<S252>' : 'pid_control_V1_OLD/PID VELOCIDAD/Anti-windup/Cont. Clamping Parallel/Dead Zone'
 * '<S253>' : 'pid_control_V1_OLD/PID VELOCIDAD/Anti-windup/Cont. Clamping Parallel/Dead Zone/Enabled'
 * '<S254>' : 'pid_control_V1_OLD/PID VELOCIDAD/D Gain/Internal Parameters'
 * '<S255>' : 'pid_control_V1_OLD/PID VELOCIDAD/External Derivative/Error'
 * '<S256>' : 'pid_control_V1_OLD/PID VELOCIDAD/Filter/Cont. Filter'
 * '<S257>' : 'pid_control_V1_OLD/PID VELOCIDAD/Filter ICs/Internal IC - Filter'
 * '<S258>' : 'pid_control_V1_OLD/PID VELOCIDAD/I Gain/Internal Parameters'
 * '<S259>' : 'pid_control_V1_OLD/PID VELOCIDAD/Ideal P Gain/Passthrough'
 * '<S260>' : 'pid_control_V1_OLD/PID VELOCIDAD/Ideal P Gain Fdbk/Disabled'
 * '<S261>' : 'pid_control_V1_OLD/PID VELOCIDAD/Integrator/Continuous'
 * '<S262>' : 'pid_control_V1_OLD/PID VELOCIDAD/Integrator ICs/Internal IC'
 * '<S263>' : 'pid_control_V1_OLD/PID VELOCIDAD/N Copy/Disabled'
 * '<S264>' : 'pid_control_V1_OLD/PID VELOCIDAD/N Gain/Internal Parameters'
 * '<S265>' : 'pid_control_V1_OLD/PID VELOCIDAD/P Copy/Disabled'
 * '<S266>' : 'pid_control_V1_OLD/PID VELOCIDAD/Parallel P Gain/Internal Parameters'
 * '<S267>' : 'pid_control_V1_OLD/PID VELOCIDAD/Reset Signal/Disabled'
 * '<S268>' : 'pid_control_V1_OLD/PID VELOCIDAD/Saturation/Enabled'
 * '<S269>' : 'pid_control_V1_OLD/PID VELOCIDAD/Saturation Fdbk/Disabled'
 * '<S270>' : 'pid_control_V1_OLD/PID VELOCIDAD/Sum/Sum_PID'
 * '<S271>' : 'pid_control_V1_OLD/PID VELOCIDAD/Sum Fdbk/Disabled'
 * '<S272>' : 'pid_control_V1_OLD/PID VELOCIDAD/Tracking Mode/Disabled'
 * '<S273>' : 'pid_control_V1_OLD/PID VELOCIDAD/Tracking Mode Sum/Passthrough'
 * '<S274>' : 'pid_control_V1_OLD/PID VELOCIDAD/Tsamp - Integral/TsSignalSpecification'
 * '<S275>' : 'pid_control_V1_OLD/PID VELOCIDAD/Tsamp - Ngain/Passthrough'
 * '<S276>' : 'pid_control_V1_OLD/PID VELOCIDAD/postSat Signal/Forward_Path'
 * '<S277>' : 'pid_control_V1_OLD/PID VELOCIDAD/preInt Signal/Internal PreInt'
 * '<S278>' : 'pid_control_V1_OLD/PID VELOCIDAD/preSat Signal/Forward_Path'
 * '<S279>' : 'pid_control_V1_OLD/PID_HEADING_OUTER/Anti-windup'
 * '<S280>' : 'pid_control_V1_OLD/PID_HEADING_OUTER/D Gain'
 * '<S281>' : 'pid_control_V1_OLD/PID_HEADING_OUTER/External Derivative'
 * '<S282>' : 'pid_control_V1_OLD/PID_HEADING_OUTER/Filter'
 * '<S283>' : 'pid_control_V1_OLD/PID_HEADING_OUTER/Filter ICs'
 * '<S284>' : 'pid_control_V1_OLD/PID_HEADING_OUTER/I Gain'
 * '<S285>' : 'pid_control_V1_OLD/PID_HEADING_OUTER/Ideal P Gain'
 * '<S286>' : 'pid_control_V1_OLD/PID_HEADING_OUTER/Ideal P Gain Fdbk'
 * '<S287>' : 'pid_control_V1_OLD/PID_HEADING_OUTER/Integrator'
 * '<S288>' : 'pid_control_V1_OLD/PID_HEADING_OUTER/Integrator ICs'
 * '<S289>' : 'pid_control_V1_OLD/PID_HEADING_OUTER/N Copy'
 * '<S290>' : 'pid_control_V1_OLD/PID_HEADING_OUTER/N Gain'
 * '<S291>' : 'pid_control_V1_OLD/PID_HEADING_OUTER/P Copy'
 * '<S292>' : 'pid_control_V1_OLD/PID_HEADING_OUTER/Parallel P Gain'
 * '<S293>' : 'pid_control_V1_OLD/PID_HEADING_OUTER/Reset Signal'
 * '<S294>' : 'pid_control_V1_OLD/PID_HEADING_OUTER/Saturation'
 * '<S295>' : 'pid_control_V1_OLD/PID_HEADING_OUTER/Saturation Fdbk'
 * '<S296>' : 'pid_control_V1_OLD/PID_HEADING_OUTER/Sum'
 * '<S297>' : 'pid_control_V1_OLD/PID_HEADING_OUTER/Sum Fdbk'
 * '<S298>' : 'pid_control_V1_OLD/PID_HEADING_OUTER/Tracking Mode'
 * '<S299>' : 'pid_control_V1_OLD/PID_HEADING_OUTER/Tracking Mode Sum'
 * '<S300>' : 'pid_control_V1_OLD/PID_HEADING_OUTER/Tsamp - Integral'
 * '<S301>' : 'pid_control_V1_OLD/PID_HEADING_OUTER/Tsamp - Ngain'
 * '<S302>' : 'pid_control_V1_OLD/PID_HEADING_OUTER/postSat Signal'
 * '<S303>' : 'pid_control_V1_OLD/PID_HEADING_OUTER/preInt Signal'
 * '<S304>' : 'pid_control_V1_OLD/PID_HEADING_OUTER/preSat Signal'
 * '<S305>' : 'pid_control_V1_OLD/PID_HEADING_OUTER/Anti-windup/Passthrough'
 * '<S306>' : 'pid_control_V1_OLD/PID_HEADING_OUTER/D Gain/Internal Parameters'
 * '<S307>' : 'pid_control_V1_OLD/PID_HEADING_OUTER/External Derivative/Error'
 * '<S308>' : 'pid_control_V1_OLD/PID_HEADING_OUTER/Filter/Cont. Filter'
 * '<S309>' : 'pid_control_V1_OLD/PID_HEADING_OUTER/Filter ICs/Internal IC - Filter'
 * '<S310>' : 'pid_control_V1_OLD/PID_HEADING_OUTER/I Gain/Internal Parameters'
 * '<S311>' : 'pid_control_V1_OLD/PID_HEADING_OUTER/Ideal P Gain/Passthrough'
 * '<S312>' : 'pid_control_V1_OLD/PID_HEADING_OUTER/Ideal P Gain Fdbk/Disabled'
 * '<S313>' : 'pid_control_V1_OLD/PID_HEADING_OUTER/Integrator/Continuous'
 * '<S314>' : 'pid_control_V1_OLD/PID_HEADING_OUTER/Integrator ICs/Internal IC'
 * '<S315>' : 'pid_control_V1_OLD/PID_HEADING_OUTER/N Copy/Disabled'
 * '<S316>' : 'pid_control_V1_OLD/PID_HEADING_OUTER/N Gain/Internal Parameters'
 * '<S317>' : 'pid_control_V1_OLD/PID_HEADING_OUTER/P Copy/Disabled'
 * '<S318>' : 'pid_control_V1_OLD/PID_HEADING_OUTER/Parallel P Gain/Internal Parameters'
 * '<S319>' : 'pid_control_V1_OLD/PID_HEADING_OUTER/Reset Signal/Disabled'
 * '<S320>' : 'pid_control_V1_OLD/PID_HEADING_OUTER/Saturation/Passthrough'
 * '<S321>' : 'pid_control_V1_OLD/PID_HEADING_OUTER/Saturation Fdbk/Disabled'
 * '<S322>' : 'pid_control_V1_OLD/PID_HEADING_OUTER/Sum/Sum_PID'
 * '<S323>' : 'pid_control_V1_OLD/PID_HEADING_OUTER/Sum Fdbk/Disabled'
 * '<S324>' : 'pid_control_V1_OLD/PID_HEADING_OUTER/Tracking Mode/Disabled'
 * '<S325>' : 'pid_control_V1_OLD/PID_HEADING_OUTER/Tracking Mode Sum/Passthrough'
 * '<S326>' : 'pid_control_V1_OLD/PID_HEADING_OUTER/Tsamp - Integral/TsSignalSpecification'
 * '<S327>' : 'pid_control_V1_OLD/PID_HEADING_OUTER/Tsamp - Ngain/Passthrough'
 * '<S328>' : 'pid_control_V1_OLD/PID_HEADING_OUTER/postSat Signal/Forward_Path'
 * '<S329>' : 'pid_control_V1_OLD/PID_HEADING_OUTER/preInt Signal/Internal PreInt'
 * '<S330>' : 'pid_control_V1_OLD/PID_HEADING_OUTER/preSat Signal/Forward_Path'
 * '<S331>' : 'pid_control_V1_OLD/SUBSYSTEM_MODEL/Band-Limited White Noise'
 * '<S332>' : 'pid_control_V1_OLD/SUBSYSTEM_MODEL/Compare To Constant'
 * '<S333>' : 'pid_control_V1_OLD/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))'
 * '<S334>' : 'pid_control_V1_OLD/SUBSYSTEM_MODEL/MATLAB Function - MODEL'
 * '<S335>' : 'pid_control_V1_OLD/SUBSYSTEM_MODEL/MATLAB Function-reset'
 * '<S336>' : 'pid_control_V1_OLD/SUBSYSTEM_MODEL/Subscribe1_TURBULENCIA1'
 * '<S337>' : 'pid_control_V1_OLD/SUBSYSTEM_MODEL/Subscribe_HEAVE'
 * '<S338>' : 'pid_control_V1_OLD/SUBSYSTEM_MODEL/Subscribe_RATE'
 * '<S339>' : 'pid_control_V1_OLD/SUBSYSTEM_MODEL/Subscribe_TURBULENCIA'
 * '<S340>' : 'pid_control_V1_OLD/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Angle Conversion'
 * '<S341>' : 'pid_control_V1_OLD/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Filters on angular rates'
 * '<S342>' : 'pid_control_V1_OLD/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Filters on velocities'
 * '<S343>' : 'pid_control_V1_OLD/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Length Conversion'
 * '<S344>' : 'pid_control_V1_OLD/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Length Conversion1'
 * '<S345>' : 'pid_control_V1_OLD/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/RMS turbulence  intensities'
 * '<S346>' : 'pid_control_V1_OLD/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Select angular rates'
 * '<S347>' : 'pid_control_V1_OLD/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Select velocities'
 * '<S348>' : 'pid_control_V1_OLD/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Turbulence scale lengths'
 * '<S349>' : 'pid_control_V1_OLD/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Velocity Conversion'
 * '<S350>' : 'pid_control_V1_OLD/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Velocity Conversion2'
 * '<S351>' : 'pid_control_V1_OLD/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/White Noise'
 * '<S352>' : 'pid_control_V1_OLD/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Filters on angular rates/Hpgw'
 * '<S353>' : 'pid_control_V1_OLD/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Filters on angular rates/Hqgw'
 * '<S354>' : 'pid_control_V1_OLD/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Filters on angular rates/Hrgw'
 * '<S355>' : 'pid_control_V1_OLD/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Filters on velocities/Hugw(s)'
 * '<S356>' : 'pid_control_V1_OLD/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Filters on velocities/Hvgw(s)'
 * '<S357>' : 'pid_control_V1_OLD/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Filters on velocities/Hwgw(s)'
 * '<S358>' : 'pid_control_V1_OLD/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/RMS turbulence  intensities/High Altitude Intensity'
 * '<S359>' : 'pid_control_V1_OLD/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/RMS turbulence  intensities/Low Altitude Intensity'
 * '<S360>' : 'pid_control_V1_OLD/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Select angular rates/Interpolate  rates'
 * '<S361>' : 'pid_control_V1_OLD/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Select angular rates/Low altitude  rates'
 * '<S362>' : 'pid_control_V1_OLD/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Select angular rates/Medium//High  altitude rates'
 * '<S363>' : 'pid_control_V1_OLD/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Select angular rates/Merge Subsystems'
 * '<S364>' : 'pid_control_V1_OLD/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Select angular rates/Interpolate  rates/wind to body transformation'
 * '<S365>' : 'pid_control_V1_OLD/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Select angular rates/Interpolate  rates/wind to body transformation/convert to earth coords'
 * '<S366>' : 'pid_control_V1_OLD/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Select angular rates/Low altitude  rates/wind to body transformation'
 * '<S367>' : 'pid_control_V1_OLD/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Select angular rates/Low altitude  rates/wind to body transformation/convert to earth coords'
 * '<S368>' : 'pid_control_V1_OLD/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Select velocities/Interpolate  velocities'
 * '<S369>' : 'pid_control_V1_OLD/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Select velocities/Low altitude  velocities'
 * '<S370>' : 'pid_control_V1_OLD/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Select velocities/Medium//High  altitude velocities'
 * '<S371>' : 'pid_control_V1_OLD/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Select velocities/Merge Subsystems'
 * '<S372>' : 'pid_control_V1_OLD/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Select velocities/Interpolate  velocities/wind to body transformation'
 * '<S373>' : 'pid_control_V1_OLD/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Select velocities/Interpolate  velocities/wind to body transformation/convert to earth coords'
 * '<S374>' : 'pid_control_V1_OLD/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Select velocities/Low altitude  velocities/wind to body transformation'
 * '<S375>' : 'pid_control_V1_OLD/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Select velocities/Low altitude  velocities/wind to body transformation/convert to earth coords'
 * '<S376>' : 'pid_control_V1_OLD/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Turbulence scale lengths/Low altitude scale length'
 * '<S377>' : 'pid_control_V1_OLD/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Turbulence scale lengths/Medium//High altitude scale length'
 * '<S378>' : 'pid_control_V1_OLD/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Turbulence scale lengths/Medium//High altitude scale length/Length Conversion'
 * '<S379>' : 'pid_control_V1_OLD/SUBSYSTEM_MODEL/Subscribe1_TURBULENCIA1/Enabled Subsystem'
 * '<S380>' : 'pid_control_V1_OLD/SUBSYSTEM_MODEL/Subscribe_HEAVE/Enabled Subsystem'
 * '<S381>' : 'pid_control_V1_OLD/SUBSYSTEM_MODEL/Subscribe_RATE/Enabled Subsystem'
 * '<S382>' : 'pid_control_V1_OLD/SUBSYSTEM_MODEL/Subscribe_TURBULENCIA/Enabled Subsystem'
 * '<S383>' : 'pid_control_V1_OLD/Subscribe-ALTURA1/Enabled Subsystem'
 * '<S384>' : 'pid_control_V1_OLD/Subscribe-YAW/Enabled Subsystem'
 */
#endif                                 /* pid_control_V1_OLD_h_ */
