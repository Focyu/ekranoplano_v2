/*
 * pid_control_V1.h
 *
 * Trial License - for use to evaluate programs for possible purchase as
 * an end-user only.
 *
 * Code generation for model "pid_control_V1".
 *
 * Model version              : 12.123
 * Simulink Coder version : 25.2 (R2025b) 28-Jul-2025
 * C++ source code generated on : Sat Mar 28 20:25:43 2026
 *
 * Target selection: ert.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Intel->x86-64 (Linux 64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#ifndef pid_control_V1_h_
#define pid_control_V1_h_
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

/* Block signals for system '<S282>/Enabled Subsystem' */
struct B_EnabledSubsystem_pid_contro_T {
  SL_Bus_std_msgs_Bool In1;            /* '<S325>/In1' */
};

/* Block signals for system '<S283>/Enabled Subsystem' */
struct B_EnabledSubsystem_pid_cont_d_T {
  SL_Bus_std_msgs_Float64 In1;         /* '<S326>/In1' */
};

/* Block signals (default storage) */
struct B_pid_control_V1_T {
  SL_Bus_gazebo_msgs_SetEntityStateRequest BusAssignment;/* '<Root>/Bus Assignment' */
  real_T IC[12];                       /* '<S10>/IC' */
  real_T x[12];                        /* '<S10>/Integrator' */
  real_T RotationAnglestoDirectionCo[9];
                        /* '<S10>/Rotation Angles to Direction Cosine Matrix' */
  real_T FA_b_tmp[9];
  real_T TmpSignalConversionAtSFunct[5];/* '<S10>/MATLAB Function - MODEL' */
  char_T b_zeroDelimTopic[25];
  real_T wbe_b[3];
  real_T F_b[3];
  real_T Product_m[3];                 /* '<S318>/Product' */
  real_T Dtot[3];
  char_T b_zeroDelimTopic_c[22];
  char_T b_zeroDelimTopic_k[22];
  char_T b_zeroDelimTopic_cx[17];
  char_T b_zeroDelimTopic_b[17];
  sJ4ih70VmKcvCeguWN0mNVF deadline;
  sJ4ih70VmKcvCeguWN0mNVF deadline_p;
  real_T w_n[2];                       /* '<S301>/w' */
  real_T w1_c[2];                      /* '<S301>/w1' */
  real_T w_d[2];                       /* '<S300>/w' */
  real_T w_e0[2];                      /* '<S299>/w' */
  real_T UnaryMinus[2];                /* '<S299>/Unary Minus' */
  real_T w_o[2];                       /* '<S298>/w' */
  real_T sigma_w[2];                   /* '<S298>/sigma_w' */
  real_T frac[2];
  real_T dv[2];
  real_T Switch3;                      /* '<Root>/Switch3' */
  real_T Gain;                         /* '<Root>/Gain' */
  real_T FilterCoefficient;            /* '<S104>/Filter Coefficient' */
  real_T Saturation;                   /* '<S108>/Saturation' */
  real_T FilterCoefficient_c;          /* '<S52>/Filter Coefficient' */
  real_T Saturation_k;                 /* '<S56>/Saturation' */
  real_T Saturation_i;                 /* '<Root>/Saturation' */
  real_T RateLimiter;                  /* '<Root>/Rate Limiter' */
  real_T FilterCoefficient_m;          /* '<S156>/Filter Coefficient' */
  real_T Saturation_f;                 /* '<S160>/Saturation' */
  real_T Switch2;                      /* '<Root>/Switch2' */
  real_T FilterCoefficient_p;          /* '<S208>/Filter Coefficient' */
  real_T Saturation_m;                 /* '<S212>/Saturation' */
  real_T FilterCoefficient_cv;         /* '<S262>/Filter Coefficient' */
  real_T Saturation_o;                 /* '<S266>/Saturation' */
  real_T Memory[3];                    /* '<S10>/Memory' */
  real_T Memory1[3];                   /* '<S10>/Memory1' */
  real_T Switch;                       /* '<S39>/Switch' */
  real_T SumI4;                        /* '<S93>/SumI4' */
  real_T SumI4_i;                      /* '<S145>/SumI4' */
  real_T IntegralGain;                 /* '<S202>/Integral Gain' */
  real_T Switch_j;                     /* '<S249>/Switch' */
  real_T Product[4];                   /* '<S297>/Product' */
  real_T Switch_p[3];                  /* '<S10>/Switch' */
  real_T Switch1[3];                   /* '<S10>/Switch1' */
  real_T data;
  real_T data_n;
  real_T Power;                        /* '<S10>/Product2' */
  real_T Gain3;                        /* '<S10>/Gain3' */
  real_T EnergykWh;                    /* '<S10>/Gain1' */
  real_T powerdemand;                  /* '<S10>/Divide' */
  real_T loadtorque;                   /* '<S10>/Divide1' */
  real_T Output;                       /* '<S277>/Output' */
  real_T Sum[3];                       /* '<S10>/Sum' */
  real_T Sum1[3];                      /* '<S10>/Sum1' */
  real_T XDOT[40];                     /* '<S10>/MATLAB Function - MODEL' */
  real_T CL_total;                     /* '<S10>/MATLAB Function - MODEL' */
  real_T mu_Lw_out;                    /* '<S10>/MATLAB Function - MODEL' */
  real_T mu_Dw_out;                    /* '<S10>/MATLAB Function - MODEL' */
  real_T w[2];                         /* '<S303>/w' */
  real_T w_a[2];                       /* '<S303>/w ' */
  real_T LwgV1[2];                     /* '<S303>/Lwg//V 1' */
  real_T w_g[2];                       /* '<S302>/w' */
  real_T w_e[2];                       /* '<S302>/w ' */
  real_T w1[2];                        /* '<S302>/w 1' */
  real_T u2;
  real_T w_r;
  real_T Va;
  real_T beta;
  real_T q_aero;
  real_T hw;
  real_T hh;
  real_T Q;
  real_T CL_h_OGE;
  real_T CL_w_IGE;
  real_T CD_iw_IGE;
  real_T CD_ih_IGE;
  real_T Dtot_c;
  real_T Ltot;
  real_T CQ;
  real_T Cl;
  real_T Vd1;
  real_T Tp1;
  real_T Tp2;
  real_T c_phi;
  real_T s_phi;
  real_T c_the;
  real_T s_the;
  real_T c_psi;
  real_T s_psi;
  real_T Sum2_l;                       /* '<Root>/Sum2' */
  real_T Sum_b;                        /* '<S110>/Sum' */
  real_T SignPreSat;                   /* '<S39>/SignPreSat' */
  real_T Sum1_g;                       /* '<Root>/Sum1' */
  real_T Sum_hl;                       /* '<S162>/Sum' */
  real_T Sum5;                         /* '<Root>/Sum5' */
  real_T SignPreSat_h;                 /* '<S249>/SignPreSat' */
  real_T FE1_b_idx_1;
  real_T Mcg_b_idx_2;
  real_T Mcg_b_idx_0;
  real_T FE2_b_idx_0;
  real_T FE2_b_idx_2;
  real_T Fg_b_idx_2;
  real_T Fg_b_idx_1;
  real_T FA_b_idx_0;
  real_T FA_b_idx_1;
  real_T FA_b_idx_2;
  real_T c_the_tmp;
  real_T c_the_tmp_f;
  real_T Ltot_tmp;
  SL_Bus_std_msgs_Float64 SourceBlock_o2_k;/* '<S284>/SourceBlock' */
  SL_Bus_std_msgs_Float64 SourceBlock_o2_p;/* '<S283>/SourceBlock' */
  uint32_T bpIndex[2];
  uint32_T lengthOut;                  /* '<Root>/MATLAB Function1' */
  uint32_T lengthOut_e;                /* '<Root>/MATLAB Function' */
  uint8_T stringOut[128];              /* '<Root>/MATLAB Function1' */
  uint8_T stringOut_l[128];            /* '<Root>/MATLAB Function' */
  boolean_T Compare;                   /* '<S278>/Compare' */
  boolean_T AND3;                      /* '<S39>/AND3' */
  boolean_T Memory_a;                  /* '<S39>/Memory' */
  boolean_T AND3_c;                    /* '<S249>/AND3' */
  boolean_T Memory_h;                  /* '<S249>/Memory' */
  boolean_T SourceBlock_o1;            /* '<S12>/SourceBlock' */
  boolean_T SourceBlock_o1_o;          /* '<S11>/SourceBlock' */
  boolean_T SourceBlock_o1_h;          /* '<S285>/SourceBlock' */
  boolean_T SourceBlock_o1_d;          /* '<S284>/SourceBlock' */
  boolean_T SourceBlock_o1_c;          /* '<S283>/SourceBlock' */
  boolean_T SourceBlock_o1_k;          /* '<S282>/SourceBlock' */
  B_EnabledSubsystem_pid_cont_d_T EnabledSubsystem_a;/* '<S12>/Enabled Subsystem' */
  B_EnabledSubsystem_pid_cont_d_T EnabledSubsystem_b;/* '<S11>/Enabled Subsystem' */
  B_EnabledSubsystem_pid_contro_T EnabledSubsystem_pt;/* '<S285>/Enabled Subsystem' */
  B_EnabledSubsystem_pid_cont_d_T EnabledSubsystem_p;/* '<S284>/Enabled Subsystem' */
  B_EnabledSubsystem_pid_cont_d_T EnabledSubsystem_k;/* '<S283>/Enabled Subsystem' */
  B_EnabledSubsystem_pid_contro_T EnabledSubsystem;/* '<S282>/Enabled Subsystem' */
};

/* Block states (default storage) for system '<Root>' */
struct DW_pid_control_V1_T {
  ros_slros2_internal_block_Ser_T obj; /* '<S2>/ServiceCaller' */
  ros_slros2_internal_block_Sub_T obj_k;/* '<S12>/SourceBlock' */
  ros_slros2_internal_block_Sub_T obj_m;/* '<S11>/SourceBlock' */
  ros_slros2_internal_block_Sub_T obj_h;/* '<S285>/SourceBlock' */
  ros_slros2_internal_block_Sub_T obj_p;/* '<S284>/SourceBlock' */
  ros_slros2_internal_block_Sub_T obj_hy;/* '<S283>/SourceBlock' */
  ros_slros2_internal_block_Sub_T obj_h4;/* '<S282>/SourceBlock' */
  real_T UnitDelay3_DSTATE;            /* '<Root>/Unit Delay3' */
  real_T UnitDelay2_DSTATE;            /* '<Root>/Unit Delay2' */
  real_T Memory2_PreviousInput[12];    /* '<S10>/Memory2' */
  real_T PrevY;                        /* '<Root>/Rate Limiter' */
  real_T LastMajorTime;                /* '<Root>/Rate Limiter' */
  real_T Memory_PreviousInput[3];      /* '<S10>/Memory' */
  real_T Memory1_PreviousInput[3];     /* '<S10>/Memory1' */
  real_T NextOutput[4];                /* '<S297>/White Noise' */
  real_T NextOutput_k;                 /* '<S277>/White Noise' */
  struct {
    void *LoggedData;
  } ToWorkspace_PWORK;                 /* '<Root>/To Workspace' */

  struct {
    void *LoggedData;
  } ToWorkspace_PWORK_g;               /* '<S10>/To Workspace' */

  struct {
    void *LoggedData;
  } ToWorkspace1_PWORK;                /* '<S10>/To Workspace1' */

  struct {
    void *LoggedData;
  } ToWorkspace10_PWORK;               /* '<S10>/To Workspace10' */

  struct {
    void *LoggedData;
  } ToWorkspace11_PWORK;               /* '<S10>/To Workspace11' */

  struct {
    void *LoggedData;
  } ToWorkspace12_PWORK;               /* '<S10>/To Workspace12' */

  struct {
    void *LoggedData;
  } ToWorkspace13_PWORK;               /* '<S10>/To Workspace13' */

  struct {
    void *LoggedData;
  } ToWorkspace14_PWORK;               /* '<S10>/To Workspace14' */

  struct {
    void *LoggedData;
  } ToWorkspace15_PWORK;               /* '<S10>/To Workspace15' */

  struct {
    void *LoggedData;
  } ToWorkspace16_PWORK;               /* '<S10>/To Workspace16' */

  struct {
    void *LoggedData;
  } ToWorkspace17_PWORK;               /* '<S10>/To Workspace17' */

  struct {
    void *LoggedData;
  } ToWorkspace18_PWORK;               /* '<S10>/To Workspace18' */

  struct {
    void *LoggedData;
  } ToWorkspace19_PWORK;               /* '<S10>/To Workspace19' */

  struct {
    void *LoggedData;
  } ToWorkspace2_PWORK;                /* '<S10>/To Workspace2' */

  struct {
    void *LoggedData;
  } ToWorkspace20_PWORK;               /* '<S10>/To Workspace20' */

  struct {
    void *LoggedData;
  } ToWorkspace21_PWORK;               /* '<S10>/To Workspace21' */

  struct {
    void *LoggedData;
  } ToWorkspace22_PWORK;               /* '<S10>/To Workspace22' */

  struct {
    void *LoggedData;
  } ToWorkspace23_PWORK;               /* '<S10>/To Workspace23' */

  struct {
    void *LoggedData;
  } ToWorkspace24_PWORK;               /* '<S10>/To Workspace24' */

  struct {
    void *LoggedData;
  } ToWorkspace25_PWORK;               /* '<S10>/To Workspace25' */

  struct {
    void *LoggedData;
  } ToWorkspace26_PWORK;               /* '<S10>/To Workspace26' */

  struct {
    void *LoggedData;
  } ToWorkspace27_PWORK;               /* '<S10>/To Workspace27' */

  struct {
    void *LoggedData;
  } ToWorkspace28_PWORK;               /* '<S10>/To Workspace28' */

  struct {
    void *LoggedData;
  } ToWorkspace29_PWORK;               /* '<S10>/To Workspace29' */

  struct {
    void *LoggedData;
  } ToWorkspace3_PWORK;                /* '<S10>/To Workspace3' */

  struct {
    void *LoggedData;
  } ToWorkspace30_PWORK;               /* '<S10>/To Workspace30' */

  struct {
    void *LoggedData;
  } ToWorkspace31_PWORK;               /* '<S10>/To Workspace31' */

  struct {
    void *LoggedData;
  } ToWorkspace32_PWORK;               /* '<S10>/To Workspace32' */

  struct {
    void *LoggedData;
  } ToWorkspace33_PWORK;               /* '<S10>/To Workspace33' */

  struct {
    void *LoggedData;
  } ToWorkspace4_PWORK;                /* '<S10>/To Workspace4' */

  struct {
    void *LoggedData;
  } ToWorkspace5_PWORK;                /* '<S10>/To Workspace5' */

  struct {
    void *LoggedData;
  } ToWorkspace6_PWORK;                /* '<S10>/To Workspace6' */

  struct {
    void *LoggedData;
  } ToWorkspace7_PWORK;                /* '<S10>/To Workspace7' */

  struct {
    void *LoggedData;
  } ToWorkspace8_PWORK;                /* '<S10>/To Workspace8' */

  struct {
    void *LoggedData;
  } ToWorkspace9_PWORK;                /* '<S10>/To Workspace9' */

  uint32_T PreLookUpIndexSearchprobofexcee;
                        /* '<S304>/PreLook-Up Index Search  (prob of exceed)' */
  uint32_T PreLookUpIndexSearchaltitude_DW;
                              /* '<S304>/PreLook-Up Index Search  (altitude)' */
  uint32_T RandSeed[4];                /* '<S297>/White Noise' */
  uint32_T RandSeed_a;                 /* '<S277>/White Noise' */
  robotics_slcore_internal_bloc_T obj_c;
                             /* '<Root>/Coordinate Transformation Conversion' */
  int8_T ifHeightMaxlowaltitudeelseifHei;
  /* '<S293>/if Height < Max low altitude  elseif Height > Min isotropic altitude ' */
  int8_T ifHeightMaxlowaltitudeelseifH_a;
  /* '<S292>/if Height < Max low altitude  elseif Height > Min isotropic altitude ' */
  boolean_T IC_FirstOutputTime;        /* '<S10>/IC' */
  boolean_T Integrator_DWORK1;         /* '<S10>/Integrator' */
  boolean_T PrevLimited;               /* '<Root>/Rate Limiter' */
  boolean_T Memory_PreviousInput_o;    /* '<S39>/Memory' */
  boolean_T Memory_PreviousInput_a;    /* '<S249>/Memory' */
  boolean_T objisempty;                /* '<S12>/SourceBlock' */
  boolean_T objisempty_g;              /* '<S11>/SourceBlock' */
  boolean_T objisempty_a;              /* '<S285>/SourceBlock' */
  boolean_T objisempty_e;              /* '<S284>/SourceBlock' */
  boolean_T objisempty_l;              /* '<S283>/SourceBlock' */
  boolean_T objisempty_c;              /* '<S282>/SourceBlock' */
  boolean_T objisempty_d;    /* '<Root>/Coordinate Transformation Conversion' */
  boolean_T objisempty_f;              /* '<S2>/ServiceCaller' */
  boolean_T Hwgws_MODE;                /* '<S288>/Hwgw(s)' */
  boolean_T Hvgws_MODE;                /* '<S288>/Hvgw(s)' */
  boolean_T Hugws_MODE;                /* '<S288>/Hugw(s)' */
  boolean_T Hrgw_MODE;                 /* '<S287>/Hrgw' */
  boolean_T Hqgw_MODE;                 /* '<S287>/Hqgw' */
  boolean_T Hpgw_MODE;                 /* '<S287>/Hpgw' */
};

/* Continuous states (default storage) */
struct X_pid_control_V1_T {
  real_T Integrator_CSTATE[12];        /* '<S10>/Integrator' */
  real_T Integrator_CSTATE_n;          /* '<S101>/Integrator' */
  real_T Filter_CSTATE;                /* '<S96>/Filter' */
  real_T Integrator_CSTATE_m;          /* '<S49>/Integrator' */
  real_T Filter_CSTATE_g;              /* '<S44>/Filter' */
  real_T Integrator_CSTATE_p;          /* '<S153>/Integrator' */
  real_T Filter_CSTATE_m;              /* '<S148>/Filter' */
  real_T Integrator_CSTATE_d;          /* '<S205>/Integrator' */
  real_T Filter_CSTATE_f;              /* '<S200>/Filter' */
  real_T Integrator_CSTATE_f;          /* '<S259>/Integrator' */
  real_T Filter_CSTATE_l;              /* '<S254>/Filter' */
  real_T Integrator1_CSTATE;           /* '<S10>/Integrator1' */
  real_T TransferFcn_CSTATE[2];        /* '<S10>/Transfer Fcn' */
  real_T TransferFcn1_CSTATE;          /* '<S10>/Transfer Fcn1' */
  real_T wg_p1_CSTATE[2];              /* '<S303>/wg_p1' */
  real_T wg_p2_CSTATE[2];              /* '<S303>/wg_p2' */
  real_T vg_p1_CSTATE[2];              /* '<S302>/vg_p1' */
  real_T vgw_p2_CSTATE[2];             /* '<S302>/vgw_p2' */
  real_T ug_p_CSTATE[2];               /* '<S301>/ug_p' */
  real_T rgw_p_CSTATE[2];              /* '<S300>/rgw_p' */
  real_T qgw_p_CSTATE[2];              /* '<S299>/qgw_p' */
  real_T pgw_p_CSTATE[2];              /* '<S298>/pgw_p' */
};

/* State derivatives (default storage) */
struct XDot_pid_control_V1_T {
  real_T Integrator_CSTATE[12];        /* '<S10>/Integrator' */
  real_T Integrator_CSTATE_n;          /* '<S101>/Integrator' */
  real_T Filter_CSTATE;                /* '<S96>/Filter' */
  real_T Integrator_CSTATE_m;          /* '<S49>/Integrator' */
  real_T Filter_CSTATE_g;              /* '<S44>/Filter' */
  real_T Integrator_CSTATE_p;          /* '<S153>/Integrator' */
  real_T Filter_CSTATE_m;              /* '<S148>/Filter' */
  real_T Integrator_CSTATE_d;          /* '<S205>/Integrator' */
  real_T Filter_CSTATE_f;              /* '<S200>/Filter' */
  real_T Integrator_CSTATE_f;          /* '<S259>/Integrator' */
  real_T Filter_CSTATE_l;              /* '<S254>/Filter' */
  real_T Integrator1_CSTATE;           /* '<S10>/Integrator1' */
  real_T TransferFcn_CSTATE[2];        /* '<S10>/Transfer Fcn' */
  real_T TransferFcn1_CSTATE;          /* '<S10>/Transfer Fcn1' */
  real_T wg_p1_CSTATE[2];              /* '<S303>/wg_p1' */
  real_T wg_p2_CSTATE[2];              /* '<S303>/wg_p2' */
  real_T vg_p1_CSTATE[2];              /* '<S302>/vg_p1' */
  real_T vgw_p2_CSTATE[2];             /* '<S302>/vgw_p2' */
  real_T ug_p_CSTATE[2];               /* '<S301>/ug_p' */
  real_T rgw_p_CSTATE[2];              /* '<S300>/rgw_p' */
  real_T qgw_p_CSTATE[2];              /* '<S299>/qgw_p' */
  real_T pgw_p_CSTATE[2];              /* '<S298>/pgw_p' */
};

/* State disabled  */
struct XDis_pid_control_V1_T {
  boolean_T Integrator_CSTATE[12];     /* '<S10>/Integrator' */
  boolean_T Integrator_CSTATE_n;       /* '<S101>/Integrator' */
  boolean_T Filter_CSTATE;             /* '<S96>/Filter' */
  boolean_T Integrator_CSTATE_m;       /* '<S49>/Integrator' */
  boolean_T Filter_CSTATE_g;           /* '<S44>/Filter' */
  boolean_T Integrator_CSTATE_p;       /* '<S153>/Integrator' */
  boolean_T Filter_CSTATE_m;           /* '<S148>/Filter' */
  boolean_T Integrator_CSTATE_d;       /* '<S205>/Integrator' */
  boolean_T Filter_CSTATE_f;           /* '<S200>/Filter' */
  boolean_T Integrator_CSTATE_f;       /* '<S259>/Integrator' */
  boolean_T Filter_CSTATE_l;           /* '<S254>/Filter' */
  boolean_T Integrator1_CSTATE;        /* '<S10>/Integrator1' */
  boolean_T TransferFcn_CSTATE[2];     /* '<S10>/Transfer Fcn' */
  boolean_T TransferFcn1_CSTATE;       /* '<S10>/Transfer Fcn1' */
  boolean_T wg_p1_CSTATE[2];           /* '<S303>/wg_p1' */
  boolean_T wg_p2_CSTATE[2];           /* '<S303>/wg_p2' */
  boolean_T vg_p1_CSTATE[2];           /* '<S302>/vg_p1' */
  boolean_T vgw_p2_CSTATE[2];          /* '<S302>/vgw_p2' */
  boolean_T ug_p_CSTATE[2];            /* '<S301>/ug_p' */
  boolean_T rgw_p_CSTATE[2];           /* '<S300>/rgw_p' */
  boolean_T qgw_p_CSTATE[2];           /* '<S299>/qgw_p' */
  boolean_T pgw_p_CSTATE[2];           /* '<S298>/pgw_p' */
};

/* Zero-crossing (trigger) state */
struct PrevZCX_pid_control_V1_T {
  ZCSigState Integrator_Reset_ZCE;     /* '<S10>/Integrator' */
};

/* Invariant block signals (default storage) */
struct ConstB_pid_control_V1_T {
  real_T UnitConversion;               /* '<S286>/Unit Conversion' */
  real_T UnitConversion_k;             /* '<S296>/Unit Conversion' */
  real_T sigma_wg;                     /* '<S305>/sigma_wg ' */
  real_T UnitConversion_n;             /* '<S290>/Unit Conversion' */
  real_T UnitConversion_c;             /* '<S324>/Unit Conversion' */
  real_T PreLookUpIndexSearchprobofe;
                        /* '<S304>/PreLook-Up Index Search  (prob of exceed)' */
  real_T Sqrt[4];                      /* '<S297>/Sqrt' */
  real_T Sqrt1;                        /* '<S297>/Sqrt1' */
  real_T Divide[4];                    /* '<S297>/Divide' */
  real_T motorspeed;                   /* '<S10>/Gain2' */
  real_T Sum;                          /* '<S314>/Sum' */
  real_T Sum_a;                        /* '<S306>/Sum' */
  real_T sqrt_a;                       /* '<S303>/sqrt' */
  real_T w4;                           /* '<S298>/w4' */
  real_T u16;                          /* '<S298>/u^1//6' */
  uint32_T PreLookUpIndexSearchprobo_g;
                        /* '<S304>/PreLook-Up Index Search  (prob of exceed)' */
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
  /* Pooled Parameter (Expression: x_nom)
   * Referenced by:
   *   '<S10>/IC'
   *   '<S10>/Memory2'
   */
  real_T pooled10[12];

  /* Expression: h_vec
   * Referenced by: '<S304>/PreLook-Up Index Search  (altitude)'
   */
  real_T PreLookUpIndexSearchaltitude_Br[12];

  /* Expression: sigma_vec'
   * Referenced by: '<S304>/Medium//High Altitude Intensity'
   */
  real_T MediumHighAltitudeIntensity_Tab[84];

  /* Computed Parameter: MediumHighAltitudeIntensity_max
   * Referenced by: '<S304>/Medium//High Altitude Intensity'
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

  /* private member function(s) for subsystem '<S282>/Enabled Subsystem'*/
  static void pid_contr_EnabledSubsystem_Init(B_EnabledSubsystem_pid_contro_T
    *localB);
  static void pid_control_V1_EnabledSubsystem(boolean_T rtu_Enable, const
    SL_Bus_std_msgs_Bool *rtu_In1, B_EnabledSubsystem_pid_contro_T *localB);

  /* private member function(s) for subsystem '<S283>/Enabled Subsystem'*/
  static void pid_con_EnabledSubsystem_i_Init(B_EnabledSubsystem_pid_cont_d_T
    *localB);
  static void pid_control__EnabledSubsystem_k(boolean_T rtu_Enable, const
    SL_Bus_std_msgs_Float64 *rtu_In1, B_EnabledSubsystem_pid_cont_d_T *localB);

  /* private member function(s) for subsystem '<Root>'*/
  void pid_c_Subscriber_setupImpl_onhg(const ros_slros2_internal_block_Sub_T
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
 * Block '<S145>/Kb' : Eliminated nontunable gain of 1
 * Block '<S279>/Cast' : Eliminate redundant data type conversion
 * Block '<S279>/Cast To Double' : Eliminate redundant data type conversion
 * Block '<S279>/Cast To Double1' : Eliminate redundant data type conversion
 * Block '<S279>/Cast To Double2' : Eliminate redundant data type conversion
 * Block '<S279>/Cast To Double3' : Eliminate redundant data type conversion
 * Block '<S279>/Cast To Double4' : Eliminate redundant data type conversion
 * Block '<S310>/Reshape' : Reshape block reduction
 * Block '<S310>/Reshape1' : Reshape block reduction
 * Block '<S312>/Reshape' : Reshape block reduction
 * Block '<S318>/Reshape' : Reshape block reduction
 * Block '<S318>/Reshape1' : Reshape block reduction
 * Block '<S320>/Reshape' : Reshape block reduction
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
 * '<S5>'   : 'pid_control_V1/PID ALERON'
 * '<S6>'   : 'pid_control_V1/PID ALTURA'
 * '<S7>'   : 'pid_control_V1/PID PITCH//ELEVATOR'
 * '<S8>'   : 'pid_control_V1/PID TIIMON'
 * '<S9>'   : 'pid_control_V1/PID VELOCIDAD'
 * '<S10>'  : 'pid_control_V1/SUBSYSTEM_MODEL'
 * '<S11>'  : 'pid_control_V1/Subscribe-ALTURA1'
 * '<S12>'  : 'pid_control_V1/Subscribe-YAW'
 * '<S13>'  : 'pid_control_V1/PID ALERON/Anti-windup'
 * '<S14>'  : 'pid_control_V1/PID ALERON/D Gain'
 * '<S15>'  : 'pid_control_V1/PID ALERON/External Derivative'
 * '<S16>'  : 'pid_control_V1/PID ALERON/Filter'
 * '<S17>'  : 'pid_control_V1/PID ALERON/Filter ICs'
 * '<S18>'  : 'pid_control_V1/PID ALERON/I Gain'
 * '<S19>'  : 'pid_control_V1/PID ALERON/Ideal P Gain'
 * '<S20>'  : 'pid_control_V1/PID ALERON/Ideal P Gain Fdbk'
 * '<S21>'  : 'pid_control_V1/PID ALERON/Integrator'
 * '<S22>'  : 'pid_control_V1/PID ALERON/Integrator ICs'
 * '<S23>'  : 'pid_control_V1/PID ALERON/N Copy'
 * '<S24>'  : 'pid_control_V1/PID ALERON/N Gain'
 * '<S25>'  : 'pid_control_V1/PID ALERON/P Copy'
 * '<S26>'  : 'pid_control_V1/PID ALERON/Parallel P Gain'
 * '<S27>'  : 'pid_control_V1/PID ALERON/Reset Signal'
 * '<S28>'  : 'pid_control_V1/PID ALERON/Saturation'
 * '<S29>'  : 'pid_control_V1/PID ALERON/Saturation Fdbk'
 * '<S30>'  : 'pid_control_V1/PID ALERON/Sum'
 * '<S31>'  : 'pid_control_V1/PID ALERON/Sum Fdbk'
 * '<S32>'  : 'pid_control_V1/PID ALERON/Tracking Mode'
 * '<S33>'  : 'pid_control_V1/PID ALERON/Tracking Mode Sum'
 * '<S34>'  : 'pid_control_V1/PID ALERON/Tsamp - Integral'
 * '<S35>'  : 'pid_control_V1/PID ALERON/Tsamp - Ngain'
 * '<S36>'  : 'pid_control_V1/PID ALERON/postSat Signal'
 * '<S37>'  : 'pid_control_V1/PID ALERON/preInt Signal'
 * '<S38>'  : 'pid_control_V1/PID ALERON/preSat Signal'
 * '<S39>'  : 'pid_control_V1/PID ALERON/Anti-windup/Cont. Clamping Parallel'
 * '<S40>'  : 'pid_control_V1/PID ALERON/Anti-windup/Cont. Clamping Parallel/Dead Zone'
 * '<S41>'  : 'pid_control_V1/PID ALERON/Anti-windup/Cont. Clamping Parallel/Dead Zone/Enabled'
 * '<S42>'  : 'pid_control_V1/PID ALERON/D Gain/Internal Parameters'
 * '<S43>'  : 'pid_control_V1/PID ALERON/External Derivative/Error'
 * '<S44>'  : 'pid_control_V1/PID ALERON/Filter/Cont. Filter'
 * '<S45>'  : 'pid_control_V1/PID ALERON/Filter ICs/Internal IC - Filter'
 * '<S46>'  : 'pid_control_V1/PID ALERON/I Gain/Internal Parameters'
 * '<S47>'  : 'pid_control_V1/PID ALERON/Ideal P Gain/Passthrough'
 * '<S48>'  : 'pid_control_V1/PID ALERON/Ideal P Gain Fdbk/Disabled'
 * '<S49>'  : 'pid_control_V1/PID ALERON/Integrator/Continuous'
 * '<S50>'  : 'pid_control_V1/PID ALERON/Integrator ICs/Internal IC'
 * '<S51>'  : 'pid_control_V1/PID ALERON/N Copy/Disabled'
 * '<S52>'  : 'pid_control_V1/PID ALERON/N Gain/Internal Parameters'
 * '<S53>'  : 'pid_control_V1/PID ALERON/P Copy/Disabled'
 * '<S54>'  : 'pid_control_V1/PID ALERON/Parallel P Gain/Internal Parameters'
 * '<S55>'  : 'pid_control_V1/PID ALERON/Reset Signal/Disabled'
 * '<S56>'  : 'pid_control_V1/PID ALERON/Saturation/Enabled'
 * '<S57>'  : 'pid_control_V1/PID ALERON/Saturation Fdbk/Disabled'
 * '<S58>'  : 'pid_control_V1/PID ALERON/Sum/Sum_PID'
 * '<S59>'  : 'pid_control_V1/PID ALERON/Sum Fdbk/Disabled'
 * '<S60>'  : 'pid_control_V1/PID ALERON/Tracking Mode/Disabled'
 * '<S61>'  : 'pid_control_V1/PID ALERON/Tracking Mode Sum/Passthrough'
 * '<S62>'  : 'pid_control_V1/PID ALERON/Tsamp - Integral/TsSignalSpecification'
 * '<S63>'  : 'pid_control_V1/PID ALERON/Tsamp - Ngain/Passthrough'
 * '<S64>'  : 'pid_control_V1/PID ALERON/postSat Signal/Forward_Path'
 * '<S65>'  : 'pid_control_V1/PID ALERON/preInt Signal/Internal PreInt'
 * '<S66>'  : 'pid_control_V1/PID ALERON/preSat Signal/Forward_Path'
 * '<S67>'  : 'pid_control_V1/PID ALTURA/Anti-windup'
 * '<S68>'  : 'pid_control_V1/PID ALTURA/D Gain'
 * '<S69>'  : 'pid_control_V1/PID ALTURA/External Derivative'
 * '<S70>'  : 'pid_control_V1/PID ALTURA/Filter'
 * '<S71>'  : 'pid_control_V1/PID ALTURA/Filter ICs'
 * '<S72>'  : 'pid_control_V1/PID ALTURA/I Gain'
 * '<S73>'  : 'pid_control_V1/PID ALTURA/Ideal P Gain'
 * '<S74>'  : 'pid_control_V1/PID ALTURA/Ideal P Gain Fdbk'
 * '<S75>'  : 'pid_control_V1/PID ALTURA/Integrator'
 * '<S76>'  : 'pid_control_V1/PID ALTURA/Integrator ICs'
 * '<S77>'  : 'pid_control_V1/PID ALTURA/N Copy'
 * '<S78>'  : 'pid_control_V1/PID ALTURA/N Gain'
 * '<S79>'  : 'pid_control_V1/PID ALTURA/P Copy'
 * '<S80>'  : 'pid_control_V1/PID ALTURA/Parallel P Gain'
 * '<S81>'  : 'pid_control_V1/PID ALTURA/Reset Signal'
 * '<S82>'  : 'pid_control_V1/PID ALTURA/Saturation'
 * '<S83>'  : 'pid_control_V1/PID ALTURA/Saturation Fdbk'
 * '<S84>'  : 'pid_control_V1/PID ALTURA/Sum'
 * '<S85>'  : 'pid_control_V1/PID ALTURA/Sum Fdbk'
 * '<S86>'  : 'pid_control_V1/PID ALTURA/Tracking Mode'
 * '<S87>'  : 'pid_control_V1/PID ALTURA/Tracking Mode Sum'
 * '<S88>'  : 'pid_control_V1/PID ALTURA/Tsamp - Integral'
 * '<S89>'  : 'pid_control_V1/PID ALTURA/Tsamp - Ngain'
 * '<S90>'  : 'pid_control_V1/PID ALTURA/postSat Signal'
 * '<S91>'  : 'pid_control_V1/PID ALTURA/preInt Signal'
 * '<S92>'  : 'pid_control_V1/PID ALTURA/preSat Signal'
 * '<S93>'  : 'pid_control_V1/PID ALTURA/Anti-windup/Back Calculation'
 * '<S94>'  : 'pid_control_V1/PID ALTURA/D Gain/Internal Parameters'
 * '<S95>'  : 'pid_control_V1/PID ALTURA/External Derivative/Error'
 * '<S96>'  : 'pid_control_V1/PID ALTURA/Filter/Cont. Filter'
 * '<S97>'  : 'pid_control_V1/PID ALTURA/Filter ICs/Internal IC - Filter'
 * '<S98>'  : 'pid_control_V1/PID ALTURA/I Gain/Internal Parameters'
 * '<S99>'  : 'pid_control_V1/PID ALTURA/Ideal P Gain/Passthrough'
 * '<S100>' : 'pid_control_V1/PID ALTURA/Ideal P Gain Fdbk/Disabled'
 * '<S101>' : 'pid_control_V1/PID ALTURA/Integrator/Continuous'
 * '<S102>' : 'pid_control_V1/PID ALTURA/Integrator ICs/Internal IC'
 * '<S103>' : 'pid_control_V1/PID ALTURA/N Copy/Disabled'
 * '<S104>' : 'pid_control_V1/PID ALTURA/N Gain/Internal Parameters'
 * '<S105>' : 'pid_control_V1/PID ALTURA/P Copy/Disabled'
 * '<S106>' : 'pid_control_V1/PID ALTURA/Parallel P Gain/Internal Parameters'
 * '<S107>' : 'pid_control_V1/PID ALTURA/Reset Signal/Disabled'
 * '<S108>' : 'pid_control_V1/PID ALTURA/Saturation/Enabled'
 * '<S109>' : 'pid_control_V1/PID ALTURA/Saturation Fdbk/Disabled'
 * '<S110>' : 'pid_control_V1/PID ALTURA/Sum/Sum_PID'
 * '<S111>' : 'pid_control_V1/PID ALTURA/Sum Fdbk/Disabled'
 * '<S112>' : 'pid_control_V1/PID ALTURA/Tracking Mode/Disabled'
 * '<S113>' : 'pid_control_V1/PID ALTURA/Tracking Mode Sum/Passthrough'
 * '<S114>' : 'pid_control_V1/PID ALTURA/Tsamp - Integral/TsSignalSpecification'
 * '<S115>' : 'pid_control_V1/PID ALTURA/Tsamp - Ngain/Passthrough'
 * '<S116>' : 'pid_control_V1/PID ALTURA/postSat Signal/Forward_Path'
 * '<S117>' : 'pid_control_V1/PID ALTURA/preInt Signal/Internal PreInt'
 * '<S118>' : 'pid_control_V1/PID ALTURA/preSat Signal/Forward_Path'
 * '<S119>' : 'pid_control_V1/PID PITCH//ELEVATOR/Anti-windup'
 * '<S120>' : 'pid_control_V1/PID PITCH//ELEVATOR/D Gain'
 * '<S121>' : 'pid_control_V1/PID PITCH//ELEVATOR/External Derivative'
 * '<S122>' : 'pid_control_V1/PID PITCH//ELEVATOR/Filter'
 * '<S123>' : 'pid_control_V1/PID PITCH//ELEVATOR/Filter ICs'
 * '<S124>' : 'pid_control_V1/PID PITCH//ELEVATOR/I Gain'
 * '<S125>' : 'pid_control_V1/PID PITCH//ELEVATOR/Ideal P Gain'
 * '<S126>' : 'pid_control_V1/PID PITCH//ELEVATOR/Ideal P Gain Fdbk'
 * '<S127>' : 'pid_control_V1/PID PITCH//ELEVATOR/Integrator'
 * '<S128>' : 'pid_control_V1/PID PITCH//ELEVATOR/Integrator ICs'
 * '<S129>' : 'pid_control_V1/PID PITCH//ELEVATOR/N Copy'
 * '<S130>' : 'pid_control_V1/PID PITCH//ELEVATOR/N Gain'
 * '<S131>' : 'pid_control_V1/PID PITCH//ELEVATOR/P Copy'
 * '<S132>' : 'pid_control_V1/PID PITCH//ELEVATOR/Parallel P Gain'
 * '<S133>' : 'pid_control_V1/PID PITCH//ELEVATOR/Reset Signal'
 * '<S134>' : 'pid_control_V1/PID PITCH//ELEVATOR/Saturation'
 * '<S135>' : 'pid_control_V1/PID PITCH//ELEVATOR/Saturation Fdbk'
 * '<S136>' : 'pid_control_V1/PID PITCH//ELEVATOR/Sum'
 * '<S137>' : 'pid_control_V1/PID PITCH//ELEVATOR/Sum Fdbk'
 * '<S138>' : 'pid_control_V1/PID PITCH//ELEVATOR/Tracking Mode'
 * '<S139>' : 'pid_control_V1/PID PITCH//ELEVATOR/Tracking Mode Sum'
 * '<S140>' : 'pid_control_V1/PID PITCH//ELEVATOR/Tsamp - Integral'
 * '<S141>' : 'pid_control_V1/PID PITCH//ELEVATOR/Tsamp - Ngain'
 * '<S142>' : 'pid_control_V1/PID PITCH//ELEVATOR/postSat Signal'
 * '<S143>' : 'pid_control_V1/PID PITCH//ELEVATOR/preInt Signal'
 * '<S144>' : 'pid_control_V1/PID PITCH//ELEVATOR/preSat Signal'
 * '<S145>' : 'pid_control_V1/PID PITCH//ELEVATOR/Anti-windup/Back Calculation'
 * '<S146>' : 'pid_control_V1/PID PITCH//ELEVATOR/D Gain/Internal Parameters'
 * '<S147>' : 'pid_control_V1/PID PITCH//ELEVATOR/External Derivative/Error'
 * '<S148>' : 'pid_control_V1/PID PITCH//ELEVATOR/Filter/Cont. Filter'
 * '<S149>' : 'pid_control_V1/PID PITCH//ELEVATOR/Filter ICs/Internal IC - Filter'
 * '<S150>' : 'pid_control_V1/PID PITCH//ELEVATOR/I Gain/Internal Parameters'
 * '<S151>' : 'pid_control_V1/PID PITCH//ELEVATOR/Ideal P Gain/Passthrough'
 * '<S152>' : 'pid_control_V1/PID PITCH//ELEVATOR/Ideal P Gain Fdbk/Disabled'
 * '<S153>' : 'pid_control_V1/PID PITCH//ELEVATOR/Integrator/Continuous'
 * '<S154>' : 'pid_control_V1/PID PITCH//ELEVATOR/Integrator ICs/Internal IC'
 * '<S155>' : 'pid_control_V1/PID PITCH//ELEVATOR/N Copy/Disabled'
 * '<S156>' : 'pid_control_V1/PID PITCH//ELEVATOR/N Gain/Internal Parameters'
 * '<S157>' : 'pid_control_V1/PID PITCH//ELEVATOR/P Copy/Disabled'
 * '<S158>' : 'pid_control_V1/PID PITCH//ELEVATOR/Parallel P Gain/Internal Parameters'
 * '<S159>' : 'pid_control_V1/PID PITCH//ELEVATOR/Reset Signal/Disabled'
 * '<S160>' : 'pid_control_V1/PID PITCH//ELEVATOR/Saturation/Enabled'
 * '<S161>' : 'pid_control_V1/PID PITCH//ELEVATOR/Saturation Fdbk/Disabled'
 * '<S162>' : 'pid_control_V1/PID PITCH//ELEVATOR/Sum/Sum_PID'
 * '<S163>' : 'pid_control_V1/PID PITCH//ELEVATOR/Sum Fdbk/Disabled'
 * '<S164>' : 'pid_control_V1/PID PITCH//ELEVATOR/Tracking Mode/Disabled'
 * '<S165>' : 'pid_control_V1/PID PITCH//ELEVATOR/Tracking Mode Sum/Passthrough'
 * '<S166>' : 'pid_control_V1/PID PITCH//ELEVATOR/Tsamp - Integral/TsSignalSpecification'
 * '<S167>' : 'pid_control_V1/PID PITCH//ELEVATOR/Tsamp - Ngain/Passthrough'
 * '<S168>' : 'pid_control_V1/PID PITCH//ELEVATOR/postSat Signal/Forward_Path'
 * '<S169>' : 'pid_control_V1/PID PITCH//ELEVATOR/preInt Signal/Internal PreInt'
 * '<S170>' : 'pid_control_V1/PID PITCH//ELEVATOR/preSat Signal/Forward_Path'
 * '<S171>' : 'pid_control_V1/PID TIIMON/Anti-windup'
 * '<S172>' : 'pid_control_V1/PID TIIMON/D Gain'
 * '<S173>' : 'pid_control_V1/PID TIIMON/External Derivative'
 * '<S174>' : 'pid_control_V1/PID TIIMON/Filter'
 * '<S175>' : 'pid_control_V1/PID TIIMON/Filter ICs'
 * '<S176>' : 'pid_control_V1/PID TIIMON/I Gain'
 * '<S177>' : 'pid_control_V1/PID TIIMON/Ideal P Gain'
 * '<S178>' : 'pid_control_V1/PID TIIMON/Ideal P Gain Fdbk'
 * '<S179>' : 'pid_control_V1/PID TIIMON/Integrator'
 * '<S180>' : 'pid_control_V1/PID TIIMON/Integrator ICs'
 * '<S181>' : 'pid_control_V1/PID TIIMON/N Copy'
 * '<S182>' : 'pid_control_V1/PID TIIMON/N Gain'
 * '<S183>' : 'pid_control_V1/PID TIIMON/P Copy'
 * '<S184>' : 'pid_control_V1/PID TIIMON/Parallel P Gain'
 * '<S185>' : 'pid_control_V1/PID TIIMON/Reset Signal'
 * '<S186>' : 'pid_control_V1/PID TIIMON/Saturation'
 * '<S187>' : 'pid_control_V1/PID TIIMON/Saturation Fdbk'
 * '<S188>' : 'pid_control_V1/PID TIIMON/Sum'
 * '<S189>' : 'pid_control_V1/PID TIIMON/Sum Fdbk'
 * '<S190>' : 'pid_control_V1/PID TIIMON/Tracking Mode'
 * '<S191>' : 'pid_control_V1/PID TIIMON/Tracking Mode Sum'
 * '<S192>' : 'pid_control_V1/PID TIIMON/Tsamp - Integral'
 * '<S193>' : 'pid_control_V1/PID TIIMON/Tsamp - Ngain'
 * '<S194>' : 'pid_control_V1/PID TIIMON/postSat Signal'
 * '<S195>' : 'pid_control_V1/PID TIIMON/preInt Signal'
 * '<S196>' : 'pid_control_V1/PID TIIMON/preSat Signal'
 * '<S197>' : 'pid_control_V1/PID TIIMON/Anti-windup/Passthrough'
 * '<S198>' : 'pid_control_V1/PID TIIMON/D Gain/Internal Parameters'
 * '<S199>' : 'pid_control_V1/PID TIIMON/External Derivative/Error'
 * '<S200>' : 'pid_control_V1/PID TIIMON/Filter/Cont. Filter'
 * '<S201>' : 'pid_control_V1/PID TIIMON/Filter ICs/Internal IC - Filter'
 * '<S202>' : 'pid_control_V1/PID TIIMON/I Gain/Internal Parameters'
 * '<S203>' : 'pid_control_V1/PID TIIMON/Ideal P Gain/Passthrough'
 * '<S204>' : 'pid_control_V1/PID TIIMON/Ideal P Gain Fdbk/Disabled'
 * '<S205>' : 'pid_control_V1/PID TIIMON/Integrator/Continuous'
 * '<S206>' : 'pid_control_V1/PID TIIMON/Integrator ICs/Internal IC'
 * '<S207>' : 'pid_control_V1/PID TIIMON/N Copy/Disabled'
 * '<S208>' : 'pid_control_V1/PID TIIMON/N Gain/Internal Parameters'
 * '<S209>' : 'pid_control_V1/PID TIIMON/P Copy/Disabled'
 * '<S210>' : 'pid_control_V1/PID TIIMON/Parallel P Gain/Internal Parameters'
 * '<S211>' : 'pid_control_V1/PID TIIMON/Reset Signal/Disabled'
 * '<S212>' : 'pid_control_V1/PID TIIMON/Saturation/Enabled'
 * '<S213>' : 'pid_control_V1/PID TIIMON/Saturation Fdbk/Disabled'
 * '<S214>' : 'pid_control_V1/PID TIIMON/Sum/Sum_PID'
 * '<S215>' : 'pid_control_V1/PID TIIMON/Sum Fdbk/Disabled'
 * '<S216>' : 'pid_control_V1/PID TIIMON/Tracking Mode/Disabled'
 * '<S217>' : 'pid_control_V1/PID TIIMON/Tracking Mode Sum/Passthrough'
 * '<S218>' : 'pid_control_V1/PID TIIMON/Tsamp - Integral/TsSignalSpecification'
 * '<S219>' : 'pid_control_V1/PID TIIMON/Tsamp - Ngain/Passthrough'
 * '<S220>' : 'pid_control_V1/PID TIIMON/postSat Signal/Forward_Path'
 * '<S221>' : 'pid_control_V1/PID TIIMON/preInt Signal/Internal PreInt'
 * '<S222>' : 'pid_control_V1/PID TIIMON/preSat Signal/Forward_Path'
 * '<S223>' : 'pid_control_V1/PID VELOCIDAD/Anti-windup'
 * '<S224>' : 'pid_control_V1/PID VELOCIDAD/D Gain'
 * '<S225>' : 'pid_control_V1/PID VELOCIDAD/External Derivative'
 * '<S226>' : 'pid_control_V1/PID VELOCIDAD/Filter'
 * '<S227>' : 'pid_control_V1/PID VELOCIDAD/Filter ICs'
 * '<S228>' : 'pid_control_V1/PID VELOCIDAD/I Gain'
 * '<S229>' : 'pid_control_V1/PID VELOCIDAD/Ideal P Gain'
 * '<S230>' : 'pid_control_V1/PID VELOCIDAD/Ideal P Gain Fdbk'
 * '<S231>' : 'pid_control_V1/PID VELOCIDAD/Integrator'
 * '<S232>' : 'pid_control_V1/PID VELOCIDAD/Integrator ICs'
 * '<S233>' : 'pid_control_V1/PID VELOCIDAD/N Copy'
 * '<S234>' : 'pid_control_V1/PID VELOCIDAD/N Gain'
 * '<S235>' : 'pid_control_V1/PID VELOCIDAD/P Copy'
 * '<S236>' : 'pid_control_V1/PID VELOCIDAD/Parallel P Gain'
 * '<S237>' : 'pid_control_V1/PID VELOCIDAD/Reset Signal'
 * '<S238>' : 'pid_control_V1/PID VELOCIDAD/Saturation'
 * '<S239>' : 'pid_control_V1/PID VELOCIDAD/Saturation Fdbk'
 * '<S240>' : 'pid_control_V1/PID VELOCIDAD/Sum'
 * '<S241>' : 'pid_control_V1/PID VELOCIDAD/Sum Fdbk'
 * '<S242>' : 'pid_control_V1/PID VELOCIDAD/Tracking Mode'
 * '<S243>' : 'pid_control_V1/PID VELOCIDAD/Tracking Mode Sum'
 * '<S244>' : 'pid_control_V1/PID VELOCIDAD/Tsamp - Integral'
 * '<S245>' : 'pid_control_V1/PID VELOCIDAD/Tsamp - Ngain'
 * '<S246>' : 'pid_control_V1/PID VELOCIDAD/postSat Signal'
 * '<S247>' : 'pid_control_V1/PID VELOCIDAD/preInt Signal'
 * '<S248>' : 'pid_control_V1/PID VELOCIDAD/preSat Signal'
 * '<S249>' : 'pid_control_V1/PID VELOCIDAD/Anti-windup/Cont. Clamping Parallel'
 * '<S250>' : 'pid_control_V1/PID VELOCIDAD/Anti-windup/Cont. Clamping Parallel/Dead Zone'
 * '<S251>' : 'pid_control_V1/PID VELOCIDAD/Anti-windup/Cont. Clamping Parallel/Dead Zone/Enabled'
 * '<S252>' : 'pid_control_V1/PID VELOCIDAD/D Gain/Internal Parameters'
 * '<S253>' : 'pid_control_V1/PID VELOCIDAD/External Derivative/Error'
 * '<S254>' : 'pid_control_V1/PID VELOCIDAD/Filter/Cont. Filter'
 * '<S255>' : 'pid_control_V1/PID VELOCIDAD/Filter ICs/Internal IC - Filter'
 * '<S256>' : 'pid_control_V1/PID VELOCIDAD/I Gain/Internal Parameters'
 * '<S257>' : 'pid_control_V1/PID VELOCIDAD/Ideal P Gain/Passthrough'
 * '<S258>' : 'pid_control_V1/PID VELOCIDAD/Ideal P Gain Fdbk/Disabled'
 * '<S259>' : 'pid_control_V1/PID VELOCIDAD/Integrator/Continuous'
 * '<S260>' : 'pid_control_V1/PID VELOCIDAD/Integrator ICs/Internal IC'
 * '<S261>' : 'pid_control_V1/PID VELOCIDAD/N Copy/Disabled'
 * '<S262>' : 'pid_control_V1/PID VELOCIDAD/N Gain/Internal Parameters'
 * '<S263>' : 'pid_control_V1/PID VELOCIDAD/P Copy/Disabled'
 * '<S264>' : 'pid_control_V1/PID VELOCIDAD/Parallel P Gain/Internal Parameters'
 * '<S265>' : 'pid_control_V1/PID VELOCIDAD/Reset Signal/Disabled'
 * '<S266>' : 'pid_control_V1/PID VELOCIDAD/Saturation/Enabled'
 * '<S267>' : 'pid_control_V1/PID VELOCIDAD/Saturation Fdbk/Disabled'
 * '<S268>' : 'pid_control_V1/PID VELOCIDAD/Sum/Sum_PID'
 * '<S269>' : 'pid_control_V1/PID VELOCIDAD/Sum Fdbk/Disabled'
 * '<S270>' : 'pid_control_V1/PID VELOCIDAD/Tracking Mode/Disabled'
 * '<S271>' : 'pid_control_V1/PID VELOCIDAD/Tracking Mode Sum/Passthrough'
 * '<S272>' : 'pid_control_V1/PID VELOCIDAD/Tsamp - Integral/TsSignalSpecification'
 * '<S273>' : 'pid_control_V1/PID VELOCIDAD/Tsamp - Ngain/Passthrough'
 * '<S274>' : 'pid_control_V1/PID VELOCIDAD/postSat Signal/Forward_Path'
 * '<S275>' : 'pid_control_V1/PID VELOCIDAD/preInt Signal/Internal PreInt'
 * '<S276>' : 'pid_control_V1/PID VELOCIDAD/preSat Signal/Forward_Path'
 * '<S277>' : 'pid_control_V1/SUBSYSTEM_MODEL/Band-Limited White Noise'
 * '<S278>' : 'pid_control_V1/SUBSYSTEM_MODEL/Compare To Constant'
 * '<S279>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))'
 * '<S280>' : 'pid_control_V1/SUBSYSTEM_MODEL/MATLAB Function - MODEL'
 * '<S281>' : 'pid_control_V1/SUBSYSTEM_MODEL/MATLAB Function1'
 * '<S282>' : 'pid_control_V1/SUBSYSTEM_MODEL/Subscribe1_TURBULENCIA1'
 * '<S283>' : 'pid_control_V1/SUBSYSTEM_MODEL/Subscribe_HEAVE'
 * '<S284>' : 'pid_control_V1/SUBSYSTEM_MODEL/Subscribe_RATE'
 * '<S285>' : 'pid_control_V1/SUBSYSTEM_MODEL/Subscribe_TURBULENCIA'
 * '<S286>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Angle Conversion'
 * '<S287>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Filters on angular rates'
 * '<S288>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Filters on velocities'
 * '<S289>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Length Conversion'
 * '<S290>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Length Conversion1'
 * '<S291>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/RMS turbulence  intensities'
 * '<S292>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Select angular rates'
 * '<S293>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Select velocities'
 * '<S294>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Turbulence scale lengths'
 * '<S295>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Velocity Conversion'
 * '<S296>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Velocity Conversion2'
 * '<S297>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/White Noise'
 * '<S298>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Filters on angular rates/Hpgw'
 * '<S299>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Filters on angular rates/Hqgw'
 * '<S300>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Filters on angular rates/Hrgw'
 * '<S301>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Filters on velocities/Hugw(s)'
 * '<S302>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Filters on velocities/Hvgw(s)'
 * '<S303>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Filters on velocities/Hwgw(s)'
 * '<S304>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/RMS turbulence  intensities/High Altitude Intensity'
 * '<S305>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/RMS turbulence  intensities/Low Altitude Intensity'
 * '<S306>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Select angular rates/Interpolate  rates'
 * '<S307>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Select angular rates/Low altitude  rates'
 * '<S308>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Select angular rates/Medium//High  altitude rates'
 * '<S309>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Select angular rates/Merge Subsystems'
 * '<S310>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Select angular rates/Interpolate  rates/wind to body transformation'
 * '<S311>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Select angular rates/Interpolate  rates/wind to body transformation/convert to earth coords'
 * '<S312>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Select angular rates/Low altitude  rates/wind to body transformation'
 * '<S313>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Select angular rates/Low altitude  rates/wind to body transformation/convert to earth coords'
 * '<S314>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Select velocities/Interpolate  velocities'
 * '<S315>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Select velocities/Low altitude  velocities'
 * '<S316>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Select velocities/Medium//High  altitude velocities'
 * '<S317>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Select velocities/Merge Subsystems'
 * '<S318>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Select velocities/Interpolate  velocities/wind to body transformation'
 * '<S319>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Select velocities/Interpolate  velocities/wind to body transformation/convert to earth coords'
 * '<S320>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Select velocities/Low altitude  velocities/wind to body transformation'
 * '<S321>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Select velocities/Low altitude  velocities/wind to body transformation/convert to earth coords'
 * '<S322>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Turbulence scale lengths/Low altitude scale length'
 * '<S323>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Turbulence scale lengths/Medium//High altitude scale length'
 * '<S324>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Turbulence scale lengths/Medium//High altitude scale length/Length Conversion'
 * '<S325>' : 'pid_control_V1/SUBSYSTEM_MODEL/Subscribe1_TURBULENCIA1/Enabled Subsystem'
 * '<S326>' : 'pid_control_V1/SUBSYSTEM_MODEL/Subscribe_HEAVE/Enabled Subsystem'
 * '<S327>' : 'pid_control_V1/SUBSYSTEM_MODEL/Subscribe_RATE/Enabled Subsystem'
 * '<S328>' : 'pid_control_V1/SUBSYSTEM_MODEL/Subscribe_TURBULENCIA/Enabled Subsystem'
 * '<S329>' : 'pid_control_V1/Subscribe-ALTURA1/Enabled Subsystem'
 * '<S330>' : 'pid_control_V1/Subscribe-YAW/Enabled Subsystem'
 */
#endif                                 /* pid_control_V1_h_ */
