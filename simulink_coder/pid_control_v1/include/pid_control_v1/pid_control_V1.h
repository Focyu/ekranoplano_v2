/*
 * pid_control_V1.h
 *
 * Trial License - for use to evaluate programs for possible purchase as
 * an end-user only.
 *
 * Code generation for model "pid_control_V1".
 *
 * Model version              : 12.131
 * Simulink Coder version : 25.2 (R2025b) 28-Jul-2025
 * C++ source code generated on : Tue Apr 14 23:25:41 2026
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

/* Block signals for system '<S283>/Enabled Subsystem' */
struct B_EnabledSubsystem_pid_contro_T {
  SL_Bus_std_msgs_Bool In1;            /* '<S326>/In1' */
};

/* Block signals for system '<S284>/Enabled Subsystem' */
struct B_EnabledSubsystem_pid_cont_d_T {
  SL_Bus_std_msgs_Float64 In1;         /* '<S327>/In1' */
};

/* Block signals (default storage) */
struct B_pid_control_V1_T {
  SL_Bus_gazebo_msgs_SetEntityStateRequest BusAssignment;/* '<Root>/Bus Assignment' */
  real_T IC[12];                       /* '<S10>/IC' */
  real_T x[12];                        /* '<S10>/Integrator' */
  real_T R[9];
  real_T RotationAnglestoDirectionCo[9];
                        /* '<S10>/Rotation Angles to Direction Cosine Matrix' */
  real_T dv[9];
  real_T TmpSignalConversionAtSFunct[5];/* '<S10>/MATLAB Function - MODEL' */
  char_T b_zeroDelimTopic[25];
  real_T wbe_b[3];
  real_T FE1_b[3];
  real_T F_b[3];
  real_T Product_m[3];                 /* '<S319>/Product' */
  real_T Dtot[3];
  char_T b_zeroDelimTopic_c[22];
  char_T b_zeroDelimTopic_k[22];
  char_T b_zeroDelimTopic_cx[17];
  char_T b_zeroDelimTopic_b[17];
  sJ4ih70VmKcvCeguWN0mNVF deadline;
  sJ4ih70VmKcvCeguWN0mNVF deadline_p;
  sJ4ih70VmKcvCeguWN0mNVF deadline_c;
  real_T LwgV1[2];                     /* '<S304>/Lwg//V 1' */
  real_T w_g[2];                       /* '<S303>/w' */
  real_T w_e[2];                       /* '<S303>/w ' */
  real_T w1[2];                        /* '<S303>/w 1' */
  real_T w_n[2];                       /* '<S302>/w' */
  real_T w1_c[2];                      /* '<S302>/w1' */
  real_T w_d[2];                       /* '<S301>/w' */
  real_T w_e0[2];                      /* '<S300>/w' */
  real_T UnaryMinus[2];                /* '<S300>/Unary Minus' */
  real_T w_o[2];                       /* '<S299>/w' */
  real_T sigma_w[2];                   /* '<S299>/sigma_w' */
  uint8_T stringOut_l[128];            /* '<Root>/MATLAB Function' */
  real_T frac[2];
  real_T dv1[2];
  real_T Switch3;                      /* '<Root>/Switch3' */
  real_T Gain;                         /* '<Root>/Gain' */
  real_T FilterCoefficient;            /* '<S105>/Filter Coefficient' */
  real_T Saturation;                   /* '<S109>/Saturation' */
  real_T Switch1;                      /* '<Root>/Switch1' */
  real_T FilterCoefficient_c;          /* '<S53>/Filter Coefficient' */
  real_T Saturation_k;                 /* '<S57>/Saturation' */
  real_T Saturation_i;                 /* '<Root>/Saturation' */
  real_T RateLimiter;                  /* '<Root>/Rate Limiter' */
  real_T FilterCoefficient_m;          /* '<S157>/Filter Coefficient' */
  real_T Saturation_f;                 /* '<S161>/Saturation' */
  real_T Switch2;                      /* '<Root>/Switch2' */
  real_T FilterCoefficient_p;          /* '<S209>/Filter Coefficient' */
  real_T Saturation_m;                 /* '<S213>/Saturation' */
  real_T FilterCoefficient_cv;         /* '<S263>/Filter Coefficient' */
  real_T Saturation_o;                 /* '<S267>/Saturation' */
  real_T Memory[3];                    /* '<S10>/Memory' */
  real_T Memory1[3];                   /* '<S10>/Memory1' */
  real_T Switch;                       /* '<S40>/Switch' */
  real_T SumI4;                        /* '<S94>/SumI4' */
  real_T SumI4_i;                      /* '<S146>/SumI4' */
  real_T IntegralGain;                 /* '<S203>/Integral Gain' */
  real_T Switch_j;                     /* '<S250>/Switch' */
  real_T Product[4];                   /* '<S298>/Product' */
  real_T Switch_p[3];                  /* '<S10>/Switch' */
  real_T Switch1_p[3];                 /* '<S10>/Switch1' */
  real_T data;
  real_T data_n;
  real_T Power;                        /* '<S10>/Product2' */
  real_T Gain3;                        /* '<S10>/Gain3' */
  real_T EnergykWh;                    /* '<S10>/Gain1' */
  real_T powerdemand;                  /* '<S10>/Divide' */
  real_T loadtorque;                   /* '<S10>/Divide1' */
  real_T Output;                       /* '<S278>/Output' */
  real_T Sum[3];                       /* '<S10>/Sum' */
  real_T Sum1[3];                      /* '<S10>/Sum1' */
  real_T XDOT[40];                     /* '<S10>/MATLAB Function - MODEL' */
  real_T CL_total;                     /* '<S10>/MATLAB Function - MODEL' */
  real_T mu_Lw_out;                    /* '<S10>/MATLAB Function - MODEL' */
  real_T mu_Dw_out;                    /* '<S10>/MATLAB Function - MODEL' */
  real_T w[2];                         /* '<S304>/w' */
  real_T w_a[2];                       /* '<S304>/w ' */
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
  real_T Dtot_f;
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
  real_T Sum_b;                        /* '<S111>/Sum' */
  real_T SignPreSat;                   /* '<S40>/SignPreSat' */
  real_T Sum1_g;                       /* '<Root>/Sum1' */
  real_T Sum_hl;                       /* '<S163>/Sum' */
  real_T Sum5;                         /* '<Root>/Sum5' */
  real_T SignPreSat_h;                 /* '<S250>/SignPreSat' */
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
  SL_Bus_std_msgs_Float64 SourceBlock_o2_k;/* '<S285>/SourceBlock' */
  SL_Bus_std_msgs_Float64 SourceBlock_o2_p;/* '<S284>/SourceBlock' */
  SL_Bus_std_msgs_Float64 SourceBlock_o2;/* '<S13>/SourceBlock' */
  uint32_T bpIndex[2];
  uint32_T lengthOut;                  /* '<Root>/MATLAB Function1' */
  uint32_T lengthOut_e;                /* '<Root>/MATLAB Function' */
  uint8_T stringOut[128];              /* '<Root>/MATLAB Function1' */
  boolean_T Compare;                   /* '<S279>/Compare' */
  boolean_T AND3;                      /* '<S40>/AND3' */
  boolean_T Memory_a;                  /* '<S40>/Memory' */
  boolean_T AND3_c;                    /* '<S250>/AND3' */
  boolean_T Memory_h;                  /* '<S250>/Memory' */
  boolean_T SourceBlock_o1;            /* '<S13>/SourceBlock' */
  boolean_T SourceBlock_o1_e;          /* '<S12>/SourceBlock' */
  boolean_T SourceBlock_o1_o;          /* '<S11>/SourceBlock' */
  boolean_T SourceBlock_o1_h;          /* '<S286>/SourceBlock' */
  boolean_T SourceBlock_o1_d;          /* '<S285>/SourceBlock' */
  boolean_T SourceBlock_o1_c;          /* '<S284>/SourceBlock' */
  boolean_T SourceBlock_o1_k;          /* '<S283>/SourceBlock' */
  B_EnabledSubsystem_pid_cont_d_T EnabledSubsystem_a;/* '<S13>/Enabled Subsystem' */
  B_EnabledSubsystem_pid_cont_d_T EnabledSubsystem_ps;/* '<S12>/Enabled Subsystem' */
  B_EnabledSubsystem_pid_cont_d_T EnabledSubsystem_b;/* '<S11>/Enabled Subsystem' */
  B_EnabledSubsystem_pid_contro_T EnabledSubsystem_pt;/* '<S286>/Enabled Subsystem' */
  B_EnabledSubsystem_pid_cont_d_T EnabledSubsystem_p;/* '<S285>/Enabled Subsystem' */
  B_EnabledSubsystem_pid_cont_d_T EnabledSubsystem_k;/* '<S284>/Enabled Subsystem' */
  B_EnabledSubsystem_pid_contro_T EnabledSubsystem;/* '<S283>/Enabled Subsystem' */
};

/* Block states (default storage) for system '<Root>' */
struct DW_pid_control_V1_T {
  ros_slros2_internal_block_Ser_T obj; /* '<S2>/ServiceCaller' */
  ros_slros2_internal_block_Sub_T obj_k;/* '<S13>/SourceBlock' */
  ros_slros2_internal_block_Sub_T obj_i;/* '<S12>/SourceBlock' */
  ros_slros2_internal_block_Sub_T obj_m;/* '<S11>/SourceBlock' */
  ros_slros2_internal_block_Sub_T obj_h;/* '<S286>/SourceBlock' */
  ros_slros2_internal_block_Sub_T obj_p;/* '<S285>/SourceBlock' */
  ros_slros2_internal_block_Sub_T obj_hy;/* '<S284>/SourceBlock' */
  ros_slros2_internal_block_Sub_T obj_h4;/* '<S283>/SourceBlock' */
  real_T UnitDelay3_DSTATE;            /* '<Root>/Unit Delay3' */
  real_T UnitDelay1_DSTATE;            /* '<Root>/Unit Delay1' */
  real_T UnitDelay2_DSTATE;            /* '<Root>/Unit Delay2' */
  real_T Memory2_PreviousInput[12];    /* '<S10>/Memory2' */
  real_T PrevY;                        /* '<Root>/Rate Limiter' */
  real_T LastMajorTime;                /* '<Root>/Rate Limiter' */
  real_T Memory_PreviousInput[3];      /* '<S10>/Memory' */
  real_T Memory1_PreviousInput[3];     /* '<S10>/Memory1' */
  real_T NextOutput[4];                /* '<S298>/White Noise' */
  real_T NextOutput_k;                 /* '<S278>/White Noise' */
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
                        /* '<S305>/PreLook-Up Index Search  (prob of exceed)' */
  uint32_T PreLookUpIndexSearchaltitude_DW;
                              /* '<S305>/PreLook-Up Index Search  (altitude)' */
  uint32_T RandSeed[4];                /* '<S298>/White Noise' */
  uint32_T RandSeed_a;                 /* '<S278>/White Noise' */
  robotics_slcore_internal_bloc_T obj_c;
                             /* '<Root>/Coordinate Transformation Conversion' */
  int8_T ifHeightMaxlowaltitudeelseifHei;
  /* '<S294>/if Height < Max low altitude  elseif Height > Min isotropic altitude ' */
  int8_T ifHeightMaxlowaltitudeelseifH_a;
  /* '<S293>/if Height < Max low altitude  elseif Height > Min isotropic altitude ' */
  boolean_T IC_FirstOutputTime;        /* '<S10>/IC' */
  boolean_T Integrator_DWORK1;         /* '<S10>/Integrator' */
  boolean_T PrevLimited;               /* '<Root>/Rate Limiter' */
  boolean_T Memory_PreviousInput_o;    /* '<S40>/Memory' */
  boolean_T Memory_PreviousInput_a;    /* '<S250>/Memory' */
  boolean_T objisempty;                /* '<S13>/SourceBlock' */
  boolean_T objisempty_f;              /* '<S12>/SourceBlock' */
  boolean_T objisempty_g;              /* '<S11>/SourceBlock' */
  boolean_T objisempty_a;              /* '<S286>/SourceBlock' */
  boolean_T objisempty_e;              /* '<S285>/SourceBlock' */
  boolean_T objisempty_l;              /* '<S284>/SourceBlock' */
  boolean_T objisempty_c;              /* '<S283>/SourceBlock' */
  boolean_T objisempty_d;    /* '<Root>/Coordinate Transformation Conversion' */
  boolean_T objisempty_ft;             /* '<S2>/ServiceCaller' */
  boolean_T Hwgws_MODE;                /* '<S289>/Hwgw(s)' */
  boolean_T Hvgws_MODE;                /* '<S289>/Hvgw(s)' */
  boolean_T Hugws_MODE;                /* '<S289>/Hugw(s)' */
  boolean_T Hrgw_MODE;                 /* '<S288>/Hrgw' */
  boolean_T Hqgw_MODE;                 /* '<S288>/Hqgw' */
  boolean_T Hpgw_MODE;                 /* '<S288>/Hpgw' */
};

/* Continuous states (default storage) */
struct X_pid_control_V1_T {
  real_T Integrator_CSTATE[12];        /* '<S10>/Integrator' */
  real_T Integrator_CSTATE_n;          /* '<S102>/Integrator' */
  real_T Filter_CSTATE;                /* '<S97>/Filter' */
  real_T Integrator_CSTATE_m;          /* '<S50>/Integrator' */
  real_T Filter_CSTATE_g;              /* '<S45>/Filter' */
  real_T Integrator_CSTATE_p;          /* '<S154>/Integrator' */
  real_T Filter_CSTATE_m;              /* '<S149>/Filter' */
  real_T Integrator_CSTATE_d;          /* '<S206>/Integrator' */
  real_T Filter_CSTATE_f;              /* '<S201>/Filter' */
  real_T Integrator_CSTATE_f;          /* '<S260>/Integrator' */
  real_T Filter_CSTATE_l;              /* '<S255>/Filter' */
  real_T Integrator1_CSTATE;           /* '<S10>/Integrator1' */
  real_T TransferFcn_CSTATE[2];        /* '<S10>/Transfer Fcn' */
  real_T TransferFcn1_CSTATE;          /* '<S10>/Transfer Fcn1' */
  real_T wg_p1_CSTATE[2];              /* '<S304>/wg_p1' */
  real_T wg_p2_CSTATE[2];              /* '<S304>/wg_p2' */
  real_T vg_p1_CSTATE[2];              /* '<S303>/vg_p1' */
  real_T vgw_p2_CSTATE[2];             /* '<S303>/vgw_p2' */
  real_T ug_p_CSTATE[2];               /* '<S302>/ug_p' */
  real_T rgw_p_CSTATE[2];              /* '<S301>/rgw_p' */
  real_T qgw_p_CSTATE[2];              /* '<S300>/qgw_p' */
  real_T pgw_p_CSTATE[2];              /* '<S299>/pgw_p' */
};

/* State derivatives (default storage) */
struct XDot_pid_control_V1_T {
  real_T Integrator_CSTATE[12];        /* '<S10>/Integrator' */
  real_T Integrator_CSTATE_n;          /* '<S102>/Integrator' */
  real_T Filter_CSTATE;                /* '<S97>/Filter' */
  real_T Integrator_CSTATE_m;          /* '<S50>/Integrator' */
  real_T Filter_CSTATE_g;              /* '<S45>/Filter' */
  real_T Integrator_CSTATE_p;          /* '<S154>/Integrator' */
  real_T Filter_CSTATE_m;              /* '<S149>/Filter' */
  real_T Integrator_CSTATE_d;          /* '<S206>/Integrator' */
  real_T Filter_CSTATE_f;              /* '<S201>/Filter' */
  real_T Integrator_CSTATE_f;          /* '<S260>/Integrator' */
  real_T Filter_CSTATE_l;              /* '<S255>/Filter' */
  real_T Integrator1_CSTATE;           /* '<S10>/Integrator1' */
  real_T TransferFcn_CSTATE[2];        /* '<S10>/Transfer Fcn' */
  real_T TransferFcn1_CSTATE;          /* '<S10>/Transfer Fcn1' */
  real_T wg_p1_CSTATE[2];              /* '<S304>/wg_p1' */
  real_T wg_p2_CSTATE[2];              /* '<S304>/wg_p2' */
  real_T vg_p1_CSTATE[2];              /* '<S303>/vg_p1' */
  real_T vgw_p2_CSTATE[2];             /* '<S303>/vgw_p2' */
  real_T ug_p_CSTATE[2];               /* '<S302>/ug_p' */
  real_T rgw_p_CSTATE[2];              /* '<S301>/rgw_p' */
  real_T qgw_p_CSTATE[2];              /* '<S300>/qgw_p' */
  real_T pgw_p_CSTATE[2];              /* '<S299>/pgw_p' */
};

/* State disabled  */
struct XDis_pid_control_V1_T {
  boolean_T Integrator_CSTATE[12];     /* '<S10>/Integrator' */
  boolean_T Integrator_CSTATE_n;       /* '<S102>/Integrator' */
  boolean_T Filter_CSTATE;             /* '<S97>/Filter' */
  boolean_T Integrator_CSTATE_m;       /* '<S50>/Integrator' */
  boolean_T Filter_CSTATE_g;           /* '<S45>/Filter' */
  boolean_T Integrator_CSTATE_p;       /* '<S154>/Integrator' */
  boolean_T Filter_CSTATE_m;           /* '<S149>/Filter' */
  boolean_T Integrator_CSTATE_d;       /* '<S206>/Integrator' */
  boolean_T Filter_CSTATE_f;           /* '<S201>/Filter' */
  boolean_T Integrator_CSTATE_f;       /* '<S260>/Integrator' */
  boolean_T Filter_CSTATE_l;           /* '<S255>/Filter' */
  boolean_T Integrator1_CSTATE;        /* '<S10>/Integrator1' */
  boolean_T TransferFcn_CSTATE[2];     /* '<S10>/Transfer Fcn' */
  boolean_T TransferFcn1_CSTATE;       /* '<S10>/Transfer Fcn1' */
  boolean_T wg_p1_CSTATE[2];           /* '<S304>/wg_p1' */
  boolean_T wg_p2_CSTATE[2];           /* '<S304>/wg_p2' */
  boolean_T vg_p1_CSTATE[2];           /* '<S303>/vg_p1' */
  boolean_T vgw_p2_CSTATE[2];          /* '<S303>/vgw_p2' */
  boolean_T ug_p_CSTATE[2];            /* '<S302>/ug_p' */
  boolean_T rgw_p_CSTATE[2];           /* '<S301>/rgw_p' */
  boolean_T qgw_p_CSTATE[2];           /* '<S300>/qgw_p' */
  boolean_T pgw_p_CSTATE[2];           /* '<S299>/pgw_p' */
};

/* Zero-crossing (trigger) state */
struct PrevZCX_pid_control_V1_T {
  ZCSigState Integrator_Reset_ZCE;     /* '<S10>/Integrator' */
};

/* Invariant block signals (default storage) */
struct ConstB_pid_control_V1_T {
  real_T UnitConversion;               /* '<S287>/Unit Conversion' */
  real_T UnitConversion_k;             /* '<S297>/Unit Conversion' */
  real_T sigma_wg;                     /* '<S306>/sigma_wg ' */
  real_T UnitConversion_n;             /* '<S291>/Unit Conversion' */
  real_T UnitConversion_c;             /* '<S325>/Unit Conversion' */
  real_T PreLookUpIndexSearchprobofe;
                        /* '<S305>/PreLook-Up Index Search  (prob of exceed)' */
  real_T Sqrt[4];                      /* '<S298>/Sqrt' */
  real_T Sqrt1;                        /* '<S298>/Sqrt1' */
  real_T Divide[4];                    /* '<S298>/Divide' */
  real_T motorspeed;                   /* '<S10>/Gain2' */
  real_T Sum;                          /* '<S315>/Sum' */
  real_T Sum_a;                        /* '<S307>/Sum' */
  real_T sqrt_a;                       /* '<S304>/sqrt' */
  real_T w4;                           /* '<S299>/w4' */
  real_T u16;                          /* '<S299>/u^1//6' */
  uint32_T PreLookUpIndexSearchprobo_g;
                        /* '<S305>/PreLook-Up Index Search  (prob of exceed)' */
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
   * Referenced by: '<S305>/PreLook-Up Index Search  (altitude)'
   */
  real_T PreLookUpIndexSearchaltitude_Br[12];

  /* Expression: sigma_vec'
   * Referenced by: '<S305>/Medium//High Altitude Intensity'
   */
  real_T MediumHighAltitudeIntensity_Tab[84];

  /* Computed Parameter: MediumHighAltitudeIntensity_max
   * Referenced by: '<S305>/Medium//High Altitude Intensity'
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

  /* private member function(s) for subsystem '<S283>/Enabled Subsystem'*/
  static void pid_contr_EnabledSubsystem_Init(B_EnabledSubsystem_pid_contro_T
    *localB);
  static void pid_control_V1_EnabledSubsystem(boolean_T rtu_Enable, const
    SL_Bus_std_msgs_Bool *rtu_In1, B_EnabledSubsystem_pid_contro_T *localB);

  /* private member function(s) for subsystem '<S284>/Enabled Subsystem'*/
  static void pid_con_EnabledSubsystem_i_Init(B_EnabledSubsystem_pid_cont_d_T
    *localB);
  static void pid_control__EnabledSubsystem_k(boolean_T rtu_Enable, const
    SL_Bus_std_msgs_Float64 *rtu_In1, B_EnabledSubsystem_pid_cont_d_T *localB);

  /* private member function(s) for subsystem '<Root>'*/
  void pid_c_Subscriber_setupImpl_onhg(const ros_slros2_internal_block_Sub_T
    *obj);
  void pid__Subscriber_setupImpl_onhgd(const ros_slros2_internal_block_Sub_T
    *obj);
  void pid_Subscriber_setupImpl_onhgd0(const ros_slros2_internal_block_Sub_T
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
 * Block '<S146>/Kb' : Eliminated nontunable gain of 1
 * Block '<Root>/Rate Transition' : Eliminated since input and output rates are identical
 * Block '<S280>/Cast' : Eliminate redundant data type conversion
 * Block '<S280>/Cast To Double' : Eliminate redundant data type conversion
 * Block '<S280>/Cast To Double1' : Eliminate redundant data type conversion
 * Block '<S280>/Cast To Double2' : Eliminate redundant data type conversion
 * Block '<S280>/Cast To Double3' : Eliminate redundant data type conversion
 * Block '<S280>/Cast To Double4' : Eliminate redundant data type conversion
 * Block '<S311>/Reshape' : Reshape block reduction
 * Block '<S311>/Reshape1' : Reshape block reduction
 * Block '<S313>/Reshape' : Reshape block reduction
 * Block '<S319>/Reshape' : Reshape block reduction
 * Block '<S319>/Reshape1' : Reshape block reduction
 * Block '<S321>/Reshape' : Reshape block reduction
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
 * '<S12>'  : 'pid_control_V1/Subscribe-ROLL'
 * '<S13>'  : 'pid_control_V1/Subscribe-YAW'
 * '<S14>'  : 'pid_control_V1/PID ALERON/Anti-windup'
 * '<S15>'  : 'pid_control_V1/PID ALERON/D Gain'
 * '<S16>'  : 'pid_control_V1/PID ALERON/External Derivative'
 * '<S17>'  : 'pid_control_V1/PID ALERON/Filter'
 * '<S18>'  : 'pid_control_V1/PID ALERON/Filter ICs'
 * '<S19>'  : 'pid_control_V1/PID ALERON/I Gain'
 * '<S20>'  : 'pid_control_V1/PID ALERON/Ideal P Gain'
 * '<S21>'  : 'pid_control_V1/PID ALERON/Ideal P Gain Fdbk'
 * '<S22>'  : 'pid_control_V1/PID ALERON/Integrator'
 * '<S23>'  : 'pid_control_V1/PID ALERON/Integrator ICs'
 * '<S24>'  : 'pid_control_V1/PID ALERON/N Copy'
 * '<S25>'  : 'pid_control_V1/PID ALERON/N Gain'
 * '<S26>'  : 'pid_control_V1/PID ALERON/P Copy'
 * '<S27>'  : 'pid_control_V1/PID ALERON/Parallel P Gain'
 * '<S28>'  : 'pid_control_V1/PID ALERON/Reset Signal'
 * '<S29>'  : 'pid_control_V1/PID ALERON/Saturation'
 * '<S30>'  : 'pid_control_V1/PID ALERON/Saturation Fdbk'
 * '<S31>'  : 'pid_control_V1/PID ALERON/Sum'
 * '<S32>'  : 'pid_control_V1/PID ALERON/Sum Fdbk'
 * '<S33>'  : 'pid_control_V1/PID ALERON/Tracking Mode'
 * '<S34>'  : 'pid_control_V1/PID ALERON/Tracking Mode Sum'
 * '<S35>'  : 'pid_control_V1/PID ALERON/Tsamp - Integral'
 * '<S36>'  : 'pid_control_V1/PID ALERON/Tsamp - Ngain'
 * '<S37>'  : 'pid_control_V1/PID ALERON/postSat Signal'
 * '<S38>'  : 'pid_control_V1/PID ALERON/preInt Signal'
 * '<S39>'  : 'pid_control_V1/PID ALERON/preSat Signal'
 * '<S40>'  : 'pid_control_V1/PID ALERON/Anti-windup/Cont. Clamping Parallel'
 * '<S41>'  : 'pid_control_V1/PID ALERON/Anti-windup/Cont. Clamping Parallel/Dead Zone'
 * '<S42>'  : 'pid_control_V1/PID ALERON/Anti-windup/Cont. Clamping Parallel/Dead Zone/Enabled'
 * '<S43>'  : 'pid_control_V1/PID ALERON/D Gain/Internal Parameters'
 * '<S44>'  : 'pid_control_V1/PID ALERON/External Derivative/Error'
 * '<S45>'  : 'pid_control_V1/PID ALERON/Filter/Cont. Filter'
 * '<S46>'  : 'pid_control_V1/PID ALERON/Filter ICs/Internal IC - Filter'
 * '<S47>'  : 'pid_control_V1/PID ALERON/I Gain/Internal Parameters'
 * '<S48>'  : 'pid_control_V1/PID ALERON/Ideal P Gain/Passthrough'
 * '<S49>'  : 'pid_control_V1/PID ALERON/Ideal P Gain Fdbk/Disabled'
 * '<S50>'  : 'pid_control_V1/PID ALERON/Integrator/Continuous'
 * '<S51>'  : 'pid_control_V1/PID ALERON/Integrator ICs/Internal IC'
 * '<S52>'  : 'pid_control_V1/PID ALERON/N Copy/Disabled'
 * '<S53>'  : 'pid_control_V1/PID ALERON/N Gain/Internal Parameters'
 * '<S54>'  : 'pid_control_V1/PID ALERON/P Copy/Disabled'
 * '<S55>'  : 'pid_control_V1/PID ALERON/Parallel P Gain/Internal Parameters'
 * '<S56>'  : 'pid_control_V1/PID ALERON/Reset Signal/Disabled'
 * '<S57>'  : 'pid_control_V1/PID ALERON/Saturation/Enabled'
 * '<S58>'  : 'pid_control_V1/PID ALERON/Saturation Fdbk/Disabled'
 * '<S59>'  : 'pid_control_V1/PID ALERON/Sum/Sum_PID'
 * '<S60>'  : 'pid_control_V1/PID ALERON/Sum Fdbk/Disabled'
 * '<S61>'  : 'pid_control_V1/PID ALERON/Tracking Mode/Disabled'
 * '<S62>'  : 'pid_control_V1/PID ALERON/Tracking Mode Sum/Passthrough'
 * '<S63>'  : 'pid_control_V1/PID ALERON/Tsamp - Integral/TsSignalSpecification'
 * '<S64>'  : 'pid_control_V1/PID ALERON/Tsamp - Ngain/Passthrough'
 * '<S65>'  : 'pid_control_V1/PID ALERON/postSat Signal/Forward_Path'
 * '<S66>'  : 'pid_control_V1/PID ALERON/preInt Signal/Internal PreInt'
 * '<S67>'  : 'pid_control_V1/PID ALERON/preSat Signal/Forward_Path'
 * '<S68>'  : 'pid_control_V1/PID ALTURA/Anti-windup'
 * '<S69>'  : 'pid_control_V1/PID ALTURA/D Gain'
 * '<S70>'  : 'pid_control_V1/PID ALTURA/External Derivative'
 * '<S71>'  : 'pid_control_V1/PID ALTURA/Filter'
 * '<S72>'  : 'pid_control_V1/PID ALTURA/Filter ICs'
 * '<S73>'  : 'pid_control_V1/PID ALTURA/I Gain'
 * '<S74>'  : 'pid_control_V1/PID ALTURA/Ideal P Gain'
 * '<S75>'  : 'pid_control_V1/PID ALTURA/Ideal P Gain Fdbk'
 * '<S76>'  : 'pid_control_V1/PID ALTURA/Integrator'
 * '<S77>'  : 'pid_control_V1/PID ALTURA/Integrator ICs'
 * '<S78>'  : 'pid_control_V1/PID ALTURA/N Copy'
 * '<S79>'  : 'pid_control_V1/PID ALTURA/N Gain'
 * '<S80>'  : 'pid_control_V1/PID ALTURA/P Copy'
 * '<S81>'  : 'pid_control_V1/PID ALTURA/Parallel P Gain'
 * '<S82>'  : 'pid_control_V1/PID ALTURA/Reset Signal'
 * '<S83>'  : 'pid_control_V1/PID ALTURA/Saturation'
 * '<S84>'  : 'pid_control_V1/PID ALTURA/Saturation Fdbk'
 * '<S85>'  : 'pid_control_V1/PID ALTURA/Sum'
 * '<S86>'  : 'pid_control_V1/PID ALTURA/Sum Fdbk'
 * '<S87>'  : 'pid_control_V1/PID ALTURA/Tracking Mode'
 * '<S88>'  : 'pid_control_V1/PID ALTURA/Tracking Mode Sum'
 * '<S89>'  : 'pid_control_V1/PID ALTURA/Tsamp - Integral'
 * '<S90>'  : 'pid_control_V1/PID ALTURA/Tsamp - Ngain'
 * '<S91>'  : 'pid_control_V1/PID ALTURA/postSat Signal'
 * '<S92>'  : 'pid_control_V1/PID ALTURA/preInt Signal'
 * '<S93>'  : 'pid_control_V1/PID ALTURA/preSat Signal'
 * '<S94>'  : 'pid_control_V1/PID ALTURA/Anti-windup/Back Calculation'
 * '<S95>'  : 'pid_control_V1/PID ALTURA/D Gain/Internal Parameters'
 * '<S96>'  : 'pid_control_V1/PID ALTURA/External Derivative/Error'
 * '<S97>'  : 'pid_control_V1/PID ALTURA/Filter/Cont. Filter'
 * '<S98>'  : 'pid_control_V1/PID ALTURA/Filter ICs/Internal IC - Filter'
 * '<S99>'  : 'pid_control_V1/PID ALTURA/I Gain/Internal Parameters'
 * '<S100>' : 'pid_control_V1/PID ALTURA/Ideal P Gain/Passthrough'
 * '<S101>' : 'pid_control_V1/PID ALTURA/Ideal P Gain Fdbk/Disabled'
 * '<S102>' : 'pid_control_V1/PID ALTURA/Integrator/Continuous'
 * '<S103>' : 'pid_control_V1/PID ALTURA/Integrator ICs/Internal IC'
 * '<S104>' : 'pid_control_V1/PID ALTURA/N Copy/Disabled'
 * '<S105>' : 'pid_control_V1/PID ALTURA/N Gain/Internal Parameters'
 * '<S106>' : 'pid_control_V1/PID ALTURA/P Copy/Disabled'
 * '<S107>' : 'pid_control_V1/PID ALTURA/Parallel P Gain/Internal Parameters'
 * '<S108>' : 'pid_control_V1/PID ALTURA/Reset Signal/Disabled'
 * '<S109>' : 'pid_control_V1/PID ALTURA/Saturation/Enabled'
 * '<S110>' : 'pid_control_V1/PID ALTURA/Saturation Fdbk/Disabled'
 * '<S111>' : 'pid_control_V1/PID ALTURA/Sum/Sum_PID'
 * '<S112>' : 'pid_control_V1/PID ALTURA/Sum Fdbk/Disabled'
 * '<S113>' : 'pid_control_V1/PID ALTURA/Tracking Mode/Disabled'
 * '<S114>' : 'pid_control_V1/PID ALTURA/Tracking Mode Sum/Passthrough'
 * '<S115>' : 'pid_control_V1/PID ALTURA/Tsamp - Integral/TsSignalSpecification'
 * '<S116>' : 'pid_control_V1/PID ALTURA/Tsamp - Ngain/Passthrough'
 * '<S117>' : 'pid_control_V1/PID ALTURA/postSat Signal/Forward_Path'
 * '<S118>' : 'pid_control_V1/PID ALTURA/preInt Signal/Internal PreInt'
 * '<S119>' : 'pid_control_V1/PID ALTURA/preSat Signal/Forward_Path'
 * '<S120>' : 'pid_control_V1/PID PITCH//ELEVATOR/Anti-windup'
 * '<S121>' : 'pid_control_V1/PID PITCH//ELEVATOR/D Gain'
 * '<S122>' : 'pid_control_V1/PID PITCH//ELEVATOR/External Derivative'
 * '<S123>' : 'pid_control_V1/PID PITCH//ELEVATOR/Filter'
 * '<S124>' : 'pid_control_V1/PID PITCH//ELEVATOR/Filter ICs'
 * '<S125>' : 'pid_control_V1/PID PITCH//ELEVATOR/I Gain'
 * '<S126>' : 'pid_control_V1/PID PITCH//ELEVATOR/Ideal P Gain'
 * '<S127>' : 'pid_control_V1/PID PITCH//ELEVATOR/Ideal P Gain Fdbk'
 * '<S128>' : 'pid_control_V1/PID PITCH//ELEVATOR/Integrator'
 * '<S129>' : 'pid_control_V1/PID PITCH//ELEVATOR/Integrator ICs'
 * '<S130>' : 'pid_control_V1/PID PITCH//ELEVATOR/N Copy'
 * '<S131>' : 'pid_control_V1/PID PITCH//ELEVATOR/N Gain'
 * '<S132>' : 'pid_control_V1/PID PITCH//ELEVATOR/P Copy'
 * '<S133>' : 'pid_control_V1/PID PITCH//ELEVATOR/Parallel P Gain'
 * '<S134>' : 'pid_control_V1/PID PITCH//ELEVATOR/Reset Signal'
 * '<S135>' : 'pid_control_V1/PID PITCH//ELEVATOR/Saturation'
 * '<S136>' : 'pid_control_V1/PID PITCH//ELEVATOR/Saturation Fdbk'
 * '<S137>' : 'pid_control_V1/PID PITCH//ELEVATOR/Sum'
 * '<S138>' : 'pid_control_V1/PID PITCH//ELEVATOR/Sum Fdbk'
 * '<S139>' : 'pid_control_V1/PID PITCH//ELEVATOR/Tracking Mode'
 * '<S140>' : 'pid_control_V1/PID PITCH//ELEVATOR/Tracking Mode Sum'
 * '<S141>' : 'pid_control_V1/PID PITCH//ELEVATOR/Tsamp - Integral'
 * '<S142>' : 'pid_control_V1/PID PITCH//ELEVATOR/Tsamp - Ngain'
 * '<S143>' : 'pid_control_V1/PID PITCH//ELEVATOR/postSat Signal'
 * '<S144>' : 'pid_control_V1/PID PITCH//ELEVATOR/preInt Signal'
 * '<S145>' : 'pid_control_V1/PID PITCH//ELEVATOR/preSat Signal'
 * '<S146>' : 'pid_control_V1/PID PITCH//ELEVATOR/Anti-windup/Back Calculation'
 * '<S147>' : 'pid_control_V1/PID PITCH//ELEVATOR/D Gain/Internal Parameters'
 * '<S148>' : 'pid_control_V1/PID PITCH//ELEVATOR/External Derivative/Error'
 * '<S149>' : 'pid_control_V1/PID PITCH//ELEVATOR/Filter/Cont. Filter'
 * '<S150>' : 'pid_control_V1/PID PITCH//ELEVATOR/Filter ICs/Internal IC - Filter'
 * '<S151>' : 'pid_control_V1/PID PITCH//ELEVATOR/I Gain/Internal Parameters'
 * '<S152>' : 'pid_control_V1/PID PITCH//ELEVATOR/Ideal P Gain/Passthrough'
 * '<S153>' : 'pid_control_V1/PID PITCH//ELEVATOR/Ideal P Gain Fdbk/Disabled'
 * '<S154>' : 'pid_control_V1/PID PITCH//ELEVATOR/Integrator/Continuous'
 * '<S155>' : 'pid_control_V1/PID PITCH//ELEVATOR/Integrator ICs/Internal IC'
 * '<S156>' : 'pid_control_V1/PID PITCH//ELEVATOR/N Copy/Disabled'
 * '<S157>' : 'pid_control_V1/PID PITCH//ELEVATOR/N Gain/Internal Parameters'
 * '<S158>' : 'pid_control_V1/PID PITCH//ELEVATOR/P Copy/Disabled'
 * '<S159>' : 'pid_control_V1/PID PITCH//ELEVATOR/Parallel P Gain/Internal Parameters'
 * '<S160>' : 'pid_control_V1/PID PITCH//ELEVATOR/Reset Signal/Disabled'
 * '<S161>' : 'pid_control_V1/PID PITCH//ELEVATOR/Saturation/Enabled'
 * '<S162>' : 'pid_control_V1/PID PITCH//ELEVATOR/Saturation Fdbk/Disabled'
 * '<S163>' : 'pid_control_V1/PID PITCH//ELEVATOR/Sum/Sum_PID'
 * '<S164>' : 'pid_control_V1/PID PITCH//ELEVATOR/Sum Fdbk/Disabled'
 * '<S165>' : 'pid_control_V1/PID PITCH//ELEVATOR/Tracking Mode/Disabled'
 * '<S166>' : 'pid_control_V1/PID PITCH//ELEVATOR/Tracking Mode Sum/Passthrough'
 * '<S167>' : 'pid_control_V1/PID PITCH//ELEVATOR/Tsamp - Integral/TsSignalSpecification'
 * '<S168>' : 'pid_control_V1/PID PITCH//ELEVATOR/Tsamp - Ngain/Passthrough'
 * '<S169>' : 'pid_control_V1/PID PITCH//ELEVATOR/postSat Signal/Forward_Path'
 * '<S170>' : 'pid_control_V1/PID PITCH//ELEVATOR/preInt Signal/Internal PreInt'
 * '<S171>' : 'pid_control_V1/PID PITCH//ELEVATOR/preSat Signal/Forward_Path'
 * '<S172>' : 'pid_control_V1/PID TIIMON/Anti-windup'
 * '<S173>' : 'pid_control_V1/PID TIIMON/D Gain'
 * '<S174>' : 'pid_control_V1/PID TIIMON/External Derivative'
 * '<S175>' : 'pid_control_V1/PID TIIMON/Filter'
 * '<S176>' : 'pid_control_V1/PID TIIMON/Filter ICs'
 * '<S177>' : 'pid_control_V1/PID TIIMON/I Gain'
 * '<S178>' : 'pid_control_V1/PID TIIMON/Ideal P Gain'
 * '<S179>' : 'pid_control_V1/PID TIIMON/Ideal P Gain Fdbk'
 * '<S180>' : 'pid_control_V1/PID TIIMON/Integrator'
 * '<S181>' : 'pid_control_V1/PID TIIMON/Integrator ICs'
 * '<S182>' : 'pid_control_V1/PID TIIMON/N Copy'
 * '<S183>' : 'pid_control_V1/PID TIIMON/N Gain'
 * '<S184>' : 'pid_control_V1/PID TIIMON/P Copy'
 * '<S185>' : 'pid_control_V1/PID TIIMON/Parallel P Gain'
 * '<S186>' : 'pid_control_V1/PID TIIMON/Reset Signal'
 * '<S187>' : 'pid_control_V1/PID TIIMON/Saturation'
 * '<S188>' : 'pid_control_V1/PID TIIMON/Saturation Fdbk'
 * '<S189>' : 'pid_control_V1/PID TIIMON/Sum'
 * '<S190>' : 'pid_control_V1/PID TIIMON/Sum Fdbk'
 * '<S191>' : 'pid_control_V1/PID TIIMON/Tracking Mode'
 * '<S192>' : 'pid_control_V1/PID TIIMON/Tracking Mode Sum'
 * '<S193>' : 'pid_control_V1/PID TIIMON/Tsamp - Integral'
 * '<S194>' : 'pid_control_V1/PID TIIMON/Tsamp - Ngain'
 * '<S195>' : 'pid_control_V1/PID TIIMON/postSat Signal'
 * '<S196>' : 'pid_control_V1/PID TIIMON/preInt Signal'
 * '<S197>' : 'pid_control_V1/PID TIIMON/preSat Signal'
 * '<S198>' : 'pid_control_V1/PID TIIMON/Anti-windup/Passthrough'
 * '<S199>' : 'pid_control_V1/PID TIIMON/D Gain/Internal Parameters'
 * '<S200>' : 'pid_control_V1/PID TIIMON/External Derivative/Error'
 * '<S201>' : 'pid_control_V1/PID TIIMON/Filter/Cont. Filter'
 * '<S202>' : 'pid_control_V1/PID TIIMON/Filter ICs/Internal IC - Filter'
 * '<S203>' : 'pid_control_V1/PID TIIMON/I Gain/Internal Parameters'
 * '<S204>' : 'pid_control_V1/PID TIIMON/Ideal P Gain/Passthrough'
 * '<S205>' : 'pid_control_V1/PID TIIMON/Ideal P Gain Fdbk/Disabled'
 * '<S206>' : 'pid_control_V1/PID TIIMON/Integrator/Continuous'
 * '<S207>' : 'pid_control_V1/PID TIIMON/Integrator ICs/Internal IC'
 * '<S208>' : 'pid_control_V1/PID TIIMON/N Copy/Disabled'
 * '<S209>' : 'pid_control_V1/PID TIIMON/N Gain/Internal Parameters'
 * '<S210>' : 'pid_control_V1/PID TIIMON/P Copy/Disabled'
 * '<S211>' : 'pid_control_V1/PID TIIMON/Parallel P Gain/Internal Parameters'
 * '<S212>' : 'pid_control_V1/PID TIIMON/Reset Signal/Disabled'
 * '<S213>' : 'pid_control_V1/PID TIIMON/Saturation/Enabled'
 * '<S214>' : 'pid_control_V1/PID TIIMON/Saturation Fdbk/Disabled'
 * '<S215>' : 'pid_control_V1/PID TIIMON/Sum/Sum_PID'
 * '<S216>' : 'pid_control_V1/PID TIIMON/Sum Fdbk/Disabled'
 * '<S217>' : 'pid_control_V1/PID TIIMON/Tracking Mode/Disabled'
 * '<S218>' : 'pid_control_V1/PID TIIMON/Tracking Mode Sum/Passthrough'
 * '<S219>' : 'pid_control_V1/PID TIIMON/Tsamp - Integral/TsSignalSpecification'
 * '<S220>' : 'pid_control_V1/PID TIIMON/Tsamp - Ngain/Passthrough'
 * '<S221>' : 'pid_control_V1/PID TIIMON/postSat Signal/Forward_Path'
 * '<S222>' : 'pid_control_V1/PID TIIMON/preInt Signal/Internal PreInt'
 * '<S223>' : 'pid_control_V1/PID TIIMON/preSat Signal/Forward_Path'
 * '<S224>' : 'pid_control_V1/PID VELOCIDAD/Anti-windup'
 * '<S225>' : 'pid_control_V1/PID VELOCIDAD/D Gain'
 * '<S226>' : 'pid_control_V1/PID VELOCIDAD/External Derivative'
 * '<S227>' : 'pid_control_V1/PID VELOCIDAD/Filter'
 * '<S228>' : 'pid_control_V1/PID VELOCIDAD/Filter ICs'
 * '<S229>' : 'pid_control_V1/PID VELOCIDAD/I Gain'
 * '<S230>' : 'pid_control_V1/PID VELOCIDAD/Ideal P Gain'
 * '<S231>' : 'pid_control_V1/PID VELOCIDAD/Ideal P Gain Fdbk'
 * '<S232>' : 'pid_control_V1/PID VELOCIDAD/Integrator'
 * '<S233>' : 'pid_control_V1/PID VELOCIDAD/Integrator ICs'
 * '<S234>' : 'pid_control_V1/PID VELOCIDAD/N Copy'
 * '<S235>' : 'pid_control_V1/PID VELOCIDAD/N Gain'
 * '<S236>' : 'pid_control_V1/PID VELOCIDAD/P Copy'
 * '<S237>' : 'pid_control_V1/PID VELOCIDAD/Parallel P Gain'
 * '<S238>' : 'pid_control_V1/PID VELOCIDAD/Reset Signal'
 * '<S239>' : 'pid_control_V1/PID VELOCIDAD/Saturation'
 * '<S240>' : 'pid_control_V1/PID VELOCIDAD/Saturation Fdbk'
 * '<S241>' : 'pid_control_V1/PID VELOCIDAD/Sum'
 * '<S242>' : 'pid_control_V1/PID VELOCIDAD/Sum Fdbk'
 * '<S243>' : 'pid_control_V1/PID VELOCIDAD/Tracking Mode'
 * '<S244>' : 'pid_control_V1/PID VELOCIDAD/Tracking Mode Sum'
 * '<S245>' : 'pid_control_V1/PID VELOCIDAD/Tsamp - Integral'
 * '<S246>' : 'pid_control_V1/PID VELOCIDAD/Tsamp - Ngain'
 * '<S247>' : 'pid_control_V1/PID VELOCIDAD/postSat Signal'
 * '<S248>' : 'pid_control_V1/PID VELOCIDAD/preInt Signal'
 * '<S249>' : 'pid_control_V1/PID VELOCIDAD/preSat Signal'
 * '<S250>' : 'pid_control_V1/PID VELOCIDAD/Anti-windup/Cont. Clamping Parallel'
 * '<S251>' : 'pid_control_V1/PID VELOCIDAD/Anti-windup/Cont. Clamping Parallel/Dead Zone'
 * '<S252>' : 'pid_control_V1/PID VELOCIDAD/Anti-windup/Cont. Clamping Parallel/Dead Zone/Enabled'
 * '<S253>' : 'pid_control_V1/PID VELOCIDAD/D Gain/Internal Parameters'
 * '<S254>' : 'pid_control_V1/PID VELOCIDAD/External Derivative/Error'
 * '<S255>' : 'pid_control_V1/PID VELOCIDAD/Filter/Cont. Filter'
 * '<S256>' : 'pid_control_V1/PID VELOCIDAD/Filter ICs/Internal IC - Filter'
 * '<S257>' : 'pid_control_V1/PID VELOCIDAD/I Gain/Internal Parameters'
 * '<S258>' : 'pid_control_V1/PID VELOCIDAD/Ideal P Gain/Passthrough'
 * '<S259>' : 'pid_control_V1/PID VELOCIDAD/Ideal P Gain Fdbk/Disabled'
 * '<S260>' : 'pid_control_V1/PID VELOCIDAD/Integrator/Continuous'
 * '<S261>' : 'pid_control_V1/PID VELOCIDAD/Integrator ICs/Internal IC'
 * '<S262>' : 'pid_control_V1/PID VELOCIDAD/N Copy/Disabled'
 * '<S263>' : 'pid_control_V1/PID VELOCIDAD/N Gain/Internal Parameters'
 * '<S264>' : 'pid_control_V1/PID VELOCIDAD/P Copy/Disabled'
 * '<S265>' : 'pid_control_V1/PID VELOCIDAD/Parallel P Gain/Internal Parameters'
 * '<S266>' : 'pid_control_V1/PID VELOCIDAD/Reset Signal/Disabled'
 * '<S267>' : 'pid_control_V1/PID VELOCIDAD/Saturation/Enabled'
 * '<S268>' : 'pid_control_V1/PID VELOCIDAD/Saturation Fdbk/Disabled'
 * '<S269>' : 'pid_control_V1/PID VELOCIDAD/Sum/Sum_PID'
 * '<S270>' : 'pid_control_V1/PID VELOCIDAD/Sum Fdbk/Disabled'
 * '<S271>' : 'pid_control_V1/PID VELOCIDAD/Tracking Mode/Disabled'
 * '<S272>' : 'pid_control_V1/PID VELOCIDAD/Tracking Mode Sum/Passthrough'
 * '<S273>' : 'pid_control_V1/PID VELOCIDAD/Tsamp - Integral/TsSignalSpecification'
 * '<S274>' : 'pid_control_V1/PID VELOCIDAD/Tsamp - Ngain/Passthrough'
 * '<S275>' : 'pid_control_V1/PID VELOCIDAD/postSat Signal/Forward_Path'
 * '<S276>' : 'pid_control_V1/PID VELOCIDAD/preInt Signal/Internal PreInt'
 * '<S277>' : 'pid_control_V1/PID VELOCIDAD/preSat Signal/Forward_Path'
 * '<S278>' : 'pid_control_V1/SUBSYSTEM_MODEL/Band-Limited White Noise'
 * '<S279>' : 'pid_control_V1/SUBSYSTEM_MODEL/Compare To Constant'
 * '<S280>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))'
 * '<S281>' : 'pid_control_V1/SUBSYSTEM_MODEL/MATLAB Function - MODEL'
 * '<S282>' : 'pid_control_V1/SUBSYSTEM_MODEL/MATLAB Function-reset'
 * '<S283>' : 'pid_control_V1/SUBSYSTEM_MODEL/Subscribe1_TURBULENCIA1'
 * '<S284>' : 'pid_control_V1/SUBSYSTEM_MODEL/Subscribe_HEAVE'
 * '<S285>' : 'pid_control_V1/SUBSYSTEM_MODEL/Subscribe_RATE'
 * '<S286>' : 'pid_control_V1/SUBSYSTEM_MODEL/Subscribe_TURBULENCIA'
 * '<S287>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Angle Conversion'
 * '<S288>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Filters on angular rates'
 * '<S289>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Filters on velocities'
 * '<S290>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Length Conversion'
 * '<S291>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Length Conversion1'
 * '<S292>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/RMS turbulence  intensities'
 * '<S293>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Select angular rates'
 * '<S294>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Select velocities'
 * '<S295>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Turbulence scale lengths'
 * '<S296>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Velocity Conversion'
 * '<S297>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Velocity Conversion2'
 * '<S298>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/White Noise'
 * '<S299>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Filters on angular rates/Hpgw'
 * '<S300>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Filters on angular rates/Hqgw'
 * '<S301>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Filters on angular rates/Hrgw'
 * '<S302>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Filters on velocities/Hugw(s)'
 * '<S303>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Filters on velocities/Hvgw(s)'
 * '<S304>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Filters on velocities/Hwgw(s)'
 * '<S305>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/RMS turbulence  intensities/High Altitude Intensity'
 * '<S306>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/RMS turbulence  intensities/Low Altitude Intensity'
 * '<S307>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Select angular rates/Interpolate  rates'
 * '<S308>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Select angular rates/Low altitude  rates'
 * '<S309>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Select angular rates/Medium//High  altitude rates'
 * '<S310>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Select angular rates/Merge Subsystems'
 * '<S311>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Select angular rates/Interpolate  rates/wind to body transformation'
 * '<S312>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Select angular rates/Interpolate  rates/wind to body transformation/convert to earth coords'
 * '<S313>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Select angular rates/Low altitude  rates/wind to body transformation'
 * '<S314>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Select angular rates/Low altitude  rates/wind to body transformation/convert to earth coords'
 * '<S315>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Select velocities/Interpolate  velocities'
 * '<S316>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Select velocities/Low altitude  velocities'
 * '<S317>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Select velocities/Medium//High  altitude velocities'
 * '<S318>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Select velocities/Merge Subsystems'
 * '<S319>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Select velocities/Interpolate  velocities/wind to body transformation'
 * '<S320>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Select velocities/Interpolate  velocities/wind to body transformation/convert to earth coords'
 * '<S321>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Select velocities/Low altitude  velocities/wind to body transformation'
 * '<S322>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Select velocities/Low altitude  velocities/wind to body transformation/convert to earth coords'
 * '<S323>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Turbulence scale lengths/Low altitude scale length'
 * '<S324>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Turbulence scale lengths/Medium//High altitude scale length'
 * '<S325>' : 'pid_control_V1/SUBSYSTEM_MODEL/Dryden Wind Turbulence Model  (Continuous (-q +r))/Turbulence scale lengths/Medium//High altitude scale length/Length Conversion'
 * '<S326>' : 'pid_control_V1/SUBSYSTEM_MODEL/Subscribe1_TURBULENCIA1/Enabled Subsystem'
 * '<S327>' : 'pid_control_V1/SUBSYSTEM_MODEL/Subscribe_HEAVE/Enabled Subsystem'
 * '<S328>' : 'pid_control_V1/SUBSYSTEM_MODEL/Subscribe_RATE/Enabled Subsystem'
 * '<S329>' : 'pid_control_V1/SUBSYSTEM_MODEL/Subscribe_TURBULENCIA/Enabled Subsystem'
 * '<S330>' : 'pid_control_V1/Subscribe-ALTURA1/Enabled Subsystem'
 * '<S331>' : 'pid_control_V1/Subscribe-ROLL/Enabled Subsystem'
 * '<S332>' : 'pid_control_V1/Subscribe-YAW/Enabled Subsystem'
 */
#endif                                 /* pid_control_V1_h_ */
