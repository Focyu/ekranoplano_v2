/*
 * pid_control_V1.cpp
 *
 * Trial License - for use to evaluate programs for possible purchase as
 * an end-user only.
 *
 * Code generation for model "pid_control_V1".
 *
 * Model version              : 12.132
 * Simulink Coder version : 25.2 (R2025b) 28-Jul-2025
 * C++ source code generated on : Wed Apr 15 00:50:02 2026
 *
 * Target selection: ert.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Intel->x86-64 (Linux 64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "pid_control_V1.h"
#include "rtwtypes.h"
#include "pid_control_V1_types.h"
#include <string.h>
#include "pid_control_V1_private.h"
#include <math.h>
#include <emmintrin.h>

extern "C"
{

#include "rt_nonfinite.h"

}

#include "rmw/qos_profiles.h"
#include <stddef.h>
#include "zero_crossing_types.h"
#include "rt_defines.h"

uint32_T plook_bincpa(real_T u, const real_T bp[], uint32_T maxIndex, real_T
                      *fraction, uint32_T *prevIndex)
{
  uint32_T bpIndex;

  /* Prelookup - Index and Fraction
     Index Search method: 'binary'
     Extrapolation method: 'Clip'
     Use previous index: 'on'
     Use last breakpoint for index at or above upper limit: 'on'
     Remove protection against out-of-range input in generated code: 'off'
   */
  if (u <= bp[0U]) {
    bpIndex = 0U;
    *fraction = 0.0;
  } else if (u < bp[maxIndex]) {
    bpIndex = binsearch_u32d_prevIdx(u, bp, *prevIndex, maxIndex);
    *fraction = (u - bp[bpIndex]) / (bp[bpIndex + 1U] - bp[bpIndex]);
  } else {
    bpIndex = maxIndex;
    *fraction = 0.0;
  }

  *prevIndex = bpIndex;
  return bpIndex;
}

real_T intrp2d_la_pw(const uint32_T bpIndex[], const real_T frac[], const real_T
                     table[], const uint32_T stride, const uint32_T maxIndex[])
{
  real_T y;
  real_T yL_0d0;
  uint32_T offset_1d;

  /* Column-major Interpolation 2-D
     Interpolation method: 'Linear point-slope'
     Use last breakpoint for index at or above upper limit: 'on'
     Overflow mode: 'portable wrapping'
   */
  offset_1d = bpIndex[1U] * stride + bpIndex[0U];
  if (bpIndex[0U] == maxIndex[0U]) {
    y = table[offset_1d];
  } else {
    yL_0d0 = table[offset_1d];
    y = (table[offset_1d + 1U] - yL_0d0) * frac[0U] + yL_0d0;
  }

  if (bpIndex[1U] == maxIndex[1U]) {
  } else {
    offset_1d += stride;
    if (bpIndex[0U] == maxIndex[0U]) {
      yL_0d0 = table[offset_1d];
    } else {
      yL_0d0 = table[offset_1d];
      yL_0d0 += (table[offset_1d + 1U] - yL_0d0) * frac[0U];
    }

    y += (yL_0d0 - y) * frac[1U];
  }

  return y;
}

uint32_T binsearch_u32d_prevIdx(real_T u, const real_T bp[], uint32_T startIndex,
  uint32_T maxIndex)
{
  uint32_T bpIndex;
  uint32_T found;
  uint32_T iLeft;
  uint32_T iRght;

  /* Binary Search using Previous Index */
  bpIndex = startIndex;
  iLeft = 0U;
  iRght = maxIndex;
  found = 0U;
  while (found == 0U) {
    if (u < bp[bpIndex]) {
      iRght = bpIndex - 1U;
      bpIndex = ((bpIndex + iLeft) - 1U) >> 1U;
    } else if (u < bp[bpIndex + 1U]) {
      found = 1U;
    } else {
      iLeft = bpIndex + 1U;
      bpIndex = ((bpIndex + iRght) + 1U) >> 1U;
    }
  }

  return bpIndex;
}

/*
 * This function updates continuous states using the ODE4 fixed-step
 * solver algorithm
 */
void pid_control_V1::rt_ertODEUpdateContinuousStates(RTWSolverInfo *si )
{
  time_T t = rtsiGetT(si);
  time_T tnew = rtsiGetSolverStopTime(si);
  time_T h = rtsiGetStepSize(si);
  real_T *x = rtsiGetContStates(si);
  ODE4_IntgData *id = static_cast<ODE4_IntgData *>(rtsiGetSolverData(si));
  real_T *y = id->y;
  real_T *f0 = id->f[0];
  real_T *f1 = id->f[1];
  real_T *f2 = id->f[2];
  real_T *f3 = id->f[3];
  real_T temp;
  int_T i;
  int_T nXc = 42;
  rtsiSetSimTimeStep(si,MINOR_TIME_STEP);

  /* Save the state values at time t in y, we'll use x as ynew. */
  (void) memcpy(y, x,
                static_cast<uint_T>(nXc)*sizeof(real_T));

  /* Assumes that rtsiSetT and ModelOutputs are up-to-date */
  /* f0 = f(t,y) */
  rtsiSetdX(si, f0);
  pid_control_V1_derivatives();

  /* f1 = f(t + (h/2), y + (h/2)*f0) */
  temp = 0.5 * h;
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (temp*f0[i]);
  }

  rtsiSetT(si, t + temp);
  rtsiSetdX(si, f1);
  this->step();
  pid_control_V1_derivatives();

  /* f2 = f(t + (h/2), y + (h/2)*f1) */
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (temp*f1[i]);
  }

  rtsiSetdX(si, f2);
  this->step();
  pid_control_V1_derivatives();

  /* f3 = f(t + h, y + h*f2) */
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + (h*f2[i]);
  }

  rtsiSetT(si, tnew);
  rtsiSetdX(si, f3);
  this->step();
  pid_control_V1_derivatives();

  /* tnew = t + h
     ynew = y + (h/6)*(f0 + 2*f1 + 2*f2 + 2*f3) */
  temp = h / 6.0;
  for (i = 0; i < nXc; i++) {
    x[i] = y[i] + temp*(f0[i] + 2.0*f1[i] + 2.0*f2[i] + f3[i]);
  }

  rtsiSetSimTimeStep(si,MAJOR_TIME_STEP);
}

/*
 * System initialize for enable system:
 *    '<S284>/Enabled Subsystem'
 *    '<S287>/Enabled Subsystem'
 */
void pid_control_V1::pid_contr_EnabledSubsystem_Init
  (B_EnabledSubsystem_pid_contro_T *localB)
{
  /* SystemInitialize for SignalConversion generated from: '<S327>/In1' */
  memset(&localB->In1, 0, sizeof(SL_Bus_std_msgs_Bool));
}

/*
 * Output and update for enable system:
 *    '<S284>/Enabled Subsystem'
 *    '<S287>/Enabled Subsystem'
 */
void pid_control_V1::pid_control_V1_EnabledSubsystem(boolean_T rtu_Enable, const
  SL_Bus_std_msgs_Bool *rtu_In1, B_EnabledSubsystem_pid_contro_T *localB)
{
  /* Outputs for Enabled SubSystem: '<S284>/Enabled Subsystem' incorporates:
   *  EnablePort: '<S327>/Enable'
   */
  if (rtu_Enable) {
    /* SignalConversion generated from: '<S327>/In1' */
    localB->In1 = *rtu_In1;
  }

  /* End of Outputs for SubSystem: '<S284>/Enabled Subsystem' */
}

/*
 * System initialize for enable system:
 *    '<S285>/Enabled Subsystem'
 *    '<S286>/Enabled Subsystem'
 *    '<S12>/Enabled Subsystem'
 *    '<S13>/Enabled Subsystem'
 *    '<S14>/Enabled Subsystem'
 */
void pid_control_V1::pid_con_EnabledSubsystem_i_Init
  (B_EnabledSubsystem_pid_cont_d_T *localB)
{
  /* SystemInitialize for SignalConversion generated from: '<S328>/In1' */
  memset(&localB->In1, 0, sizeof(SL_Bus_std_msgs_Float64));
}

/*
 * Output and update for enable system:
 *    '<S285>/Enabled Subsystem'
 *    '<S286>/Enabled Subsystem'
 *    '<S12>/Enabled Subsystem'
 *    '<S13>/Enabled Subsystem'
 *    '<S14>/Enabled Subsystem'
 */
void pid_control_V1::pid_control__EnabledSubsystem_k(boolean_T rtu_Enable, const
  SL_Bus_std_msgs_Float64 *rtu_In1, B_EnabledSubsystem_pid_cont_d_T *localB)
{
  /* Outputs for Enabled SubSystem: '<S285>/Enabled Subsystem' incorporates:
   *  EnablePort: '<S328>/Enable'
   */
  if (rtu_Enable) {
    /* SignalConversion generated from: '<S328>/In1' */
    localB->In1 = *rtu_In1;
  }

  /* End of Outputs for SubSystem: '<S285>/Enabled Subsystem' */
}

void pid_control_V1::pid_c_Subscriber_setupImpl_onhg(const
  ros_slros2_internal_block_Sub_T *obj)
{
  rmw_qos_profile_t qos_profile;
  sJ4ih70VmKcvCeguWN0mNVF deadline;
  sJ4ih70VmKcvCeguWN0mNVF lifespan;
  sJ4ih70VmKcvCeguWN0mNVF liveliness_lease_duration;
  static const char_T b_zeroDelimTopic[17] = "/setpoint/altura";
  qos_profile = rmw_qos_profile_default;

  /* Start for MATLABSystem: '<S12>/SourceBlock' */
  deadline.sec = 0.0;
  deadline.nsec = 0.0;
  lifespan.sec = 0.0;
  lifespan.nsec = 0.0;
  liveliness_lease_duration.sec = 0.0;
  liveliness_lease_duration.nsec = 0.0;
  SET_QOS_VALUES(qos_profile, RMW_QOS_POLICY_HISTORY_KEEP_LAST, (size_t)10.0,
                 RMW_QOS_POLICY_DURABILITY_VOLATILE,
                 RMW_QOS_POLICY_RELIABILITY_RELIABLE, deadline, lifespan,
                 RMW_QOS_POLICY_LIVELINESS_AUTOMATIC, liveliness_lease_duration,
                 (bool)obj->QOSAvoidROSNamespaceConventions);
  for (int32_T i = 0; i < 17; i++) {
    /* Start for MATLABSystem: '<S12>/SourceBlock' */
    pid_control_V1_B.b_zeroDelimTopic_b[i] = b_zeroDelimTopic[i];
  }

  Sub_pid_control_V1_435.createSubscriber(&pid_control_V1_B.b_zeroDelimTopic_b[0],
    qos_profile);
}

void pid_control_V1::pid__Subscriber_setupImpl_onhgd(const
  ros_slros2_internal_block_Sub_T *obj)
{
  rmw_qos_profile_t qos_profile;
  sJ4ih70VmKcvCeguWN0mNVF lifespan;
  sJ4ih70VmKcvCeguWN0mNVF liveliness_lease_duration;
  char_T b_zeroDelimTopic[15];
  static const char_T b_zeroDelimTopic_0[15] = "/setpoint/roll";
  qos_profile = rmw_qos_profile_default;

  /* Start for MATLABSystem: '<S13>/SourceBlock' */
  pid_control_V1_B.deadline.sec = 0.0;
  pid_control_V1_B.deadline.nsec = 0.0;
  lifespan.sec = 0.0;
  lifespan.nsec = 0.0;
  liveliness_lease_duration.sec = 0.0;
  liveliness_lease_duration.nsec = 0.0;
  SET_QOS_VALUES(qos_profile, RMW_QOS_POLICY_HISTORY_KEEP_LAST, (size_t)10.0,
                 RMW_QOS_POLICY_DURABILITY_VOLATILE,
                 RMW_QOS_POLICY_RELIABILITY_RELIABLE, pid_control_V1_B.deadline,
                 lifespan, RMW_QOS_POLICY_LIVELINESS_AUTOMATIC,
                 liveliness_lease_duration, (bool)
                 obj->QOSAvoidROSNamespaceConventions);
  for (int32_T i = 0; i < 15; i++) {
    /* Start for MATLABSystem: '<S13>/SourceBlock' */
    b_zeroDelimTopic[i] = b_zeroDelimTopic_0[i];
  }

  Sub_pid_control_V1_538.createSubscriber(&b_zeroDelimTopic[0], qos_profile);
}

void pid_control_V1::pid_Subscriber_setupImpl_onhgd0(const
  ros_slros2_internal_block_Sub_T *obj)
{
  rmw_qos_profile_t qos_profile;
  sJ4ih70VmKcvCeguWN0mNVF lifespan;
  sJ4ih70VmKcvCeguWN0mNVF liveliness_lease_duration;
  char_T b_zeroDelimTopic[14];
  static const char_T b_zeroDelimTopic_0[14] = "/setpoint/yaw";
  qos_profile = rmw_qos_profile_default;

  /* Start for MATLABSystem: '<S14>/SourceBlock' */
  pid_control_V1_B.deadline_p.sec = 0.0;
  pid_control_V1_B.deadline_p.nsec = 0.0;
  lifespan.sec = 0.0;
  lifespan.nsec = 0.0;
  liveliness_lease_duration.sec = 0.0;
  liveliness_lease_duration.nsec = 0.0;
  SET_QOS_VALUES(qos_profile, RMW_QOS_POLICY_HISTORY_KEEP_LAST, (size_t)10.0,
                 RMW_QOS_POLICY_DURABILITY_VOLATILE,
                 RMW_QOS_POLICY_RELIABILITY_RELIABLE,
                 pid_control_V1_B.deadline_p, lifespan,
                 RMW_QOS_POLICY_LIVELINESS_AUTOMATIC, liveliness_lease_duration,
                 (bool)obj->QOSAvoidROSNamespaceConventions);
  for (int32_T i = 0; i < 14; i++) {
    /* Start for MATLABSystem: '<S14>/SourceBlock' */
    b_zeroDelimTopic[i] = b_zeroDelimTopic_0[i];
  }

  Sub_pid_control_V1_377.createSubscriber(&b_zeroDelimTopic[0], qos_profile);
}

void pid_control_V1::pid_con_ServiceCaller_setupImpl(const
  ros_slros2_internal_block_Ser_T *obj)
{
  rmw_qos_profile_t qos_profile;
  sJ4ih70VmKcvCeguWN0mNVF deadline;
  sJ4ih70VmKcvCeguWN0mNVF lifespan;
  sJ4ih70VmKcvCeguWN0mNVF liveliness_lease_duration;
  static const char_T b_zeroDelimTopic[25] = "/gazebo/set_entity_state";
  qos_profile = rmw_qos_profile_default;

  /* Start for MATLABSystem: '<S2>/ServiceCaller' */
  deadline.sec = 0.0;
  deadline.nsec = 0.0;
  lifespan.sec = 0.0;
  lifespan.nsec = 0.0;
  liveliness_lease_duration.sec = 0.0;
  liveliness_lease_duration.nsec = 0.0;
  SET_QOS_VALUES(qos_profile, RMW_QOS_POLICY_HISTORY_KEEP_LAST, (size_t)1.0,
                 RMW_QOS_POLICY_DURABILITY_VOLATILE,
                 RMW_QOS_POLICY_RELIABILITY_RELIABLE, deadline, lifespan,
                 RMW_QOS_POLICY_LIVELINESS_AUTOMATIC, liveliness_lease_duration,
                 (bool)obj->QOSAvoidROSNamespaceConventions);
  for (int32_T i = 0; i < 25; i++) {
    /* Start for MATLABSystem: '<S2>/ServiceCaller' */
    pid_control_V1_B.b_zeroDelimTopic[i] = b_zeroDelimTopic[i];
  }

  ServCall_pid_control_V1_326.createServiceCaller
    (&pid_control_V1_B.b_zeroDelimTopic[0], qos_profile);
}

void pid_control_V1::pid_co_Subscriber_setupImpl_onh(const
  ros_slros2_internal_block_Sub_T *obj)
{
  rmw_qos_profile_t qos_profile;
  sJ4ih70VmKcvCeguWN0mNVF deadline;
  sJ4ih70VmKcvCeguWN0mNVF lifespan;
  sJ4ih70VmKcvCeguWN0mNVF liveliness_lease_duration;
  static const char_T b_zeroDelimTopic[22] = "/setpoint/turbulencia";
  qos_profile = rmw_qos_profile_default;

  /* Start for MATLABSystem: '<S287>/SourceBlock' */
  deadline.sec = 0.0;
  deadline.nsec = 0.0;
  lifespan.sec = 0.0;
  lifespan.nsec = 0.0;
  liveliness_lease_duration.sec = 0.0;
  liveliness_lease_duration.nsec = 0.0;
  SET_QOS_VALUES(qos_profile, RMW_QOS_POLICY_HISTORY_KEEP_LAST, (size_t)10.0,
                 RMW_QOS_POLICY_DURABILITY_VOLATILE,
                 RMW_QOS_POLICY_RELIABILITY_RELIABLE, deadline, lifespan,
                 RMW_QOS_POLICY_LIVELINESS_AUTOMATIC, liveliness_lease_duration,
                 (bool)obj->QOSAvoidROSNamespaceConventions);
  for (int32_T i = 0; i < 22; i++) {
    /* Start for MATLABSystem: '<S287>/SourceBlock' */
    pid_control_V1_B.b_zeroDelimTopic_k[i] = b_zeroDelimTopic[i];
  }

  Sub_pid_control_V1_417.createSubscriber(&pid_control_V1_B.b_zeroDelimTopic_k[0],
    qos_profile);
}

void pid_control_V1::pid_contro_Subscriber_setupImpl(const
  ros_slros2_internal_block_Sub_T *obj)
{
  rmw_qos_profile_t qos_profile;
  sJ4ih70VmKcvCeguWN0mNVF deadline;
  sJ4ih70VmKcvCeguWN0mNVF lifespan;
  sJ4ih70VmKcvCeguWN0mNVF liveliness_lease_duration;
  static const char_T b_zeroDelimTopic[22] = "/setpoint/turbulencia";
  qos_profile = rmw_qos_profile_default;

  /* Start for MATLABSystem: '<S284>/SourceBlock' */
  deadline.sec = 0.0;
  deadline.nsec = 0.0;
  lifespan.sec = 0.0;
  lifespan.nsec = 0.0;
  liveliness_lease_duration.sec = 0.0;
  liveliness_lease_duration.nsec = 0.0;
  SET_QOS_VALUES(qos_profile, RMW_QOS_POLICY_HISTORY_KEEP_LAST, (size_t)10.0,
                 RMW_QOS_POLICY_DURABILITY_VOLATILE,
                 RMW_QOS_POLICY_RELIABILITY_RELIABLE, deadline, lifespan,
                 RMW_QOS_POLICY_LIVELINESS_AUTOMATIC, liveliness_lease_duration,
                 (bool)obj->QOSAvoidROSNamespaceConventions);
  for (int32_T i = 0; i < 22; i++) {
    /* Start for MATLABSystem: '<S284>/SourceBlock' */
    pid_control_V1_B.b_zeroDelimTopic_c[i] = b_zeroDelimTopic[i];
  }

  Sub_pid_control_V1_423.createSubscriber(&pid_control_V1_B.b_zeroDelimTopic_c[0],
    qos_profile);
}

void pid_control_V1::pid_cont_Subscriber_setupImpl_o(const
  ros_slros2_internal_block_Sub_T *obj)
{
  rmw_qos_profile_t qos_profile;
  sJ4ih70VmKcvCeguWN0mNVF lifespan;
  sJ4ih70VmKcvCeguWN0mNVF liveliness_lease_duration;
  char_T b_zeroDelimTopic[12];
  static const char_T b_zeroDelimTopic_0[12] = "/olas/heave";
  qos_profile = rmw_qos_profile_default;

  /* Start for MATLABSystem: '<S285>/SourceBlock' */
  pid_control_V1_B.deadline_c.sec = 0.0;
  pid_control_V1_B.deadline_c.nsec = 0.0;
  lifespan.sec = 0.0;
  lifespan.nsec = 0.0;
  liveliness_lease_duration.sec = 0.0;
  liveliness_lease_duration.nsec = 0.0;
  SET_QOS_VALUES(qos_profile, RMW_QOS_POLICY_HISTORY_KEEP_LAST, (size_t)10.0,
                 RMW_QOS_POLICY_DURABILITY_VOLATILE,
                 RMW_QOS_POLICY_RELIABILITY_RELIABLE,
                 pid_control_V1_B.deadline_c, lifespan,
                 RMW_QOS_POLICY_LIVELINESS_AUTOMATIC, liveliness_lease_duration,
                 (bool)obj->QOSAvoidROSNamespaceConventions);
  for (int32_T i = 0; i < 12; i++) {
    /* Start for MATLABSystem: '<S285>/SourceBlock' */
    b_zeroDelimTopic[i] = b_zeroDelimTopic_0[i];
  }

  Sub_pid_control_V1_443.createSubscriber(&b_zeroDelimTopic[0], qos_profile);
}

void pid_control_V1::pid_con_Subscriber_setupImpl_on(const
  ros_slros2_internal_block_Sub_T *obj)
{
  rmw_qos_profile_t qos_profile;
  sJ4ih70VmKcvCeguWN0mNVF deadline;
  sJ4ih70VmKcvCeguWN0mNVF lifespan;
  sJ4ih70VmKcvCeguWN0mNVF liveliness_lease_duration;
  static const char_T b_zeroDelimTopic[17] = "/olas/pitch_rate";
  qos_profile = rmw_qos_profile_default;

  /* Start for MATLABSystem: '<S286>/SourceBlock' */
  deadline.sec = 0.0;
  deadline.nsec = 0.0;
  lifespan.sec = 0.0;
  lifespan.nsec = 0.0;
  liveliness_lease_duration.sec = 0.0;
  liveliness_lease_duration.nsec = 0.0;
  SET_QOS_VALUES(qos_profile, RMW_QOS_POLICY_HISTORY_KEEP_LAST, (size_t)10.0,
                 RMW_QOS_POLICY_DURABILITY_VOLATILE,
                 RMW_QOS_POLICY_RELIABILITY_RELIABLE, deadline, lifespan,
                 RMW_QOS_POLICY_LIVELINESS_AUTOMATIC, liveliness_lease_duration,
                 (bool)obj->QOSAvoidROSNamespaceConventions);
  for (int32_T i = 0; i < 17; i++) {
    /* Start for MATLABSystem: '<S286>/SourceBlock' */
    pid_control_V1_B.b_zeroDelimTopic_cx[i] = b_zeroDelimTopic[i];
  }

  Sub_pid_control_V1_445.createSubscriber(&pid_control_V1_B.b_zeroDelimTopic_cx
    [0], qos_profile);
}

real_T rt_urand_Upu32_Yd_f_pw_snf(uint32_T *u)
{
  uint32_T hi;
  uint32_T lo;

  /* Uniform random number generator (random number between 0 and 1)

     #define IA      16807                      magic multiplier = 7^5
     #define IM      2147483647                 modulus = 2^31-1
     #define IQ      127773                     IM div IA
     #define IR      2836                       IM modulo IA
     #define S       4.656612875245797e-10      reciprocal of 2^31-1
     test = IA * (seed % IQ) - IR * (seed/IQ)
     seed = test < 0 ? (test + IM) : test
     return (seed*S)
   */
  lo = *u % 127773U * 16807U;
  hi = *u / 127773U * 2836U;
  if (lo < hi) {
    *u = 2147483647U - (hi - lo);
  } else {
    *u = lo - hi;
  }

  return static_cast<real_T>(*u) * 4.6566128752457969E-10;
}

real_T rt_nrand_Upu32_Yd_f_pw_snf(uint32_T *u)
{
  real_T si;
  real_T sr;
  real_T y;

  /* Normal (Gaussian) random number generator */
  do {
    sr = 2.0 * rt_urand_Upu32_Yd_f_pw_snf(u) - 1.0;
    si = 2.0 * rt_urand_Upu32_Yd_f_pw_snf(u) - 1.0;
    si = sr * sr + si * si;
  } while (si > 1.0);

  y = sqrt(-2.0 * log(si) / si) * sr;
  return y;
}

real_T rt_atan2d_snf(real_T u0, real_T u1)
{
  real_T y;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = (rtNaN);
  } else if (rtIsInf(u0) && rtIsInf(u1)) {
    int32_T tmp;
    int32_T tmp_0;
    if (u0 > 0.0) {
      tmp = 1;
    } else {
      tmp = -1;
    }

    if (u1 > 0.0) {
      tmp_0 = 1;
    } else {
      tmp_0 = -1;
    }

    y = atan2(static_cast<real_T>(tmp), static_cast<real_T>(tmp_0));
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(RT_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = atan2(u0, u1);
  }

  return y;
}

real_T rt_powd_snf(real_T u0, real_T u1)
{
  real_T y;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = (rtNaN);
  } else {
    real_T tmp;
    real_T tmp_0;
    tmp = fabs(u0);
    tmp_0 = fabs(u1);
    if (rtIsInf(u1)) {
      if (tmp == 1.0) {
        y = 1.0;
      } else if (tmp > 1.0) {
        if (u1 > 0.0) {
          y = (rtInf);
        } else {
          y = 0.0;
        }
      } else if (u1 > 0.0) {
        y = 0.0;
      } else {
        y = (rtInf);
      }
    } else if (tmp_0 == 0.0) {
      y = 1.0;
    } else if (tmp_0 == 1.0) {
      if (u1 > 0.0) {
        y = u0;
      } else {
        y = 1.0 / u0;
      }
    } else if (u1 == 2.0) {
      y = u0 * u0;
    } else if ((u1 == 0.5) && (u0 >= 0.0)) {
      y = sqrt(u0);
    } else if ((u0 < 0.0) && (u1 > floor(u1))) {
      y = (rtNaN);
    } else {
      y = pow(u0, u1);
    }
  }

  return y;
}

/* Model step function */
void pid_control_V1::step()
{
  /* local block i/o variables */
  SL_Bus_std_msgs_Float64 rtb_SourceBlock_o2_o;
  SL_Bus_std_msgs_Float64 rtb_SourceBlock_o2_d;
  SL_Bus_std_msgs_Bool rtb_SourceBlock_o2_j;
  SL_Bus_std_msgs_Bool rtb_SourceBlock_o2_dd;
  __m128d tmp_3;
  SL_Bus_gazebo_msgs_SetEntityStateResponse tmp;
  int32_T i;
  int32_T tmp_2;
  int8_T rtAction;
  int8_T rtPrevAction;
  boolean_T serverAvailableOnTime;
  boolean_T tmp_0;
  boolean_T tmp_1;
  static const uint8_T b[11] = { 101U, 107U, 114U, 97U, 110U, 111U, 112U, 108U,
    97U, 110U, 111U };

  static const uint8_T b_0[5] = { 119U, 111U, 114U, 108U, 100U };

  if (rtmIsMajorTimeStep((&pid_control_V1_M))) {
    /* set solver stop time */
    if (!((&pid_control_V1_M)->Timing.clockTick0+1)) {
      rtsiSetSolverStopTime(&(&pid_control_V1_M)->solverInfo,
                            (((&pid_control_V1_M)->Timing.clockTickH0 + 1) *
        (&pid_control_V1_M)->Timing.stepSize0 * 4294967296.0));
    } else {
      rtsiSetSolverStopTime(&(&pid_control_V1_M)->solverInfo,
                            (((&pid_control_V1_M)->Timing.clockTick0 + 1) *
        (&pid_control_V1_M)->Timing.stepSize0 + (&pid_control_V1_M)
        ->Timing.clockTickH0 * (&pid_control_V1_M)->Timing.stepSize0 *
        4294967296.0));
    }
  }                                    /* end MajorTimeStep */

  /* Update absolute time of base rate at minor time step */
  if (rtmIsMinorTimeStep((&pid_control_V1_M))) {
    (&pid_control_V1_M)->Timing.t[0] = rtsiGetT(&(&pid_control_V1_M)->solverInfo);
  }

  /* RelationalOperator: '<S280>/Compare' incorporates:
   *  Constant: '<S280>/Constant'
   */
  pid_control_V1_B.Compare = (pid_control_V1_X.Integrator_CSTATE[11] >= 0.05);

  /* Outputs for Enabled SubSystem: '<S289>/Hrgw' incorporates:
   *  EnablePort: '<S302>/Enable'
   */
  /* Outputs for Enabled SubSystem: '<S289>/Hqgw' incorporates:
   *  EnablePort: '<S301>/Enable'
   */
  /* Outputs for Enabled SubSystem: '<S289>/Hpgw' incorporates:
   *  EnablePort: '<S300>/Enable'
   */
  /* Outputs for Enabled SubSystem: '<S290>/Hwgw(s)' incorporates:
   *  EnablePort: '<S305>/Enable'
   */
  /* Outputs for Enabled SubSystem: '<S290>/Hvgw(s)' incorporates:
   *  EnablePort: '<S304>/Enable'
   */
  tmp_0 = rtmIsMajorTimeStep((&pid_control_V1_M));

  /* End of Outputs for SubSystem: '<S290>/Hvgw(s)' */
  /* End of Outputs for SubSystem: '<S290>/Hwgw(s)' */
  /* End of Outputs for SubSystem: '<S289>/Hpgw' */
  /* End of Outputs for SubSystem: '<S289>/Hqgw' */
  /* End of Outputs for SubSystem: '<S289>/Hrgw' */
  if (tmp_0) {
    /* MATLAB Function: '<S11>/MATLAB Function-reset' incorporates:
     *  Memory: '<S11>/Memory2'
     */
    memcpy(&pid_control_V1_B.IC[0], &pid_control_V1_DW.Memory2_PreviousInput[0],
           12U * sizeof(real_T));
    pid_control_V1_B.IC[2] = 0.0;
    pid_control_V1_B.IC[11] = 0.0;

    /* InitialCondition: '<S11>/IC' */
    if (pid_control_V1_DW.IC_FirstOutputTime) {
      pid_control_V1_DW.IC_FirstOutputTime = false;

      /* InitialCondition: '<S11>/IC' */
      memcpy(&pid_control_V1_B.IC[0], &pid_control_V1_ConstP.pooled10[0], 12U *
             sizeof(real_T));
    }

    /* End of InitialCondition: '<S11>/IC' */
  }

  /* Outputs for Enabled SubSystem: '<S289>/Hrgw' incorporates:
   *  EnablePort: '<S302>/Enable'
   */
  /* Outputs for Enabled SubSystem: '<S289>/Hqgw' incorporates:
   *  EnablePort: '<S301>/Enable'
   */
  /* Outputs for Enabled SubSystem: '<S289>/Hpgw' incorporates:
   *  EnablePort: '<S300>/Enable'
   */
  /* Outputs for Enabled SubSystem: '<S290>/Hwgw(s)' incorporates:
   *  EnablePort: '<S305>/Enable'
   */
  /* Outputs for Enabled SubSystem: '<S290>/Hvgw(s)' incorporates:
   *  EnablePort: '<S304>/Enable'
   */
  /* Outputs for Enabled SubSystem: '<S290>/Hugw(s)' incorporates:
   *  EnablePort: '<S303>/Enable'
   */
  /* If: '<S294>/if Height < Max low altitude  elseif Height > Min isotropic altitude ' incorporates:
   *  If: '<S295>/if Height < Max low altitude  elseif Height > Min isotropic altitude '
   *  Integrator: '<S11>/Integrator'
   *  RateLimiter: '<Root>/Rate Limiter'
   */
  tmp_1 = rtsiIsModeUpdateTimeStep(&(&pid_control_V1_M)->solverInfo);

  /* End of Outputs for SubSystem: '<S290>/Hugw(s)' */
  /* End of Outputs for SubSystem: '<S290>/Hvgw(s)' */
  /* End of Outputs for SubSystem: '<S290>/Hwgw(s)' */
  /* End of Outputs for SubSystem: '<S289>/Hpgw' */
  /* End of Outputs for SubSystem: '<S289>/Hqgw' */
  /* End of Outputs for SubSystem: '<S289>/Hrgw' */

  /* Integrator: '<S11>/Integrator' incorporates:
   *  InitialCondition: '<S11>/IC'
   */
  if (tmp_1) {
    serverAvailableOnTime = (((pid_control_V1_PrevZCX.Integrator_Reset_ZCE ==
      POS_ZCSIG) != pid_control_V1_B.Compare) &&
      (pid_control_V1_PrevZCX.Integrator_Reset_ZCE != UNINITIALIZED_ZCSIG));
    pid_control_V1_PrevZCX.Integrator_Reset_ZCE = pid_control_V1_B.Compare;

    /* evaluate zero-crossings and the level of the reset signal */
    if (serverAvailableOnTime || pid_control_V1_B.Compare ||
        pid_control_V1_DW.Integrator_DWORK1) {
      memcpy(&pid_control_V1_X.Integrator_CSTATE[0], &pid_control_V1_B.IC[0],
             12U * sizeof(real_T));
      rtsiSetBlockStateForSolverChangedAtMajorStep(&(&pid_control_V1_M)
        ->solverInfo, true);
      rtsiSetContTimeOutputInconsistentWithStateAtMajorStep(&(&pid_control_V1_M
        )->solverInfo, true);
    }
  }

  /* Integrator: '<S11>/Integrator' */
  memcpy(&pid_control_V1_B.x[0], &pid_control_V1_X.Integrator_CSTATE[0], 12U *
         sizeof(real_T));
  if (tmp_0) {
    /* MATLABSystem: '<S12>/SourceBlock' */
    pid_control_V1_B.SourceBlock_o1_o = Sub_pid_control_V1_435.getLatestMessage(
      &rtb_SourceBlock_o2_d);

    /* Outputs for Enabled SubSystem: '<S12>/Enabled Subsystem' */
    pid_control__EnabledSubsystem_k(pid_control_V1_B.SourceBlock_o1_o,
      &rtb_SourceBlock_o2_d, &pid_control_V1_B.EnabledSubsystem_b);

    /* End of Outputs for SubSystem: '<S12>/Enabled Subsystem' */

    /* UnitDelay: '<Root>/Unit Delay3' */
    pid_control_V1_B.WhiteNoise_p = pid_control_V1_DW.UnitDelay3_DSTATE;

    /* Switch: '<Root>/Switch3' */
    if (pid_control_V1_B.SourceBlock_o1_o) {
      /* Switch: '<Root>/Switch3' */
      pid_control_V1_B.Switch3 = pid_control_V1_B.EnabledSubsystem_b.In1.data;
    } else {
      /* Switch: '<Root>/Switch3' incorporates:
       *  UnitDelay: '<Root>/Unit Delay3'
       */
      pid_control_V1_B.Switch3 = pid_control_V1_DW.UnitDelay3_DSTATE;
    }

    /* End of Switch: '<Root>/Switch3' */
  }

  /* Gain: '<Root>/Gain' */
  pid_control_V1_B.Gain = -pid_control_V1_B.x[11];

  /* Sum: '<Root>/Sum2' incorporates:
   *  Gain: '<Root>/Gain4'
   */
  pid_control_V1_B.Sum2_l = pid_control_V1_B.Switch3 - (-pid_control_V1_B.x[11]);

  /* Gain: '<S106>/Filter Coefficient' incorporates:
   *  Gain: '<S96>/Derivative Gain'
   *  Integrator: '<S98>/Filter'
   *  Sum: '<S98>/SumD'
   */
  pid_control_V1_B.FilterCoefficient = (4.5 * pid_control_V1_B.Sum2_l -
    pid_control_V1_X.Filter_CSTATE) * 100.0;

  /* Sum: '<S112>/Sum' incorporates:
   *  Gain: '<S108>/Proportional Gain'
   *  Integrator: '<S103>/Integrator'
   */
  pid_control_V1_B.Sum_b = (2.0 * pid_control_V1_B.Sum2_l +
    pid_control_V1_X.Integrator_CSTATE_n) + pid_control_V1_B.FilterCoefficient;

  /* Saturate: '<S110>/Saturation' */
  if (pid_control_V1_B.Sum_b > 20.0) {
    /* Saturate: '<S110>/Saturation' */
    pid_control_V1_B.Saturation = 20.0;
  } else if (pid_control_V1_B.Sum_b < 0.0) {
    /* Saturate: '<S110>/Saturation' */
    pid_control_V1_B.Saturation = 0.0;
  } else {
    /* Saturate: '<S110>/Saturation' */
    pid_control_V1_B.Saturation = pid_control_V1_B.Sum_b;
  }

  /* End of Saturate: '<S110>/Saturation' */
  if (tmp_0) {
    /* MATLABSystem: '<S13>/SourceBlock' */
    pid_control_V1_B.SourceBlock_o1_e = Sub_pid_control_V1_538.getLatestMessage(
      &rtb_SourceBlock_o2_o);

    /* Outputs for Enabled SubSystem: '<S13>/Enabled Subsystem' */
    pid_control__EnabledSubsystem_k(pid_control_V1_B.SourceBlock_o1_e,
      &rtb_SourceBlock_o2_o, &pid_control_V1_B.EnabledSubsystem_ps);

    /* End of Outputs for SubSystem: '<S13>/Enabled Subsystem' */

    /* Switch: '<Root>/Switch1' */
    if (pid_control_V1_B.SourceBlock_o1_e) {
      /* Switch: '<Root>/Switch1' */
      pid_control_V1_B.Switch1_g = pid_control_V1_B.EnabledSubsystem_ps.In1.data;
    } else {
      /* Switch: '<Root>/Switch1' incorporates:
       *  UnitDelay: '<Root>/Unit Delay1'
       */
      pid_control_V1_B.Switch1_g = pid_control_V1_DW.UnitDelay1_DSTATE;
    }

    /* End of Switch: '<Root>/Switch1' */

    /* MATLABSystem: '<S14>/SourceBlock' */
    pid_control_V1_B.SourceBlock_o1 = Sub_pid_control_V1_377.getLatestMessage
      (&pid_control_V1_B.SourceBlock_o2);

    /* Outputs for Enabled SubSystem: '<S14>/Enabled Subsystem' */
    pid_control__EnabledSubsystem_k(pid_control_V1_B.SourceBlock_o1,
      &pid_control_V1_B.SourceBlock_o2, &pid_control_V1_B.EnabledSubsystem_a);

    /* End of Outputs for SubSystem: '<S14>/Enabled Subsystem' */

    /* UnitDelay: '<Root>/Unit Delay2' */
    pid_control_V1_B.WhiteNoise_p = pid_control_V1_DW.UnitDelay2_DSTATE;

    /* Switch: '<Root>/Switch2' */
    if (pid_control_V1_B.SourceBlock_o1) {
      /* Switch: '<Root>/Switch2' */
      pid_control_V1_B.Switch2 = pid_control_V1_B.EnabledSubsystem_a.In1.data;
    } else {
      /* Switch: '<Root>/Switch2' */
      pid_control_V1_B.Switch2 = pid_control_V1_B.WhiteNoise_p;
    }

    /* End of Switch: '<Root>/Switch2' */

    /* UnitDelay: '<Root>/Unit Delay4' */
    pid_control_V1_B.UnitDelay4 = pid_control_V1_DW.UnitDelay4_DSTATE;
  }

  /* Sum: '<Root>/Sum5' */
  pid_control_V1_B.Gain2 = pid_control_V1_B.Switch2 - pid_control_V1_B.x[8];

  /* Gain: '<Root>/Kp_Heading' */
  pid_control_V1_B.SignPreSat = 0.5 * pid_control_V1_B.Gain2;

  /* MATLAB Function: '<Root>/MATLAB Function2' */
  if (!(pid_control_V1_B.SignPreSat <= 0.3)) {
    pid_control_V1_B.SignPreSat = 0.3;
  }

  if (!(pid_control_V1_B.SignPreSat >= -0.3)) {
    pid_control_V1_B.SignPreSat = -0.3;
  }

  /* Gain: '<S48>/Integral Gain' incorporates:
   *  MATLAB Function: '<Root>/MATLAB Function2'
   *  Sum: '<Root>/Sum4'
   */
  pid_control_V1_B.Switch = atan(pid_control_V1_B.UnitDelay4 *
    pid_control_V1_B.SignPreSat / 9.81) - pid_control_V1_B.x[6];

  /* Gain: '<S54>/Filter Coefficient' incorporates:
   *  Gain: '<S44>/Derivative Gain'
   *  Integrator: '<S46>/Filter'
   *  Sum: '<S46>/SumD'
   */
  pid_control_V1_B.FilterCoefficient_c = (-0.5 * pid_control_V1_B.Switch -
    pid_control_V1_X.Filter_CSTATE_g) * 100.0;

  /* Sum: '<S60>/Sum' incorporates:
   *  Gain: '<S56>/Proportional Gain'
   *  Integrator: '<S51>/Integrator'
   */
  pid_control_V1_B.SignPreSat = (-1.2 * pid_control_V1_B.Switch +
    pid_control_V1_X.Integrator_CSTATE_m) + pid_control_V1_B.FilterCoefficient_c;

  /* Saturate: '<S58>/Saturation' */
  if (pid_control_V1_B.SignPreSat > 0.17453292519943295) {
    /* Saturate: '<S58>/Saturation' */
    pid_control_V1_B.Saturation_k = 0.17453292519943295;
  } else if (pid_control_V1_B.SignPreSat < -0.17453292519943295) {
    /* Saturate: '<S58>/Saturation' */
    pid_control_V1_B.Saturation_k = -0.17453292519943295;
  } else {
    /* Saturate: '<S58>/Saturation' */
    pid_control_V1_B.Saturation_k = pid_control_V1_B.SignPreSat;
  }

  /* End of Saturate: '<S58>/Saturation' */

  /* Saturate: '<Root>/Saturation' */
  if (pid_control_V1_B.Saturation > 0.13962634015954636) {
    /* Saturate: '<Root>/Saturation' */
    pid_control_V1_B.Saturation_i = 0.13962634015954636;
  } else if (pid_control_V1_B.Saturation < -0.13962634015954636) {
    /* Saturate: '<Root>/Saturation' */
    pid_control_V1_B.Saturation_i = -0.13962634015954636;
  } else {
    /* Saturate: '<Root>/Saturation' */
    pid_control_V1_B.Saturation_i = pid_control_V1_B.Saturation;
  }

  /* End of Saturate: '<Root>/Saturation' */

  /* RateLimiter: '<Root>/Rate Limiter' */
  if (pid_control_V1_DW.LastMajorTime == (rtInf)) {
    /* RateLimiter: '<Root>/Rate Limiter' */
    pid_control_V1_B.RateLimiter = pid_control_V1_B.Saturation_i;
  } else {
    pid_control_V1_B.Sum1_g = (&pid_control_V1_M)->Timing.t[0];
    pid_control_V1_B.SignPreSat_h = pid_control_V1_B.Sum1_g -
      pid_control_V1_DW.LastMajorTime;
    if (pid_control_V1_DW.LastMajorTime == pid_control_V1_B.Sum1_g) {
      if (pid_control_V1_DW.PrevLimited) {
        /* RateLimiter: '<Root>/Rate Limiter' */
        pid_control_V1_B.RateLimiter = pid_control_V1_DW.PrevY;
      } else {
        /* RateLimiter: '<Root>/Rate Limiter' */
        pid_control_V1_B.RateLimiter = pid_control_V1_B.Saturation_i;
      }
    } else {
      pid_control_V1_B.Sum_hl = pid_control_V1_B.SignPreSat_h * 0.06;
      pid_control_V1_B.Sum1_g = pid_control_V1_B.Saturation_i -
        pid_control_V1_DW.PrevY;
      if (pid_control_V1_B.Sum1_g > pid_control_V1_B.Sum_hl) {
        /* RateLimiter: '<Root>/Rate Limiter' */
        pid_control_V1_B.RateLimiter = pid_control_V1_DW.PrevY +
          pid_control_V1_B.Sum_hl;
        serverAvailableOnTime = true;
      } else {
        pid_control_V1_B.SignPreSat_h *= -0.06;
        if (pid_control_V1_B.Sum1_g < pid_control_V1_B.SignPreSat_h) {
          /* RateLimiter: '<Root>/Rate Limiter' */
          pid_control_V1_B.RateLimiter = pid_control_V1_DW.PrevY +
            pid_control_V1_B.SignPreSat_h;
          serverAvailableOnTime = true;
        } else {
          /* RateLimiter: '<Root>/Rate Limiter' */
          pid_control_V1_B.RateLimiter = pid_control_V1_B.Saturation_i;
          serverAvailableOnTime = false;
        }
      }

      if (tmp_1) {
        pid_control_V1_DW.PrevLimited = serverAvailableOnTime;
      }
    }
  }

  /* Sum: '<Root>/Sum1' */
  pid_control_V1_B.Sum1_g = pid_control_V1_B.RateLimiter - pid_control_V1_B.x[7];

  /* Gain: '<S158>/Filter Coefficient' incorporates:
   *  Gain: '<S148>/Derivative Gain'
   *  Integrator: '<S150>/Filter'
   *  Sum: '<S150>/SumD'
   */
  pid_control_V1_B.FilterCoefficient_m = (-0.2 * pid_control_V1_B.Sum1_g -
    pid_control_V1_X.Filter_CSTATE_m) * 15.0;

  /* Sum: '<S164>/Sum' incorporates:
   *  Gain: '<S160>/Proportional Gain'
   *  Integrator: '<S155>/Integrator'
   */
  pid_control_V1_B.Sum_hl = (-0.35 * pid_control_V1_B.Sum1_g +
    pid_control_V1_X.Integrator_CSTATE_p) + pid_control_V1_B.FilterCoefficient_m;

  /* Saturate: '<S162>/Saturation' */
  if (pid_control_V1_B.Sum_hl > 0.3490658503988659) {
    /* Saturate: '<S162>/Saturation' */
    pid_control_V1_B.Saturation_f = 0.3490658503988659;
  } else if (pid_control_V1_B.Sum_hl < -0.3490658503988659) {
    /* Saturate: '<S162>/Saturation' */
    pid_control_V1_B.Saturation_f = -0.3490658503988659;
  } else {
    /* Saturate: '<S162>/Saturation' */
    pid_control_V1_B.Saturation_f = pid_control_V1_B.Sum_hl;
  }

  /* End of Saturate: '<S162>/Saturation' */
  if (tmp_0) {
    /* Sum: '<Root>/Sum6' incorporates:
     *  Constant: '<Root>/Constant1'
     *  UnitDelay: '<Root>/Unit Delay'
     */
    pid_control_V1_B.WhiteNoise_p = 0.0 - pid_control_V1_DW.UnitDelay_DSTATE;

    /* Gain: '<S212>/Proportional Gain' incorporates:
     *  Gain: '<S200>/Derivative Gain'
     */
    pid_control_V1_B.ProportionalGain = -0.4 * pid_control_V1_B.WhiteNoise_p;

    /* Gain: '<S200>/Derivative Gain' */
    pid_control_V1_B.DerivativeGain = pid_control_V1_B.ProportionalGain;
  }

  /* Gain: '<S210>/Filter Coefficient' incorporates:
   *  Integrator: '<S202>/Filter'
   *  Sum: '<S202>/SumD'
   */
  pid_control_V1_B.FilterCoefficient_p = (pid_control_V1_B.DerivativeGain -
    pid_control_V1_X.Filter_CSTATE_f) * 100.0;

  /* Sum: '<S216>/Sum' incorporates:
   *  Integrator: '<S207>/Integrator'
   */
  pid_control_V1_B.Saturation_m = (pid_control_V1_B.ProportionalGain +
    pid_control_V1_X.Integrator_CSTATE_d) + pid_control_V1_B.FilterCoefficient_p;

  /* Saturate: '<S214>/Saturation' */
  if (pid_control_V1_B.Saturation_m > 0.26179938779914941) {
    /* Sum: '<S216>/Sum' incorporates:
     *  Saturate: '<S214>/Saturation'
     */
    pid_control_V1_B.Saturation_m = 0.26179938779914941;
  } else if (pid_control_V1_B.Saturation_m < -0.26179938779914941) {
    /* Sum: '<S216>/Sum' incorporates:
     *  Saturate: '<S214>/Saturation'
     */
    pid_control_V1_B.Saturation_m = -0.26179938779914941;
  }

  /* End of Saturate: '<S214>/Saturation' */

  /* Gain: '<S264>/Filter Coefficient' incorporates:
   *  Constant: '<Root>/Constant_U'
   *  Gain: '<S254>/Derivative Gain'
   *  Integrator: '<S256>/Filter'
   *  Sum: '<Root>/Sum3'
   *  Sum: '<S256>/SumD'
   */
  pid_control_V1_B.FilterCoefficient_cv = ((20.2 - pid_control_V1_B.x[0]) *
    0.005 - pid_control_V1_X.Filter_CSTATE_l) * 100.0;

  /* Sum: '<S270>/Sum' incorporates:
   *  Constant: '<Root>/Constant_U'
   *  Gain: '<S266>/Proportional Gain'
   *  Integrator: '<S261>/Integrator'
   *  Sum: '<Root>/Sum3'
   */
  pid_control_V1_B.SignPreSat_h = ((20.2 - pid_control_V1_B.x[0]) * 0.08 +
    pid_control_V1_X.Integrator_CSTATE_f) +
    pid_control_V1_B.FilterCoefficient_cv;

  /* Saturate: '<S268>/Saturation' */
  if (pid_control_V1_B.SignPreSat_h > 1.0) {
    pid_control_V1_B.Square1 = 1.0;
  } else if (pid_control_V1_B.SignPreSat_h < 0.0) {
    pid_control_V1_B.Square1 = 0.0;
  } else {
    pid_control_V1_B.Square1 = pid_control_V1_B.SignPreSat_h;
  }

  /* End of Saturate: '<S268>/Saturation' */

  /* Gain: '<Root>/K_diff_thrust' */
  pid_control_V1_B.Gain2 *= 0.1;

  /* Sum: '<Root>/Sum7' */
  pid_control_V1_B.Saturation1 = pid_control_V1_B.Square1 +
    pid_control_V1_B.Gain2;

  /* Saturate: '<Root>/Saturation1' */
  if (pid_control_V1_B.Saturation1 > 1.0) {
    /* Sum: '<Root>/Sum7' incorporates:
     *  Saturate: '<Root>/Saturation1'
     */
    pid_control_V1_B.Saturation1 = 1.0;
  } else if (pid_control_V1_B.Saturation1 < 0.0) {
    /* Sum: '<Root>/Sum7' incorporates:
     *  Saturate: '<Root>/Saturation1'
     */
    pid_control_V1_B.Saturation1 = 0.0;
  }

  /* End of Saturate: '<Root>/Saturation1' */

  /* Sum: '<Root>/Sum8' */
  pid_control_V1_B.Saturation2 = pid_control_V1_B.Square1 -
    pid_control_V1_B.Gain2;

  /* Saturate: '<Root>/Saturation2' */
  if (pid_control_V1_B.Saturation2 > 1.0) {
    /* Sum: '<Root>/Sum8' incorporates:
     *  Saturate: '<Root>/Saturation2'
     */
    pid_control_V1_B.Saturation2 = 1.0;
  } else if (pid_control_V1_B.Saturation2 < 0.0) {
    /* Sum: '<Root>/Sum8' incorporates:
     *  Saturate: '<Root>/Saturation2'
     */
    pid_control_V1_B.Saturation2 = 0.0;
  }

  /* End of Saturate: '<Root>/Saturation2' */
  if (tmp_0) {
    /* Memory: '<S11>/Memory' */
    pid_control_V1_B.Memory[0] = pid_control_V1_DW.Memory_PreviousInput[0];

    /* Memory: '<S11>/Memory1' */
    pid_control_V1_B.Memory1[0] = pid_control_V1_DW.Memory1_PreviousInput[0];

    /* Memory: '<S11>/Memory' */
    pid_control_V1_B.Memory[1] = pid_control_V1_DW.Memory_PreviousInput[1];

    /* Memory: '<S11>/Memory1' */
    pid_control_V1_B.Memory1[1] = pid_control_V1_DW.Memory1_PreviousInput[1];

    /* Memory: '<S11>/Memory' */
    pid_control_V1_B.Memory[2] = pid_control_V1_DW.Memory_PreviousInput[2];

    /* Memory: '<S11>/Memory1' */
    pid_control_V1_B.Memory1[2] = pid_control_V1_DW.Memory1_PreviousInput[2];
  }

  /* SignalConversion generated from: '<S282>/ SFunction ' incorporates:
   *  MATLAB Function: '<S11>/MATLAB Function - MODEL'
   */
  pid_control_V1_B.TmpSignalConversionAtSFunct[0] =
    pid_control_V1_B.Saturation_k;
  pid_control_V1_B.TmpSignalConversionAtSFunct[1] =
    pid_control_V1_B.Saturation_f;
  pid_control_V1_B.TmpSignalConversionAtSFunct[2] =
    pid_control_V1_B.Saturation_m;
  pid_control_V1_B.TmpSignalConversionAtSFunct[3] = pid_control_V1_B.Saturation1;
  pid_control_V1_B.TmpSignalConversionAtSFunct[4] = pid_control_V1_B.Saturation2;

  /* MATLAB Function: '<S11>/MATLAB Function - MODEL' incorporates:
   *  Memory: '<S11>/Memory'
   */
  if (pid_control_V1_B.TmpSignalConversionAtSFunct[1] <= 0.3490658503988659) {
    pid_control_V1_B.u2 = pid_control_V1_B.TmpSignalConversionAtSFunct[1];
  } else {
    pid_control_V1_B.u2 = 0.3490658503988659;
  }

  if (!(pid_control_V1_B.u2 >= -0.3490658503988659)) {
    pid_control_V1_B.u2 = -0.3490658503988659;
  }

  tmp_3 = _mm_add_pd(_mm_loadu_pd(&pid_control_V1_B.x[0]), _mm_loadu_pd
                     (&pid_control_V1_B.Memory[0]));
  _mm_storeu_pd(&pid_control_V1_B.dv1[0], tmp_3);

  /* MATLAB Function: '<S11>/MATLAB Function - MODEL' incorporates:
   *  Memory: '<S11>/Memory'
   *  Memory: '<S11>/Memory1'
   */
  pid_control_V1_B.Gain2 = pid_control_V1_B.x[2] + pid_control_V1_B.Memory[2];
  pid_control_V1_B.Va_out = sqrt((pid_control_V1_B.dv1[0] *
    pid_control_V1_B.dv1[0] + pid_control_V1_B.dv1[1] * pid_control_V1_B.dv1[1])
    + pid_control_V1_B.Gain2 * pid_control_V1_B.Gain2);
  if (pid_control_V1_B.Va_out < 0.5) {
    pid_control_V1_B.Va_out = 0.5;
    pid_control_V1_B.Gain2 = 0.0;
    pid_control_V1_B.Square1 = 0.0;
  } else {
    pid_control_V1_B.Gain2 = rt_atan2d_snf(pid_control_V1_B.Gain2,
      pid_control_V1_B.dv1[0]);
    pid_control_V1_B.Square1 = pid_control_V1_B.dv1[1] / pid_control_V1_B.Va_out;
    if ((pid_control_V1_B.Square1 >= 1.0) || rtIsNaN(pid_control_V1_B.Square1))
    {
      pid_control_V1_B.Square1 = 1.0;
    }

    if (pid_control_V1_B.Square1 <= -1.0) {
      pid_control_V1_B.Square1 = -1.0;
    }

    pid_control_V1_B.Square1 = asin(pid_control_V1_B.Square1);
  }

  pid_control_V1_B.q_aero = pid_control_V1_B.Memory1[1] + pid_control_V1_B.x[4];
  if ((-pid_control_V1_B.x[11] - 0.0505 <= 0.001) || rtIsNaN
      (-pid_control_V1_B.x[11] - 0.0505)) {
    pid_control_V1_B.hw = 0.001;
  } else {
    pid_control_V1_B.hw = -pid_control_V1_B.x[11] - 0.0505;
  }

  if ((-pid_control_V1_B.x[11] + 0.3475 <= 0.001) || rtIsNaN
      (-pid_control_V1_B.x[11] + 0.3475)) {
    pid_control_V1_B.hh = 0.001;
  } else {
    pid_control_V1_B.hh = -pid_control_V1_B.x[11] + 0.3475;
  }

  pid_control_V1_B.Q = pid_control_V1_B.Va_out * pid_control_V1_B.Va_out *
    0.6125;
  pid_control_V1_B.wbe_b[0] = pid_control_V1_B.x[3];
  pid_control_V1_B.wbe_b[1] = pid_control_V1_B.x[4];
  pid_control_V1_B.wbe_b[2] = pid_control_V1_B.x[5];
  pid_control_V1_B.Vd1 = pid_control_V1_B.hw / 0.6977;
  pid_control_V1_B.mu_Lw_out = rt_powd_snf(pid_control_V1_B.Vd1, 0.787) * 288.0 *
    exp(rt_powd_snf(pid_control_V1_B.Vd1, 0.327) * -9.14) * 0.97986308862072491 /
    5.9129476540958859 + 1.0;
  pid_control_V1_B.mu_Dw_out = (1.0 - exp(rt_powd_snf(pid_control_V1_B.Vd1,
    0.814) * -4.74) * 0.97916641726789588) - exp(rt_powd_snf
    (pid_control_V1_B.Vd1, 0.758) * -3.88) * (pid_control_V1_B.Vd1 *
    pid_control_V1_B.Vd1);
  pid_control_V1_B.CD_ih_IGE = pid_control_V1_B.hh / 0.3808;
  pid_control_V1_B.hh = ((pid_control_V1_B.Gain2 - -0.065449846949787352) +
    0.026179938779914941) * 4.9604094530365153;
  pid_control_V1_B.hw = (((pid_control_V1_B.Gain2 - -0.043633231299858237) +
    0.0087266462599716477) - (0.56 / pid_control_V1_B.Va_out * 0.35 *
    pid_control_V1_B.q_aero + (pid_control_V1_B.Gain2 - -0.065449846949787352) *
    0.35)) * 4.8387748917360032;
  pid_control_V1_B.CL_w_IGE = pid_control_V1_B.hh * pid_control_V1_B.mu_Lw_out;
  pid_control_V1_B.CL_h_IGE = (rt_powd_snf(pid_control_V1_B.CD_ih_IGE, 0.787) *
    288.0 * exp(rt_powd_snf(pid_control_V1_B.CD_ih_IGE, 0.327) * -9.14) *
    0.95628590200128227 / 5.35300902982722 + 1.0) * pid_control_V1_B.hw;
  pid_control_V1_B.CD_iw_IGE = pid_control_V1_B.CL_w_IGE *
    pid_control_V1_B.CL_w_IGE / 21.205750411731103 * pid_control_V1_B.mu_Dw_out;
  pid_control_V1_B.CD_ih_IGE = ((1.0 - exp(rt_powd_snf
    (pid_control_V1_B.CD_ih_IGE, 0.814) * -4.74) * 0.96770634751485862) - exp
    (rt_powd_snf(pid_control_V1_B.CD_ih_IGE, 0.758) * -3.88) *
    (pid_control_V1_B.CD_ih_IGE * pid_control_V1_B.CD_ih_IGE)) *
    (pid_control_V1_B.CL_h_IGE * pid_control_V1_B.CL_h_IGE / 18.943803701146454);
  pid_control_V1_B.Dtot_f = ((pid_control_V1_B.u2 * pid_control_V1_B.u2 *
    -1.08E-5 + 0.000715 * pid_control_V1_B.u2) * 0.02164 +
    ((pid_control_V1_B.CD_iw_IGE * 0.0649 + 0.0027258) +
     pid_control_V1_B.CD_ih_IGE * 0.02164)) * pid_control_V1_B.Q;
  pid_control_V1_B.Ltot_tmp = pid_control_V1_B.CL_w_IGE * 0.0649 +
    pid_control_V1_B.CL_h_IGE * 0.02164;
  pid_control_V1_B.Ltot = pid_control_V1_B.Ltot_tmp * pid_control_V1_B.Q;
  pid_control_V1_B.CQ = -0.019 * pid_control_V1_B.Square1 * 180.0 /
    3.1415926535897931;
  pid_control_V1_B.FA_b_idx_0 = sin(pid_control_V1_B.Gain2);
  pid_control_V1_B.FA_b_idx_1 = cos(pid_control_V1_B.Gain2);
  pid_control_V1_B.R[0] = pid_control_V1_B.FA_b_idx_1;
  pid_control_V1_B.R[3] = 0.0;
  pid_control_V1_B.R[6] = -pid_control_V1_B.FA_b_idx_0;
  pid_control_V1_B.R[2] = pid_control_V1_B.FA_b_idx_0;
  pid_control_V1_B.R[5] = 0.0;
  pid_control_V1_B.R[8] = pid_control_V1_B.FA_b_idx_1;
  pid_control_V1_B.Dtot[0] = -pid_control_V1_B.Dtot_f;
  pid_control_V1_B.Dtot[1] = pid_control_V1_B.CQ * pid_control_V1_B.Q * 0.0649;
  pid_control_V1_B.Dtot[2] = -pid_control_V1_B.Ltot;
  pid_control_V1_B.R[1] = 0.0;
  pid_control_V1_B.FA_b_idx_0 = 0.0;
  pid_control_V1_B.R[4] = 1.0;
  pid_control_V1_B.FA_b_idx_1 = 0.0;
  pid_control_V1_B.R[7] = 0.0;
  pid_control_V1_B.FA_b_idx_2 = 0.0;
  for (i = 0; i < 3; i++) {
    tmp_3 = _mm_add_pd(_mm_mul_pd(_mm_loadu_pd(&pid_control_V1_B.R[3 * i]),
      _mm_set1_pd(pid_control_V1_B.Dtot[i])), _mm_set_pd
                       (pid_control_V1_B.FA_b_idx_1, pid_control_V1_B.FA_b_idx_0));
    _mm_storeu_pd(&pid_control_V1_B.dv1[0], tmp_3);
    pid_control_V1_B.FA_b_idx_0 = pid_control_V1_B.dv1[0];
    pid_control_V1_B.FA_b_idx_1 = pid_control_V1_B.dv1[1];
    pid_control_V1_B.FA_b_idx_2 += pid_control_V1_B.R[3 * i + 2] *
      pid_control_V1_B.Dtot[i];
  }

  if (pid_control_V1_B.TmpSignalConversionAtSFunct[0] <= 0.3490658503988659) {
    pid_control_V1_B.Vd2 = pid_control_V1_B.TmpSignalConversionAtSFunct[0];
  } else {
    pid_control_V1_B.Vd2 = 0.3490658503988659;
  }

  if (!(pid_control_V1_B.Vd2 >= -0.3490658503988659)) {
    pid_control_V1_B.Vd2 = -0.3490658503988659;
  }

  pid_control_V1_B.Fg_b_idx_2 = 2.0 * pid_control_V1_B.Va_out;
  pid_control_V1_B.Cl = ((pid_control_V1_B.Memory1[0] + pid_control_V1_B.x[3]) *
    0.6977 / pid_control_V1_B.Fg_b_idx_2 * -2.0 + -0.0286 *
    pid_control_V1_B.Square1) + -0.5 * pid_control_V1_B.Vd2;
  pid_control_V1_B.u2 = ((exp(pid_control_V1_B.Vd1 * -4.0) * -0.05 + -1.14 *
    pid_control_V1_B.Gain2) + pid_control_V1_B.q_aero * 0.093 /
    pid_control_V1_B.Fg_b_idx_2 * -5.0) + -3.0 * pid_control_V1_B.u2;
  if (pid_control_V1_B.TmpSignalConversionAtSFunct[2] <= 0.26179938779914941) {
    pid_control_V1_B.Vd2 = pid_control_V1_B.TmpSignalConversionAtSFunct[2];
  } else {
    pid_control_V1_B.Vd2 = 0.26179938779914941;
  }

  if (!(pid_control_V1_B.Vd2 >= -0.26179938779914941)) {
    pid_control_V1_B.Vd2 = -0.26179938779914941;
  }

  pid_control_V1_B.q_aero = ((pid_control_V1_B.Memory1[2] + pid_control_V1_B.x[5])
    * 0.6977 / pid_control_V1_B.Fg_b_idx_2 * -1.5 + -0.1146 *
    pid_control_V1_B.Square1) + -0.3 * pid_control_V1_B.Vd2;
  if (pid_control_V1_B.TmpSignalConversionAtSFunct[3] <= 1.0) {
    pid_control_V1_B.Vd2 = pid_control_V1_B.TmpSignalConversionAtSFunct[3];
  } else {
    pid_control_V1_B.Vd2 = 1.0;
  }

  if (!(pid_control_V1_B.Vd2 >= 0.0)) {
    pid_control_V1_B.Vd2 = 0.0;
  }

  pid_control_V1_B.Vd1 = (25.0 - pid_control_V1_B.Va_out) * pid_control_V1_B.Vd2
    + pid_control_V1_B.Va_out;
  if (pid_control_V1_B.TmpSignalConversionAtSFunct[4] <= 1.0) {
    pid_control_V1_B.Vd2 = pid_control_V1_B.TmpSignalConversionAtSFunct[4];
  } else {
    pid_control_V1_B.Vd2 = 1.0;
  }

  if (!(pid_control_V1_B.Vd2 >= 0.0)) {
    pid_control_V1_B.Vd2 = 0.0;
  }

  pid_control_V1_B.Vd2 = (25.0 - pid_control_V1_B.Va_out) * pid_control_V1_B.Vd2
    + pid_control_V1_B.Va_out;
  pid_control_V1_B.Vd1 = 0.0044226062159978471 * pid_control_V1_B.Vd1 *
    (pid_control_V1_B.Vd1 - pid_control_V1_B.Va_out);
  pid_control_V1_B.Vd2 = 0.0044226062159978471 * pid_control_V1_B.Vd2 *
    (pid_control_V1_B.Vd2 - pid_control_V1_B.Va_out);
  pid_control_V1_B.FE1_b[0] = pid_control_V1_B.Vd1 * 0.99619469809174555;
  pid_control_V1_B.FE1_b[2] = pid_control_V1_B.Vd1 * 0.087155742747658166;
  pid_control_V1_B.FE2_b_idx_0 = pid_control_V1_B.Vd2 * 0.99619469809174555;
  pid_control_V1_B.FE2_b_idx_2 = pid_control_V1_B.Vd2 * 0.087155742747658166;
  pid_control_V1_B.Vd1 = -9.81 * sin(pid_control_V1_B.x[7]) * 1.2;
  _mm_storeu_pd(&pid_control_V1_B.dv1[0], _mm_mul_pd(_mm_mul_pd(_mm_mul_pd
    (_mm_set1_pd(9.81), _mm_set1_pd(cos(pid_control_V1_B.x[7]))), _mm_set_pd(cos
    (pid_control_V1_B.x[6]), sin(pid_control_V1_B.x[6]))), _mm_set1_pd(1.2)));
  pid_control_V1_B.Vd2 = pid_control_V1_B.dv1[0];
  pid_control_V1_B.Fg_b_idx_2 = pid_control_V1_B.dv1[1];
  pid_control_V1_B.FE_b = pid_control_V1_B.FE1_b[0] +
    pid_control_V1_B.FE2_b_idx_0;
  pid_control_V1_B.FE_b_idx_0 = pid_control_V1_B.FE_b;
  pid_control_V1_B.F_b[0] = (pid_control_V1_B.Vd1 + pid_control_V1_B.FE_b) +
    pid_control_V1_B.FA_b_idx_0;
  pid_control_V1_B.F_b[1] = pid_control_V1_B.dv1[0] +
    pid_control_V1_B.FA_b_idx_1;
  pid_control_V1_B.FE_b = pid_control_V1_B.FE1_b[2] +
    pid_control_V1_B.FE2_b_idx_2;
  pid_control_V1_B.F_b[2] = (pid_control_V1_B.dv1[1] + pid_control_V1_B.FE_b) +
    pid_control_V1_B.FA_b_idx_2;
  pid_control_V1_B.c_phi = 0.6977 * pid_control_V1_B.Q * 0.0649;
  pid_control_V1_B.Mcg_b_idx_0 = (0.0834 * pid_control_V1_B.FE1_b[2] + -0.0834 *
    pid_control_V1_B.FE2_b_idx_2) + pid_control_V1_B.c_phi * pid_control_V1_B.Cl;
  pid_control_V1_B.Q = 0.093 * pid_control_V1_B.Q * 0.0649 * pid_control_V1_B.u2
    + ((-0.0396 * pid_control_V1_B.FE1_b[0] - 0.0721 * pid_control_V1_B.FE1_b[2])
       + (-0.0396 * pid_control_V1_B.FE2_b_idx_0 - 0.0721 *
          pid_control_V1_B.FE2_b_idx_2));
  pid_control_V1_B.FE2_b_idx_0 = ((0.0 - 0.0834 * pid_control_V1_B.FE1_b[0]) +
    (0.0 - -0.0834 * pid_control_V1_B.FE2_b_idx_0)) + pid_control_V1_B.c_phi *
    pid_control_V1_B.q_aero;
  memcpy(&pid_control_V1_B.R[0], &pid_control_V1_B.x[0], 9U * sizeof(real_T));
  pid_control_V1_B.FE1_b[0] = pid_control_V1_B.R[0];
  pid_control_V1_B.FE1_b[1] = pid_control_V1_B.R[1];
  pid_control_V1_B.FE1_b[2] = pid_control_V1_B.R[2];
  pid_control_V1_B.c_phi = cos(pid_control_V1_B.R[6]);
  pid_control_V1_B.FE2_b_idx_2 = sin(pid_control_V1_B.R[6]);
  pid_control_V1_B.c_the = cos(pid_control_V1_B.R[7]);
  pid_control_V1_B.s_the = sin(pid_control_V1_B.R[7]);
  pid_control_V1_B.c_psi = cos(pid_control_V1_B.R[8]);
  pid_control_V1_B.s_psi = sin(pid_control_V1_B.R[8]);
  pid_control_V1_B.R[0] = pid_control_V1_B.c_the * pid_control_V1_B.c_psi;
  pid_control_V1_B.R_tmp = pid_control_V1_B.FE2_b_idx_2 * pid_control_V1_B.s_the;
  pid_control_V1_B.R[3] = pid_control_V1_B.R_tmp * pid_control_V1_B.c_psi -
    pid_control_V1_B.c_phi * pid_control_V1_B.s_psi;
  pid_control_V1_B.R_tmp_g = pid_control_V1_B.c_phi * pid_control_V1_B.s_the;
  pid_control_V1_B.R[6] = pid_control_V1_B.R_tmp_g * pid_control_V1_B.c_psi +
    pid_control_V1_B.FE2_b_idx_2 * pid_control_V1_B.s_psi;
  pid_control_V1_B.R[1] = pid_control_V1_B.c_the * pid_control_V1_B.s_psi;
  pid_control_V1_B.R[4] = pid_control_V1_B.R_tmp * pid_control_V1_B.s_psi +
    pid_control_V1_B.c_phi * pid_control_V1_B.c_psi;
  pid_control_V1_B.R[7] = pid_control_V1_B.R_tmp_g * pid_control_V1_B.s_psi -
    pid_control_V1_B.FE2_b_idx_2 * pid_control_V1_B.c_psi;
  pid_control_V1_B.R[2] = -pid_control_V1_B.s_the;
  pid_control_V1_B.R[5] = pid_control_V1_B.FE2_b_idx_2 * pid_control_V1_B.c_the;
  pid_control_V1_B.R[8] = pid_control_V1_B.c_phi * pid_control_V1_B.c_the;
  tmp_3 = _mm_sub_pd(_mm_mul_pd(_mm_set_pd(pid_control_V1_B.x[0],
    pid_control_V1_B.x[2]), _mm_loadu_pd(&pid_control_V1_B.x[4])), _mm_mul_pd
                     (_mm_loadu_pd(&pid_control_V1_B.x[1]), _mm_set_pd
                      (pid_control_V1_B.x[3], pid_control_V1_B.x[5])));
  _mm_storeu_pd(&pid_control_V1_B.Dtot[0], tmp_3);
  pid_control_V1_B.Dtot[2] = pid_control_V1_B.x[1] * pid_control_V1_B.x[3] -
    pid_control_V1_B.x[0] * pid_control_V1_B.x[4];
  pid_control_V1_B.dv[0] = 1.0;
  _mm_storeu_pd(&pid_control_V1_B.dv1[0], _mm_mul_pd(_mm_set_pd(cos
    (pid_control_V1_B.x[6]), sin(pid_control_V1_B.x[6])), _mm_set1_pd(tan
    (pid_control_V1_B.x[7]))));
  pid_control_V1_B.dv[3] = pid_control_V1_B.dv1[0];
  pid_control_V1_B.dv[6] = pid_control_V1_B.dv1[1];
  pid_control_V1_B.dv[1] = 0.0;
  pid_control_V1_B.dv[4] = cos(pid_control_V1_B.x[6]);
  pid_control_V1_B.dv[7] = -sin(pid_control_V1_B.x[6]);
  pid_control_V1_B.dv[2] = 0.0;
  _mm_storeu_pd(&pid_control_V1_B.dv1[0], _mm_div_pd(_mm_set_pd(cos
    (pid_control_V1_B.x[6]), sin(pid_control_V1_B.x[6])), _mm_set1_pd(cos
    (pid_control_V1_B.x[7]))));
  pid_control_V1_B.dv[5] = pid_control_V1_B.dv1[0];
  pid_control_V1_B.dv[8] = pid_control_V1_B.dv1[1];
  pid_control_V1_B.c_phi = 0.0;
  pid_control_V1_B.FE2_b_idx_2 = 0.0;
  pid_control_V1_B.c_the = 0.0;
  pid_control_V1_B.s_the = 0.0;
  pid_control_V1_B.c_psi = 0.0;
  pid_control_V1_B.s_psi = 0.0;
  for (i = 0; i < 3; i++) {
    tmp_3 = _mm_add_pd(_mm_mul_pd(_mm_loadu_pd(&pid_control_V1_B.dv[3 * i]),
      _mm_set1_pd(pid_control_V1_B.wbe_b[i])), _mm_set_pd
                       (pid_control_V1_B.FE2_b_idx_2, pid_control_V1_B.c_phi));
    _mm_storeu_pd(&pid_control_V1_B.dv1[0], tmp_3);
    pid_control_V1_B.c_phi = pid_control_V1_B.dv1[0];
    pid_control_V1_B.FE2_b_idx_2 = pid_control_V1_B.dv1[1];
    _mm_storeu_pd(&pid_control_V1_B.dv1[0], _mm_add_pd(_mm_mul_pd(_mm_set_pd
      (pid_control_V1_B.R[3 * i], pid_control_V1_B.dv[3 * i + 2]), _mm_set_pd
      (pid_control_V1_B.FE1_b[i], pid_control_V1_B.wbe_b[i])), _mm_set_pd
      (pid_control_V1_B.s_the, pid_control_V1_B.c_the)));
    pid_control_V1_B.c_the = pid_control_V1_B.dv1[0];
    pid_control_V1_B.s_the = pid_control_V1_B.dv1[1];
    tmp_3 = _mm_add_pd(_mm_mul_pd(_mm_loadu_pd(&pid_control_V1_B.R[3 * i + 1]),
      _mm_set1_pd(pid_control_V1_B.FE1_b[i])), _mm_set_pd(pid_control_V1_B.s_psi,
      pid_control_V1_B.c_psi));
    _mm_storeu_pd(&pid_control_V1_B.dv1[0], tmp_3);
    pid_control_V1_B.c_psi = pid_control_V1_B.dv1[0];
    pid_control_V1_B.s_psi = pid_control_V1_B.dv1[1];
    pid_control_V1_B.XDOT[i] = 0.83333333333333337 * pid_control_V1_B.F_b[i] -
      pid_control_V1_B.Dtot[i];
  }

  pid_control_V1_B.XDOT[3] = ((0.023669 * pid_control_V1_B.Mcg_b_idx_0 +
    0.001856 * pid_control_V1_B.FE2_b_idx_0) - (-2.6338496E-5 *
    pid_control_V1_B.x[3] + 0.000144843342 * pid_control_V1_B.x[5]) *
    pid_control_V1_B.x[4]) / 0.000191043437;
  pid_control_V1_B.XDOT[4] = ((pid_control_V1_B.Q - -0.015451999999999999 *
    pid_control_V1_B.x[3] * pid_control_V1_B.x[5]) - (pid_control_V1_B.x[3] *
    pid_control_V1_B.x[3] - pid_control_V1_B.x[5] * pid_control_V1_B.x[5]) *
    0.001856) / 0.017695;
  pid_control_V1_B.XDOT[5] = ((-2.6338496E-5 * pid_control_V1_B.x[5] +
    -7.4435989999999989E-5 * pid_control_V1_B.x[3]) * pid_control_V1_B.x[4] +
    (0.001856 * pid_control_V1_B.Mcg_b_idx_0 + 0.008217 *
     pid_control_V1_B.FE2_b_idx_0)) / 0.000191043437;
  pid_control_V1_B.XDOT[9] = pid_control_V1_B.s_the;
  pid_control_V1_B.XDOT[10] = pid_control_V1_B.c_psi;
  pid_control_V1_B.XDOT[11] = pid_control_V1_B.s_psi;
  if (!(pid_control_V1_B.Dtot_f >= 0.001)) {
    pid_control_V1_B.Dtot_f = 0.001;
  }

  pid_control_V1_B.XDOT[12] = pid_control_V1_B.Ltot / pid_control_V1_B.Dtot_f;
  pid_control_V1_B.XDOT[19] = pid_control_V1_B.CQ;
  pid_control_V1_B.XDOT[20] = pid_control_V1_B.Cl;
  pid_control_V1_B.XDOT[21] = pid_control_V1_B.u2;
  pid_control_V1_B.XDOT[22] = pid_control_V1_B.q_aero;
  pid_control_V1_B.XDOT[23] = pid_control_V1_B.Gain2;
  pid_control_V1_B.XDOT[24] = pid_control_V1_B.Square1;
  pid_control_V1_B.XDOT[25] = pid_control_V1_B.hh;
  pid_control_V1_B.XDOT[26] = pid_control_V1_B.hw;
  pid_control_V1_B.XDOT[27] = pid_control_V1_B.CL_w_IGE;
  pid_control_V1_B.XDOT[28] = pid_control_V1_B.CL_h_IGE;
  pid_control_V1_B.XDOT[29] = pid_control_V1_B.CD_iw_IGE;
  pid_control_V1_B.XDOT[30] = pid_control_V1_B.CD_ih_IGE;
  pid_control_V1_B.XDOT[6] = pid_control_V1_B.c_phi;
  pid_control_V1_B.XDOT[13] = pid_control_V1_B.F_b[0];
  pid_control_V1_B.XDOT[16] = pid_control_V1_B.Mcg_b_idx_0;
  pid_control_V1_B.XDOT[31] = pid_control_V1_B.Vd1;
  pid_control_V1_B.XDOT[34] = pid_control_V1_B.FE_b_idx_0;
  pid_control_V1_B.XDOT[37] = pid_control_V1_B.FA_b_idx_0;
  pid_control_V1_B.XDOT[7] = pid_control_V1_B.FE2_b_idx_2;
  pid_control_V1_B.XDOT[14] = pid_control_V1_B.F_b[1];
  pid_control_V1_B.XDOT[17] = pid_control_V1_B.Q;
  pid_control_V1_B.XDOT[32] = pid_control_V1_B.Vd2;
  pid_control_V1_B.XDOT[35] = 0.0;
  pid_control_V1_B.XDOT[38] = pid_control_V1_B.FA_b_idx_1;
  pid_control_V1_B.XDOT[8] = pid_control_V1_B.c_the;
  pid_control_V1_B.XDOT[15] = pid_control_V1_B.F_b[2];
  pid_control_V1_B.XDOT[18] = pid_control_V1_B.FE2_b_idx_0;
  pid_control_V1_B.XDOT[33] = pid_control_V1_B.Fg_b_idx_2;
  pid_control_V1_B.XDOT[36] = pid_control_V1_B.FE_b;
  pid_control_V1_B.XDOT[39] = pid_control_V1_B.FA_b_idx_2;
  pid_control_V1_B.CL_total = pid_control_V1_B.Ltot_tmp / 0.08654;
  if (tmp_0) {
    /* MATLAB Function: '<Root>/MATLAB Function' */
    memset(&pid_control_V1_B.stringOut_l[0], 0, sizeof(uint8_T) << 7U);
    for (i = 0; i < 11; i++) {
      pid_control_V1_B.stringOut_l[i] = b[i];
    }

    pid_control_V1_B.lengthOut_e = 11U;

    /* End of MATLAB Function: '<Root>/MATLAB Function' */

    /* MATLAB Function: '<Root>/MATLAB Function1' */
    memset(&pid_control_V1_B.stringOut[0], 0, sizeof(uint8_T) << 7U);
    for (i = 0; i < 5; i++) {
      pid_control_V1_B.stringOut[i] = b_0[i];
    }

    pid_control_V1_B.lengthOut = 5U;

    /* End of MATLAB Function: '<Root>/MATLAB Function1' */
  }

  /* BusAssignment: '<Root>/Bus Assignment' */
  memset(&pid_control_V1_B.BusAssignment, 0, sizeof
         (SL_Bus_gazebo_msgs_SetEntityStateRequest));

  /* Gain: '<Root>/Gain2' incorporates:
   *  Gain: '<Root>/Gain1'
   *  MATLABSystem: '<Root>/Coordinate Transformation Conversion'
   */
  _mm_storeu_pd(&pid_control_V1_B.wbe_b[0], _mm_div_pd(_mm_set_pd
    (-pid_control_V1_B.x[7], -pid_control_V1_B.x[8]), _mm_set1_pd(2.0)));

  /* MATLABSystem: '<Root>/Coordinate Transformation Conversion' incorporates:
   *  Constant: '<Root>/Constant'
   *  Sum: '<Root>/Sum'
   */
  pid_control_V1_B.wbe_b[2] = (pid_control_V1_B.x[6] + 1.57) / 2.0;
  pid_control_V1_B.Square1 = sin(pid_control_V1_B.wbe_b[0]);
  pid_control_V1_B.hh = sin(pid_control_V1_B.wbe_b[1]);
  pid_control_V1_B.hw = sin(pid_control_V1_B.wbe_b[2]);
  pid_control_V1_B.CL_w_IGE = cos(pid_control_V1_B.wbe_b[0]);
  pid_control_V1_B.CL_h_IGE = cos(pid_control_V1_B.wbe_b[1]);
  pid_control_V1_B.CD_iw_IGE = cos(pid_control_V1_B.wbe_b[2]);

  /* BusAssignment: '<Root>/Bus Assignment' incorporates:
   *  Gain: '<Root>/Gain3'
   */
  pid_control_V1_B.BusAssignment.state.pose.position.x = pid_control_V1_B.x[9];
  pid_control_V1_B.BusAssignment.state.pose.position.y = -pid_control_V1_B.x[10];
  pid_control_V1_B.BusAssignment.state.pose.position.z = pid_control_V1_B.Gain;

  /* Start for MATLABSystem: '<Root>/Coordinate Transformation Conversion' */
  pid_control_V1_B.Gain2 = pid_control_V1_B.CL_w_IGE * pid_control_V1_B.CL_h_IGE;

  /* BusAssignment: '<Root>/Bus Assignment' incorporates:
   *  MATLABSystem: '<Root>/Coordinate Transformation Conversion'
   * */
  pid_control_V1_B.BusAssignment.state.pose.orientation.w =
    pid_control_V1_B.Square1 * pid_control_V1_B.hh * pid_control_V1_B.hw +
    pid_control_V1_B.Gain2 * pid_control_V1_B.CD_iw_IGE;
  pid_control_V1_B.BusAssignment.state.pose.orientation.z =
    pid_control_V1_B.Gain2 * pid_control_V1_B.hw - pid_control_V1_B.CD_iw_IGE *
    pid_control_V1_B.Square1 * pid_control_V1_B.hh;
  pid_control_V1_B.BusAssignment.state.pose.orientation.y =
    pid_control_V1_B.CL_w_IGE * pid_control_V1_B.CD_iw_IGE * pid_control_V1_B.hh
    + pid_control_V1_B.CL_h_IGE * pid_control_V1_B.Square1 * pid_control_V1_B.hw;
  pid_control_V1_B.BusAssignment.state.pose.orientation.x =
    pid_control_V1_B.CL_h_IGE * pid_control_V1_B.CD_iw_IGE *
    pid_control_V1_B.Square1 - pid_control_V1_B.CL_w_IGE * pid_control_V1_B.hh *
    pid_control_V1_B.hw;
  memcpy(&pid_control_V1_B.BusAssignment.state.name[0],
         &pid_control_V1_B.stringOut_l[0], sizeof(uint8_T) << 7U);
  memcpy(&pid_control_V1_B.BusAssignment.state.reference_frame[0],
         &pid_control_V1_B.stringOut[0], sizeof(uint8_T) << 7U);
  pid_control_V1_B.BusAssignment.state.name_SL_Info.CurrentLength =
    pid_control_V1_B.lengthOut_e;
  pid_control_V1_B.BusAssignment.state.reference_frame_SL_Info.CurrentLength =
    pid_control_V1_B.lengthOut;

  /* Outputs for Atomic SubSystem: '<Root>/Call Service' */
  /* MATLABSystem: '<S2>/ServiceCaller' */
  serverAvailableOnTime = ServCall_pid_control_V1_326.waitForServer(5.0);
  if (serverAvailableOnTime) {
    ServCall_pid_control_V1_326.call(&pid_control_V1_B.BusAssignment, &tmp);
  }

  /* End of MATLABSystem: '<S2>/ServiceCaller' */
  /* End of Outputs for SubSystem: '<Root>/Call Service' */

  /* Gain: '<S41>/ZeroGain' */
  pid_control_V1_B.Square1 = 0.0 * pid_control_V1_B.SignPreSat;

  /* DeadZone: '<S43>/DeadZone' */
  if (pid_control_V1_B.SignPreSat > 0.17453292519943295) {
    pid_control_V1_B.SignPreSat -= 0.17453292519943295;
  } else if (pid_control_V1_B.SignPreSat >= -0.17453292519943295) {
    pid_control_V1_B.SignPreSat = 0.0;
  } else {
    pid_control_V1_B.SignPreSat -= -0.17453292519943295;
  }

  /* End of DeadZone: '<S43>/DeadZone' */

  /* Gain: '<S48>/Integral Gain' */
  pid_control_V1_B.Switch *= -0.08;

  /* Signum: '<S41>/SignPreSat' */
  if (rtIsNaN(pid_control_V1_B.SignPreSat)) {
    /* DataTypeConversion: '<S41>/DataTypeConv1' */
    i = 0;
  } else {
    if (pid_control_V1_B.SignPreSat < 0.0) {
      /* DataTypeConversion: '<S41>/DataTypeConv1' */
      pid_control_V1_B.Gain2 = -1.0;
    } else {
      /* DataTypeConversion: '<S41>/DataTypeConv1' */
      pid_control_V1_B.Gain2 = (pid_control_V1_B.SignPreSat > 0.0);
    }

    /* DataTypeConversion: '<S41>/DataTypeConv1' */
    i = static_cast<int32_T>(fmod(pid_control_V1_B.Gain2, 256.0));
  }

  /* End of Signum: '<S41>/SignPreSat' */

  /* Signum: '<S41>/SignPreIntegrator' */
  if (rtIsNaN(pid_control_V1_B.Switch)) {
    /* DataTypeConversion: '<S41>/DataTypeConv2' */
    tmp_2 = 0;
  } else {
    if (pid_control_V1_B.Switch < 0.0) {
      /* DataTypeConversion: '<S41>/DataTypeConv2' */
      pid_control_V1_B.Gain2 = -1.0;
    } else {
      /* DataTypeConversion: '<S41>/DataTypeConv2' */
      pid_control_V1_B.Gain2 = (pid_control_V1_B.Switch > 0.0);
    }

    /* DataTypeConversion: '<S41>/DataTypeConv2' */
    tmp_2 = static_cast<int32_T>(fmod(pid_control_V1_B.Gain2, 256.0));
  }

  /* End of Signum: '<S41>/SignPreIntegrator' */

  /* DataTypeConversion: '<S41>/DataTypeConv1' */
  if (i < 0) {
    i = static_cast<int8_T>(-static_cast<int8_T>(static_cast<uint8_T>(-
      static_cast<real_T>(i))));
  }

  /* DataTypeConversion: '<S41>/DataTypeConv2' */
  if (tmp_2 < 0) {
    tmp_2 = static_cast<int8_T>(-static_cast<int8_T>(static_cast<uint8_T>(-
      static_cast<real_T>(tmp_2))));
  }

  /* Logic: '<S41>/AND3' incorporates:
   *  DataTypeConversion: '<S41>/DataTypeConv1'
   *  DataTypeConversion: '<S41>/DataTypeConv2'
   *  RelationalOperator: '<S41>/Equal1'
   *  RelationalOperator: '<S41>/NotEqual'
   */
  pid_control_V1_B.AND3 = ((pid_control_V1_B.Square1 !=
    pid_control_V1_B.SignPreSat) && (i == tmp_2));
  if (tmp_0) {
    /* Memory: '<S41>/Memory' */
    pid_control_V1_B.Memory_a = pid_control_V1_DW.Memory_PreviousInput_o;
  }

  /* Switch: '<S41>/Switch' */
  if (pid_control_V1_B.Memory_a) {
    /* Gain: '<S48>/Integral Gain' incorporates:
     *  Constant: '<S41>/Constant1'
     *  Switch: '<S41>/Switch'
     */
    pid_control_V1_B.Switch = 0.0;
  }

  /* End of Switch: '<S41>/Switch' */

  /* Sum: '<S95>/SumI4' incorporates:
   *  Gain: '<S100>/Integral Gain'
   *  Gain: '<S95>/Kb'
   *  Sum: '<S95>/SumI2'
   */
  pid_control_V1_B.SumI4 = (pid_control_V1_B.Saturation - pid_control_V1_B.Sum_b)
    * 0.1 + 0.25 * pid_control_V1_B.Sum2_l;

  /* Sum: '<S147>/SumI4' incorporates:
   *  Gain: '<S152>/Integral Gain'
   *  Sum: '<S147>/SumI2'
   */
  pid_control_V1_B.SumI4_i = (pid_control_V1_B.Saturation_f -
    pid_control_V1_B.Sum_hl) + -0.05 * pid_control_V1_B.Sum1_g;
  if (tmp_0) {
    /* Gain: '<S204>/Integral Gain' */
    pid_control_V1_B.IntegralGain = -0.005 * pid_control_V1_B.WhiteNoise_p;
  }

  /* Gain: '<S251>/ZeroGain' */
  pid_control_V1_B.WhiteNoise_p = 0.0 * pid_control_V1_B.SignPreSat_h;

  /* DeadZone: '<S253>/DeadZone' */
  if (pid_control_V1_B.SignPreSat_h > 1.0) {
    pid_control_V1_B.SignPreSat_h--;
  } else if (pid_control_V1_B.SignPreSat_h >= 0.0) {
    pid_control_V1_B.SignPreSat_h = 0.0;
  }

  /* End of DeadZone: '<S253>/DeadZone' */

  /* Gain: '<S258>/Integral Gain' incorporates:
   *  Constant: '<Root>/Constant_U'
   *  Sum: '<Root>/Sum3'
   */
  pid_control_V1_B.Switch_j = (20.2 - pid_control_V1_B.x[0]) * 0.015;

  /* Signum: '<S251>/SignPreSat' */
  if (rtIsNaN(pid_control_V1_B.SignPreSat_h)) {
    /* DataTypeConversion: '<S251>/DataTypeConv1' */
    i = 0;
  } else {
    if (pid_control_V1_B.SignPreSat_h < 0.0) {
      /* DataTypeConversion: '<S251>/DataTypeConv1' */
      pid_control_V1_B.Gain2 = -1.0;
    } else {
      /* DataTypeConversion: '<S251>/DataTypeConv1' */
      pid_control_V1_B.Gain2 = (pid_control_V1_B.SignPreSat_h > 0.0);
    }

    /* DataTypeConversion: '<S251>/DataTypeConv1' */
    i = static_cast<int32_T>(fmod(pid_control_V1_B.Gain2, 256.0));
  }

  /* End of Signum: '<S251>/SignPreSat' */

  /* Signum: '<S251>/SignPreIntegrator' */
  if (rtIsNaN(pid_control_V1_B.Switch_j)) {
    /* DataTypeConversion: '<S251>/DataTypeConv2' */
    tmp_2 = 0;
  } else {
    if (pid_control_V1_B.Switch_j < 0.0) {
      /* DataTypeConversion: '<S251>/DataTypeConv2' */
      pid_control_V1_B.Gain2 = -1.0;
    } else {
      /* DataTypeConversion: '<S251>/DataTypeConv2' */
      pid_control_V1_B.Gain2 = (pid_control_V1_B.Switch_j > 0.0);
    }

    /* DataTypeConversion: '<S251>/DataTypeConv2' */
    tmp_2 = static_cast<int32_T>(fmod(pid_control_V1_B.Gain2, 256.0));
  }

  /* End of Signum: '<S251>/SignPreIntegrator' */

  /* DataTypeConversion: '<S251>/DataTypeConv1' */
  if (i < 0) {
    i = static_cast<int8_T>(-static_cast<int8_T>(static_cast<uint8_T>(-
      static_cast<real_T>(i))));
  }

  /* DataTypeConversion: '<S251>/DataTypeConv2' */
  if (tmp_2 < 0) {
    tmp_2 = static_cast<int8_T>(-static_cast<int8_T>(static_cast<uint8_T>(-
      static_cast<real_T>(tmp_2))));
  }

  /* Logic: '<S251>/AND3' incorporates:
   *  DataTypeConversion: '<S251>/DataTypeConv1'
   *  DataTypeConversion: '<S251>/DataTypeConv2'
   *  RelationalOperator: '<S251>/Equal1'
   *  RelationalOperator: '<S251>/NotEqual'
   */
  pid_control_V1_B.AND3_c = ((pid_control_V1_B.WhiteNoise_p !=
    pid_control_V1_B.SignPreSat_h) && (i == tmp_2));
  if (tmp_0) {
    /* Memory: '<S251>/Memory' */
    pid_control_V1_B.Memory_h = pid_control_V1_DW.Memory_PreviousInput_a;
  }

  /* Switch: '<S251>/Switch' */
  if (pid_control_V1_B.Memory_h) {
    /* Gain: '<S258>/Integral Gain' incorporates:
     *  Constant: '<S251>/Constant1'
     *  Switch: '<S251>/Switch'
     */
    pid_control_V1_B.Switch_j = 0.0;
  }

  /* End of Switch: '<S251>/Switch' */

  /* UnitConversion: '<S291>/Unit Conversion' incorporates:
   *  Gain: '<S11>/Gain4'
   */
  /* Unit Conversion - from: m to: ft
     Expression: output = (3.28084*input) + (0) */
  pid_control_V1_B.WhiteNoise_p = 3.280839895013123 * -pid_control_V1_B.x[11];

  /* UnitConversion: '<S297>/Unit Conversion' */
  /* Unit Conversion - from: m/s to: ft/s
     Expression: output = (3.28084*input) + (0) */
  pid_control_V1_B.Sum2_l = 3.280839895013123 * pid_control_V1_B.Va_out;

  /* Saturate: '<S324>/Limit Function 10ft to 1000ft' incorporates:
   *  Saturate: '<S307>/Limit Height h<1000ft'
   */
  if (pid_control_V1_B.WhiteNoise_p > 1000.0) {
    pid_control_V1_B.Sum_b = 1000.0;
    pid_control_V1_B.SignPreSat = 1000.0;
  } else {
    if (pid_control_V1_B.WhiteNoise_p < 10.0) {
      pid_control_V1_B.Sum_b = 10.0;
    } else {
      pid_control_V1_B.Sum_b = pid_control_V1_B.WhiteNoise_p;
    }

    if (pid_control_V1_B.WhiteNoise_p < 0.0) {
      pid_control_V1_B.SignPreSat = 0.0;
    } else {
      pid_control_V1_B.SignPreSat = pid_control_V1_B.WhiteNoise_p;
    }
  }

  /* End of Saturate: '<S324>/Limit Function 10ft to 1000ft' */

  /* Fcn: '<S324>/Low Altitude Scale Length' */
  pid_control_V1_B.Sum1_g = pid_control_V1_B.Sum_b / rt_powd_snf(0.000823 *
    pid_control_V1_B.Sum_b + 0.177, 1.2);

  /* Product: '<S307>/sigma_ug, sigma_vg' incorporates:
   *  Fcn: '<S307>/Low Altitude Intensity'
   */
  pid_control_V1_B.SignPreSat_h = 1.0 / rt_powd_snf(0.000823 *
    pid_control_V1_B.SignPreSat + 0.177, 0.4) * pid_control_V1_ConstB.sigma_wg;

  /* Interpolation_n-D: '<S306>/Medium//High Altitude Intensity' incorporates:
   *  PreLookup: '<S306>/PreLook-Up Index Search  (altitude)'
   */
  pid_control_V1_B.bpIndex[0] = plook_bincpa(pid_control_V1_B.WhiteNoise_p,
    pid_control_V1_ConstP.PreLookUpIndexSearchaltitude_Br, 11U,
    &pid_control_V1_B.SignPreSat,
    &pid_control_V1_DW.PreLookUpIndexSearchaltitude_DW);
  pid_control_V1_B.frac[0] = pid_control_V1_B.SignPreSat;
  pid_control_V1_B.frac[1] = pid_control_V1_ConstB.PreLookUpIndexSearchprobofe;
  pid_control_V1_B.bpIndex[1] =
    pid_control_V1_ConstB.PreLookUpIndexSearchprobo_g;
  pid_control_V1_B.SignPreSat = intrp2d_la_pw(pid_control_V1_B.bpIndex,
    pid_control_V1_B.frac, pid_control_V1_ConstP.MediumHighAltitudeIntensity_Tab,
    12U, pid_control_V1_ConstP.MediumHighAltitudeIntensity_max);
  if (tmp_0) {
    /* Product: '<S299>/Divide' incorporates:
     *  Product: '<S299>/Product'
     *  RandomNumber: '<S299>/White Noise'
     */
    tmp_3 = _mm_mul_pd(_mm_loadu_pd(&pid_control_V1_ConstB.Divide[0]),
                       _mm_loadu_pd(&pid_control_V1_DW.NextOutput[0]));

    /* Product: '<S299>/Product' */
    _mm_storeu_pd(&pid_control_V1_B.Product[0], tmp_3);

    /* Product: '<S299>/Divide' incorporates:
     *  Product: '<S299>/Product'
     *  RandomNumber: '<S299>/White Noise'
     */
    tmp_3 = _mm_mul_pd(_mm_loadu_pd(&pid_control_V1_ConstB.Divide[2]),
                       _mm_loadu_pd(&pid_control_V1_DW.NextOutput[2]));

    /* Product: '<S299>/Product' */
    _mm_storeu_pd(&pid_control_V1_B.Product[2], tmp_3);

    /* Outputs for Enabled SubSystem: '<S290>/Hugw(s)' incorporates:
     *  EnablePort: '<S303>/Enable'
     */
    if (tmp_1 && (!pid_control_V1_DW.Hugws_MODE)) {
      (void) memset(&(pid_control_V1_XDis.ug_p_CSTATE), 0,
                    2*sizeof(boolean_T));

      /* InitializeConditions for Integrator: '<S303>/ug_p' */
      pid_control_V1_X.ug_p_CSTATE[0] = 0.0;
      pid_control_V1_X.ug_p_CSTATE[1] = 0.0;
      pid_control_V1_DW.Hugws_MODE = true;
    }

    /* End of Outputs for SubSystem: '<S290>/Hugw(s)' */
  }

  /* Outputs for Enabled SubSystem: '<S290>/Hugw(s)' incorporates:
   *  EnablePort: '<S303>/Enable'
   */
  if (pid_control_V1_DW.Hugws_MODE) {
    /* Product: '<S303>/Lug//V' */
    pid_control_V1_B.frac[0] = pid_control_V1_B.Sum1_g / pid_control_V1_B.Sum2_l;
    pid_control_V1_B.frac[1] = pid_control_V1_ConstB.UnitConversion_c /
      pid_control_V1_B.Sum2_l;

    /* Sqrt: '<S303>/sqrt' incorporates:
     *  Gain: '<S303>/(2//pi)'
     *  Integrator: '<S303>/ug_p'
     *  Product: '<S303>/Lug//V1'
     */
    tmp_3 = _mm_div_pd(_mm_sub_pd(_mm_mul_pd(_mm_set_pd(sqrt(0.63661977236758138
      * pid_control_V1_B.frac[1]), sqrt(0.63661977236758138 *
      pid_control_V1_B.frac[0])), _mm_set1_pd(pid_control_V1_B.Product[0])),
      _mm_loadu_pd(&pid_control_V1_X.ug_p_CSTATE[0])), _mm_loadu_pd
                       (&pid_control_V1_B.frac[0]));

    /* Product: '<S303>/w' */
    _mm_storeu_pd(&pid_control_V1_B.w_n[0], tmp_3);

    /* Integrator: '<S303>/ug_p' incorporates:
     *  Product: '<S303>/w1'
     */
    tmp_3 = _mm_mul_pd(_mm_loadu_pd(&pid_control_V1_X.ug_p_CSTATE[0]),
                       _mm_set_pd(pid_control_V1_B.SignPreSat,
      pid_control_V1_B.SignPreSat_h));

    /* Product: '<S303>/w1' */
    _mm_storeu_pd(&pid_control_V1_B.w1_c[0], tmp_3);
  }

  /* End of Outputs for SubSystem: '<S290>/Hugw(s)' */

  /* Gain: '<S296>/Lv' */
  pid_control_V1_B.frac[1] = pid_control_V1_ConstB.UnitConversion_c;

  /* Outputs for Enabled SubSystem: '<S290>/Hvgw(s)' incorporates:
   *  EnablePort: '<S304>/Enable'
   */
  if (tmp_0 && tmp_1 && (!pid_control_V1_DW.Hvgws_MODE)) {
    (void) memset(&(pid_control_V1_XDis.vg_p1_CSTATE), 0,
                  4*sizeof(boolean_T));

    /* InitializeConditions for Integrator: '<S304>/vg_p1' */
    pid_control_V1_X.vg_p1_CSTATE[0] = 0.0;

    /* InitializeConditions for Integrator: '<S304>/vgw_p2' */
    pid_control_V1_X.vgw_p2_CSTATE[0] = 0.0;

    /* InitializeConditions for Integrator: '<S304>/vg_p1' */
    pid_control_V1_X.vg_p1_CSTATE[1] = 0.0;

    /* InitializeConditions for Integrator: '<S304>/vgw_p2' */
    pid_control_V1_X.vgw_p2_CSTATE[1] = 0.0;
    pid_control_V1_DW.Hvgws_MODE = true;
  }

  if (pid_control_V1_DW.Hvgws_MODE) {
    /* Product: '<S304>/Lvg//V' incorporates:
     *  Gain: '<S296>/Lv'
     */
    pid_control_V1_B.Sum1_g /= pid_control_V1_B.Sum2_l;

    /* Product: '<S304>/w' incorporates:
     *  Gain: '<S304>/(1//pi)'
     *  Integrator: '<S304>/vg_p1'
     *  Product: '<S304>/Lug//V1'
     *  Sqrt: '<S304>/sqrt'
     *  Sum: '<S304>/Sum'
     */
    pid_control_V1_B.Sum_hl = (sqrt(0.31830988618379069 *
      pid_control_V1_B.Sum1_g) * pid_control_V1_B.Product[1] -
      pid_control_V1_X.vg_p1_CSTATE[0]) / pid_control_V1_B.Sum1_g;
    pid_control_V1_B.w_g[0] = pid_control_V1_B.Sum_hl;

    /* Product: '<S304>/w ' incorporates:
     *  Gain: '<S304>/sqrt(3)'
     *  Integrator: '<S304>/vg_p1'
     *  Integrator: '<S304>/vgw_p2'
     *  Product: '<S304>/Lvg//V '
     *  Sum: '<S304>/Sum1'
     */
    pid_control_V1_B.w_e[0] = (pid_control_V1_B.Sum_hl * pid_control_V1_B.Sum1_g
      * 1.7320508075688772 + (pid_control_V1_X.vg_p1_CSTATE[0] -
      pid_control_V1_X.vgw_p2_CSTATE[0])) / pid_control_V1_B.Sum1_g;

    /* Product: '<S304>/Lvg//V' */
    pid_control_V1_B.Sum1_g = pid_control_V1_B.frac[1] / pid_control_V1_B.Sum2_l;

    /* Product: '<S304>/w' incorporates:
     *  Gain: '<S304>/(1//pi)'
     *  Integrator: '<S304>/vg_p1'
     *  Product: '<S304>/Lug//V1'
     *  Sqrt: '<S304>/sqrt'
     *  Sum: '<S304>/Sum'
     */
    pid_control_V1_B.Sum_hl = (sqrt(0.31830988618379069 *
      pid_control_V1_B.Sum1_g) * pid_control_V1_B.Product[1] -
      pid_control_V1_X.vg_p1_CSTATE[1]) / pid_control_V1_B.Sum1_g;
    pid_control_V1_B.w_g[1] = pid_control_V1_B.Sum_hl;

    /* Product: '<S304>/w ' incorporates:
     *  Gain: '<S304>/sqrt(3)'
     *  Integrator: '<S304>/vg_p1'
     *  Integrator: '<S304>/vgw_p2'
     *  Product: '<S304>/Lvg//V '
     *  Sum: '<S304>/Sum1'
     */
    pid_control_V1_B.w_e[1] = (pid_control_V1_B.Sum_hl * pid_control_V1_B.Sum1_g
      * 1.7320508075688772 + (pid_control_V1_X.vg_p1_CSTATE[1] -
      pid_control_V1_X.vgw_p2_CSTATE[1])) / pid_control_V1_B.Sum1_g;

    /* Product: '<S304>/w 1' incorporates:
     *  Integrator: '<S304>/vgw_p2'
     */
    tmp_3 = _mm_mul_pd(_mm_set_pd(pid_control_V1_B.SignPreSat,
      pid_control_V1_B.SignPreSat_h), _mm_loadu_pd
                       (&pid_control_V1_X.vgw_p2_CSTATE[0]));

    /* Product: '<S304>/w 1' */
    _mm_storeu_pd(&pid_control_V1_B.w1[0], tmp_3);
  }

  /* End of Outputs for SubSystem: '<S290>/Hvgw(s)' */

  /* Gain: '<S296>/Lw' */
  pid_control_V1_B.frac[1] = pid_control_V1_ConstB.UnitConversion_c;

  /* Outputs for Enabled SubSystem: '<S290>/Hwgw(s)' incorporates:
   *  EnablePort: '<S305>/Enable'
   */
  if (tmp_0 && tmp_1 && (!pid_control_V1_DW.Hwgws_MODE)) {
    (void) memset(&(pid_control_V1_XDis.wg_p1_CSTATE), 0,
                  4*sizeof(boolean_T));

    /* InitializeConditions for Integrator: '<S305>/wg_p1' */
    pid_control_V1_X.wg_p1_CSTATE[0] = 0.0;

    /* InitializeConditions for Integrator: '<S305>/wg_p2' */
    pid_control_V1_X.wg_p2_CSTATE[0] = 0.0;

    /* InitializeConditions for Integrator: '<S305>/wg_p1' */
    pid_control_V1_X.wg_p1_CSTATE[1] = 0.0;

    /* InitializeConditions for Integrator: '<S305>/wg_p2' */
    pid_control_V1_X.wg_p2_CSTATE[1] = 0.0;
    pid_control_V1_DW.Hwgws_MODE = true;
  }

  if (pid_control_V1_DW.Hwgws_MODE) {
    /* Product: '<S305>/Lwg//V' incorporates:
     *  Gain: '<S296>/Lw'
     */
    pid_control_V1_B.SignPreSat_h = pid_control_V1_B.Sum_b /
      pid_control_V1_B.Sum2_l;

    /* Product: '<S305>/w' incorporates:
     *  Gain: '<S305>/1//pi'
     *  Integrator: '<S305>/wg_p1'
     *  Product: '<S305>/Lug//V1'
     *  Sqrt: '<S305>/sqrt1'
     *  Sum: '<S305>/Sum'
     */
    pid_control_V1_B.Sum1_g = (sqrt(0.31830988618379069 *
      pid_control_V1_B.SignPreSat_h) * pid_control_V1_B.Product[2] -
      pid_control_V1_X.wg_p1_CSTATE[0]) / pid_control_V1_B.SignPreSat_h;
    pid_control_V1_B.w[0] = pid_control_V1_B.Sum1_g;

    /* Product: '<S305>/w ' incorporates:
     *  Integrator: '<S305>/wg_p1'
     *  Integrator: '<S305>/wg_p2'
     *  Product: '<S305>/Lwg//V'
     *  Product: '<S305>/Lwg//V '
     *  Sum: '<S305>/Sum1'
     */
    pid_control_V1_B.w_a[0] = (pid_control_V1_B.Sum1_g *
      pid_control_V1_ConstB.sqrt_a * pid_control_V1_B.SignPreSat_h +
      (pid_control_V1_X.wg_p1_CSTATE[0] - pid_control_V1_X.wg_p2_CSTATE[0])) /
      pid_control_V1_B.SignPreSat_h;

    /* Product: '<S305>/Lwg//V' */
    pid_control_V1_B.SignPreSat_h = pid_control_V1_B.frac[1] /
      pid_control_V1_B.Sum2_l;

    /* Product: '<S305>/w' incorporates:
     *  Gain: '<S305>/1//pi'
     *  Integrator: '<S305>/wg_p1'
     *  Product: '<S305>/Lug//V1'
     *  Sqrt: '<S305>/sqrt1'
     *  Sum: '<S305>/Sum'
     */
    pid_control_V1_B.Sum1_g = (sqrt(0.31830988618379069 *
      pid_control_V1_B.SignPreSat_h) * pid_control_V1_B.Product[2] -
      pid_control_V1_X.wg_p1_CSTATE[1]) / pid_control_V1_B.SignPreSat_h;
    pid_control_V1_B.w[1] = pid_control_V1_B.Sum1_g;

    /* Product: '<S305>/w ' incorporates:
     *  Integrator: '<S305>/wg_p1'
     *  Integrator: '<S305>/wg_p2'
     *  Product: '<S305>/Lwg//V'
     *  Product: '<S305>/Lwg//V '
     *  Sum: '<S305>/Sum1'
     */
    pid_control_V1_B.w_a[1] = (pid_control_V1_B.Sum1_g *
      pid_control_V1_ConstB.sqrt_a * pid_control_V1_B.SignPreSat_h +
      (pid_control_V1_X.wg_p1_CSTATE[1] - pid_control_V1_X.wg_p2_CSTATE[1])) /
      pid_control_V1_B.SignPreSat_h;

    /* Product: '<S305>/Lwg//V 1' incorporates:
     *  Integrator: '<S305>/wg_p2'
     */
    tmp_3 = _mm_mul_pd(_mm_set_pd(pid_control_V1_B.SignPreSat,
      pid_control_V1_ConstB.sigma_wg), _mm_loadu_pd
                       (&pid_control_V1_X.wg_p2_CSTATE[0]));

    /* Product: '<S305>/Lwg//V 1' */
    _mm_storeu_pd(&pid_control_V1_B.LwgV1[0], tmp_3);
  }

  /* End of Outputs for SubSystem: '<S290>/Hwgw(s)' */

  /* Angle2Dcm: '<S11>/Rotation Angles to Direction Cosine Matrix' */
  pid_control_V1_B.SignPreSat_h = cos(pid_control_V1_B.x[6]);
  pid_control_V1_B.Sum1_g = sin(pid_control_V1_B.x[6]);
  pid_control_V1_B.Sum_hl = -sin(pid_control_V1_B.x[6]);
  pid_control_V1_B.Gain2 = cos(pid_control_V1_B.x[6]);
  pid_control_V1_B.Dtot_f = cos(pid_control_V1_B.x[7]);
  pid_control_V1_B.FA_b_idx_0 = -sin(pid_control_V1_B.x[7]);
  pid_control_V1_B.Square1 = sin(pid_control_V1_B.x[7]);
  pid_control_V1_B.hh = cos(pid_control_V1_B.x[7]);
  pid_control_V1_B.hw = cos(pid_control_V1_B.x[8]);
  pid_control_V1_B.CL_w_IGE = sin(pid_control_V1_B.x[8]);
  pid_control_V1_B.CL_h_IGE = -sin(pid_control_V1_B.x[8]);
  pid_control_V1_B.Ltot_tmp = cos(pid_control_V1_B.x[8]);
  pid_control_V1_B.CD_iw_IGE = 0.0 * pid_control_V1_B.Square1 +
    pid_control_V1_B.Dtot_f;
  pid_control_V1_B.CD_ih_IGE = 0.0 * pid_control_V1_B.hh +
    pid_control_V1_B.FA_b_idx_0;
  pid_control_V1_B.Ltot = pid_control_V1_B.hw * 0.0;
  pid_control_V1_B.CQ = 0.0 * pid_control_V1_B.Dtot_f;
  pid_control_V1_B.Dtot_f = (pid_control_V1_B.CQ + pid_control_V1_B.Ltot) +
    pid_control_V1_B.CL_w_IGE * pid_control_V1_B.Square1;
  pid_control_V1_B.hw += pid_control_V1_B.CL_w_IGE * 0.0;
  pid_control_V1_B.FA_b_idx_0 *= 0.0;
  pid_control_V1_B.CL_w_IGE = (pid_control_V1_B.FA_b_idx_0 +
    pid_control_V1_B.Ltot) + pid_control_V1_B.CL_w_IGE * pid_control_V1_B.hh;
  pid_control_V1_B.Ltot = pid_control_V1_B.CL_h_IGE * 0.0;
  pid_control_V1_B.Square1 = (pid_control_V1_B.CQ + pid_control_V1_B.Ltot) +
    pid_control_V1_B.Square1 * pid_control_V1_B.Ltot_tmp;
  pid_control_V1_B.CL_h_IGE += pid_control_V1_B.Ltot_tmp * 0.0;
  pid_control_V1_B.hh = (pid_control_V1_B.FA_b_idx_0 + pid_control_V1_B.Ltot) +
    pid_control_V1_B.Ltot_tmp * pid_control_V1_B.hh;
  pid_control_V1_B.Ltot_tmp = pid_control_V1_B.CD_ih_IGE * 0.0;
  pid_control_V1_B.RotationAnglestoDirectionCo[0] = (pid_control_V1_B.CD_iw_IGE *
    pid_control_V1_B.SignPreSat_h + 0.0 * pid_control_V1_B.Sum_hl) +
    pid_control_V1_B.Ltot_tmp;
  pid_control_V1_B.Ltot = pid_control_V1_B.CL_w_IGE * 0.0;
  pid_control_V1_B.RotationAnglestoDirectionCo[1] =
    (pid_control_V1_B.SignPreSat_h * pid_control_V1_B.Dtot_f +
     pid_control_V1_B.Sum_hl * pid_control_V1_B.hw) + pid_control_V1_B.Ltot;
  pid_control_V1_B.CQ = pid_control_V1_B.hh * 0.0;
  pid_control_V1_B.RotationAnglestoDirectionCo[2] =
    (pid_control_V1_B.SignPreSat_h * pid_control_V1_B.Square1 +
     pid_control_V1_B.Sum_hl * pid_control_V1_B.CL_h_IGE) + pid_control_V1_B.CQ;
  pid_control_V1_B.RotationAnglestoDirectionCo[3] = (pid_control_V1_B.CD_iw_IGE *
    pid_control_V1_B.Sum1_g + 0.0 * pid_control_V1_B.Gain2) +
    pid_control_V1_B.Ltot_tmp;
  pid_control_V1_B.RotationAnglestoDirectionCo[4] = (pid_control_V1_B.Sum1_g *
    pid_control_V1_B.Dtot_f + pid_control_V1_B.hw * pid_control_V1_B.Gain2) +
    pid_control_V1_B.Ltot;
  pid_control_V1_B.RotationAnglestoDirectionCo[5] = (pid_control_V1_B.Sum1_g *
    pid_control_V1_B.Square1 + pid_control_V1_B.Gain2 *
    pid_control_V1_B.CL_h_IGE) + pid_control_V1_B.CQ;
  pid_control_V1_B.RotationAnglestoDirectionCo[6] = pid_control_V1_B.CD_iw_IGE *
    0.0 + pid_control_V1_B.CD_ih_IGE;
  pid_control_V1_B.RotationAnglestoDirectionCo[7] = (pid_control_V1_B.Dtot_f *
    0.0 + pid_control_V1_B.hw * 0.0) + pid_control_V1_B.CL_w_IGE;
  pid_control_V1_B.RotationAnglestoDirectionCo[8] = (pid_control_V1_B.Square1 *
    0.0 + pid_control_V1_B.CL_h_IGE * 0.0) + pid_control_V1_B.hh;

  /* If: '<S295>/if Height < Max low altitude  elseif Height > Min isotropic altitude ' incorporates:
   *  Constant: '<S316>/max_height_low'
   *  Product: '<S316>/Product1'
   *  Product: '<S321>/Product1'
   *  Product: '<S321>/Product2'
   *  Product: '<S323>/Product1'
   *  Product: '<S323>/Product2'
   *  Sum: '<S316>/Sum1'
   *  Sum: '<S316>/Sum2'
   *  Sum: '<S316>/Sum3'
   *  Sum: '<S321>/Sum'
   *  Sum: '<S323>/Sum'
   */
  rtPrevAction = pid_control_V1_DW.ifHeightMaxlowaltitudeelseifHei;
  if (tmp_1) {
    if (pid_control_V1_B.WhiteNoise_p <= 1000.0) {
      rtAction = 0;
    } else if (pid_control_V1_B.WhiteNoise_p >= 2000.0) {
      rtAction = 1;
    } else {
      rtAction = 2;
    }

    pid_control_V1_DW.ifHeightMaxlowaltitudeelseifHei = rtAction;
  } else {
    rtAction = pid_control_V1_DW.ifHeightMaxlowaltitudeelseifHei;
  }

  if (rtPrevAction != rtAction) {
    rtsiSetBlockStateForSolverChangedAtMajorStep(&(&pid_control_V1_M)
      ->solverInfo, true);
  }

  switch (rtAction) {
   case 0:
    /* Outputs for IfAction SubSystem: '<S295>/Low altitude  velocities' incorporates:
     *  ActionPort: '<S317>/Action Port'
     */
    /* SignalConversion generated from: '<S322>/Vector Concatenate' */
    pid_control_V1_B.Product_m[2] = pid_control_V1_B.LwgV1[0];

    /* Trigonometry: '<S323>/Trigonometric Function' incorporates:
     *  UnitConversion: '<S288>/Unit Conversion'
     */
    pid_control_V1_B.SignPreSat_h = sin(pid_control_V1_ConstB.UnitConversion);
    pid_control_V1_B.Sum1_g = cos(pid_control_V1_ConstB.UnitConversion);
    _mm_storeu_pd(&pid_control_V1_B.Product_m[0], _mm_add_pd(_mm_mul_pd
      (_mm_set_pd(pid_control_V1_B.SignPreSat_h, pid_control_V1_B.w1_c[0]),
       _mm_set_pd(pid_control_V1_B.w1_c[0], pid_control_V1_B.Sum1_g)),
      _mm_mul_pd(_mm_mul_pd(_mm_set_pd(pid_control_V1_B.w1[0],
      pid_control_V1_B.SignPreSat_h), _mm_set_pd(pid_control_V1_B.Sum1_g,
      pid_control_V1_B.w1[0])), _mm_set_pd(1.0, -1.0))));

    /* Product: '<S322>/Product' incorporates:
     *  Angle2Dcm: '<S11>/Rotation Angles to Direction Cosine Matrix'
     *  Concatenate: '<S322>/Vector Concatenate'
     *  Product: '<S323>/Product1'
     *  Product: '<S323>/Product2'
     *  Reshape: '<S322>/Reshape1'
     *  Sum: '<S323>/Sum'
     */
    pid_control_V1_B.SignPreSat_h = 0.0;
    pid_control_V1_B.Sum1_g = 0.0;
    pid_control_V1_B.Sum_hl = 0.0;
    for (i = 0; i < 3; i++) {
      tmp_3 = _mm_add_pd(_mm_mul_pd(_mm_loadu_pd
        (&pid_control_V1_B.RotationAnglestoDirectionCo[3 * i]), _mm_set1_pd
        (pid_control_V1_B.Product_m[i])), _mm_set_pd(pid_control_V1_B.Sum1_g,
        pid_control_V1_B.SignPreSat_h));
      _mm_storeu_pd(&pid_control_V1_B.dv1[0], tmp_3);
      pid_control_V1_B.SignPreSat_h = pid_control_V1_B.dv1[0];
      pid_control_V1_B.Sum1_g = pid_control_V1_B.dv1[1];
      pid_control_V1_B.Sum_hl += pid_control_V1_B.RotationAnglestoDirectionCo[3 *
        i + 2] * pid_control_V1_B.Product_m[i];
    }

    pid_control_V1_B.wbe_b[2] = pid_control_V1_B.Sum_hl;
    pid_control_V1_B.wbe_b[1] = pid_control_V1_B.Sum1_g;
    pid_control_V1_B.wbe_b[0] = pid_control_V1_B.SignPreSat_h;

    /* End of Product: '<S322>/Product' */
    /* End of Outputs for SubSystem: '<S295>/Low altitude  velocities' */
    break;

   case 1:
    /* Outputs for IfAction SubSystem: '<S295>/Medium//High  altitude velocities' incorporates:
     *  ActionPort: '<S318>/Action Port'
     */
    /* Gain: '<S318>/Gain' */
    pid_control_V1_B.wbe_b[0] = pid_control_V1_B.w1_c[1];
    pid_control_V1_B.wbe_b[1] = pid_control_V1_B.w1[1];
    pid_control_V1_B.wbe_b[2] = pid_control_V1_B.LwgV1[1];

    /* End of Outputs for SubSystem: '<S295>/Medium//High  altitude velocities' */
    break;

   default:
    /* Outputs for IfAction SubSystem: '<S295>/Interpolate  velocities' incorporates:
     *  ActionPort: '<S316>/Action Port'
     */
    /* Trigonometry: '<S321>/Trigonometric Function' incorporates:
     *  UnitConversion: '<S288>/Unit Conversion'
     */
    pid_control_V1_B.SignPreSat_h = sin(pid_control_V1_ConstB.UnitConversion);
    pid_control_V1_B.Sum1_g = cos(pid_control_V1_ConstB.UnitConversion);
    _mm_storeu_pd(&pid_control_V1_B.wbe_b[0], _mm_add_pd(_mm_mul_pd(_mm_set_pd
      (pid_control_V1_B.SignPreSat_h, pid_control_V1_B.w1_c[0]), _mm_set_pd
      (pid_control_V1_B.w1_c[0], pid_control_V1_B.Sum1_g)), _mm_mul_pd
      (_mm_mul_pd(_mm_set_pd(pid_control_V1_B.w1[0],
      pid_control_V1_B.SignPreSat_h), _mm_set_pd(pid_control_V1_B.Sum1_g,
      pid_control_V1_B.w1[0])), _mm_set_pd(1.0, -1.0))));

    /* SignalConversion generated from: '<S320>/Vector Concatenate' incorporates:
     *  Product: '<S321>/Product1'
     *  Product: '<S321>/Product2'
     *  Sum: '<S321>/Sum'
     */
    pid_control_V1_B.wbe_b[2] = pid_control_V1_B.LwgV1[0];

    /* Product: '<S320>/Product' incorporates:
     *  Angle2Dcm: '<S11>/Rotation Angles to Direction Cosine Matrix'
     *  Concatenate: '<S320>/Vector Concatenate'
     */
    pid_control_V1_B.SignPreSat_h = 0.0;
    pid_control_V1_B.Sum1_g = 0.0;
    pid_control_V1_B.Sum_hl = 0.0;
    for (i = 0; i < 3; i++) {
      tmp_3 = _mm_add_pd(_mm_mul_pd(_mm_loadu_pd
        (&pid_control_V1_B.RotationAnglestoDirectionCo[3 * i]), _mm_set1_pd
        (pid_control_V1_B.wbe_b[i])), _mm_set_pd(pid_control_V1_B.Sum1_g,
        pid_control_V1_B.SignPreSat_h));
      _mm_storeu_pd(&pid_control_V1_B.dv1[0], tmp_3);
      pid_control_V1_B.SignPreSat_h = pid_control_V1_B.dv1[0];
      pid_control_V1_B.Sum1_g = pid_control_V1_B.dv1[1];
      pid_control_V1_B.Sum_hl += pid_control_V1_B.RotationAnglestoDirectionCo[3 *
        i + 2] * pid_control_V1_B.wbe_b[i];
    }

    pid_control_V1_B.Product_m[2] = pid_control_V1_B.Sum_hl;
    pid_control_V1_B.Product_m[1] = pid_control_V1_B.Sum1_g;
    pid_control_V1_B.Product_m[0] = pid_control_V1_B.SignPreSat_h;
    tmp_3 = _mm_add_pd(_mm_div_pd(_mm_mul_pd(_mm_sub_pd(_mm_set_pd
      (pid_control_V1_B.w1[1], pid_control_V1_B.w1_c[1]), _mm_loadu_pd
      (&pid_control_V1_B.Product_m[0])), _mm_sub_pd(_mm_set1_pd
      (pid_control_V1_B.WhiteNoise_p), _mm_set1_pd(1000.0))), _mm_set1_pd
      (pid_control_V1_ConstB.Sum)), _mm_loadu_pd(&pid_control_V1_B.Product_m[0]));
    _mm_storeu_pd(&pid_control_V1_B.wbe_b[0], tmp_3);

    /* Sum: '<S316>/Sum3' incorporates:
     *  Constant: '<S316>/max_height_low'
     *  Product: '<S316>/Product1'
     *  Product: '<S320>/Product'
     *  Sum: '<S316>/Sum1'
     *  Sum: '<S316>/Sum2'
     */
    pid_control_V1_B.wbe_b[2] = (pid_control_V1_B.LwgV1[1] -
      pid_control_V1_B.Sum_hl) * (pid_control_V1_B.WhiteNoise_p - 1000.0) /
      pid_control_V1_ConstB.Sum + pid_control_V1_B.Sum_hl;

    /* End of Outputs for SubSystem: '<S295>/Interpolate  velocities' */
    break;
  }

  /* UnitConversion: '<S281>/Unit Conversion' */
  /* Unit Conversion - from: ft/s to: m/s
     Expression: output = (0.3048*input) + (0) */
  tmp_3 = _mm_mul_pd(_mm_set1_pd(0.3048), _mm_loadu_pd(&pid_control_V1_B.wbe_b[0]));
  _mm_storeu_pd(&pid_control_V1_B.wbe_b[0], tmp_3);
  pid_control_V1_B.wbe_b[2] *= 0.3048;
  if (tmp_0) {
    /* MATLABSystem: '<S287>/SourceBlock' */
    pid_control_V1_B.SourceBlock_o1_h = Sub_pid_control_V1_417.getLatestMessage(
      &rtb_SourceBlock_o2_j);

    /* Outputs for Enabled SubSystem: '<S287>/Enabled Subsystem' */
    pid_control_V1_EnabledSubsystem(pid_control_V1_B.SourceBlock_o1_h,
      &rtb_SourceBlock_o2_j, &pid_control_V1_B.EnabledSubsystem_pt);

    /* End of Outputs for SubSystem: '<S287>/Enabled Subsystem' */
  }

  /* Switch: '<S11>/Switch' */
  if (pid_control_V1_B.EnabledSubsystem_pt.In1.data) {
    /* Switch: '<S11>/Switch' */
    pid_control_V1_B.Switch_p[0] = pid_control_V1_B.wbe_b[0];
    pid_control_V1_B.Switch_p[1] = pid_control_V1_B.wbe_b[1];
    pid_control_V1_B.Switch_p[2] = pid_control_V1_B.wbe_b[2];
  } else {
    /* Switch: '<S11>/Switch' incorporates:
     *  Constant: '<S11>/Constant'
     */
    pid_control_V1_B.Switch_p[0] = 0.0;
    pid_control_V1_B.Switch_p[1] = 0.0;
    pid_control_V1_B.Switch_p[2] = 0.0;
  }

  /* End of Switch: '<S11>/Switch' */

  /* Outputs for Enabled SubSystem: '<S289>/Hpgw' incorporates:
   *  EnablePort: '<S300>/Enable'
   */
  if (tmp_0 && tmp_1 && (!pid_control_V1_DW.Hpgw_MODE)) {
    (void) memset(&(pid_control_V1_XDis.pgw_p_CSTATE), 0,
                  2*sizeof(boolean_T));

    /* InitializeConditions for Integrator: '<S300>/pgw_p' */
    pid_control_V1_X.pgw_p_CSTATE[0] = 0.0;
    pid_control_V1_X.pgw_p_CSTATE[1] = 0.0;
    pid_control_V1_DW.Hpgw_MODE = true;
  }

  if (pid_control_V1_DW.Hpgw_MODE) {
    /* Fcn: '<S300>/sqrt(0.8//V)' */
    pid_control_V1_B.Gain2 = 0.8 / pid_control_V1_B.Sum2_l;
    if (pid_control_V1_B.Gain2 < 0.0) {
      pid_control_V1_B.Gain2 = -sqrt(-pid_control_V1_B.Gain2);
    } else {
      pid_control_V1_B.Gain2 = sqrt(pid_control_V1_B.Gain2);
    }

    /* Product: '<S300>/w3' */
    pid_control_V1_B.SignPreSat_h = pid_control_V1_B.Sum2_l *
      pid_control_V1_ConstB.w4;

    /* Product: '<S300>/w' incorporates:
     *  Fcn: '<S300>/sqrt(0.8//V)'
     *  Gain: '<S296>/Lw'
     *  Integrator: '<S300>/pgw_p'
     *  Math: '<S300>/L^1//3'
     *  Product: '<S300>/Lug//V1'
     *  Product: '<S300>/w1'
     *  Product: '<S300>/w2'
     *  Sum: '<S300>/Sum'
     */
    pid_control_V1_B.w_o[0] = (pid_control_V1_B.Gain2 / rt_powd_snf
      (pid_control_V1_B.Sum_b, 0.33333333333333331) * pid_control_V1_ConstB.u16 *
      pid_control_V1_B.Product[3] - pid_control_V1_X.pgw_p_CSTATE[0]) *
      pid_control_V1_B.SignPreSat_h;

    /* Math: '<S300>/L^1//3' */
    if (pid_control_V1_B.frac[1] < 0.0) {
      pid_control_V1_B.Sum_b = -rt_powd_snf(-pid_control_V1_B.frac[1],
        0.33333333333333331);
    } else {
      pid_control_V1_B.Sum_b = rt_powd_snf(pid_control_V1_B.frac[1],
        0.33333333333333331);
    }

    /* Product: '<S300>/w' incorporates:
     *  Fcn: '<S300>/sqrt(0.8//V)'
     *  Integrator: '<S300>/pgw_p'
     *  Math: '<S300>/L^1//3'
     *  Product: '<S300>/Lug//V1'
     *  Product: '<S300>/w1'
     *  Product: '<S300>/w2'
     *  Sum: '<S300>/Sum'
     */
    pid_control_V1_B.w_o[1] = (pid_control_V1_B.Gain2 / pid_control_V1_B.Sum_b *
      pid_control_V1_ConstB.u16 * pid_control_V1_B.Product[3] -
      pid_control_V1_X.pgw_p_CSTATE[1]) * pid_control_V1_B.SignPreSat_h;

    /* Product: '<S300>/sigma_w' incorporates:
     *  Integrator: '<S300>/pgw_p'
     */
    tmp_3 = _mm_mul_pd(_mm_set_pd(pid_control_V1_B.SignPreSat,
      pid_control_V1_ConstB.sigma_wg), _mm_loadu_pd
                       (&pid_control_V1_X.pgw_p_CSTATE[0]));

    /* Product: '<S300>/sigma_w' */
    _mm_storeu_pd(&pid_control_V1_B.sigma_w[0], tmp_3);
  }

  /* End of Outputs for SubSystem: '<S289>/Hpgw' */

  /* Outputs for Enabled SubSystem: '<S289>/Hqgw' incorporates:
   *  EnablePort: '<S301>/Enable'
   */
  if (tmp_0 && tmp_1 && (!pid_control_V1_DW.Hqgw_MODE)) {
    (void) memset(&(pid_control_V1_XDis.qgw_p_CSTATE), 0,
                  2*sizeof(boolean_T));

    /* InitializeConditions for Integrator: '<S301>/qgw_p' */
    pid_control_V1_X.qgw_p_CSTATE[0] = 0.0;
    pid_control_V1_X.qgw_p_CSTATE[1] = 0.0;
    pid_control_V1_DW.Hqgw_MODE = true;
  }

  if (pid_control_V1_DW.Hqgw_MODE) {
    /* Gain: '<S301>/pi//4' */
    pid_control_V1_B.Sum_b = 0.78539816339744828 * pid_control_V1_B.Sum2_l;

    /* Product: '<S301>/w' incorporates:
     *  Integrator: '<S301>/qgw_p'
     *  Product: '<S301>/wg//V'
     *  Sum: '<S301>/Sum'
     */
    pid_control_V1_B.SignPreSat = (pid_control_V1_B.LwgV1[0] /
      pid_control_V1_B.Sum2_l - pid_control_V1_X.qgw_p_CSTATE[0]) *
      (pid_control_V1_B.Sum_b / pid_control_V1_ConstB.UnitConversion_n);
    pid_control_V1_B.w_e0[0] = pid_control_V1_B.SignPreSat;

    /* UnaryMinus: '<S301>/Unary Minus' */
    pid_control_V1_B.UnaryMinus[0] = -pid_control_V1_B.SignPreSat;

    /* Product: '<S301>/w' incorporates:
     *  Integrator: '<S301>/qgw_p'
     *  Product: '<S301>/wg//V'
     *  Sum: '<S301>/Sum'
     */
    pid_control_V1_B.SignPreSat = (pid_control_V1_B.LwgV1[1] /
      pid_control_V1_B.Sum2_l - pid_control_V1_X.qgw_p_CSTATE[1]) *
      (pid_control_V1_B.Sum_b / pid_control_V1_ConstB.UnitConversion_n);
    pid_control_V1_B.w_e0[1] = pid_control_V1_B.SignPreSat;

    /* UnaryMinus: '<S301>/Unary Minus' */
    pid_control_V1_B.UnaryMinus[1] = -pid_control_V1_B.SignPreSat;
  }

  /* End of Outputs for SubSystem: '<S289>/Hqgw' */

  /* Outputs for Enabled SubSystem: '<S289>/Hrgw' incorporates:
   *  EnablePort: '<S302>/Enable'
   */
  if (tmp_0 && tmp_1 && (!pid_control_V1_DW.Hrgw_MODE)) {
    (void) memset(&(pid_control_V1_XDis.rgw_p_CSTATE), 0,
                  2*sizeof(boolean_T));

    /* InitializeConditions for Integrator: '<S302>/rgw_p' */
    pid_control_V1_X.rgw_p_CSTATE[0] = 0.0;
    pid_control_V1_X.rgw_p_CSTATE[1] = 0.0;
    pid_control_V1_DW.Hrgw_MODE = true;
  }

  if (pid_control_V1_DW.Hrgw_MODE) {
    /* Product: '<S302>/vg//V' incorporates:
     *  Gain: '<S302>/pi//3'
     *  Integrator: '<S302>/rgw_p'
     *  Product: '<S302>/w'
     */
    tmp_3 = _mm_mul_pd(_mm_sub_pd(_mm_div_pd(_mm_loadu_pd(&pid_control_V1_B.w1[0]),
      _mm_set1_pd(pid_control_V1_B.Sum2_l)), _mm_loadu_pd
      (&pid_control_V1_X.rgw_p_CSTATE[0])), _mm_div_pd(_mm_set1_pd
      (1.0471975511965976 * pid_control_V1_B.Sum2_l), _mm_set1_pd
      (pid_control_V1_ConstB.UnitConversion_n)));

    /* Product: '<S302>/w' */
    _mm_storeu_pd(&pid_control_V1_B.w_d[0], tmp_3);
  }

  /* End of Outputs for SubSystem: '<S289>/Hrgw' */

  /* If: '<S294>/if Height < Max low altitude  elseif Height > Min isotropic altitude ' incorporates:
   *  Constant: '<S308>/max_height_low'
   *  Product: '<S308>/Product1'
   *  Product: '<S313>/Product1'
   *  Product: '<S313>/Product2'
   *  Product: '<S315>/Product1'
   *  Product: '<S315>/Product2'
   *  Sum: '<S308>/Sum1'
   *  Sum: '<S308>/Sum2'
   *  Sum: '<S308>/Sum3'
   *  Sum: '<S313>/Sum'
   *  Sum: '<S315>/Sum'
   */
  rtPrevAction = pid_control_V1_DW.ifHeightMaxlowaltitudeelseifH_a;
  if (tmp_1) {
    if (pid_control_V1_B.WhiteNoise_p <= 1000.0) {
      rtAction = 0;
    } else if (pid_control_V1_B.WhiteNoise_p >= 2000.0) {
      rtAction = 1;
    } else {
      rtAction = 2;
    }

    pid_control_V1_DW.ifHeightMaxlowaltitudeelseifH_a = rtAction;
  } else {
    rtAction = pid_control_V1_DW.ifHeightMaxlowaltitudeelseifH_a;
  }

  if (rtPrevAction != rtAction) {
    rtsiSetBlockStateForSolverChangedAtMajorStep(&(&pid_control_V1_M)
      ->solverInfo, true);
  }

  switch (rtAction) {
   case 0:
    /* Outputs for IfAction SubSystem: '<S294>/Low altitude  rates' incorporates:
     *  ActionPort: '<S309>/Action Port'
     */
    /* SignalConversion generated from: '<S314>/Vector Concatenate' */
    pid_control_V1_B.Product_m[2] = pid_control_V1_B.w_d[0];

    /* Trigonometry: '<S315>/Trigonometric Function1' incorporates:
     *  UnitConversion: '<S288>/Unit Conversion'
     */
    pid_control_V1_B.WhiteNoise_p = sin(pid_control_V1_ConstB.UnitConversion);
    pid_control_V1_B.Sum2_l = cos(pid_control_V1_ConstB.UnitConversion);
    _mm_storeu_pd(&pid_control_V1_B.Product_m[0], _mm_add_pd(_mm_mul_pd
      (_mm_set_pd(pid_control_V1_B.WhiteNoise_p, pid_control_V1_B.sigma_w[0]),
       _mm_set_pd(pid_control_V1_B.sigma_w[0], pid_control_V1_B.Sum2_l)),
      _mm_mul_pd(_mm_mul_pd(_mm_set_pd(pid_control_V1_B.UnaryMinus[0],
      pid_control_V1_B.WhiteNoise_p), _mm_set_pd(pid_control_V1_B.Sum2_l,
      pid_control_V1_B.UnaryMinus[0])), _mm_set_pd(1.0, -1.0))));

    /* Product: '<S314>/Product' incorporates:
     *  Angle2Dcm: '<S11>/Rotation Angles to Direction Cosine Matrix'
     *  Concatenate: '<S314>/Vector Concatenate'
     *  Product: '<S315>/Product1'
     *  Product: '<S315>/Product2'
     *  Reshape: '<S314>/Reshape1'
     *  Sum: '<S315>/Sum'
     */
    pid_control_V1_B.SignPreSat_h = 0.0;
    pid_control_V1_B.Sum1_g = 0.0;
    pid_control_V1_B.Sum_hl = 0.0;
    for (i = 0; i < 3; i++) {
      tmp_3 = _mm_add_pd(_mm_mul_pd(_mm_loadu_pd
        (&pid_control_V1_B.RotationAnglestoDirectionCo[3 * i]), _mm_set1_pd
        (pid_control_V1_B.Product_m[i])), _mm_set_pd(pid_control_V1_B.Sum1_g,
        pid_control_V1_B.SignPreSat_h));
      _mm_storeu_pd(&pid_control_V1_B.dv1[0], tmp_3);
      pid_control_V1_B.SignPreSat_h = pid_control_V1_B.dv1[0];
      pid_control_V1_B.Sum1_g = pid_control_V1_B.dv1[1];
      pid_control_V1_B.Sum_hl += pid_control_V1_B.RotationAnglestoDirectionCo[3 *
        i + 2] * pid_control_V1_B.Product_m[i];
    }

    pid_control_V1_B.wbe_b[2] = pid_control_V1_B.Sum_hl;
    pid_control_V1_B.wbe_b[1] = pid_control_V1_B.Sum1_g;
    pid_control_V1_B.wbe_b[0] = pid_control_V1_B.SignPreSat_h;

    /* End of Product: '<S314>/Product' */
    /* End of Outputs for SubSystem: '<S294>/Low altitude  rates' */
    break;

   case 1:
    /* Outputs for IfAction SubSystem: '<S294>/Medium//High  altitude rates' incorporates:
     *  ActionPort: '<S310>/Action Port'
     */
    /* Gain: '<S310>/Gain' */
    pid_control_V1_B.wbe_b[0] = pid_control_V1_B.sigma_w[1];
    pid_control_V1_B.wbe_b[1] = pid_control_V1_B.UnaryMinus[1];
    pid_control_V1_B.wbe_b[2] = pid_control_V1_B.w_d[1];

    /* End of Outputs for SubSystem: '<S294>/Medium//High  altitude rates' */
    break;

   default:
    /* Outputs for IfAction SubSystem: '<S294>/Interpolate  rates' incorporates:
     *  ActionPort: '<S308>/Action Port'
     */
    /* Trigonometry: '<S313>/Trigonometric Function' incorporates:
     *  UnitConversion: '<S288>/Unit Conversion'
     */
    pid_control_V1_B.Sum2_l = sin(pid_control_V1_ConstB.UnitConversion);
    pid_control_V1_B.Sum_b = cos(pid_control_V1_ConstB.UnitConversion);
    _mm_storeu_pd(&pid_control_V1_B.wbe_b[0], _mm_add_pd(_mm_mul_pd(_mm_set_pd
      (pid_control_V1_B.Sum2_l, pid_control_V1_B.sigma_w[0]), _mm_set_pd
      (pid_control_V1_B.sigma_w[0], pid_control_V1_B.Sum_b)), _mm_mul_pd
      (_mm_mul_pd(_mm_set_pd(pid_control_V1_B.UnaryMinus[0],
      pid_control_V1_B.Sum2_l), _mm_set_pd(pid_control_V1_B.Sum_b,
      pid_control_V1_B.UnaryMinus[0])), _mm_set_pd(1.0, -1.0))));

    /* SignalConversion generated from: '<S312>/Vector Concatenate' incorporates:
     *  Product: '<S313>/Product1'
     *  Product: '<S313>/Product2'
     *  Sum: '<S313>/Sum'
     */
    pid_control_V1_B.wbe_b[2] = pid_control_V1_B.w_d[0];

    /* Product: '<S312>/Product' incorporates:
     *  Angle2Dcm: '<S11>/Rotation Angles to Direction Cosine Matrix'
     *  Concatenate: '<S312>/Vector Concatenate'
     */
    pid_control_V1_B.SignPreSat_h = 0.0;
    pid_control_V1_B.Sum1_g = 0.0;
    pid_control_V1_B.Sum_hl = 0.0;
    for (i = 0; i < 3; i++) {
      tmp_3 = _mm_add_pd(_mm_mul_pd(_mm_loadu_pd
        (&pid_control_V1_B.RotationAnglestoDirectionCo[3 * i]), _mm_set1_pd
        (pid_control_V1_B.wbe_b[i])), _mm_set_pd(pid_control_V1_B.Sum1_g,
        pid_control_V1_B.SignPreSat_h));
      _mm_storeu_pd(&pid_control_V1_B.dv1[0], tmp_3);
      pid_control_V1_B.SignPreSat_h = pid_control_V1_B.dv1[0];
      pid_control_V1_B.Sum1_g = pid_control_V1_B.dv1[1];
      pid_control_V1_B.Sum_hl += pid_control_V1_B.RotationAnglestoDirectionCo[3 *
        i + 2] * pid_control_V1_B.wbe_b[i];
    }

    pid_control_V1_B.Product_m[2] = pid_control_V1_B.Sum_hl;
    pid_control_V1_B.Product_m[1] = pid_control_V1_B.Sum1_g;
    pid_control_V1_B.Product_m[0] = pid_control_V1_B.SignPreSat_h;
    tmp_3 = _mm_add_pd(_mm_div_pd(_mm_mul_pd(_mm_sub_pd(_mm_set_pd
      (pid_control_V1_B.UnaryMinus[1], pid_control_V1_B.sigma_w[1]),
      _mm_loadu_pd(&pid_control_V1_B.Product_m[0])), _mm_sub_pd(_mm_set1_pd
      (pid_control_V1_B.WhiteNoise_p), _mm_set1_pd(1000.0))), _mm_set1_pd
      (pid_control_V1_ConstB.Sum_a)), _mm_loadu_pd(&pid_control_V1_B.Product_m[0]));
    _mm_storeu_pd(&pid_control_V1_B.wbe_b[0], tmp_3);

    /* Sum: '<S308>/Sum3' incorporates:
     *  Constant: '<S308>/max_height_low'
     *  Product: '<S308>/Product1'
     *  Product: '<S312>/Product'
     *  Sum: '<S308>/Sum1'
     *  Sum: '<S308>/Sum2'
     */
    pid_control_V1_B.wbe_b[2] = (pid_control_V1_B.w_d[1] -
      pid_control_V1_B.Sum_hl) * (pid_control_V1_B.WhiteNoise_p - 1000.0) /
      pid_control_V1_ConstB.Sum_a + pid_control_V1_B.Sum_hl;

    /* End of Outputs for SubSystem: '<S294>/Interpolate  rates' */
    break;
  }

  if (tmp_0) {
    /* MATLABSystem: '<S284>/SourceBlock' */
    pid_control_V1_B.SourceBlock_o1_k = Sub_pid_control_V1_423.getLatestMessage(
      &rtb_SourceBlock_o2_dd);

    /* Outputs for Enabled SubSystem: '<S284>/Enabled Subsystem' */
    pid_control_V1_EnabledSubsystem(pid_control_V1_B.SourceBlock_o1_k,
      &rtb_SourceBlock_o2_dd, &pid_control_V1_B.EnabledSubsystem);

    /* End of Outputs for SubSystem: '<S284>/Enabled Subsystem' */
  }

  /* Switch: '<S11>/Switch1' */
  if (pid_control_V1_B.EnabledSubsystem.In1.data) {
    /* Switch: '<S11>/Switch1' */
    pid_control_V1_B.Switch1[0] = pid_control_V1_B.wbe_b[0];
    pid_control_V1_B.Switch1[1] = pid_control_V1_B.wbe_b[1];
    pid_control_V1_B.Switch1[2] = pid_control_V1_B.wbe_b[2];
  } else {
    /* Switch: '<S11>/Switch1' incorporates:
     *  Constant: '<S11>/Constant2'
     */
    pid_control_V1_B.Switch1[0] = 0.0;
    pid_control_V1_B.Switch1[1] = 0.0;
    pid_control_V1_B.Switch1[2] = 0.0;
  }

  /* End of Switch: '<S11>/Switch1' */
  if (tmp_0) {
    /* MATLABSystem: '<S285>/SourceBlock' */
    pid_control_V1_B.SourceBlock_o1_c = Sub_pid_control_V1_443.getLatestMessage(
      &pid_control_V1_B.SourceBlock_o2_p);

    /* Outputs for Enabled SubSystem: '<S285>/Enabled Subsystem' */
    pid_control__EnabledSubsystem_k(pid_control_V1_B.SourceBlock_o1_c,
      &pid_control_V1_B.SourceBlock_o2_p, &pid_control_V1_B.EnabledSubsystem_k);

    /* End of Outputs for SubSystem: '<S285>/Enabled Subsystem' */

    /* SignalConversion generated from: '<S11>/Bus Selector2' */
    pid_control_V1_B.data = pid_control_V1_B.EnabledSubsystem_k.In1.data;

    /* MATLABSystem: '<S286>/SourceBlock' */
    pid_control_V1_B.SourceBlock_o1_d = Sub_pid_control_V1_445.getLatestMessage(
      &pid_control_V1_B.SourceBlock_o2_k);

    /* Outputs for Enabled SubSystem: '<S286>/Enabled Subsystem' */
    pid_control__EnabledSubsystem_k(pid_control_V1_B.SourceBlock_o1_d,
      &pid_control_V1_B.SourceBlock_o2_k, &pid_control_V1_B.EnabledSubsystem_p);

    /* End of Outputs for SubSystem: '<S286>/Enabled Subsystem' */

    /* SignalConversion generated from: '<S11>/Bus Selector3' */
    pid_control_V1_B.data_n = pid_control_V1_B.EnabledSubsystem_p.In1.data;
  }

  /* Product: '<S11>/Product2' incorporates:
   *  Math: '<S11>/Square'
   *  Math: '<S11>/Square1'
   *  Math: '<S11>/Square2'
   *  Sqrt: '<S11>/Sqrt'
   *  Sum: '<S11>/Sum2'
   */
  pid_control_V1_B.Power = sqrt((pid_control_V1_B.x[0] * pid_control_V1_B.x[0] +
    pid_control_V1_B.x[1] * pid_control_V1_B.x[1]) + pid_control_V1_B.x[2] *
    pid_control_V1_B.x[2]) * pid_control_V1_B.XDOT[34];

  /* Gain: '<S11>/Gain3' */
  pid_control_V1_B.Gain3 = 0.001 * pid_control_V1_B.Power;
  if (tmp_0) {
  }

  /* Gain: '<S11>/Gain1' incorporates:
   *  Integrator: '<S11>/Integrator1'
   */
  pid_control_V1_B.EnergykWh = 2.7777777777777776E-7 *
    pid_control_V1_X.Integrator1_CSTATE;
  if (tmp_0) {
    /* SignalConversion generated from: '<S11>/To Workspace1' */
    pid_control_V1_B.TmpSignalConversionAtSFunct[0] =
      pid_control_V1_B.Saturation_k;
    pid_control_V1_B.TmpSignalConversionAtSFunct[1] =
      pid_control_V1_B.Saturation_f;
    pid_control_V1_B.TmpSignalConversionAtSFunct[2] =
      pid_control_V1_B.Saturation_m;
    pid_control_V1_B.TmpSignalConversionAtSFunct[3] =
      pid_control_V1_B.Saturation1;
    pid_control_V1_B.TmpSignalConversionAtSFunct[4] =
      pid_control_V1_B.Saturation2;
  }

  /* Product: '<S11>/Divide' incorporates:
   *  Constant: '<S11>/thrust efficiency Cp?'
   */
  pid_control_V1_B.powerdemand = pid_control_V1_B.Gain3 / 0.57;
  if (tmp_0) {
  }

  /* Product: '<S11>/Divide1' */
  pid_control_V1_B.loadtorque = pid_control_V1_B.powerdemand /
    pid_control_V1_ConstB.motorspeed;
  if (tmp_0) {
    /* Gain: '<S279>/Output' incorporates:
     *  RandomNumber: '<S279>/White Noise'
     */
    pid_control_V1_B.Output = 10.0 * pid_control_V1_DW.NextOutput_k;
  }

  /* TransferFcn: '<S11>/Transfer Fcn' */
  pid_control_V1_B.Sum_b = 0.5303 * pid_control_V1_X.TransferFcn_CSTATE[0] + 0.0
    * pid_control_V1_X.TransferFcn_CSTATE[1];

  /* Switch: '<S11>/Switch2' incorporates:
   *  Constant: '<S11>/Constant3'
   */
  if (!(pid_control_V1_B.data != 0.0)) {
    pid_control_V1_B.Sum_b = 0.0;
  }

  /* End of Switch: '<S11>/Switch2' */

  /* Sum: '<S11>/Sum' */
  pid_control_V1_B.Sum[0] = pid_control_V1_B.Switch_p[0];
  pid_control_V1_B.Sum[1] = pid_control_V1_B.Switch_p[1] +
    pid_control_V1_B.Sum_b;
  pid_control_V1_B.Sum[2] = pid_control_V1_B.Switch_p[2];

  /* Sum: '<S11>/Sum1' */
  pid_control_V1_B.Sum1[0] = pid_control_V1_B.Switch1[0];

  /* Switch: '<S11>/Switch3' incorporates:
   *  Constant: '<S11>/Constant4'
   *  TransferFcn: '<S11>/Transfer Fcn1'
   */
  if (pid_control_V1_B.data_n != 0.0) {
    pid_control_V1_B.Gain2 = -0.0003571 * pid_control_V1_X.TransferFcn1_CSTATE +
      0.03571 * pid_control_V1_B.Output;
  } else {
    pid_control_V1_B.Gain2 = 0.0;
  }

  /* Sum: '<S11>/Sum1' incorporates:
   *  Switch: '<S11>/Switch3'
   */
  pid_control_V1_B.Sum1[1] = pid_control_V1_B.Switch1[1] +
    pid_control_V1_B.Gain2;
  pid_control_V1_B.Sum1[2] = pid_control_V1_B.Switch1[2];
  if (rtmIsMajorTimeStep((&pid_control_V1_M))) {
    if (rtmIsMajorTimeStep((&pid_control_V1_M))) {
      /* Update for Memory: '<S11>/Memory2' incorporates:
       *  Integrator: '<S11>/Integrator'
       */
      memcpy(&pid_control_V1_DW.Memory2_PreviousInput[0], &pid_control_V1_B.x[0],
             12U * sizeof(real_T));

      /* Update for UnitDelay: '<Root>/Unit Delay3' */
      pid_control_V1_DW.UnitDelay3_DSTATE = pid_control_V1_B.Switch3;

      /* Update for UnitDelay: '<Root>/Unit Delay1' */
      pid_control_V1_DW.UnitDelay1_DSTATE = pid_control_V1_B.Switch1_g;

      /* Update for UnitDelay: '<Root>/Unit Delay2' */
      pid_control_V1_DW.UnitDelay2_DSTATE = pid_control_V1_B.Switch2;

      /* Update for UnitDelay: '<Root>/Unit Delay4' */
      pid_control_V1_DW.UnitDelay4_DSTATE = pid_control_V1_B.Va_out;

      /* Update for UnitDelay: '<Root>/Unit Delay' */
      pid_control_V1_DW.UnitDelay_DSTATE = pid_control_V1_B.XDOT[24];

      /* Update for Memory: '<S11>/Memory' incorporates:
       *  Sum: '<S11>/Sum'
       */
      pid_control_V1_DW.Memory_PreviousInput[0] = pid_control_V1_B.Sum[0];

      /* Update for Memory: '<S11>/Memory1' incorporates:
       *  Sum: '<S11>/Sum1'
       */
      pid_control_V1_DW.Memory1_PreviousInput[0] = pid_control_V1_B.Sum1[0];

      /* Update for Memory: '<S11>/Memory' incorporates:
       *  Sum: '<S11>/Sum'
       */
      pid_control_V1_DW.Memory_PreviousInput[1] = pid_control_V1_B.Sum[1];

      /* Update for Memory: '<S11>/Memory1' incorporates:
       *  Sum: '<S11>/Sum1'
       */
      pid_control_V1_DW.Memory1_PreviousInput[1] = pid_control_V1_B.Sum1[1];

      /* Update for Memory: '<S11>/Memory' incorporates:
       *  Sum: '<S11>/Sum'
       */
      pid_control_V1_DW.Memory_PreviousInput[2] = pid_control_V1_B.Sum[2];

      /* Update for Memory: '<S11>/Memory1' incorporates:
       *  Sum: '<S11>/Sum1'
       */
      pid_control_V1_DW.Memory1_PreviousInput[2] = pid_control_V1_B.Sum1[2];

      /* Update for Memory: '<S41>/Memory' */
      pid_control_V1_DW.Memory_PreviousInput_o = pid_control_V1_B.AND3;

      /* Update for Memory: '<S251>/Memory' */
      pid_control_V1_DW.Memory_PreviousInput_a = pid_control_V1_B.AND3_c;

      /* Update for RandomNumber: '<S299>/White Noise' */
      pid_control_V1_DW.NextOutput[0] = rt_nrand_Upu32_Yd_f_pw_snf
        (&pid_control_V1_DW.RandSeed[0]);
      pid_control_V1_DW.NextOutput[1] = rt_nrand_Upu32_Yd_f_pw_snf
        (&pid_control_V1_DW.RandSeed[1]);
      pid_control_V1_DW.NextOutput[2] = rt_nrand_Upu32_Yd_f_pw_snf
        (&pid_control_V1_DW.RandSeed[2]);
      pid_control_V1_DW.NextOutput[3] = rt_nrand_Upu32_Yd_f_pw_snf
        (&pid_control_V1_DW.RandSeed[3]);

      /* Update for RandomNumber: '<S279>/White Noise' */
      pid_control_V1_DW.NextOutput_k = rt_nrand_Upu32_Yd_f_pw_snf
        (&pid_control_V1_DW.RandSeed_a);
    }

    /* Update for Integrator: '<S11>/Integrator' */
    pid_control_V1_DW.Integrator_DWORK1 = false;

    /* Update for RateLimiter: '<Root>/Rate Limiter' */
    pid_control_V1_DW.PrevY = pid_control_V1_B.RateLimiter;
    pid_control_V1_DW.LastMajorTime = (&pid_control_V1_M)->Timing.t[0];

    /* ContTimeOutputInconsistentWithStateAtMajorOutputFlag is set, need to run a minor output */
    if (rtmIsMajorTimeStep((&pid_control_V1_M))) {
      if (rtsiGetContTimeOutputInconsistentWithStateAtMajorStep
          (&(&pid_control_V1_M)->solverInfo)) {
        rtsiSetSimTimeStep(&(&pid_control_V1_M)->solverInfo,MINOR_TIME_STEP);
        rtsiSetContTimeOutputInconsistentWithStateAtMajorStep
          (&(&pid_control_V1_M)->solverInfo, false);
        pid_control_V1::step();
        rtsiSetSimTimeStep(&(&pid_control_V1_M)->solverInfo, MAJOR_TIME_STEP);
      }
    }
  }                                    /* end MajorTimeStep */

  if (rtmIsMajorTimeStep((&pid_control_V1_M))) {
    rt_ertODEUpdateContinuousStates(&(&pid_control_V1_M)->solverInfo);

    /* Update absolute time for base rate */
    /* The "clockTick0" counts the number of times the code of this task has
     * been executed. The absolute time is the multiplication of "clockTick0"
     * and "Timing.stepSize0". Size of "clockTick0" ensures timer will not
     * overflow during the application lifespan selected.
     * Timer of this task consists of two 32 bit unsigned integers.
     * The two integers represent the low bits Timing.clockTick0 and the high bits
     * Timing.clockTickH0. When the low bit overflows to 0, the high bits increment.
     */
    if (!(++(&pid_control_V1_M)->Timing.clockTick0)) {
      ++(&pid_control_V1_M)->Timing.clockTickH0;
    }

    (&pid_control_V1_M)->Timing.t[0] = rtsiGetSolverStopTime(&(&pid_control_V1_M)
      ->solverInfo);

    {
      /* Update absolute timer for sample time: [0.01s, 0.0s] */
      /* The "clockTick1" counts the number of times the code of this task has
       * been executed. The resolution of this integer timer is 0.01, which is the step size
       * of the task. Size of "clockTick1" ensures timer will not overflow during the
       * application lifespan selected.
       * Timer of this task consists of two 32 bit unsigned integers.
       * The two integers represent the low bits Timing.clockTick1 and the high bits
       * Timing.clockTickH1. When the low bit overflows to 0, the high bits increment.
       */
      (&pid_control_V1_M)->Timing.clockTick1++;
      if (!(&pid_control_V1_M)->Timing.clockTick1) {
        (&pid_control_V1_M)->Timing.clockTickH1++;
      }
    }
  }                                    /* end MajorTimeStep */
}

/* Derivatives for root system: '<Root>' */
void pid_control_V1::pid_control_V1_derivatives()
{
  XDot_pid_control_V1_T *_rtXdot;
  real_T tmp[2];
  _rtXdot = ((XDot_pid_control_V1_T *) (&pid_control_V1_M)->derivs);

  /* Derivatives for Integrator: '<S11>/Integrator' */
  if (!pid_control_V1_B.Compare) {
    memcpy(&_rtXdot->Integrator_CSTATE[0], &pid_control_V1_B.XDOT[0], 12U *
           sizeof(real_T));
  } else {
    /* level reset is active */
    memset(&_rtXdot->Integrator_CSTATE[0], 0, 12U * sizeof(real_T));
  }

  /* End of Derivatives for Integrator: '<S11>/Integrator' */

  /* Derivatives for Integrator: '<S103>/Integrator' */
  _rtXdot->Integrator_CSTATE_n = pid_control_V1_B.SumI4;

  /* Derivatives for Integrator: '<S98>/Filter' */
  _rtXdot->Filter_CSTATE = pid_control_V1_B.FilterCoefficient;

  /* Derivatives for Integrator: '<S51>/Integrator' */
  _rtXdot->Integrator_CSTATE_m = pid_control_V1_B.Switch;

  /* Derivatives for Integrator: '<S46>/Filter' */
  _rtXdot->Filter_CSTATE_g = pid_control_V1_B.FilterCoefficient_c;

  /* Derivatives for Integrator: '<S155>/Integrator' */
  _rtXdot->Integrator_CSTATE_p = pid_control_V1_B.SumI4_i;

  /* Derivatives for Integrator: '<S150>/Filter' */
  _rtXdot->Filter_CSTATE_m = pid_control_V1_B.FilterCoefficient_m;

  /* Derivatives for Integrator: '<S207>/Integrator' */
  _rtXdot->Integrator_CSTATE_d = pid_control_V1_B.IntegralGain;

  /* Derivatives for Integrator: '<S202>/Filter' */
  _rtXdot->Filter_CSTATE_f = pid_control_V1_B.FilterCoefficient_p;

  /* Derivatives for Integrator: '<S261>/Integrator' */
  _rtXdot->Integrator_CSTATE_f = pid_control_V1_B.Switch_j;

  /* Derivatives for Integrator: '<S256>/Filter' */
  _rtXdot->Filter_CSTATE_l = pid_control_V1_B.FilterCoefficient_cv;

  /* Derivatives for Enabled SubSystem: '<S290>/Hugw(s)' */
  if (pid_control_V1_DW.Hugws_MODE) {
    /* Derivatives for Integrator: '<S303>/ug_p' */
    _rtXdot->ug_p_CSTATE[0] = pid_control_V1_B.w_n[0];
    _rtXdot->ug_p_CSTATE[1] = pid_control_V1_B.w_n[1];
  } else {
    {
      real_T *dx;
      int_T i1;
      dx = &(((XDot_pid_control_V1_T *) (&pid_control_V1_M)->derivs)
             ->ug_p_CSTATE[0]);
      for (i1=0; i1 < 2; i1++) {
        dx[i1] = 0.0;
      }
    }
  }

  /* End of Derivatives for SubSystem: '<S290>/Hugw(s)' */

  /* Derivatives for Enabled SubSystem: '<S290>/Hvgw(s)' */
  if (pid_control_V1_DW.Hvgws_MODE) {
    /* Derivatives for Integrator: '<S304>/vg_p1' */
    _rtXdot->vg_p1_CSTATE[0] = pid_control_V1_B.w_g[0];

    /* Derivatives for Integrator: '<S304>/vgw_p2' */
    _rtXdot->vgw_p2_CSTATE[0] = pid_control_V1_B.w_e[0];

    /* Derivatives for Integrator: '<S304>/vg_p1' */
    _rtXdot->vg_p1_CSTATE[1] = pid_control_V1_B.w_g[1];

    /* Derivatives for Integrator: '<S304>/vgw_p2' */
    _rtXdot->vgw_p2_CSTATE[1] = pid_control_V1_B.w_e[1];
  } else {
    {
      real_T *dx;
      int_T i1;
      dx = &(((XDot_pid_control_V1_T *) (&pid_control_V1_M)->derivs)
             ->vg_p1_CSTATE[0]);
      for (i1=0; i1 < 4; i1++) {
        dx[i1] = 0.0;
      }
    }
  }

  /* End of Derivatives for SubSystem: '<S290>/Hvgw(s)' */

  /* Derivatives for Enabled SubSystem: '<S290>/Hwgw(s)' */
  if (pid_control_V1_DW.Hwgws_MODE) {
    /* Derivatives for Integrator: '<S305>/wg_p1' */
    _rtXdot->wg_p1_CSTATE[0] = pid_control_V1_B.w[0];

    /* Derivatives for Integrator: '<S305>/wg_p2' */
    _rtXdot->wg_p2_CSTATE[0] = pid_control_V1_B.w_a[0];

    /* Derivatives for Integrator: '<S305>/wg_p1' */
    _rtXdot->wg_p1_CSTATE[1] = pid_control_V1_B.w[1];

    /* Derivatives for Integrator: '<S305>/wg_p2' */
    _rtXdot->wg_p2_CSTATE[1] = pid_control_V1_B.w_a[1];
  } else {
    {
      real_T *dx;
      int_T i1;
      dx = &(((XDot_pid_control_V1_T *) (&pid_control_V1_M)->derivs)
             ->wg_p1_CSTATE[0]);
      for (i1=0; i1 < 4; i1++) {
        dx[i1] = 0.0;
      }
    }
  }

  /* End of Derivatives for SubSystem: '<S290>/Hwgw(s)' */

  /* Derivatives for Enabled SubSystem: '<S289>/Hpgw' */
  if (pid_control_V1_DW.Hpgw_MODE) {
    /* Derivatives for Integrator: '<S300>/pgw_p' */
    _rtXdot->pgw_p_CSTATE[0] = pid_control_V1_B.w_o[0];
    _rtXdot->pgw_p_CSTATE[1] = pid_control_V1_B.w_o[1];
  } else {
    {
      real_T *dx;
      int_T i1;
      dx = &(((XDot_pid_control_V1_T *) (&pid_control_V1_M)->derivs)
             ->pgw_p_CSTATE[0]);
      for (i1=0; i1 < 2; i1++) {
        dx[i1] = 0.0;
      }
    }
  }

  /* End of Derivatives for SubSystem: '<S289>/Hpgw' */

  /* Derivatives for Enabled SubSystem: '<S289>/Hqgw' */
  if (pid_control_V1_DW.Hqgw_MODE) {
    /* Derivatives for Integrator: '<S301>/qgw_p' */
    _rtXdot->qgw_p_CSTATE[0] = pid_control_V1_B.w_e0[0];
    _rtXdot->qgw_p_CSTATE[1] = pid_control_V1_B.w_e0[1];
  } else {
    {
      real_T *dx;
      int_T i1;
      dx = &(((XDot_pid_control_V1_T *) (&pid_control_V1_M)->derivs)
             ->qgw_p_CSTATE[0]);
      for (i1=0; i1 < 2; i1++) {
        dx[i1] = 0.0;
      }
    }
  }

  /* End of Derivatives for SubSystem: '<S289>/Hqgw' */

  /* Derivatives for Enabled SubSystem: '<S289>/Hrgw' */
  if (pid_control_V1_DW.Hrgw_MODE) {
    /* Derivatives for Integrator: '<S302>/rgw_p' */
    _rtXdot->rgw_p_CSTATE[0] = pid_control_V1_B.w_d[0];
    _rtXdot->rgw_p_CSTATE[1] = pid_control_V1_B.w_d[1];
  } else {
    {
      real_T *dx;
      int_T i1;
      dx = &(((XDot_pid_control_V1_T *) (&pid_control_V1_M)->derivs)
             ->rgw_p_CSTATE[0]);
      for (i1=0; i1 < 2; i1++) {
        dx[i1] = 0.0;
      }
    }
  }

  /* End of Derivatives for SubSystem: '<S289>/Hrgw' */

  /* Derivatives for Integrator: '<S11>/Integrator1' */
  _rtXdot->Integrator1_CSTATE = pid_control_V1_B.Power;

  /* Derivatives for TransferFcn: '<S11>/Transfer Fcn' */
  _rtXdot->TransferFcn_CSTATE[0] = 0.0;
  _rtXdot->TransferFcn_CSTATE[0] += -0.898 *
    pid_control_V1_X.TransferFcn_CSTATE[0];
  _rtXdot->TransferFcn_CSTATE[1] = 0.0;
  _rtXdot->TransferFcn_CSTATE[0] += -0.806 *
    pid_control_V1_X.TransferFcn_CSTATE[1];
  _mm_storeu_pd(&tmp[0], _mm_add_pd(_mm_set_pd(_rtXdot->TransferFcn_CSTATE[0],
    pid_control_V1_X.TransferFcn_CSTATE[0]), _mm_set_pd(pid_control_V1_B.Output,
    _rtXdot->TransferFcn_CSTATE[1])));
  _rtXdot->TransferFcn_CSTATE[1] = tmp[0];
  _rtXdot->TransferFcn_CSTATE[0] = tmp[1];

  /* Derivatives for TransferFcn: '<S11>/Transfer Fcn1' */
  _rtXdot->TransferFcn1_CSTATE = 0.0;
  _rtXdot->TransferFcn1_CSTATE += -0.01 * pid_control_V1_X.TransferFcn1_CSTATE;
  _rtXdot->TransferFcn1_CSTATE += pid_control_V1_B.Output;
}

/* Model initialize function */
void pid_control_V1::initialize()
{
  /* Registration code */

  /* initialize non-finites */
  rt_InitInfAndNaN(sizeof(real_T));

  {
    /* Setup solver object */
    rtsiSetSimTimeStepPtr(&(&pid_control_V1_M)->solverInfo, &(&pid_control_V1_M
                          )->Timing.simTimeStep);
    rtsiSetTPtr(&(&pid_control_V1_M)->solverInfo, &rtmGetTPtr((&pid_control_V1_M)));
    rtsiSetStepSizePtr(&(&pid_control_V1_M)->solverInfo, &(&pid_control_V1_M)
                       ->Timing.stepSize0);
    rtsiSetdXPtr(&(&pid_control_V1_M)->solverInfo, &(&pid_control_V1_M)->derivs);
    rtsiSetContStatesPtr(&(&pid_control_V1_M)->solverInfo, (real_T **)
                         &(&pid_control_V1_M)->contStates);
    rtsiSetNumContStatesPtr(&(&pid_control_V1_M)->solverInfo,
      &(&pid_control_V1_M)->Sizes.numContStates);
    rtsiSetNumPeriodicContStatesPtr(&(&pid_control_V1_M)->solverInfo,
      &(&pid_control_V1_M)->Sizes.numPeriodicContStates);
    rtsiSetPeriodicContStateIndicesPtr(&(&pid_control_V1_M)->solverInfo,
      &(&pid_control_V1_M)->periodicContStateIndices);
    rtsiSetPeriodicContStateRangesPtr(&(&pid_control_V1_M)->solverInfo,
      &(&pid_control_V1_M)->periodicContStateRanges);
    rtsiSetContStateDisabledPtr(&(&pid_control_V1_M)->solverInfo, (boolean_T**)
      &(&pid_control_V1_M)->contStateDisabled);
    rtsiSetErrorStatusPtr(&(&pid_control_V1_M)->solverInfo, (&rtmGetErrorStatus
      ((&pid_control_V1_M))));
    rtsiSetRTModelPtr(&(&pid_control_V1_M)->solverInfo, (&pid_control_V1_M));
  }

  rtsiSetSimTimeStep(&(&pid_control_V1_M)->solverInfo, MAJOR_TIME_STEP);
  rtsiSetIsMinorTimeStepWithModeChange(&(&pid_control_V1_M)->solverInfo, false);
  rtsiSetIsContModeFrozen(&(&pid_control_V1_M)->solverInfo, false);
  (&pid_control_V1_M)->intgData.y = (&pid_control_V1_M)->odeY;
  (&pid_control_V1_M)->intgData.f[0] = (&pid_control_V1_M)->odeF[0];
  (&pid_control_V1_M)->intgData.f[1] = (&pid_control_V1_M)->odeF[1];
  (&pid_control_V1_M)->intgData.f[2] = (&pid_control_V1_M)->odeF[2];
  (&pid_control_V1_M)->intgData.f[3] = (&pid_control_V1_M)->odeF[3];
  (&pid_control_V1_M)->contStates = ((X_pid_control_V1_T *) &pid_control_V1_X);
  (&pid_control_V1_M)->contStateDisabled = ((XDis_pid_control_V1_T *)
    &pid_control_V1_XDis);
  (&pid_control_V1_M)->Timing.tStart = (0.0);
  rtsiSetSolverData(&(&pid_control_V1_M)->solverInfo, static_cast<void *>
                    (&(&pid_control_V1_M)->intgData));
  rtsiSetSolverName(&(&pid_control_V1_M)->solverInfo,"ode4");
  rtmSetTPtr((&pid_control_V1_M), &(&pid_control_V1_M)->Timing.tArray[0]);
  (&pid_control_V1_M)->Timing.stepSize0 = 0.01;
  rtmSetFirstInitCond((&pid_control_V1_M), 1);

  /* Start for InitialCondition: '<S11>/IC' */
  memcpy(&pid_control_V1_B.IC[0], &pid_control_V1_ConstP.pooled10[0], 12U *
         sizeof(real_T));

  /* Start for InitialCondition: '<S11>/IC' */
  pid_control_V1_DW.IC_FirstOutputTime = true;

  /* Start for MATLABSystem: '<S12>/SourceBlock' */
  pid_control_V1_DW.obj_m.QOSAvoidROSNamespaceConventions = false;
  pid_control_V1_DW.obj_m.matlabCodegenIsDeleted = false;
  pid_control_V1_DW.objisempty_g = true;
  pid_control_V1_DW.obj_m.isSetupComplete = false;
  pid_control_V1_DW.obj_m.isInitialized = 1;
  pid_c_Subscriber_setupImpl_onhg(&pid_control_V1_DW.obj_m);
  pid_control_V1_DW.obj_m.isSetupComplete = true;

  /* Start for MATLABSystem: '<S13>/SourceBlock' */
  pid_control_V1_DW.obj_i.QOSAvoidROSNamespaceConventions = false;
  pid_control_V1_DW.obj_i.matlabCodegenIsDeleted = false;
  pid_control_V1_DW.objisempty_f = true;
  pid_control_V1_DW.obj_i.isSetupComplete = false;
  pid_control_V1_DW.obj_i.isInitialized = 1;
  pid__Subscriber_setupImpl_onhgd(&pid_control_V1_DW.obj_i);
  pid_control_V1_DW.obj_i.isSetupComplete = true;

  /* Start for MATLABSystem: '<S14>/SourceBlock' */
  pid_control_V1_DW.obj_k.QOSAvoidROSNamespaceConventions = false;
  pid_control_V1_DW.obj_k.matlabCodegenIsDeleted = false;
  pid_control_V1_DW.objisempty = true;
  pid_control_V1_DW.obj_k.isSetupComplete = false;
  pid_control_V1_DW.obj_k.isInitialized = 1;
  pid_Subscriber_setupImpl_onhgd0(&pid_control_V1_DW.obj_k);
  pid_control_V1_DW.obj_k.isSetupComplete = true;

  /* Start for MATLABSystem: '<Root>/Coordinate Transformation Conversion' */
  pid_control_V1_DW.objisempty_d = true;
  pid_control_V1_DW.obj_c.isInitialized = 1;

  /* Start for Atomic SubSystem: '<Root>/Call Service' */
  /* Start for MATLABSystem: '<S2>/ServiceCaller' */
  pid_control_V1_DW.obj.QOSAvoidROSNamespaceConventions = false;
  pid_control_V1_DW.obj.matlabCodegenIsDeleted = false;
  pid_control_V1_DW.objisempty_ft = true;
  pid_control_V1_DW.obj.isSetupComplete = false;
  pid_control_V1_DW.obj.isInitialized = 1;
  pid_con_ServiceCaller_setupImpl(&pid_control_V1_DW.obj);
  pid_control_V1_DW.obj.isSetupComplete = true;

  /* End of Start for SubSystem: '<Root>/Call Service' */

  /* Start for Enabled SubSystem: '<S290>/Hugw(s)' */
  (void) memset(&(pid_control_V1_XDis.ug_p_CSTATE), 1,
                2*sizeof(boolean_T));

  /* End of Start for SubSystem: '<S290>/Hugw(s)' */

  /* Start for Enabled SubSystem: '<S290>/Hvgw(s)' */
  (void) memset(&(pid_control_V1_XDis.vg_p1_CSTATE), 1,
                4*sizeof(boolean_T));

  /* End of Start for SubSystem: '<S290>/Hvgw(s)' */

  /* Start for Enabled SubSystem: '<S290>/Hwgw(s)' */
  (void) memset(&(pid_control_V1_XDis.wg_p1_CSTATE), 1,
                4*sizeof(boolean_T));

  /* End of Start for SubSystem: '<S290>/Hwgw(s)' */

  /* Start for If: '<S295>/if Height < Max low altitude  elseif Height > Min isotropic altitude ' */
  pid_control_V1_DW.ifHeightMaxlowaltitudeelseifHei = -1;

  /* Start for MATLABSystem: '<S287>/SourceBlock' */
  pid_control_V1_DW.obj_h.QOSAvoidROSNamespaceConventions = false;
  pid_control_V1_DW.obj_h.matlabCodegenIsDeleted = false;
  pid_control_V1_DW.objisempty_a = true;
  pid_control_V1_DW.obj_h.isSetupComplete = false;
  pid_control_V1_DW.obj_h.isInitialized = 1;
  pid_co_Subscriber_setupImpl_onh(&pid_control_V1_DW.obj_h);
  pid_control_V1_DW.obj_h.isSetupComplete = true;

  /* Start for Enabled SubSystem: '<S289>/Hpgw' */
  (void) memset(&(pid_control_V1_XDis.pgw_p_CSTATE), 1,
                2*sizeof(boolean_T));

  /* End of Start for SubSystem: '<S289>/Hpgw' */

  /* Start for Enabled SubSystem: '<S289>/Hqgw' */
  (void) memset(&(pid_control_V1_XDis.qgw_p_CSTATE), 1,
                2*sizeof(boolean_T));

  /* End of Start for SubSystem: '<S289>/Hqgw' */

  /* Start for Enabled SubSystem: '<S289>/Hrgw' */
  (void) memset(&(pid_control_V1_XDis.rgw_p_CSTATE), 1,
                2*sizeof(boolean_T));

  /* End of Start for SubSystem: '<S289>/Hrgw' */

  /* Start for If: '<S294>/if Height < Max low altitude  elseif Height > Min isotropic altitude ' */
  pid_control_V1_DW.ifHeightMaxlowaltitudeelseifH_a = -1;

  /* Start for MATLABSystem: '<S284>/SourceBlock' */
  pid_control_V1_DW.obj_h4.QOSAvoidROSNamespaceConventions = false;
  pid_control_V1_DW.obj_h4.matlabCodegenIsDeleted = false;
  pid_control_V1_DW.objisempty_c = true;
  pid_control_V1_DW.obj_h4.isSetupComplete = false;
  pid_control_V1_DW.obj_h4.isInitialized = 1;
  pid_contro_Subscriber_setupImpl(&pid_control_V1_DW.obj_h4);
  pid_control_V1_DW.obj_h4.isSetupComplete = true;

  /* Start for MATLABSystem: '<S285>/SourceBlock' */
  pid_control_V1_DW.obj_hy.QOSAvoidROSNamespaceConventions = false;
  pid_control_V1_DW.obj_hy.matlabCodegenIsDeleted = false;
  pid_control_V1_DW.objisempty_l = true;
  pid_control_V1_DW.obj_hy.isSetupComplete = false;
  pid_control_V1_DW.obj_hy.isInitialized = 1;
  pid_cont_Subscriber_setupImpl_o(&pid_control_V1_DW.obj_hy);
  pid_control_V1_DW.obj_hy.isSetupComplete = true;

  /* Start for MATLABSystem: '<S286>/SourceBlock' */
  pid_control_V1_DW.obj_p.QOSAvoidROSNamespaceConventions = false;
  pid_control_V1_DW.obj_p.matlabCodegenIsDeleted = false;
  pid_control_V1_DW.objisempty_e = true;
  pid_control_V1_DW.obj_p.isSetupComplete = false;
  pid_control_V1_DW.obj_p.isInitialized = 1;
  pid_con_Subscriber_setupImpl_on(&pid_control_V1_DW.obj_p);
  pid_control_V1_DW.obj_p.isSetupComplete = true;
  pid_control_V1_PrevZCX.Integrator_Reset_ZCE = UNINITIALIZED_ZCSIG;

  /* InitializeConditions for Memory: '<S11>/Memory2' */
  memcpy(&pid_control_V1_DW.Memory2_PreviousInput[0],
         &pid_control_V1_ConstP.pooled10[0], 12U * sizeof(real_T));

  /* InitializeConditions for Integrator: '<S11>/Integrator' */
  if (rtmIsFirstInitCond((&pid_control_V1_M))) {
    pid_control_V1_X.Integrator_CSTATE[0] = 20.2;
    pid_control_V1_X.Integrator_CSTATE[1] = 0.0;
    pid_control_V1_X.Integrator_CSTATE[2] = 0.0;
    pid_control_V1_X.Integrator_CSTATE[3] = 0.0;
    pid_control_V1_X.Integrator_CSTATE[4] = 0.0;
    pid_control_V1_X.Integrator_CSTATE[5] = 0.0;
    pid_control_V1_X.Integrator_CSTATE[6] = 0.0;
    pid_control_V1_X.Integrator_CSTATE[7] = 0.026179938779914945;
    pid_control_V1_X.Integrator_CSTATE[8] = 0.0;
    pid_control_V1_X.Integrator_CSTATE[9] = 0.0;
    pid_control_V1_X.Integrator_CSTATE[10] = 0.0;
    pid_control_V1_X.Integrator_CSTATE[11] = -0.55;
  }

  pid_control_V1_DW.Integrator_DWORK1 = true;

  /* End of InitializeConditions for Integrator: '<S11>/Integrator' */

  /* InitializeConditions for UnitDelay: '<Root>/Unit Delay3' */
  pid_control_V1_DW.UnitDelay3_DSTATE = 0.55;

  /* InitializeConditions for Integrator: '<S103>/Integrator' */
  pid_control_V1_X.Integrator_CSTATE_n = 0.0;

  /* InitializeConditions for Integrator: '<S98>/Filter' */
  pid_control_V1_X.Filter_CSTATE = 0.0;

  /* InitializeConditions for Integrator: '<S51>/Integrator' */
  pid_control_V1_X.Integrator_CSTATE_m = 0.0;

  /* InitializeConditions for Integrator: '<S46>/Filter' */
  pid_control_V1_X.Filter_CSTATE_g = 0.0;

  /* InitializeConditions for RateLimiter: '<Root>/Rate Limiter' */
  pid_control_V1_DW.LastMajorTime = (rtInf);

  /* InitializeConditions for Integrator: '<S155>/Integrator' */
  pid_control_V1_X.Integrator_CSTATE_p = 0.0;

  /* InitializeConditions for Integrator: '<S150>/Filter' */
  pid_control_V1_X.Filter_CSTATE_m = 0.0;

  /* InitializeConditions for Integrator: '<S207>/Integrator' */
  pid_control_V1_X.Integrator_CSTATE_d = 0.0;

  /* InitializeConditions for Integrator: '<S202>/Filter' */
  pid_control_V1_X.Filter_CSTATE_f = 0.0;

  /* InitializeConditions for Integrator: '<S261>/Integrator' */
  pid_control_V1_X.Integrator_CSTATE_f = 0.0;

  /* InitializeConditions for Integrator: '<S256>/Filter' */
  pid_control_V1_X.Filter_CSTATE_l = 0.0;

  /* InitializeConditions for RandomNumber: '<S299>/White Noise' */
  pid_control_V1_DW.RandSeed[0] = 1529675776U;
  pid_control_V1_DW.NextOutput[0] = rt_nrand_Upu32_Yd_f_pw_snf
    (&pid_control_V1_DW.RandSeed[0]);
  pid_control_V1_DW.RandSeed[1] = 1529741312U;
  pid_control_V1_DW.NextOutput[1] = rt_nrand_Upu32_Yd_f_pw_snf
    (&pid_control_V1_DW.RandSeed[1]);
  pid_control_V1_DW.RandSeed[2] = 1529806848U;
  pid_control_V1_DW.NextOutput[2] = rt_nrand_Upu32_Yd_f_pw_snf
    (&pid_control_V1_DW.RandSeed[2]);
  pid_control_V1_DW.RandSeed[3] = 1529872384U;
  pid_control_V1_DW.NextOutput[3] = rt_nrand_Upu32_Yd_f_pw_snf
    (&pid_control_V1_DW.RandSeed[3]);

  /* InitializeConditions for Integrator: '<S11>/Integrator1' */
  pid_control_V1_X.Integrator1_CSTATE = 0.0;

  /* InitializeConditions for RandomNumber: '<S279>/White Noise' */
  pid_control_V1_DW.RandSeed_a = 1529675776U;
  pid_control_V1_DW.NextOutput_k = rt_nrand_Upu32_Yd_f_pw_snf
    (&pid_control_V1_DW.RandSeed_a);

  /* InitializeConditions for TransferFcn: '<S11>/Transfer Fcn' */
  pid_control_V1_X.TransferFcn_CSTATE[0] = 0.0;
  pid_control_V1_X.TransferFcn_CSTATE[1] = 0.0;

  /* InitializeConditions for TransferFcn: '<S11>/Transfer Fcn1' */
  pid_control_V1_X.TransferFcn1_CSTATE = 0.0;

  /* SystemInitialize for Enabled SubSystem: '<S12>/Enabled Subsystem' */
  pid_con_EnabledSubsystem_i_Init(&pid_control_V1_B.EnabledSubsystem_b);

  /* End of SystemInitialize for SubSystem: '<S12>/Enabled Subsystem' */

  /* SystemInitialize for Enabled SubSystem: '<S13>/Enabled Subsystem' */
  pid_con_EnabledSubsystem_i_Init(&pid_control_V1_B.EnabledSubsystem_ps);

  /* End of SystemInitialize for SubSystem: '<S13>/Enabled Subsystem' */

  /* SystemInitialize for Enabled SubSystem: '<S14>/Enabled Subsystem' */
  pid_con_EnabledSubsystem_i_Init(&pid_control_V1_B.EnabledSubsystem_a);

  /* End of SystemInitialize for SubSystem: '<S14>/Enabled Subsystem' */

  /* SystemInitialize for Enabled SubSystem: '<S290>/Hugw(s)' */
  /* InitializeConditions for Integrator: '<S303>/ug_p' */
  pid_control_V1_X.ug_p_CSTATE[0] = 0.0;

  /* End of SystemInitialize for SubSystem: '<S290>/Hugw(s)' */

  /* SystemInitialize for Enabled SubSystem: '<S290>/Hvgw(s)' */
  /* InitializeConditions for Integrator: '<S304>/vg_p1' */
  pid_control_V1_X.vg_p1_CSTATE[0] = 0.0;

  /* InitializeConditions for Integrator: '<S304>/vgw_p2' */
  pid_control_V1_X.vgw_p2_CSTATE[0] = 0.0;

  /* End of SystemInitialize for SubSystem: '<S290>/Hvgw(s)' */

  /* SystemInitialize for Enabled SubSystem: '<S290>/Hwgw(s)' */
  /* InitializeConditions for Integrator: '<S305>/wg_p1' */
  pid_control_V1_X.wg_p1_CSTATE[0] = 0.0;

  /* InitializeConditions for Integrator: '<S305>/wg_p2' */
  pid_control_V1_X.wg_p2_CSTATE[0] = 0.0;

  /* End of SystemInitialize for SubSystem: '<S290>/Hwgw(s)' */

  /* SystemInitialize for Enabled SubSystem: '<S290>/Hugw(s)' */
  /* InitializeConditions for Integrator: '<S303>/ug_p' */
  pid_control_V1_X.ug_p_CSTATE[1] = 0.0;

  /* End of SystemInitialize for SubSystem: '<S290>/Hugw(s)' */

  /* SystemInitialize for Enabled SubSystem: '<S290>/Hvgw(s)' */
  /* InitializeConditions for Integrator: '<S304>/vg_p1' */
  pid_control_V1_X.vg_p1_CSTATE[1] = 0.0;

  /* InitializeConditions for Integrator: '<S304>/vgw_p2' */
  pid_control_V1_X.vgw_p2_CSTATE[1] = 0.0;

  /* End of SystemInitialize for SubSystem: '<S290>/Hvgw(s)' */

  /* SystemInitialize for Enabled SubSystem: '<S290>/Hwgw(s)' */
  /* InitializeConditions for Integrator: '<S305>/wg_p1' */
  pid_control_V1_X.wg_p1_CSTATE[1] = 0.0;

  /* InitializeConditions for Integrator: '<S305>/wg_p2' */
  pid_control_V1_X.wg_p2_CSTATE[1] = 0.0;

  /* End of SystemInitialize for SubSystem: '<S290>/Hwgw(s)' */

  /* SystemInitialize for Enabled SubSystem: '<S287>/Enabled Subsystem' */
  pid_contr_EnabledSubsystem_Init(&pid_control_V1_B.EnabledSubsystem_pt);

  /* End of SystemInitialize for SubSystem: '<S287>/Enabled Subsystem' */

  /* SystemInitialize for Enabled SubSystem: '<S289>/Hpgw' */
  /* InitializeConditions for Integrator: '<S300>/pgw_p' */
  pid_control_V1_X.pgw_p_CSTATE[0] = 0.0;

  /* End of SystemInitialize for SubSystem: '<S289>/Hpgw' */

  /* SystemInitialize for Enabled SubSystem: '<S289>/Hqgw' */
  /* InitializeConditions for Integrator: '<S301>/qgw_p' */
  pid_control_V1_X.qgw_p_CSTATE[0] = 0.0;

  /* End of SystemInitialize for SubSystem: '<S289>/Hqgw' */

  /* SystemInitialize for Enabled SubSystem: '<S289>/Hrgw' */
  /* InitializeConditions for Integrator: '<S302>/rgw_p' */
  pid_control_V1_X.rgw_p_CSTATE[0] = 0.0;

  /* End of SystemInitialize for SubSystem: '<S289>/Hrgw' */

  /* SystemInitialize for Enabled SubSystem: '<S289>/Hpgw' */
  /* InitializeConditions for Integrator: '<S300>/pgw_p' */
  pid_control_V1_X.pgw_p_CSTATE[1] = 0.0;

  /* End of SystemInitialize for SubSystem: '<S289>/Hpgw' */

  /* SystemInitialize for Enabled SubSystem: '<S289>/Hqgw' */
  /* InitializeConditions for Integrator: '<S301>/qgw_p' */
  pid_control_V1_X.qgw_p_CSTATE[1] = 0.0;

  /* End of SystemInitialize for SubSystem: '<S289>/Hqgw' */

  /* SystemInitialize for Enabled SubSystem: '<S289>/Hrgw' */
  /* InitializeConditions for Integrator: '<S302>/rgw_p' */
  pid_control_V1_X.rgw_p_CSTATE[1] = 0.0;

  /* End of SystemInitialize for SubSystem: '<S289>/Hrgw' */

  /* SystemInitialize for Enabled SubSystem: '<S284>/Enabled Subsystem' */
  pid_contr_EnabledSubsystem_Init(&pid_control_V1_B.EnabledSubsystem);

  /* End of SystemInitialize for SubSystem: '<S284>/Enabled Subsystem' */

  /* SystemInitialize for Enabled SubSystem: '<S285>/Enabled Subsystem' */
  pid_con_EnabledSubsystem_i_Init(&pid_control_V1_B.EnabledSubsystem_k);

  /* End of SystemInitialize for SubSystem: '<S285>/Enabled Subsystem' */

  /* SystemInitialize for Enabled SubSystem: '<S286>/Enabled Subsystem' */
  pid_con_EnabledSubsystem_i_Init(&pid_control_V1_B.EnabledSubsystem_p);

  /* End of SystemInitialize for SubSystem: '<S286>/Enabled Subsystem' */

  /* set "at time zero" to false */
  if (rtmIsFirstInitCond((&pid_control_V1_M))) {
    rtmSetFirstInitCond((&pid_control_V1_M), 0);
  }
}

/* Model terminate function */
void pid_control_V1::terminate()
{
  /* Terminate for MATLABSystem: '<S12>/SourceBlock' */
  if (!pid_control_V1_DW.obj_m.matlabCodegenIsDeleted) {
    pid_control_V1_DW.obj_m.matlabCodegenIsDeleted = true;
    if ((pid_control_V1_DW.obj_m.isInitialized == 1) &&
        pid_control_V1_DW.obj_m.isSetupComplete) {
      Sub_pid_control_V1_435.resetSubscriberPtr();//();
    }
  }

  /* End of Terminate for MATLABSystem: '<S12>/SourceBlock' */

  /* Terminate for MATLABSystem: '<S13>/SourceBlock' */
  if (!pid_control_V1_DW.obj_i.matlabCodegenIsDeleted) {
    pid_control_V1_DW.obj_i.matlabCodegenIsDeleted = true;
    if ((pid_control_V1_DW.obj_i.isInitialized == 1) &&
        pid_control_V1_DW.obj_i.isSetupComplete) {
      Sub_pid_control_V1_538.resetSubscriberPtr();//();
    }
  }

  /* End of Terminate for MATLABSystem: '<S13>/SourceBlock' */

  /* Terminate for MATLABSystem: '<S14>/SourceBlock' */
  if (!pid_control_V1_DW.obj_k.matlabCodegenIsDeleted) {
    pid_control_V1_DW.obj_k.matlabCodegenIsDeleted = true;
    if ((pid_control_V1_DW.obj_k.isInitialized == 1) &&
        pid_control_V1_DW.obj_k.isSetupComplete) {
      Sub_pid_control_V1_377.resetSubscriberPtr();//();
    }
  }

  /* End of Terminate for MATLABSystem: '<S14>/SourceBlock' */

  /* Terminate for Atomic SubSystem: '<Root>/Call Service' */
  /* Terminate for MATLABSystem: '<S2>/ServiceCaller' */
  if (!pid_control_V1_DW.obj.matlabCodegenIsDeleted) {
    pid_control_V1_DW.obj.matlabCodegenIsDeleted = true;
    if ((pid_control_V1_DW.obj.isInitialized == 1) &&
        pid_control_V1_DW.obj.isSetupComplete) {
      ServCall_pid_control_V1_326.resetSvcClientPtr();//();
    }
  }

  /* End of Terminate for MATLABSystem: '<S2>/ServiceCaller' */
  /* End of Terminate for SubSystem: '<Root>/Call Service' */

  /* Terminate for MATLABSystem: '<S287>/SourceBlock' */
  if (!pid_control_V1_DW.obj_h.matlabCodegenIsDeleted) {
    pid_control_V1_DW.obj_h.matlabCodegenIsDeleted = true;
    if ((pid_control_V1_DW.obj_h.isInitialized == 1) &&
        pid_control_V1_DW.obj_h.isSetupComplete) {
      Sub_pid_control_V1_417.resetSubscriberPtr();//();
    }
  }

  /* End of Terminate for MATLABSystem: '<S287>/SourceBlock' */

  /* Terminate for MATLABSystem: '<S284>/SourceBlock' */
  if (!pid_control_V1_DW.obj_h4.matlabCodegenIsDeleted) {
    pid_control_V1_DW.obj_h4.matlabCodegenIsDeleted = true;
    if ((pid_control_V1_DW.obj_h4.isInitialized == 1) &&
        pid_control_V1_DW.obj_h4.isSetupComplete) {
      Sub_pid_control_V1_423.resetSubscriberPtr();//();
    }
  }

  /* End of Terminate for MATLABSystem: '<S284>/SourceBlock' */

  /* Terminate for MATLABSystem: '<S285>/SourceBlock' */
  if (!pid_control_V1_DW.obj_hy.matlabCodegenIsDeleted) {
    pid_control_V1_DW.obj_hy.matlabCodegenIsDeleted = true;
    if ((pid_control_V1_DW.obj_hy.isInitialized == 1) &&
        pid_control_V1_DW.obj_hy.isSetupComplete) {
      Sub_pid_control_V1_443.resetSubscriberPtr();//();
    }
  }

  /* End of Terminate for MATLABSystem: '<S285>/SourceBlock' */

  /* Terminate for MATLABSystem: '<S286>/SourceBlock' */
  if (!pid_control_V1_DW.obj_p.matlabCodegenIsDeleted) {
    pid_control_V1_DW.obj_p.matlabCodegenIsDeleted = true;
    if ((pid_control_V1_DW.obj_p.isInitialized == 1) &&
        pid_control_V1_DW.obj_p.isSetupComplete) {
      Sub_pid_control_V1_445.resetSubscriberPtr();//();
    }
  }

  /* End of Terminate for MATLABSystem: '<S286>/SourceBlock' */
}

/* Constructor */
pid_control_V1::pid_control_V1() :
  pid_control_V1_B(),
  pid_control_V1_DW(),
  pid_control_V1_X(),
  pid_control_V1_XDis(),
  pid_control_V1_PrevZCX(),
  pid_control_V1_M()
{
  /* Currently there is no constructor body generated.*/
}

/* Destructor */
pid_control_V1::~pid_control_V1()
{
  /* Currently there is no destructor body generated.*/
}

/* Real-Time Model get method */
RT_MODEL_pid_control_V1_T * pid_control_V1::getRTM()
{
  return (&pid_control_V1_M);
}
