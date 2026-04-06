/*
 * pid_control_V1.cpp
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

#include "pid_control_V1.h"
#include "rtwtypes.h"
#include "pid_control_V1_types.h"
#include <string.h>
#include "pid_control_V1_private.h"
#include <math.h>

extern "C"
{

#include "rt_nonfinite.h"

}

#include <emmintrin.h>
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
 *    '<S285>/Enabled Subsystem'
 *    '<S288>/Enabled Subsystem'
 */
void pid_control_V1::pid_contr_EnabledSubsystem_Init
  (B_EnabledSubsystem_pid_contro_T *localB)
{
  /* SystemInitialize for SignalConversion generated from: '<S328>/In1' */
  memset(&localB->In1, 0, sizeof(SL_Bus_std_msgs_Bool));
}

/*
 * Output and update for enable system:
 *    '<S285>/Enabled Subsystem'
 *    '<S288>/Enabled Subsystem'
 */
void pid_control_V1::pid_control_V1_EnabledSubsystem(boolean_T rtu_Enable, const
  SL_Bus_std_msgs_Bool *rtu_In1, B_EnabledSubsystem_pid_contro_T *localB)
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

/*
 * System initialize for enable system:
 *    '<S286>/Enabled Subsystem'
 *    '<S287>/Enabled Subsystem'
 *    '<S13>/Enabled Subsystem'
 *    '<S14>/Enabled Subsystem'
 *    '<S15>/Enabled Subsystem'
 */
void pid_control_V1::pid_con_EnabledSubsystem_i_Init
  (B_EnabledSubsystem_pid_cont_d_T *localB)
{
  /* SystemInitialize for SignalConversion generated from: '<S329>/In1' */
  memset(&localB->In1, 0, sizeof(SL_Bus_std_msgs_Float64));
}

/*
 * Output and update for enable system:
 *    '<S286>/Enabled Subsystem'
 *    '<S287>/Enabled Subsystem'
 *    '<S13>/Enabled Subsystem'
 *    '<S14>/Enabled Subsystem'
 *    '<S15>/Enabled Subsystem'
 */
void pid_control_V1::pid_control__EnabledSubsystem_k(boolean_T rtu_Enable, const
  SL_Bus_std_msgs_Float64 *rtu_In1, B_EnabledSubsystem_pid_cont_d_T *localB)
{
  /* Outputs for Enabled SubSystem: '<S286>/Enabled Subsystem' incorporates:
   *  EnablePort: '<S329>/Enable'
   */
  if (rtu_Enable) {
    /* SignalConversion generated from: '<S329>/In1' */
    localB->In1 = *rtu_In1;
  }

  /* End of Outputs for SubSystem: '<S286>/Enabled Subsystem' */
}

void pid_control_V1::IMUSensorParameters_updateSyste(real_T obj_MeasurementRange,
  real_T obj_Resolution, const real_T obj_ConstantBias[3], const real_T
  obj_AxesMisalignment[9], const real_T obj_NoiseDensity[3], const real_T
  obj_BiasInstability[3], const real_T obj_RandomWalk[3], real_T
  obj_BiasInstabilityCoefficients, const real_T obj_BiasInstabilityCoefficien_0
  [2], const char_T obj_NoiseType_Value[12], const real_T obj_TemperatureBias[3],
  const real_T obj_TemperatureScaleFactor[3], d_fusion_internal_Acceleromet_T
  *sobj)
{
  /* Start for MATLABSystem: '<Root>/IMU1' */
  if (sobj->isInitialized == 1) {
    sobj->TunablePropsChanged = true;
    sobj->tunablePropertyChanged[0] = true;
  }

  sobj->MeasurementRange = obj_MeasurementRange;

  /* Start for MATLABSystem: '<Root>/IMU1' */
  if (sobj->isInitialized == 1) {
    sobj->TunablePropsChanged = true;
    sobj->tunablePropertyChanged[1] = true;
  }

  sobj->Resolution = obj_Resolution;

  /* Start for MATLABSystem: '<Root>/IMU1' */
  if (sobj->isInitialized == 1) {
    sobj->TunablePropsChanged = true;
    sobj->tunablePropertyChanged[2] = true;
  }

  sobj->ConstantBias[0] = obj_ConstantBias[0];
  sobj->ConstantBias[1] = obj_ConstantBias[1];
  sobj->ConstantBias[2] = obj_ConstantBias[2];

  /* Start for MATLABSystem: '<Root>/IMU1' */
  if (sobj->isInitialized == 1) {
    sobj->TunablePropsChanged = true;
    sobj->tunablePropertyChanged[3] = true;
  }

  memcpy(&sobj->AxesMisalignment[0], &obj_AxesMisalignment[0], 9U * sizeof
         (real_T));

  /* Start for MATLABSystem: '<Root>/IMU1' */
  if (sobj->isInitialized == 1) {
    sobj->TunablePropsChanged = true;
    sobj->tunablePropertyChanged[4] = true;
  }

  sobj->NoiseDensity[0] = obj_NoiseDensity[0];
  sobj->NoiseDensity[1] = obj_NoiseDensity[1];
  sobj->NoiseDensity[2] = obj_NoiseDensity[2];

  /* Start for MATLABSystem: '<Root>/IMU1' */
  if (sobj->isInitialized == 1) {
    sobj->TunablePropsChanged = true;
    sobj->tunablePropertyChanged[5] = true;
  }

  sobj->BiasInstability[0] = obj_BiasInstability[0];
  sobj->BiasInstability[1] = obj_BiasInstability[1];
  sobj->BiasInstability[2] = obj_BiasInstability[2];

  /* Start for MATLABSystem: '<Root>/IMU1' */
  if (sobj->isInitialized == 1) {
    sobj->TunablePropsChanged = true;
    sobj->tunablePropertyChanged[6] = true;
  }

  sobj->RandomWalk[0] = obj_RandomWalk[0];
  sobj->RandomWalk[1] = obj_RandomWalk[1];
  sobj->RandomWalk[2] = obj_RandomWalk[2];

  /* Start for MATLABSystem: '<Root>/IMU1' */
  if (sobj->isInitialized == 1) {
    sobj->TunablePropsChanged = true;
    sobj->tunablePropertyChanged[7] = true;
  }

  sobj->BiasInstabilityCoefficients.Numerator = obj_BiasInstabilityCoefficients;
  sobj->BiasInstabilityCoefficients.Denominator[0] =
    obj_BiasInstabilityCoefficien_0[0];
  sobj->BiasInstabilityCoefficients.Denominator[1] =
    obj_BiasInstabilityCoefficien_0[1];
  if (sobj->isInitialized == 1) {
    sobj->TunablePropsChanged = true;
    sobj->tunablePropertyChanged[8] = true;
  }

  for (int32_T i = 0; i < 12; i++) {
    sobj->NoiseType.Value[i] = obj_NoiseType_Value[i];
  }

  /* Start for MATLABSystem: '<Root>/IMU1' */
  if (sobj->isInitialized == 1) {
    sobj->TunablePropsChanged = true;
    sobj->tunablePropertyChanged[9] = true;
  }

  sobj->TemperatureBias[0] = obj_TemperatureBias[0];
  sobj->TemperatureBias[1] = obj_TemperatureBias[1];
  sobj->TemperatureBias[2] = obj_TemperatureBias[2];

  /* Start for MATLABSystem: '<Root>/IMU1' */
  if (sobj->isInitialized == 1) {
    sobj->TunablePropsChanged = true;
    sobj->tunablePropertyChanged[10] = true;
  }

  sobj->TemperatureScaleFactor[0] = obj_TemperatureScaleFactor[0];
  sobj->TemperatureScaleFactor[1] = obj_TemperatureScaleFactor[1];
  sobj->TemperatureScaleFactor[2] = obj_TemperatureScaleFactor[2];
}

void pid_control_V1::IMUSensorParameters_updateSys_o(real_T obj_MeasurementRange,
  real_T obj_Resolution, const real_T obj_ConstantBias[3], const real_T
  obj_AxesMisalignment[9], const real_T obj_NoiseDensity[3], const real_T
  obj_BiasInstability[3], const real_T obj_RandomWalk[3], real_T
  obj_BiasInstabilityCoefficients, const real_T obj_BiasInstabilityCoefficien_0
  [2], const char_T obj_NoiseType_Value[12], const real_T obj_TemperatureBias[3],
  const real_T obj_TemperatureScaleFactor[3], const real_T obj_AccelerationBias
  [3], e_fusion_internal_GyroscopeSi_T *sobj)
{
  /* Start for MATLABSystem: '<Root>/IMU1' */
  if (sobj->isInitialized == 1) {
    sobj->TunablePropsChanged = true;
    sobj->tunablePropertyChanged[0] = true;
  }

  sobj->AccelerationBias[0] = obj_AccelerationBias[0];
  sobj->AccelerationBias[1] = obj_AccelerationBias[1];
  sobj->AccelerationBias[2] = obj_AccelerationBias[2];
  if (sobj->isInitialized == 1) {
    sobj->TunablePropsChanged = true;
    sobj->tunablePropertyChanged[1] = true;
  }

  sobj->MeasurementRange = obj_MeasurementRange;

  /* Start for MATLABSystem: '<Root>/IMU1' */
  if (sobj->isInitialized == 1) {
    sobj->TunablePropsChanged = true;
    sobj->tunablePropertyChanged[2] = true;
  }

  sobj->Resolution = obj_Resolution;

  /* Start for MATLABSystem: '<Root>/IMU1' */
  if (sobj->isInitialized == 1) {
    sobj->TunablePropsChanged = true;
    sobj->tunablePropertyChanged[3] = true;
  }

  sobj->ConstantBias[0] = obj_ConstantBias[0];
  sobj->ConstantBias[1] = obj_ConstantBias[1];
  sobj->ConstantBias[2] = obj_ConstantBias[2];

  /* Start for MATLABSystem: '<Root>/IMU1' */
  if (sobj->isInitialized == 1) {
    sobj->TunablePropsChanged = true;
    sobj->tunablePropertyChanged[4] = true;
  }

  memcpy(&sobj->AxesMisalignment[0], &obj_AxesMisalignment[0], 9U * sizeof
         (real_T));

  /* Start for MATLABSystem: '<Root>/IMU1' */
  if (sobj->isInitialized == 1) {
    sobj->TunablePropsChanged = true;
    sobj->tunablePropertyChanged[5] = true;
  }

  sobj->NoiseDensity[0] = obj_NoiseDensity[0];
  sobj->NoiseDensity[1] = obj_NoiseDensity[1];
  sobj->NoiseDensity[2] = obj_NoiseDensity[2];

  /* Start for MATLABSystem: '<Root>/IMU1' */
  if (sobj->isInitialized == 1) {
    sobj->TunablePropsChanged = true;
    sobj->tunablePropertyChanged[6] = true;
  }

  sobj->BiasInstability[0] = obj_BiasInstability[0];
  sobj->BiasInstability[1] = obj_BiasInstability[1];
  sobj->BiasInstability[2] = obj_BiasInstability[2];

  /* Start for MATLABSystem: '<Root>/IMU1' */
  if (sobj->isInitialized == 1) {
    sobj->TunablePropsChanged = true;
    sobj->tunablePropertyChanged[7] = true;
  }

  sobj->RandomWalk[0] = obj_RandomWalk[0];
  sobj->RandomWalk[1] = obj_RandomWalk[1];
  sobj->RandomWalk[2] = obj_RandomWalk[2];

  /* Start for MATLABSystem: '<Root>/IMU1' */
  if (sobj->isInitialized == 1) {
    sobj->TunablePropsChanged = true;
    sobj->tunablePropertyChanged[8] = true;
  }

  sobj->BiasInstabilityCoefficients.Numerator = obj_BiasInstabilityCoefficients;
  sobj->BiasInstabilityCoefficients.Denominator[0] =
    obj_BiasInstabilityCoefficien_0[0];
  sobj->BiasInstabilityCoefficients.Denominator[1] =
    obj_BiasInstabilityCoefficien_0[1];
  if (sobj->isInitialized == 1) {
    sobj->TunablePropsChanged = true;
    sobj->tunablePropertyChanged[9] = true;
  }

  for (int32_T i = 0; i < 12; i++) {
    sobj->NoiseType.Value[i] = obj_NoiseType_Value[i];
  }

  /* Start for MATLABSystem: '<Root>/IMU1' */
  if (sobj->isInitialized == 1) {
    sobj->TunablePropsChanged = true;
    sobj->tunablePropertyChanged[10] = true;
  }

  sobj->TemperatureBias[0] = obj_TemperatureBias[0];
  sobj->TemperatureBias[1] = obj_TemperatureBias[1];
  sobj->TemperatureBias[2] = obj_TemperatureBias[2];

  /* Start for MATLABSystem: '<Root>/IMU1' */
  if (sobj->isInitialized == 1) {
    sobj->TunablePropsChanged = true;
    sobj->tunablePropertyChanged[11] = true;
  }

  sobj->TemperatureScaleFactor[0] = obj_TemperatureScaleFactor[0];
  sobj->TemperatureScaleFactor[1] = obj_TemperatureScaleFactor[1];
  sobj->TemperatureScaleFactor[2] = obj_TemperatureScaleFactor[2];
}

void pid_control_V1::IMUSensorParameters_updateSy_on(real_T obj_MeasurementRange,
  real_T obj_Resolution, const real_T obj_ConstantBias[3], const real_T
  obj_AxesMisalignment[9], const real_T obj_NoiseDensity[3], const real_T
  obj_BiasInstability[3], const real_T obj_RandomWalk[3], real_T
  obj_BiasInstabilityCoefficients, const real_T obj_BiasInstabilityCoefficien_0
  [2], const char_T obj_NoiseType_Value[12], const real_T obj_TemperatureBias[3],
  const real_T obj_TemperatureScaleFactor[3], e_fusion_internal_Magnetomete_T
  *sobj)
{
  /* Start for MATLABSystem: '<Root>/IMU1' */
  if (sobj->isInitialized == 1) {
    sobj->TunablePropsChanged = true;
    sobj->tunablePropertyChanged[0] = true;
  }

  sobj->MeasurementRange = obj_MeasurementRange;

  /* Start for MATLABSystem: '<Root>/IMU1' */
  if (sobj->isInitialized == 1) {
    sobj->TunablePropsChanged = true;
    sobj->tunablePropertyChanged[1] = true;
  }

  sobj->Resolution = obj_Resolution;

  /* Start for MATLABSystem: '<Root>/IMU1' */
  if (sobj->isInitialized == 1) {
    sobj->TunablePropsChanged = true;
    sobj->tunablePropertyChanged[2] = true;
  }

  sobj->ConstantBias[0] = obj_ConstantBias[0];
  sobj->ConstantBias[1] = obj_ConstantBias[1];
  sobj->ConstantBias[2] = obj_ConstantBias[2];

  /* Start for MATLABSystem: '<Root>/IMU1' */
  if (sobj->isInitialized == 1) {
    sobj->TunablePropsChanged = true;
    sobj->tunablePropertyChanged[3] = true;
  }

  memcpy(&sobj->AxesMisalignment[0], &obj_AxesMisalignment[0], 9U * sizeof
         (real_T));

  /* Start for MATLABSystem: '<Root>/IMU1' */
  if (sobj->isInitialized == 1) {
    sobj->TunablePropsChanged = true;
    sobj->tunablePropertyChanged[4] = true;
  }

  sobj->NoiseDensity[0] = obj_NoiseDensity[0];
  sobj->NoiseDensity[1] = obj_NoiseDensity[1];
  sobj->NoiseDensity[2] = obj_NoiseDensity[2];

  /* Start for MATLABSystem: '<Root>/IMU1' */
  if (sobj->isInitialized == 1) {
    sobj->TunablePropsChanged = true;
    sobj->tunablePropertyChanged[5] = true;
  }

  sobj->BiasInstability[0] = obj_BiasInstability[0];
  sobj->BiasInstability[1] = obj_BiasInstability[1];
  sobj->BiasInstability[2] = obj_BiasInstability[2];

  /* Start for MATLABSystem: '<Root>/IMU1' */
  if (sobj->isInitialized == 1) {
    sobj->TunablePropsChanged = true;
    sobj->tunablePropertyChanged[6] = true;
  }

  sobj->RandomWalk[0] = obj_RandomWalk[0];
  sobj->RandomWalk[1] = obj_RandomWalk[1];
  sobj->RandomWalk[2] = obj_RandomWalk[2];

  /* Start for MATLABSystem: '<Root>/IMU1' */
  if (sobj->isInitialized == 1) {
    sobj->TunablePropsChanged = true;
    sobj->tunablePropertyChanged[7] = true;
  }

  sobj->BiasInstabilityCoefficients.Numerator = obj_BiasInstabilityCoefficients;
  sobj->BiasInstabilityCoefficients.Denominator[0] =
    obj_BiasInstabilityCoefficien_0[0];
  sobj->BiasInstabilityCoefficients.Denominator[1] =
    obj_BiasInstabilityCoefficien_0[1];
  if (sobj->isInitialized == 1) {
    sobj->TunablePropsChanged = true;
    sobj->tunablePropertyChanged[8] = true;
  }

  for (int32_T i = 0; i < 12; i++) {
    sobj->NoiseType.Value[i] = obj_NoiseType_Value[i];
  }

  /* Start for MATLABSystem: '<Root>/IMU1' */
  if (sobj->isInitialized == 1) {
    sobj->TunablePropsChanged = true;
    sobj->tunablePropertyChanged[9] = true;
  }

  sobj->TemperatureBias[0] = obj_TemperatureBias[0];
  sobj->TemperatureBias[1] = obj_TemperatureBias[1];
  sobj->TemperatureBias[2] = obj_TemperatureBias[2];

  /* Start for MATLABSystem: '<Root>/IMU1' */
  if (sobj->isInitialized == 1) {
    sobj->TunablePropsChanged = true;
    sobj->tunablePropertyChanged[10] = true;
  }

  sobj->TemperatureScaleFactor[0] = obj_TemperatureScaleFactor[0];
  sobj->TemperatureScaleFactor[1] = obj_TemperatureScaleFactor[1];
  sobj->TemperatureScaleFactor[2] = obj_TemperatureScaleFactor[2];
}

void pid_control_V1::pid_control_imuSensor_setupImpl
  (fusion_simulink_imuSensor_pid_T *obj)
{
  d_fusion_internal_Acceleromet_T *obj_0;
  e_fusion_internal_GyroscopeSi_T *obj_1;
  e_fusion_internal_Magnetomete_T *obj_2;
  real_T b_Numerator;
  real_T val;
  real_T val_0;
  int32_T i;
  boolean_T flag;
  static const char_T t1_Value[12] = { 'd', 'o', 'u', 'b', 'l', 'e', '-', 's',
    'i', 'd', 'e', 'd' };

  /* Start for MATLABSystem: '<Root>/IMU1' */
  val = obj->AccelParamsMeasurementRange;
  val_0 = obj->AccelParamsResolution;
  pid_control_V1_B.ap_ConstantBias_c[0] = obj->AccelParamsConstantBias[0];
  pid_control_V1_B.ap_ConstantBias_c[1] = obj->AccelParamsConstantBias[1];
  pid_control_V1_B.ap_ConstantBias_c[2] = obj->AccelParamsConstantBias[2];
  for (i = 0; i < 9; i++) {
    /* Start for MATLABSystem: '<Root>/IMU1' */
    pid_control_V1_B.valIn[i] = obj->AccelParamsAxesMisalignment[i];
  }

  /* Start for MATLABSystem: '<Root>/IMU1' */
  pid_control_V1_B.ap_NoiseDensity[0] = obj->AccelParamsNoiseDensity[0];
  pid_control_V1_B.ap_NoiseDensity[1] = obj->AccelParamsNoiseDensity[1];
  pid_control_V1_B.ap_NoiseDensity[2] = obj->AccelParamsNoiseDensity[2];
  pid_control_V1_B.ap_BiasInstability_k[0] = obj->AccelParamsBiasInstability[0];
  pid_control_V1_B.ap_BiasInstability_k[1] = obj->AccelParamsBiasInstability[1];
  pid_control_V1_B.ap_BiasInstability_k[2] = obj->AccelParamsBiasInstability[2];
  pid_control_V1_B.ap_RandomWalk[0] = obj->AccelParamsRandomWalk[0];
  pid_control_V1_B.ap_RandomWalk[1] = obj->AccelParamsRandomWalk[1];
  pid_control_V1_B.ap_RandomWalk[2] = obj->AccelParamsRandomWalk[2];
  pid_control_V1_B.ap_TemperatureBias_c[0] = obj->AccelParamsTemperatureBias[0];
  pid_control_V1_B.ap_TemperatureBias_c[1] = obj->AccelParamsTemperatureBias[1];
  pid_control_V1_B.ap_TemperatureBias_c[2] = obj->AccelParamsTemperatureBias[2];
  pid_control_V1_B.val_p[0] = obj->AccelParamsTemperatureScaleFactor[0];
  pid_control_V1_B.val_p[1] = obj->AccelParamsTemperatureScaleFactor[1];
  pid_control_V1_B.val_p[2] = obj->AccelParamsTemperatureScaleFactor[2];
  b_Numerator = obj->AccelParamsBiasInstabilityNumerator;
  pid_control_V1_B.b_Denominator_l[0] =
    obj->AccelParamsBiasInstabilityDenominator[0];
  pid_control_V1_B.b_Denominator_l[1] =
    obj->AccelParamsBiasInstabilityDenominator[1];
  obj->coder_buffer_pobj2.isInitialized = 0;
  for (i = 0; i < 12; i++) {
    /* Start for MATLABSystem: '<Root>/IMU1' */
    obj->coder_buffer_pobj2.tunablePropertyChanged[i] = false;
  }

  /* Start for MATLABSystem: '<Root>/IMU1' */
  IMUSensorParameters_updateSyste(val, val_0, pid_control_V1_B.ap_ConstantBias_c,
    pid_control_V1_B.valIn, pid_control_V1_B.ap_NoiseDensity,
    pid_control_V1_B.ap_BiasInstability_k, pid_control_V1_B.ap_RandomWalk,
    b_Numerator, pid_control_V1_B.b_Denominator_l, t1_Value,
    pid_control_V1_B.ap_TemperatureBias_c, pid_control_V1_B.val_p,
    &obj->coder_buffer_pobj2);
  obj->pAccel = &obj->coder_buffer_pobj2;
  obj_0 = obj->pAccel;

  /* Start for MATLABSystem: '<Root>/IMU1' */
  flag = (obj_0->isInitialized == 1);
  if (flag) {
    obj_0->TunablePropsChanged = true;
    obj_0->tunablePropertyChanged[11] = true;
  }

  obj->pAccel->Temperature = obj->Temperature;

  /* Start for MATLABSystem: '<Root>/IMU1' */
  val = obj->GyroParamsMeasurementRange;
  val_0 = obj->GyroParamsResolution;
  pid_control_V1_B.ap_ConstantBias_c[0] = obj->GyroParamsConstantBias[0];
  pid_control_V1_B.ap_ConstantBias_c[1] = obj->GyroParamsConstantBias[1];
  pid_control_V1_B.ap_ConstantBias_c[2] = obj->GyroParamsConstantBias[2];
  for (i = 0; i < 9; i++) {
    /* Start for MATLABSystem: '<Root>/IMU1' */
    pid_control_V1_B.valIn[i] = obj->GyroParamsAxesMisalignment[i];
  }

  /* Start for MATLABSystem: '<Root>/IMU1' */
  pid_control_V1_B.ap_NoiseDensity[0] = obj->GyroParamsNoiseDensity[0];
  pid_control_V1_B.ap_NoiseDensity[1] = obj->GyroParamsNoiseDensity[1];
  pid_control_V1_B.ap_NoiseDensity[2] = obj->GyroParamsNoiseDensity[2];
  pid_control_V1_B.ap_BiasInstability_k[0] = obj->GyroParamsBiasInstability[0];
  pid_control_V1_B.ap_BiasInstability_k[1] = obj->GyroParamsBiasInstability[1];
  pid_control_V1_B.ap_BiasInstability_k[2] = obj->GyroParamsBiasInstability[2];
  pid_control_V1_B.ap_RandomWalk[0] = obj->GyroParamsRandomWalk[0];
  pid_control_V1_B.ap_RandomWalk[1] = obj->GyroParamsRandomWalk[1];
  pid_control_V1_B.ap_RandomWalk[2] = obj->GyroParamsRandomWalk[2];
  pid_control_V1_B.ap_TemperatureBias_c[0] = obj->GyroParamsTemperatureBias[0];
  pid_control_V1_B.ap_TemperatureBias_c[1] = obj->GyroParamsTemperatureBias[1];
  pid_control_V1_B.ap_TemperatureBias_c[2] = obj->GyroParamsTemperatureBias[2];
  pid_control_V1_B.gp_TemperatureScaleFactor_b[0] =
    obj->GyroParamsTemperatureScaleFactor[0];
  pid_control_V1_B.gp_TemperatureScaleFactor_b[1] =
    obj->GyroParamsTemperatureScaleFactor[1];
  pid_control_V1_B.gp_TemperatureScaleFactor_b[2] =
    obj->GyroParamsTemperatureScaleFactor[2];
  pid_control_V1_B.val_p[0] = obj->GyroParamsAccelerationBias[0];
  pid_control_V1_B.val_p[1] = obj->GyroParamsAccelerationBias[1];
  pid_control_V1_B.val_p[2] = obj->GyroParamsAccelerationBias[2];
  b_Numerator = obj->GyroParamsBiasInstabilityNumerator;
  pid_control_V1_B.b_Denominator_l[0] =
    obj->GyroParamsBiasInstabilityDenominator[0];
  pid_control_V1_B.b_Denominator_l[1] =
    obj->GyroParamsBiasInstabilityDenominator[1];
  obj->coder_buffer_pobj1.isInitialized = 0;
  for (i = 0; i < 13; i++) {
    /* Start for MATLABSystem: '<Root>/IMU1' */
    obj->coder_buffer_pobj1.tunablePropertyChanged[i] = false;
  }

  /* Start for MATLABSystem: '<Root>/IMU1' */
  IMUSensorParameters_updateSys_o(val, val_0, pid_control_V1_B.ap_ConstantBias_c,
    pid_control_V1_B.valIn, pid_control_V1_B.ap_NoiseDensity,
    pid_control_V1_B.ap_BiasInstability_k, pid_control_V1_B.ap_RandomWalk,
    b_Numerator, pid_control_V1_B.b_Denominator_l, t1_Value,
    pid_control_V1_B.ap_TemperatureBias_c,
    pid_control_V1_B.gp_TemperatureScaleFactor_b, pid_control_V1_B.val_p,
    &obj->coder_buffer_pobj1);
  obj->pGyro = &obj->coder_buffer_pobj1;
  obj_1 = obj->pGyro;

  /* Start for MATLABSystem: '<Root>/IMU1' */
  flag = (obj_1->isInitialized == 1);
  if (flag) {
    obj_1->TunablePropsChanged = true;
    obj_1->tunablePropertyChanged[12] = true;
  }

  obj->pGyro->Temperature = obj->Temperature;

  /* Start for MATLABSystem: '<Root>/IMU1' */
  val = obj->MagParamsMeasurementRange;
  val_0 = obj->MagParamsResolution;
  pid_control_V1_B.ap_ConstantBias_c[0] = obj->MagParamsConstantBias[0];
  pid_control_V1_B.ap_ConstantBias_c[1] = obj->MagParamsConstantBias[1];
  pid_control_V1_B.ap_ConstantBias_c[2] = obj->MagParamsConstantBias[2];
  for (i = 0; i < 9; i++) {
    /* Start for MATLABSystem: '<Root>/IMU1' */
    pid_control_V1_B.valIn[i] = obj->MagParamsAxesMisalignment[i];
  }

  /* Start for MATLABSystem: '<Root>/IMU1' */
  pid_control_V1_B.ap_NoiseDensity[0] = obj->MagParamsNoiseDensity[0];
  pid_control_V1_B.ap_NoiseDensity[1] = obj->MagParamsNoiseDensity[1];
  pid_control_V1_B.ap_NoiseDensity[2] = obj->MagParamsNoiseDensity[2];
  pid_control_V1_B.ap_BiasInstability_k[0] = obj->MagParamsBiasInstability[0];
  pid_control_V1_B.ap_BiasInstability_k[1] = obj->MagParamsBiasInstability[1];
  pid_control_V1_B.ap_BiasInstability_k[2] = obj->MagParamsBiasInstability[2];
  pid_control_V1_B.ap_RandomWalk[0] = obj->MagParamsRandomWalk[0];
  pid_control_V1_B.ap_RandomWalk[1] = obj->MagParamsRandomWalk[1];
  pid_control_V1_B.ap_RandomWalk[2] = obj->MagParamsRandomWalk[2];
  pid_control_V1_B.ap_TemperatureBias_c[0] = obj->MagParamsTemperatureBias[0];
  pid_control_V1_B.ap_TemperatureBias_c[1] = obj->MagParamsTemperatureBias[1];
  pid_control_V1_B.ap_TemperatureBias_c[2] = obj->MagParamsTemperatureBias[2];
  pid_control_V1_B.val_p[0] = obj->MagParamsTemperatureScaleFactor[0];
  pid_control_V1_B.val_p[1] = obj->MagParamsTemperatureScaleFactor[1];
  pid_control_V1_B.val_p[2] = obj->MagParamsTemperatureScaleFactor[2];
  b_Numerator = obj->MagParamsBiasInstabilityNumerator;
  pid_control_V1_B.b_Denominator_l[0] = obj->
    MagParamsBiasInstabilityDenominator[0];
  pid_control_V1_B.b_Denominator_l[1] = obj->
    MagParamsBiasInstabilityDenominator[1];
  obj->coder_buffer_pobj0.isInitialized = 0;
  for (i = 0; i < 12; i++) {
    /* Start for MATLABSystem: '<Root>/IMU1' */
    obj->coder_buffer_pobj0.tunablePropertyChanged[i] = false;
  }

  /* Start for MATLABSystem: '<Root>/IMU1' */
  IMUSensorParameters_updateSy_on(val, val_0, pid_control_V1_B.ap_ConstantBias_c,
    pid_control_V1_B.valIn, pid_control_V1_B.ap_NoiseDensity,
    pid_control_V1_B.ap_BiasInstability_k, pid_control_V1_B.ap_RandomWalk,
    b_Numerator, pid_control_V1_B.b_Denominator_l, t1_Value,
    pid_control_V1_B.ap_TemperatureBias_c, pid_control_V1_B.val_p,
    &obj->coder_buffer_pobj0);
  obj->pMag = &obj->coder_buffer_pobj0;
  obj_2 = obj->pMag;

  /* Start for MATLABSystem: '<Root>/IMU1' */
  flag = (obj_2->isInitialized == 1);
  if (flag) {
    obj_2->TunablePropsChanged = true;
    obj_2->tunablePropertyChanged[11] = true;
  }

  obj->pMag->Temperature = obj->Temperature;
}

void pid_control_V1::pid_c_Subscriber_setupImpl_onhg(const
  ros_slros2_internal_block_Sub_T *obj)
{
  rmw_qos_profile_t qos_profile;
  sJ4ih70VmKcvCeguWN0mNVF lifespan;
  sJ4ih70VmKcvCeguWN0mNVF liveliness_lease_duration;
  static const char_T b_zeroDelimTopic[17] = "/setpoint/altura";
  qos_profile = rmw_qos_profile_default;

  /* Start for MATLABSystem: '<S13>/SourceBlock' */
  pid_control_V1_B.deadline_j.sec = 0.0;
  pid_control_V1_B.deadline_j.nsec = 0.0;
  lifespan.sec = 0.0;
  lifespan.nsec = 0.0;
  liveliness_lease_duration.sec = 0.0;
  liveliness_lease_duration.nsec = 0.0;
  SET_QOS_VALUES(qos_profile, RMW_QOS_POLICY_HISTORY_KEEP_LAST, (size_t)10.0,
                 RMW_QOS_POLICY_DURABILITY_VOLATILE,
                 RMW_QOS_POLICY_RELIABILITY_RELIABLE,
                 pid_control_V1_B.deadline_j, lifespan,
                 RMW_QOS_POLICY_LIVELINESS_AUTOMATIC, liveliness_lease_duration,
                 (bool)obj->QOSAvoidROSNamespaceConventions);
  for (int32_T i = 0; i < 17; i++) {
    /* Start for MATLABSystem: '<S13>/SourceBlock' */
    pid_control_V1_B.b_zeroDelimTopic_m[i] = b_zeroDelimTopic[i];
  }

  Sub_pid_control_V1_435.createSubscriber(&pid_control_V1_B.b_zeroDelimTopic_m[0],
    qos_profile);
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

  /* Start for MATLABSystem: '<S15>/SourceBlock' */
  pid_control_V1_B.deadline_g.sec = 0.0;
  pid_control_V1_B.deadline_g.nsec = 0.0;
  lifespan.sec = 0.0;
  lifespan.nsec = 0.0;
  liveliness_lease_duration.sec = 0.0;
  liveliness_lease_duration.nsec = 0.0;
  SET_QOS_VALUES(qos_profile, RMW_QOS_POLICY_HISTORY_KEEP_LAST, (size_t)10.0,
                 RMW_QOS_POLICY_DURABILITY_VOLATILE,
                 RMW_QOS_POLICY_RELIABILITY_RELIABLE,
                 pid_control_V1_B.deadline_g, lifespan,
                 RMW_QOS_POLICY_LIVELINESS_AUTOMATIC, liveliness_lease_duration,
                 (bool)obj->QOSAvoidROSNamespaceConventions);
  for (int32_T i = 0; i < 14; i++) {
    /* Start for MATLABSystem: '<S15>/SourceBlock' */
    b_zeroDelimTopic[i] = b_zeroDelimTopic_0[i];
  }

  Sub_pid_control_V1_377.createSubscriber(&b_zeroDelimTopic[0], qos_profile);
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

  /* Start for MATLABSystem: '<S14>/SourceBlock' */
  pid_control_V1_B.deadline_d.sec = 0.0;
  pid_control_V1_B.deadline_d.nsec = 0.0;
  lifespan.sec = 0.0;
  lifespan.nsec = 0.0;
  liveliness_lease_duration.sec = 0.0;
  liveliness_lease_duration.nsec = 0.0;
  SET_QOS_VALUES(qos_profile, RMW_QOS_POLICY_HISTORY_KEEP_LAST, (size_t)10.0,
                 RMW_QOS_POLICY_DURABILITY_VOLATILE,
                 RMW_QOS_POLICY_RELIABILITY_RELIABLE,
                 pid_control_V1_B.deadline_d, lifespan,
                 RMW_QOS_POLICY_LIVELINESS_AUTOMATIC, liveliness_lease_duration,
                 (bool)obj->QOSAvoidROSNamespaceConventions);
  for (int32_T i = 0; i < 15; i++) {
    /* Start for MATLABSystem: '<S14>/SourceBlock' */
    b_zeroDelimTopic[i] = b_zeroDelimTopic_0[i];
  }

  Sub_pid_control_V1_538.createSubscriber(&b_zeroDelimTopic[0], qos_profile);
}

void pid_control_V1::pid_con_ServiceCaller_setupImpl(const
  ros_slros2_internal_block_Ser_T *obj)
{
  rmw_qos_profile_t qos_profile;
  sJ4ih70VmKcvCeguWN0mNVF lifespan;
  sJ4ih70VmKcvCeguWN0mNVF liveliness_lease_duration;
  static const char_T b_zeroDelimTopic[25] = "/gazebo/set_entity_state";
  qos_profile = rmw_qos_profile_default;

  /* Start for MATLABSystem: '<S2>/ServiceCaller' */
  pid_control_V1_B.deadline.sec = 0.0;
  pid_control_V1_B.deadline.nsec = 0.0;
  lifespan.sec = 0.0;
  lifespan.nsec = 0.0;
  liveliness_lease_duration.sec = 0.0;
  liveliness_lease_duration.nsec = 0.0;
  SET_QOS_VALUES(qos_profile, RMW_QOS_POLICY_HISTORY_KEEP_LAST, (size_t)1.0,
                 RMW_QOS_POLICY_DURABILITY_VOLATILE,
                 RMW_QOS_POLICY_RELIABILITY_RELIABLE, pid_control_V1_B.deadline,
                 lifespan, RMW_QOS_POLICY_LIVELINESS_AUTOMATIC,
                 liveliness_lease_duration, (bool)
                 obj->QOSAvoidROSNamespaceConventions);
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
  sJ4ih70VmKcvCeguWN0mNVF lifespan;
  sJ4ih70VmKcvCeguWN0mNVF liveliness_lease_duration;
  static const char_T b_zeroDelimTopic[22] = "/setpoint/turbulencia";
  qos_profile = rmw_qos_profile_default;

  /* Start for MATLABSystem: '<S288>/SourceBlock' */
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
  for (int32_T i = 0; i < 22; i++) {
    /* Start for MATLABSystem: '<S288>/SourceBlock' */
    pid_control_V1_B.b_zeroDelimTopic_g[i] = b_zeroDelimTopic[i];
  }

  Sub_pid_control_V1_417.createSubscriber(&pid_control_V1_B.b_zeroDelimTopic_g[0],
    qos_profile);
}

void pid_control_V1::pid_contro_Subscriber_setupImpl(const
  ros_slros2_internal_block_Sub_T *obj)
{
  rmw_qos_profile_t qos_profile;
  sJ4ih70VmKcvCeguWN0mNVF lifespan;
  sJ4ih70VmKcvCeguWN0mNVF liveliness_lease_duration;
  static const char_T b_zeroDelimTopic[22] = "/setpoint/turbulencia";
  qos_profile = rmw_qos_profile_default;

  /* Start for MATLABSystem: '<S285>/SourceBlock' */
  pid_control_V1_B.deadline_n.sec = 0.0;
  pid_control_V1_B.deadline_n.nsec = 0.0;
  lifespan.sec = 0.0;
  lifespan.nsec = 0.0;
  liveliness_lease_duration.sec = 0.0;
  liveliness_lease_duration.nsec = 0.0;
  SET_QOS_VALUES(qos_profile, RMW_QOS_POLICY_HISTORY_KEEP_LAST, (size_t)10.0,
                 RMW_QOS_POLICY_DURABILITY_VOLATILE,
                 RMW_QOS_POLICY_RELIABILITY_RELIABLE,
                 pid_control_V1_B.deadline_n, lifespan,
                 RMW_QOS_POLICY_LIVELINESS_AUTOMATIC, liveliness_lease_duration,
                 (bool)obj->QOSAvoidROSNamespaceConventions);
  for (int32_T i = 0; i < 22; i++) {
    /* Start for MATLABSystem: '<S285>/SourceBlock' */
    pid_control_V1_B.b_zeroDelimTopic_f[i] = b_zeroDelimTopic[i];
  }

  Sub_pid_control_V1_423.createSubscriber(&pid_control_V1_B.b_zeroDelimTopic_f[0],
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

  /* Start for MATLABSystem: '<S286>/SourceBlock' */
  pid_control_V1_B.deadline_ld.sec = 0.0;
  pid_control_V1_B.deadline_ld.nsec = 0.0;
  lifespan.sec = 0.0;
  lifespan.nsec = 0.0;
  liveliness_lease_duration.sec = 0.0;
  liveliness_lease_duration.nsec = 0.0;
  SET_QOS_VALUES(qos_profile, RMW_QOS_POLICY_HISTORY_KEEP_LAST, (size_t)10.0,
                 RMW_QOS_POLICY_DURABILITY_VOLATILE,
                 RMW_QOS_POLICY_RELIABILITY_RELIABLE,
                 pid_control_V1_B.deadline_ld, lifespan,
                 RMW_QOS_POLICY_LIVELINESS_AUTOMATIC, liveliness_lease_duration,
                 (bool)obj->QOSAvoidROSNamespaceConventions);
  for (int32_T i = 0; i < 12; i++) {
    /* Start for MATLABSystem: '<S286>/SourceBlock' */
    b_zeroDelimTopic[i] = b_zeroDelimTopic_0[i];
  }

  Sub_pid_control_V1_443.createSubscriber(&b_zeroDelimTopic[0], qos_profile);
}

void pid_control_V1::pid_con_Subscriber_setupImpl_on(const
  ros_slros2_internal_block_Sub_T *obj)
{
  rmw_qos_profile_t qos_profile;
  sJ4ih70VmKcvCeguWN0mNVF lifespan;
  sJ4ih70VmKcvCeguWN0mNVF liveliness_lease_duration;
  static const char_T b_zeroDelimTopic[17] = "/olas/pitch_rate";
  qos_profile = rmw_qos_profile_default;

  /* Start for MATLABSystem: '<S287>/SourceBlock' */
  pid_control_V1_B.deadline_l.sec = 0.0;
  pid_control_V1_B.deadline_l.nsec = 0.0;
  lifespan.sec = 0.0;
  lifespan.nsec = 0.0;
  liveliness_lease_duration.sec = 0.0;
  liveliness_lease_duration.nsec = 0.0;
  SET_QOS_VALUES(qos_profile, RMW_QOS_POLICY_HISTORY_KEEP_LAST, (size_t)10.0,
                 RMW_QOS_POLICY_DURABILITY_VOLATILE,
                 RMW_QOS_POLICY_RELIABILITY_RELIABLE,
                 pid_control_V1_B.deadline_l, lifespan,
                 RMW_QOS_POLICY_LIVELINESS_AUTOMATIC, liveliness_lease_duration,
                 (bool)obj->QOSAvoidROSNamespaceConventions);
  for (int32_T i = 0; i < 17; i++) {
    /* Start for MATLABSystem: '<S287>/SourceBlock' */
    pid_control_V1_B.b_zeroDelimTopic_g1[i] = b_zeroDelimTopic[i];
  }

  Sub_pid_control_V1_445.createSubscriber(&pid_control_V1_B.b_zeroDelimTopic_g1
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

void pid_control_V1::pid_con_IMUSensorBase_resetImpl
  (fusion_simulink_imuSensor_pid_T *obj)
{
  d_fusion_internal_Acceleromet_T *obj_0;
  e_fusion_internal_GyroscopeSi_T *obj_1;
  e_fusion_internal_Magnetomete_T *obj_2;
  uint32_T r;
  boolean_T flag;
  for (int32_T i = 0; i < 625; i++) {
    /* Start for MATLABSystem: '<Root>/IMU1' */
    obj->pStreamState[i] = 0U;
  }

  for (int32_T i = 0; i < 625; i++) {
    /* Start for MATLABSystem: '<Root>/IMU1' */
    pid_control_V1_B.state_m[i] = obj->pStreamState[i];
  }

  r = 67U;
  pid_control_V1_B.state_m[0] = 67U;
  for (int32_T i = 0; i < 623; i++) {
    /* Start for MATLABSystem: '<Root>/IMU1' */
    r = ((r >> 30U ^ r) * 1812433253U + static_cast<uint32_T>(i)) + 1U;
    pid_control_V1_B.state_m[i + 1] = r;
  }

  pid_control_V1_B.state_m[624] = 624U;
  for (int32_T i = 0; i < 625; i++) {
    /* Start for MATLABSystem: '<Root>/IMU1' */
    obj->pStreamState[i] = pid_control_V1_B.state_m[i];
  }

  /* Start for MATLABSystem: '<Root>/IMU1' */
  flag = (obj->isInitialized == 1);
  if (flag) {
    obj_0 = obj->pAccel;
    if (obj_0->isInitialized == 1) {
      /* Start for MATLABSystem: '<Root>/IMU1' */
      obj_0->pBiasInstFilterStates[0] = 0.0;
      obj_0->pRandWalkFilterStates[0] = 0.0;
      obj_0->pBiasInstFilterStates[1] = 0.0;
      obj_0->pRandWalkFilterStates[1] = 0.0;
      obj_0->pBiasInstFilterStates[2] = 0.0;
      obj_0->pRandWalkFilterStates[2] = 0.0;
    }

    obj_1 = obj->pGyro;
    if (obj_1->isInitialized == 1) {
      /* Start for MATLABSystem: '<Root>/IMU1' */
      obj_1->pBiasInstFilterStates[0] = 0.0;
      obj_1->pRandWalkFilterStates[0] = 0.0;
      obj_1->pBiasInstFilterStates[1] = 0.0;
      obj_1->pRandWalkFilterStates[1] = 0.0;
      obj_1->pBiasInstFilterStates[2] = 0.0;
      obj_1->pRandWalkFilterStates[2] = 0.0;
    }

    obj_2 = obj->pMag;
    if (obj_2->isInitialized == 1) {
      /* Start for MATLABSystem: '<Root>/IMU1' */
      obj_2->pBiasInstFilterStates[0] = 0.0;
      obj_2->pRandWalkFilterStates[0] = 0.0;
      obj_2->pBiasInstFilterStates[1] = 0.0;
      obj_2->pRandWalkFilterStates[1] = 0.0;
      obj_2->pBiasInstFilterStates[2] = 0.0;
      obj_2->pRandWalkFilterStates[2] = 0.0;
    }
  }
}

boolean_T pid_control_V1::pid_control_V1_isequal_o(const real_T varargin_1[3],
  const real_T varargin_2[3])
{
  int32_T b_k;
  boolean_T exitg1;
  boolean_T p;
  boolean_T p_0;
  p = false;
  p_0 = true;
  b_k = 0;
  exitg1 = false;
  while ((!exitg1) && (b_k < 3)) {
    if (!(varargin_1[b_k] == varargin_2[b_k])) {
      p_0 = false;
      exitg1 = true;
    } else {
      b_k++;
    }
  }

  if (p_0) {
    p = true;
  }

  return p;
}

void pid_control_V1::imuSensor_set_MagneticFieldNED
  (fusion_simulink_imuSensor_pid_T *obj, const real_T val[3])
{
  /* Start for MATLABSystem: '<Root>/IMU1' */
  if (obj->isInitialized == 1) {
    obj->TunablePropsChanged = true;
    obj->tunablePropertyChanged[2] = true;
  }

  obj->MagneticField[0] = val[0];
  obj->MagneticField[1] = val[1];
  obj->MagneticField[2] = val[2];

  /* End of Start for MATLABSystem: '<Root>/IMU1' */
}

boolean_T pid_control_V1::pid_control_V1_isequal_on(const real_T varargin_1[9],
  const real_T varargin_2[9])
{
  int32_T b_k;
  boolean_T exitg1;
  boolean_T p;
  boolean_T p_0;
  p = false;
  p_0 = true;
  b_k = 0;
  exitg1 = false;
  while ((!exitg1) && (b_k < 9)) {
    if (!(varargin_1[b_k] == varargin_2[b_k])) {
      p_0 = false;
      exitg1 = true;
    } else {
      b_k++;
    }
  }

  if (p_0) {
    p = true;
  }

  return p;
}

boolean_T pid_control_V1::pid_control_V1_isequal(const real_T varargin_1[2],
  const real_T varargin_2[2])
{
  int32_T b_k;
  boolean_T exitg1;
  boolean_T p;
  boolean_T p_0;
  p = false;
  p_0 = true;
  b_k = 0;
  exitg1 = false;
  while ((!exitg1) && (b_k < 2)) {
    if (!(varargin_1[b_k] == varargin_2[b_k])) {
      p_0 = false;
      exitg1 = true;
    } else {
      b_k++;
    }
  }

  if (p_0) {
    p = true;
  }

  return p;
}

boolean_T pid_control_V1::pid_control_V1_vectorAny(const boolean_T x_data[],
  const int32_T x_size[2])
{
  int32_T b_k;
  boolean_T exitg1;
  boolean_T y;
  y = false;
  b_k = 0;
  exitg1 = false;
  while ((!exitg1) && (b_k <= x_size[1] - 1)) {
    if (x_data[b_k]) {
      y = true;
      exitg1 = true;
    } else {
      b_k++;
    }
  }

  return y;
}

void pid_control_V1::pid_contr_genrand_uint32_vector(uint32_T mt[625], uint32_T
  u[2])
{
  /* Start for MATLABSystem: '<Root>/IMU1' */
  for (pid_control_V1_B.b_j = 0; pid_control_V1_B.b_j < 2; pid_control_V1_B.b_j
       ++) {
    pid_control_V1_B.mti = mt[624] + 1U;
    if (mt[624] + 1U >= 625U) {
      for (int32_T b_kk = 0; b_kk < 227; b_kk++) {
        pid_control_V1_B.y_o = (mt[b_kk + 1] & 2147483647U) | (mt[b_kk] &
          2147483648U);
        if ((pid_control_V1_B.y_o & 1U) == 0U) {
          pid_control_V1_B.mti = pid_control_V1_B.y_o >> 1U;
        } else {
          pid_control_V1_B.mti = pid_control_V1_B.y_o >> 1U ^ 2567483615U;
        }

        mt[b_kk] = mt[b_kk + 397] ^ pid_control_V1_B.mti;
      }

      for (int32_T b_kk = 0; b_kk < 396; b_kk++) {
        pid_control_V1_B.y_o = (mt[b_kk + 227] & 2147483648U) | (mt[b_kk + 228]
          & 2147483647U);
        if ((pid_control_V1_B.y_o & 1U) == 0U) {
          pid_control_V1_B.mti = pid_control_V1_B.y_o >> 1U;
        } else {
          pid_control_V1_B.mti = pid_control_V1_B.y_o >> 1U ^ 2567483615U;
        }

        mt[b_kk + 227] = mt[b_kk] ^ pid_control_V1_B.mti;
      }

      pid_control_V1_B.y_o = (mt[623] & 2147483648U) | (mt[0] & 2147483647U);
      if ((pid_control_V1_B.y_o & 1U) == 0U) {
        pid_control_V1_B.mti = pid_control_V1_B.y_o >> 1U;
      } else {
        pid_control_V1_B.mti = pid_control_V1_B.y_o >> 1U ^ 2567483615U;
      }

      mt[623] = mt[396] ^ pid_control_V1_B.mti;
      pid_control_V1_B.mti = 1U;
    }

    pid_control_V1_B.y_o = mt[static_cast<int32_T>(pid_control_V1_B.mti) - 1];
    mt[624] = pid_control_V1_B.mti;
    pid_control_V1_B.y_o ^= pid_control_V1_B.y_o >> 11U;
    pid_control_V1_B.y_o ^= pid_control_V1_B.y_o << 7U & 2636928640U;
    pid_control_V1_B.y_o ^= pid_control_V1_B.y_o << 15U & 4022730752U;
    u[pid_control_V1_B.b_j] = pid_control_V1_B.y_o >> 18U ^ pid_control_V1_B.y_o;
  }

  /* End of Start for MATLABSystem: '<Root>/IMU1' */
}

real_T pid_control_V1::pid_control_V1_genrandu(uint32_T mt[625])
{
  real_T r;
  int32_T exitg1;
  boolean_T b_isvalid;
  boolean_T exitg2;

  /* ========================= COPYRIGHT NOTICE ============================ */
  /*  This is a uniform (0,1) pseudorandom number generator based on: */
  /*  */
  /*  A C-program for MT19937, with initialization improved 2002/1/26. */
  /*  Coded by Takuji Nishimura and Makoto Matsumoto. */
  /*  */
  /*  Copyright (C) 1997 - 2002, Makoto Matsumoto and Takuji Nishimura, */
  /*  All rights reserved. */
  /*  */
  /*  Redistribution and use in source and binary forms, with or without */
  /*  modification, are permitted provided that the following conditions */
  /*  are met: */
  /*  */
  /*    1. Redistributions of source code must retain the above copyright */
  /*       notice, this list of conditions and the following disclaimer. */
  /*  */
  /*    2. Redistributions in binary form must reproduce the above copyright */
  /*       notice, this list of conditions and the following disclaimer */
  /*       in the documentation and/or other materials provided with the */
  /*       distribution. */
  /*  */
  /*    3. The names of its contributors may not be used to endorse or */
  /*       promote products derived from this software without specific */
  /*       prior written permission. */
  /*  */
  /*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS */
  /*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT */
  /*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR */
  /*  A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT */
  /*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, */
  /*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT */
  /*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, */
  /*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY */
  /*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT */
  /*  (INCLUDING  NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE */
  /*  OF THIS  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */
  /*  */
  /* =============================   END   ================================= */
  do {
    exitg1 = 0;
    pid_contr_genrand_uint32_vector(mt, pid_control_V1_B.b_u);
    pid_control_V1_B.u_idx_0 = pid_control_V1_B.b_u[0] >> 5U;
    pid_control_V1_B.u_idx_1 = pid_control_V1_B.b_u[1] >> 6U;
    if ((pid_control_V1_B.u_idx_0 == 0U) && (pid_control_V1_B.u_idx_1 == 0U)) {
      if ((mt[624] >= 1U) && (mt[624] < 625U)) {
        b_isvalid = true;
      } else {
        b_isvalid = false;
      }

      if (b_isvalid) {
        b_isvalid = false;
        pid_control_V1_B.k = 0;
        exitg2 = false;
        while ((!exitg2) && (pid_control_V1_B.k + 1 < 625)) {
          if (mt[pid_control_V1_B.k] == 0U) {
            pid_control_V1_B.k++;
          } else {
            b_isvalid = true;
            exitg2 = true;
          }
        }
      }

      if (!b_isvalid) {
        pid_control_V1_B.u_idx_0 = 67U;
        mt[0] = 67U;
        for (pid_control_V1_B.k = 0; pid_control_V1_B.k < 623;
             pid_control_V1_B.k++) {
          pid_control_V1_B.u_idx_0 = ((pid_control_V1_B.u_idx_0 >> 30U ^
            pid_control_V1_B.u_idx_0) * 1812433253U + static_cast<uint32_T>
            (pid_control_V1_B.k)) + 1U;
          mt[pid_control_V1_B.k + 1] = pid_control_V1_B.u_idx_0;
        }

        mt[624] = 624U;
      }
    } else {
      exitg1 = 1;
    }
  } while (exitg1 == 0);

  /* Start for MATLABSystem: '<Root>/IMU1' */
  r = (static_cast<real_T>(pid_control_V1_B.u_idx_0) * 6.7108864E+7 +
       static_cast<real_T>(pid_control_V1_B.u_idx_1)) * 1.1102230246251565E-16;
  return r;
}

void pid_control_V1::pid_control_V1_filter(real_T b, real_T a[2], const real_T
  x[3], const real_T zi[3], real_T y[3], real_T zf[3])
{
  /* Start for MATLABSystem: '<Root>/IMU1' */
  if ((!rtIsInf(a[0])) && (!rtIsNaN(a[0])) && (!(a[0] == 0.0)) && (a[0] != 1.0))
  {
    _mm_storeu_pd(&pid_control_V1_B.dv4[0], _mm_div_pd(_mm_set_pd(a[1], b),
      _mm_set1_pd(a[0])));
    b = pid_control_V1_B.dv4[0];
    a[1] = pid_control_V1_B.dv4[1];
  }

  pid_control_V1_B.y_d = x[0] * b + zi[0];
  y[0] = pid_control_V1_B.y_d;
  zf[0] = -pid_control_V1_B.y_d * a[1];
  pid_control_V1_B.y_d = x[1] * b + zi[1];
  y[1] = pid_control_V1_B.y_d;
  zf[1] = -pid_control_V1_B.y_d * a[1];
  pid_control_V1_B.y_d = x[2] * b + zi[2];
  y[2] = pid_control_V1_B.y_d;
  zf[2] = -pid_control_V1_B.y_d * a[1];

  /* End of Start for MATLABSystem: '<Root>/IMU1' */
}

real_T rt_roundd_snf(real_T u)
{
  real_T y;
  if (fabs(u) < 4.503599627370496E+15) {
    if (u >= 0.5) {
      y = floor(u + 0.5);
    } else if (u > -0.5) {
      y = u * 0.0;
    } else {
      y = ceil(u - 0.5);
    }
  } else {
    y = u;
  }

  return y;
}

void pid_control_V1::pid_control_V_SystemCore_step_o
  (d_fusion_internal_Acceleromet_T *obj, const real_T varargin_1[3], const
   real_T varargin_2[9], const real_T varargin_3[9], real_T varargout_1[3])
{
  static const char_T b[12] = { 'd', 'o', 'u', 'b', 'l', 'e', '-', 's', 'i', 'd',
    'e', 'd' };

  __m128d tmp;
  if (obj->isInitialized != 1) {
    /* Start for MATLABSystem: '<Root>/IMU1' */
    obj->isInitialized = 1;
    for (pid_control_V1_B.ret_j = 0; pid_control_V1_B.ret_j <= 6;
         pid_control_V1_B.ret_j += 2) {
      /* Start for MATLABSystem: '<Root>/IMU1' */
      tmp = _mm_loadu_pd(&obj->AxesMisalignment[pid_control_V1_B.ret_j]);
      _mm_storeu_pd(&obj->pGain[pid_control_V1_B.ret_j], _mm_div_pd(tmp,
        _mm_set1_pd(100.0)));
    }

    for (pid_control_V1_B.ret_j = 8; pid_control_V1_B.ret_j < 9;
         pid_control_V1_B.ret_j++) {
      /* Start for MATLABSystem: '<Root>/IMU1' */
      obj->pGain[pid_control_V1_B.ret_j] = obj->
        AxesMisalignment[pid_control_V1_B.ret_j] / 100.0;
    }

    /* Start for MATLABSystem: '<Root>/IMU1' */
    pid_control_V1_B.ret_j = std::memcmp(&obj->NoiseType.Value[0], &b[0], 12);
    if (pid_control_V1_B.ret_j == 0) {
      obj->pBandwidth = 50.0;
    } else {
      obj->pBandwidth = 100.0;
    }

    obj->pBiasInstFilterNum = obj->BiasInstabilityCoefficients.Numerator;
    obj->pBiasInstFilterDen[0] = obj->BiasInstabilityCoefficients.Denominator[0];
    obj->pBiasInstFilterDen[1] = obj->BiasInstabilityCoefficients.Denominator[1];
    obj->pCorrelationTime = 0.02;
    pid_control_V1_B.b_x_e = sqrt(obj->pBandwidth);
    obj->TunablePropsChanged = false;
    tmp = _mm_set_pd(sqrt(obj->pBandwidth), sqrt(2.0 / (100.0 *
      obj->pCorrelationTime)));
    _mm_storeu_pd(&pid_control_V1_B.dv5[0], _mm_mul_pd(tmp, _mm_set_pd
      (obj->NoiseDensity[0], obj->BiasInstability[0])));
    obj->pStdDevBiasInst[0] = pid_control_V1_B.dv5[0];
    obj->pStdDevWhiteNoise[0] = pid_control_V1_B.dv5[1];
    obj->pStdDevRandWalk[0] = obj->RandomWalk[0] / pid_control_V1_B.b_x_e;
    obj->pBiasInstFilterStates[0] = 0.0;
    obj->pRandWalkFilterStates[0] = 0.0;
    _mm_storeu_pd(&pid_control_V1_B.dv5[0], _mm_mul_pd(tmp, _mm_set_pd
      (obj->NoiseDensity[1], obj->BiasInstability[1])));
    obj->pStdDevBiasInst[1] = pid_control_V1_B.dv5[0];
    obj->pStdDevWhiteNoise[1] = pid_control_V1_B.dv5[1];
    obj->pStdDevRandWalk[1] = obj->RandomWalk[1] / pid_control_V1_B.b_x_e;
    obj->pBiasInstFilterStates[1] = 0.0;
    obj->pRandWalkFilterStates[1] = 0.0;
    _mm_storeu_pd(&pid_control_V1_B.dv5[0], _mm_mul_pd(tmp, _mm_set_pd
      (obj->NoiseDensity[2], obj->BiasInstability[2])));
    obj->pStdDevBiasInst[2] = pid_control_V1_B.dv5[0];
    obj->pStdDevWhiteNoise[2] = pid_control_V1_B.dv5[1];
    obj->pStdDevRandWalk[2] = obj->RandomWalk[2] / pid_control_V1_B.b_x_e;
    obj->pBiasInstFilterStates[2] = 0.0;
    obj->pRandWalkFilterStates[2] = 0.0;
  }

  /* Start for MATLABSystem: '<Root>/IMU1' */
  if (obj->TunablePropsChanged) {
    obj->TunablePropsChanged = false;
    if (obj->tunablePropertyChanged[3]) {
      for (pid_control_V1_B.ret_j = 0; pid_control_V1_B.ret_j <= 6;
           pid_control_V1_B.ret_j += 2) {
        tmp = _mm_loadu_pd(&obj->AxesMisalignment[pid_control_V1_B.ret_j]);
        _mm_storeu_pd(&obj->pGain[pid_control_V1_B.ret_j], _mm_div_pd(tmp,
          _mm_set1_pd(100.0)));
      }

      for (pid_control_V1_B.ret_j = 8; pid_control_V1_B.ret_j < 9;
           pid_control_V1_B.ret_j++) {
        obj->pGain[pid_control_V1_B.ret_j] = obj->
          AxesMisalignment[pid_control_V1_B.ret_j] / 100.0;
      }
    }

    if (obj->tunablePropertyChanged[4] || obj->tunablePropertyChanged[8]) {
      pid_control_V1_B.ret_j = std::memcmp(&obj->NoiseType.Value[0], &b[0], 12);
      if (pid_control_V1_B.ret_j == 0) {
        obj->pBandwidth = 50.0;
      } else {
        obj->pBandwidth = 100.0;
      }

      pid_control_V1_B.b_x_e = sqrt(obj->pBandwidth);
      tmp = _mm_mul_pd(_mm_set1_pd(pid_control_V1_B.b_x_e), _mm_loadu_pd
                       (&obj->NoiseDensity[0]));
      _mm_storeu_pd(&obj->pStdDevWhiteNoise[0], tmp);
      obj->pStdDevWhiteNoise[2] = pid_control_V1_B.b_x_e * obj->NoiseDensity[2];
    }

    if (obj->tunablePropertyChanged[5]) {
      pid_control_V1_B.b_x_e = sqrt(2.0 / (100.0 * obj->pCorrelationTime));
      tmp = _mm_mul_pd(_mm_set1_pd(pid_control_V1_B.b_x_e), _mm_loadu_pd
                       (&obj->BiasInstability[0]));
      _mm_storeu_pd(&obj->pStdDevBiasInst[0], tmp);
      obj->pStdDevBiasInst[2] = pid_control_V1_B.b_x_e * obj->BiasInstability[2];
    }

    if (obj->tunablePropertyChanged[6] || obj->tunablePropertyChanged[8]) {
      pid_control_V1_B.ret_j = std::memcmp(&obj->NoiseType.Value[0], &b[0], 12);
      if (pid_control_V1_B.ret_j == 0) {
        obj->pBandwidth = 50.0;
      } else {
        obj->pBandwidth = 100.0;
      }

      pid_control_V1_B.b_x_e = sqrt(obj->pBandwidth);
      tmp = _mm_div_pd(_mm_loadu_pd(&obj->RandomWalk[0]), _mm_set1_pd
                       (pid_control_V1_B.b_x_e));
      _mm_storeu_pd(&obj->pStdDevRandWalk[0], tmp);
      obj->pStdDevRandWalk[2] = obj->RandomWalk[2] / pid_control_V1_B.b_x_e;
    }

    if (obj->tunablePropertyChanged[7]) {
      obj->pBiasInstFilterNum = obj->BiasInstabilityCoefficients.Numerator;
      obj->pBiasInstFilterDen[0] = obj->BiasInstabilityCoefficients.Denominator
        [0];
      obj->pBiasInstFilterDen[1] = obj->BiasInstabilityCoefficients.Denominator
        [1];
    }

    for (pid_control_V1_B.ret_j = 0; pid_control_V1_B.ret_j < 12;
         pid_control_V1_B.ret_j++) {
      obj->tunablePropertyChanged[pid_control_V1_B.ret_j] = false;
    }
  }

  pid_control_V1_B.y[0] = -varargin_1[0];
  pid_control_V1_B.y[1] = -varargin_1[1];
  pid_control_V1_B.y[2] = -varargin_1[2] + 9.81;
  pid_control_V1_B.b_x_e = 0.0;
  pid_control_V1_B.x_idx_5_j = 0.0;
  pid_control_V1_B.varargin_2_b = 0.0;
  for (pid_control_V1_B.ret_j = 0; pid_control_V1_B.ret_j < 3;
       pid_control_V1_B.ret_j++) {
    _mm_storeu_pd(&pid_control_V1_B.dv5[0], _mm_add_pd(_mm_mul_pd(_mm_loadu_pd
      (&varargin_2[3 * pid_control_V1_B.ret_j]), _mm_set1_pd
      (pid_control_V1_B.y[pid_control_V1_B.ret_j])), _mm_set_pd
      (pid_control_V1_B.x_idx_5_j, pid_control_V1_B.b_x_e)));
    pid_control_V1_B.b_x_e = pid_control_V1_B.dv5[0];
    pid_control_V1_B.x_idx_5_j = pid_control_V1_B.dv5[1];
    pid_control_V1_B.varargin_2_b += varargin_2[3 * pid_control_V1_B.ret_j + 2] *
      pid_control_V1_B.y[pid_control_V1_B.ret_j];
  }

  pid_control_V1_B.varargin_2[2] = pid_control_V1_B.varargin_2_b;
  pid_control_V1_B.varargin_2[1] = pid_control_V1_B.x_idx_5_j;
  pid_control_V1_B.varargin_2[0] = pid_control_V1_B.b_x_e;
  pid_control_V1_B.b_x_e = 0.0;
  pid_control_V1_B.x_idx_5_j = 0.0;
  pid_control_V1_B.varargin_2_b = 0.0;
  for (pid_control_V1_B.ret_j = 0; pid_control_V1_B.ret_j < 3;
       pid_control_V1_B.ret_j++) {
    /* Start for MATLABSystem: '<Root>/IMU1' */
    tmp = _mm_add_pd(_mm_mul_pd(_mm_loadu_pd(&obj->pGain[3 *
      pid_control_V1_B.ret_j]), _mm_set1_pd
      (pid_control_V1_B.varargin_2[pid_control_V1_B.ret_j])), _mm_set_pd
                     (pid_control_V1_B.x_idx_5_j, pid_control_V1_B.b_x_e));
    _mm_storeu_pd(&pid_control_V1_B.dv5[0], tmp);
    pid_control_V1_B.b_x_e = pid_control_V1_B.dv5[0];
    pid_control_V1_B.x_idx_5_j = pid_control_V1_B.dv5[1];

    /* Start for MATLABSystem: '<Root>/IMU1' */
    pid_control_V1_B.varargin_2_b += obj->pGain[3 * pid_control_V1_B.ret_j + 2] *
      pid_control_V1_B.varargin_2[pid_control_V1_B.ret_j];
  }

  /* Start for MATLABSystem: '<Root>/IMU1' */
  varargout_1[0] = pid_control_V1_B.b_x_e + obj->ConstantBias[0];
  pid_control_V1_B.y[0] = varargin_3[0] * obj->pStdDevBiasInst[0];
  varargout_1[1] = pid_control_V1_B.x_idx_5_j + obj->ConstantBias[1];
  pid_control_V1_B.y[1] = varargin_3[1] * obj->pStdDevBiasInst[1];
  varargout_1[2] = pid_control_V1_B.varargin_2_b + obj->ConstantBias[2];
  pid_control_V1_B.y[2] = varargin_3[2] * obj->pStdDevBiasInst[2];
  for (pid_control_V1_B.ret_j = 0; pid_control_V1_B.ret_j < 3;
       pid_control_V1_B.ret_j++) {
    /* Start for MATLABSystem: '<Root>/IMU1' */
    pid_control_V1_B.obj_c[pid_control_V1_B.ret_j] = obj->
      pBiasInstFilterStates[pid_control_V1_B.ret_j];
  }

  for (pid_control_V1_B.i3 = 0; pid_control_V1_B.i3 < 2; pid_control_V1_B.i3++)
  {
    /* Start for MATLABSystem: '<Root>/IMU1' */
    pid_control_V1_B.obj_o[pid_control_V1_B.i3] = obj->
      pBiasInstFilterDen[pid_control_V1_B.i3];
  }

  /* Start for MATLABSystem: '<Root>/IMU1' */
  pid_control_V1_filter(obj->pBiasInstFilterNum, pid_control_V1_B.obj_o,
                        pid_control_V1_B.y, pid_control_V1_B.obj_c,
                        pid_control_V1_B.varargin_2, obj->pBiasInstFilterStates);
  tmp = _mm_add_pd(_mm_mul_pd(_mm_loadu_pd(&obj->pStdDevRandWalk[0]),
    _mm_loadu_pd(&varargin_3[6])), _mm_loadu_pd(&obj->pRandWalkFilterStates[0]));
  _mm_storeu_pd(&pid_control_V1_B.dv5[0], tmp);

  /* Start for MATLABSystem: '<Root>/IMU1' */
  pid_control_V1_B.x_idx_5_j = obj->pStdDevRandWalk[2] * varargin_3[8] +
    obj->pRandWalkFilterStates[2];
  pid_control_V1_B.b_x_e = (obj->Temperature - 25.0) * 0.01;
  obj->pRandWalkFilterStates[0] = pid_control_V1_B.dv5[0];
  varargout_1[0] = ((((obj->pStdDevWhiteNoise[0] * varargin_3[3] +
                       pid_control_V1_B.varargin_2[0]) + pid_control_V1_B.dv5[0])
                     + (obj->Temperature - 25.0) * obj->TemperatureBias[0]) +
                    varargout_1[0]) * (pid_control_V1_B.b_x_e *
    obj->TemperatureScaleFactor[0] + 1.0);
  obj->pRandWalkFilterStates[1] = pid_control_V1_B.dv5[1];
  varargout_1[1] = ((((obj->pStdDevWhiteNoise[1] * varargin_3[4] +
                       pid_control_V1_B.varargin_2[1]) + pid_control_V1_B.dv5[1])
                     + (obj->Temperature - 25.0) * obj->TemperatureBias[1]) +
                    varargout_1[1]) * (pid_control_V1_B.b_x_e *
    obj->TemperatureScaleFactor[1] + 1.0);
  obj->pRandWalkFilterStates[2] = pid_control_V1_B.x_idx_5_j;
  varargout_1[2] = ((((obj->pStdDevWhiteNoise[2] * varargin_3[5] +
                       pid_control_V1_B.varargin_2[2]) +
                      pid_control_V1_B.x_idx_5_j) + (obj->Temperature - 25.0) *
                     obj->TemperatureBias[2]) + varargout_1[2]) *
    (pid_control_V1_B.b_x_e * obj->TemperatureScaleFactor[2] + 1.0);
  if (!rtIsInf(obj->MeasurementRange)) {
    pid_control_V1_B.b_x_e = fabs(varargout_1[0]);
    pid_control_V1_B.y[0] = pid_control_V1_B.b_x_e;
    pid_control_V1_B.x_idx_5_j = fabs(varargout_1[1]);
    pid_control_V1_B.y[1] = pid_control_V1_B.x_idx_5_j;
    pid_control_V1_B.varargin_2_b = fabs(varargout_1[2]);
    pid_control_V1_B.y[2] = pid_control_V1_B.varargin_2_b;
    pid_control_V1_B.partialTrueCount_n = 0;
    for (pid_control_V1_B.ret_j = 0; pid_control_V1_B.ret_j < 3;
         pid_control_V1_B.ret_j++) {
      if (pid_control_V1_B.y[pid_control_V1_B.ret_j] > obj->MeasurementRange) {
        pid_control_V1_B.b_data_c[pid_control_V1_B.partialTrueCount_n] =
          static_cast<int8_T>(pid_control_V1_B.ret_j);
        pid_control_V1_B.partialTrueCount_n++;
      }
    }

    pid_control_V1_B.y[0] = pid_control_V1_B.b_x_e;
    pid_control_V1_B.y[1] = pid_control_V1_B.x_idx_5_j;
    pid_control_V1_B.y[2] = pid_control_V1_B.varargin_2_b;
    pid_control_V1_B.trueCount_o = 0;
    for (pid_control_V1_B.ret_j = 0; pid_control_V1_B.ret_j < 3;
         pid_control_V1_B.ret_j++) {
      if (pid_control_V1_B.y[pid_control_V1_B.ret_j] > obj->MeasurementRange) {
        pid_control_V1_B.trueCount_o++;
      }
    }

    pid_control_V1_B.partialTrueCount_n = 0;
    for (pid_control_V1_B.ret_j = 0; pid_control_V1_B.ret_j < 3;
         pid_control_V1_B.ret_j++) {
      if (pid_control_V1_B.y[pid_control_V1_B.ret_j] > obj->MeasurementRange) {
        pid_control_V1_B.tmp_data_md[pid_control_V1_B.partialTrueCount_n] =
          static_cast<int8_T>(pid_control_V1_B.ret_j);
        pid_control_V1_B.partialTrueCount_n++;
      }
    }

    pid_control_V1_B.partialTrueCount_n = pid_control_V1_B.trueCount_o;
    for (pid_control_V1_B.ret_j = 0; pid_control_V1_B.ret_j <
         pid_control_V1_B.trueCount_o; pid_control_V1_B.ret_j++) {
      pid_control_V1_B.varargin_2[pid_control_V1_B.ret_j] =
        varargout_1[pid_control_V1_B.tmp_data_md[pid_control_V1_B.ret_j]];
    }

    pid_control_V1_B.trueCount_o = 0;
    for (pid_control_V1_B.ret_j = 0; pid_control_V1_B.ret_j < 3;
         pid_control_V1_B.ret_j++) {
      if (pid_control_V1_B.y[pid_control_V1_B.ret_j] > obj->MeasurementRange) {
        pid_control_V1_B.trueCount_o++;
      }
    }

    for (pid_control_V1_B.ret_j = 0; pid_control_V1_B.ret_j <
         pid_control_V1_B.trueCount_o; pid_control_V1_B.ret_j++) {
      pid_control_V1_B.b_x_e =
        pid_control_V1_B.varargin_2[pid_control_V1_B.ret_j];
      if (rtIsNaN(pid_control_V1_B.b_x_e)) {
        pid_control_V1_B.varargin_2[pid_control_V1_B.ret_j] = (rtNaN);
      } else if (pid_control_V1_B.b_x_e < 0.0) {
        pid_control_V1_B.varargin_2[pid_control_V1_B.ret_j] = -1.0;
      } else {
        pid_control_V1_B.varargin_2[pid_control_V1_B.ret_j] =
          (pid_control_V1_B.b_x_e > 0.0);
      }
    }

    for (pid_control_V1_B.ret_j = 0; pid_control_V1_B.ret_j <
         pid_control_V1_B.partialTrueCount_n; pid_control_V1_B.ret_j++) {
      varargout_1[pid_control_V1_B.b_data_c[pid_control_V1_B.ret_j]] =
        pid_control_V1_B.varargin_2[pid_control_V1_B.ret_j] *
        obj->MeasurementRange;
    }
  }

  if (obj->Resolution != 0.0) {
    _mm_storeu_pd(&varargout_1[0], _mm_mul_pd(_mm_set_pd(rt_roundd_snf
      (varargout_1[1] / obj->Resolution), rt_roundd_snf(varargout_1[0] /
      obj->Resolution)), _mm_set1_pd(obj->Resolution)));
    varargout_1[2] = rt_roundd_snf(varargout_1[2] / obj->Resolution) *
      obj->Resolution;
  }
}

void pid_control_V1::pid_control__SystemCore_step_on
  (e_fusion_internal_GyroscopeSi_T *obj, const real_T varargin_1[3], const
   real_T varargin_2[3], const real_T varargin_3[9], const real_T varargin_4[9],
   real_T varargout_1[3])
{
  static const char_T b[12] = { 'd', 'o', 'u', 'b', 'l', 'e', '-', 's', 'i', 'd',
    'e', 'd' };

  __m128d tmp;
  if (obj->isInitialized != 1) {
    /* Start for MATLABSystem: '<Root>/IMU1' */
    obj->isInitialized = 1;
    for (pid_control_V1_B.ret = 0; pid_control_V1_B.ret <= 6;
         pid_control_V1_B.ret += 2) {
      /* Start for MATLABSystem: '<Root>/IMU1' */
      tmp = _mm_loadu_pd(&obj->AxesMisalignment[pid_control_V1_B.ret]);
      _mm_storeu_pd(&obj->pGain[pid_control_V1_B.ret], _mm_div_pd(tmp,
        _mm_set1_pd(100.0)));
    }

    for (pid_control_V1_B.ret = 8; pid_control_V1_B.ret < 9;
         pid_control_V1_B.ret++) {
      /* Start for MATLABSystem: '<Root>/IMU1' */
      obj->pGain[pid_control_V1_B.ret] = obj->
        AxesMisalignment[pid_control_V1_B.ret] / 100.0;
    }

    /* Start for MATLABSystem: '<Root>/IMU1' */
    pid_control_V1_B.ret = std::memcmp(&obj->NoiseType.Value[0], &b[0], 12);
    if (pid_control_V1_B.ret == 0) {
      obj->pBandwidth = 50.0;
    } else {
      obj->pBandwidth = 100.0;
    }

    obj->pBiasInstFilterNum = obj->BiasInstabilityCoefficients.Numerator;
    obj->pBiasInstFilterDen[0] = obj->BiasInstabilityCoefficients.Denominator[0];
    obj->pBiasInstFilterDen[1] = obj->BiasInstabilityCoefficients.Denominator[1];
    obj->pCorrelationTime = 0.02;
    pid_control_V1_B.b_x = sqrt(obj->pBandwidth);
    obj->TunablePropsChanged = false;
    tmp = _mm_set_pd(sqrt(obj->pBandwidth), sqrt(2.0 / (100.0 *
      obj->pCorrelationTime)));
    _mm_storeu_pd(&pid_control_V1_B.dv3[0], _mm_mul_pd(tmp, _mm_set_pd
      (obj->NoiseDensity[0], obj->BiasInstability[0])));
    obj->pStdDevBiasInst[0] = pid_control_V1_B.dv3[0];
    obj->pStdDevWhiteNoise[0] = pid_control_V1_B.dv3[1];
    obj->pStdDevRandWalk[0] = obj->RandomWalk[0] / pid_control_V1_B.b_x;
    obj->pBiasInstFilterStates[0] = 0.0;
    obj->pRandWalkFilterStates[0] = 0.0;
    _mm_storeu_pd(&pid_control_V1_B.dv3[0], _mm_mul_pd(tmp, _mm_set_pd
      (obj->NoiseDensity[1], obj->BiasInstability[1])));
    obj->pStdDevBiasInst[1] = pid_control_V1_B.dv3[0];
    obj->pStdDevWhiteNoise[1] = pid_control_V1_B.dv3[1];
    obj->pStdDevRandWalk[1] = obj->RandomWalk[1] / pid_control_V1_B.b_x;
    obj->pBiasInstFilterStates[1] = 0.0;
    obj->pRandWalkFilterStates[1] = 0.0;
    _mm_storeu_pd(&pid_control_V1_B.dv3[0], _mm_mul_pd(tmp, _mm_set_pd
      (obj->NoiseDensity[2], obj->BiasInstability[2])));
    obj->pStdDevBiasInst[2] = pid_control_V1_B.dv3[0];
    obj->pStdDevWhiteNoise[2] = pid_control_V1_B.dv3[1];
    obj->pStdDevRandWalk[2] = obj->RandomWalk[2] / pid_control_V1_B.b_x;
    obj->pBiasInstFilterStates[2] = 0.0;
    obj->pRandWalkFilterStates[2] = 0.0;
  }

  /* Start for MATLABSystem: '<Root>/IMU1' */
  if (obj->TunablePropsChanged) {
    obj->TunablePropsChanged = false;
    if (obj->tunablePropertyChanged[4]) {
      for (pid_control_V1_B.ret = 0; pid_control_V1_B.ret <= 6;
           pid_control_V1_B.ret += 2) {
        tmp = _mm_loadu_pd(&obj->AxesMisalignment[pid_control_V1_B.ret]);
        _mm_storeu_pd(&obj->pGain[pid_control_V1_B.ret], _mm_div_pd(tmp,
          _mm_set1_pd(100.0)));
      }

      for (pid_control_V1_B.ret = 8; pid_control_V1_B.ret < 9;
           pid_control_V1_B.ret++) {
        obj->pGain[pid_control_V1_B.ret] = obj->
          AxesMisalignment[pid_control_V1_B.ret] / 100.0;
      }
    }

    if (obj->tunablePropertyChanged[5] || obj->tunablePropertyChanged[9]) {
      pid_control_V1_B.ret = std::memcmp(&obj->NoiseType.Value[0], &b[0], 12);
      if (pid_control_V1_B.ret == 0) {
        obj->pBandwidth = 50.0;
      } else {
        obj->pBandwidth = 100.0;
      }

      pid_control_V1_B.b_x = sqrt(obj->pBandwidth);
      tmp = _mm_mul_pd(_mm_set1_pd(pid_control_V1_B.b_x), _mm_loadu_pd
                       (&obj->NoiseDensity[0]));
      _mm_storeu_pd(&obj->pStdDevWhiteNoise[0], tmp);
      obj->pStdDevWhiteNoise[2] = pid_control_V1_B.b_x * obj->NoiseDensity[2];
    }

    if (obj->tunablePropertyChanged[6]) {
      pid_control_V1_B.b_x = sqrt(2.0 / (100.0 * obj->pCorrelationTime));
      tmp = _mm_mul_pd(_mm_set1_pd(pid_control_V1_B.b_x), _mm_loadu_pd
                       (&obj->BiasInstability[0]));
      _mm_storeu_pd(&obj->pStdDevBiasInst[0], tmp);
      obj->pStdDevBiasInst[2] = pid_control_V1_B.b_x * obj->BiasInstability[2];
    }

    if (obj->tunablePropertyChanged[7] || obj->tunablePropertyChanged[9]) {
      pid_control_V1_B.ret = std::memcmp(&obj->NoiseType.Value[0], &b[0], 12);
      if (pid_control_V1_B.ret == 0) {
        obj->pBandwidth = 50.0;
      } else {
        obj->pBandwidth = 100.0;
      }

      pid_control_V1_B.b_x = sqrt(obj->pBandwidth);
      tmp = _mm_div_pd(_mm_loadu_pd(&obj->RandomWalk[0]), _mm_set1_pd
                       (pid_control_V1_B.b_x));
      _mm_storeu_pd(&obj->pStdDevRandWalk[0], tmp);
      obj->pStdDevRandWalk[2] = obj->RandomWalk[2] / pid_control_V1_B.b_x;
    }

    if (obj->tunablePropertyChanged[8]) {
      obj->pBiasInstFilterNum = obj->BiasInstabilityCoefficients.Numerator;
      obj->pBiasInstFilterDen[0] = obj->BiasInstabilityCoefficients.Denominator
        [0];
      obj->pBiasInstFilterDen[1] = obj->BiasInstabilityCoefficients.Denominator
        [1];
    }

    for (pid_control_V1_B.ret = 0; pid_control_V1_B.ret < 13;
         pid_control_V1_B.ret++) {
      obj->tunablePropertyChanged[pid_control_V1_B.ret] = false;
    }
  }

  obj->pAcceleration[0] = varargin_2[0];
  obj->pAcceleration[1] = varargin_2[1];
  obj->pAcceleration[2] = varargin_2[2];
  pid_control_V1_B.b_x = 0.0;
  pid_control_V1_B.x_idx_5 = 0.0;
  pid_control_V1_B.varargin_3_b = 0.0;
  for (pid_control_V1_B.ret = 0; pid_control_V1_B.ret < 3; pid_control_V1_B.ret
       ++) {
    _mm_storeu_pd(&pid_control_V1_B.dv3[0], _mm_add_pd(_mm_mul_pd(_mm_loadu_pd
      (&varargin_3[3 * pid_control_V1_B.ret]), _mm_set1_pd
      (varargin_1[pid_control_V1_B.ret])), _mm_set_pd(pid_control_V1_B.x_idx_5,
      pid_control_V1_B.b_x)));
    pid_control_V1_B.b_x = pid_control_V1_B.dv3[0];
    pid_control_V1_B.x_idx_5 = pid_control_V1_B.dv3[1];
    pid_control_V1_B.varargin_3_b += varargin_3[3 * pid_control_V1_B.ret + 2] *
      varargin_1[pid_control_V1_B.ret];
  }

  pid_control_V1_B.varargin_3[2] = pid_control_V1_B.varargin_3_b;
  pid_control_V1_B.varargin_3[1] = pid_control_V1_B.x_idx_5;
  pid_control_V1_B.varargin_3[0] = pid_control_V1_B.b_x;
  pid_control_V1_B.b_x = 0.0;
  pid_control_V1_B.x_idx_5 = 0.0;
  pid_control_V1_B.varargin_3_b = 0.0;
  for (pid_control_V1_B.ret = 0; pid_control_V1_B.ret < 3; pid_control_V1_B.ret
       ++) {
    /* Start for MATLABSystem: '<Root>/IMU1' */
    tmp = _mm_add_pd(_mm_mul_pd(_mm_loadu_pd(&obj->pGain[3 *
      pid_control_V1_B.ret]), _mm_set1_pd
      (pid_control_V1_B.varargin_3[pid_control_V1_B.ret])), _mm_set_pd
                     (pid_control_V1_B.x_idx_5, pid_control_V1_B.b_x));
    _mm_storeu_pd(&pid_control_V1_B.dv3[0], tmp);
    pid_control_V1_B.b_x = pid_control_V1_B.dv3[0];
    pid_control_V1_B.x_idx_5 = pid_control_V1_B.dv3[1];

    /* Start for MATLABSystem: '<Root>/IMU1' */
    pid_control_V1_B.varargin_3_b += obj->pGain[3 * pid_control_V1_B.ret + 2] *
      pid_control_V1_B.varargin_3[pid_control_V1_B.ret];
  }

  /* Start for MATLABSystem: '<Root>/IMU1' */
  varargout_1[0] = pid_control_V1_B.b_x + obj->ConstantBias[0];
  pid_control_V1_B.accelerationDrift[0] = varargin_4[0] * obj->pStdDevBiasInst[0];
  varargout_1[1] = pid_control_V1_B.x_idx_5 + obj->ConstantBias[1];
  pid_control_V1_B.accelerationDrift[1] = varargin_4[1] * obj->pStdDevBiasInst[1];
  varargout_1[2] = pid_control_V1_B.varargin_3_b + obj->ConstantBias[2];
  pid_control_V1_B.accelerationDrift[2] = varargin_4[2] * obj->pStdDevBiasInst[2];
  for (pid_control_V1_B.ret = 0; pid_control_V1_B.ret < 3; pid_control_V1_B.ret
       ++) {
    /* Start for MATLABSystem: '<Root>/IMU1' */
    pid_control_V1_B.obj[pid_control_V1_B.ret] = obj->
      pBiasInstFilterStates[pid_control_V1_B.ret];
  }

  for (pid_control_V1_B.i2 = 0; pid_control_V1_B.i2 < 2; pid_control_V1_B.i2++)
  {
    /* Start for MATLABSystem: '<Root>/IMU1' */
    pid_control_V1_B.obj_d[pid_control_V1_B.i2] = obj->
      pBiasInstFilterDen[pid_control_V1_B.i2];
  }

  /* Start for MATLABSystem: '<Root>/IMU1' */
  pid_control_V1_filter(obj->pBiasInstFilterNum, pid_control_V1_B.obj_d,
                        pid_control_V1_B.accelerationDrift, pid_control_V1_B.obj,
                        pid_control_V1_B.varargin_3, obj->pBiasInstFilterStates);
  tmp = _mm_add_pd(_mm_mul_pd(_mm_loadu_pd(&obj->pStdDevRandWalk[0]),
    _mm_loadu_pd(&varargin_4[6])), _mm_loadu_pd(&obj->pRandWalkFilterStates[0]));
  _mm_storeu_pd(&pid_control_V1_B.dv3[0], tmp);

  /* Start for MATLABSystem: '<Root>/IMU1' */
  pid_control_V1_B.x_idx_5 = obj->pStdDevRandWalk[2] * varargin_4[8] +
    obj->pRandWalkFilterStates[2];
  pid_control_V1_B.b_x = (obj->Temperature - 25.0) * 0.01;
  obj->pRandWalkFilterStates[0] = pid_control_V1_B.dv3[0];
  varargout_1[0] = ((((obj->pStdDevWhiteNoise[0] * varargin_4[3] +
                       pid_control_V1_B.varargin_3[0]) + pid_control_V1_B.dv3[0])
                     + ((obj->Temperature - 25.0) * obj->TemperatureBias[0] +
                        obj->pAcceleration[0] * obj->AccelerationBias[0])) +
                    varargout_1[0]) * (pid_control_V1_B.b_x *
    obj->TemperatureScaleFactor[0] + 1.0);
  obj->pRandWalkFilterStates[1] = pid_control_V1_B.dv3[1];
  varargout_1[1] = ((((obj->pStdDevWhiteNoise[1] * varargin_4[4] +
                       pid_control_V1_B.varargin_3[1]) + pid_control_V1_B.dv3[1])
                     + ((obj->Temperature - 25.0) * obj->TemperatureBias[1] +
                        obj->pAcceleration[1] * obj->AccelerationBias[1])) +
                    varargout_1[1]) * (pid_control_V1_B.b_x *
    obj->TemperatureScaleFactor[1] + 1.0);
  obj->pRandWalkFilterStates[2] = pid_control_V1_B.x_idx_5;
  varargout_1[2] = ((((obj->pStdDevWhiteNoise[2] * varargin_4[5] +
                       pid_control_V1_B.varargin_3[2]) +
                      pid_control_V1_B.x_idx_5) + ((obj->Temperature - 25.0) *
    obj->TemperatureBias[2] + obj->pAcceleration[2] * obj->AccelerationBias[2]))
                    + varargout_1[2]) * (pid_control_V1_B.b_x *
    obj->TemperatureScaleFactor[2] + 1.0);
  if (!rtIsInf(obj->MeasurementRange)) {
    pid_control_V1_B.b_x = fabs(varargout_1[0]);
    pid_control_V1_B.accelerationDrift[0] = pid_control_V1_B.b_x;
    pid_control_V1_B.x_idx_5 = fabs(varargout_1[1]);
    pid_control_V1_B.accelerationDrift[1] = pid_control_V1_B.x_idx_5;
    pid_control_V1_B.varargin_3_b = fabs(varargout_1[2]);
    pid_control_V1_B.accelerationDrift[2] = pid_control_V1_B.varargin_3_b;
    pid_control_V1_B.partialTrueCount = 0;
    for (pid_control_V1_B.ret = 0; pid_control_V1_B.ret < 3;
         pid_control_V1_B.ret++) {
      if (pid_control_V1_B.accelerationDrift[pid_control_V1_B.ret] >
          obj->MeasurementRange) {
        pid_control_V1_B.b_data_n[pid_control_V1_B.partialTrueCount] =
          static_cast<int8_T>(pid_control_V1_B.ret);
        pid_control_V1_B.partialTrueCount++;
      }
    }

    pid_control_V1_B.accelerationDrift[0] = pid_control_V1_B.b_x;
    pid_control_V1_B.accelerationDrift[1] = pid_control_V1_B.x_idx_5;
    pid_control_V1_B.accelerationDrift[2] = pid_control_V1_B.varargin_3_b;
    pid_control_V1_B.trueCount_j = 0;
    for (pid_control_V1_B.ret = 0; pid_control_V1_B.ret < 3;
         pid_control_V1_B.ret++) {
      if (pid_control_V1_B.accelerationDrift[pid_control_V1_B.ret] >
          obj->MeasurementRange) {
        pid_control_V1_B.trueCount_j++;
      }
    }

    pid_control_V1_B.partialTrueCount = 0;
    for (pid_control_V1_B.ret = 0; pid_control_V1_B.ret < 3;
         pid_control_V1_B.ret++) {
      if (pid_control_V1_B.accelerationDrift[pid_control_V1_B.ret] >
          obj->MeasurementRange) {
        pid_control_V1_B.tmp_data_m[pid_control_V1_B.partialTrueCount] =
          static_cast<int8_T>(pid_control_V1_B.ret);
        pid_control_V1_B.partialTrueCount++;
      }
    }

    pid_control_V1_B.partialTrueCount = pid_control_V1_B.trueCount_j;
    for (pid_control_V1_B.ret = 0; pid_control_V1_B.ret <
         pid_control_V1_B.trueCount_j; pid_control_V1_B.ret++) {
      pid_control_V1_B.varargin_3[pid_control_V1_B.ret] =
        varargout_1[pid_control_V1_B.tmp_data_m[pid_control_V1_B.ret]];
    }

    pid_control_V1_B.trueCount_j = 0;
    for (pid_control_V1_B.ret = 0; pid_control_V1_B.ret < 3;
         pid_control_V1_B.ret++) {
      if (pid_control_V1_B.accelerationDrift[pid_control_V1_B.ret] >
          obj->MeasurementRange) {
        pid_control_V1_B.trueCount_j++;
      }
    }

    for (pid_control_V1_B.ret = 0; pid_control_V1_B.ret <
         pid_control_V1_B.trueCount_j; pid_control_V1_B.ret++) {
      pid_control_V1_B.b_x = pid_control_V1_B.varargin_3[pid_control_V1_B.ret];
      if (rtIsNaN(pid_control_V1_B.b_x)) {
        pid_control_V1_B.varargin_3[pid_control_V1_B.ret] = (rtNaN);
      } else if (pid_control_V1_B.b_x < 0.0) {
        pid_control_V1_B.varargin_3[pid_control_V1_B.ret] = -1.0;
      } else {
        pid_control_V1_B.varargin_3[pid_control_V1_B.ret] =
          (pid_control_V1_B.b_x > 0.0);
      }
    }

    for (pid_control_V1_B.ret = 0; pid_control_V1_B.ret <
         pid_control_V1_B.partialTrueCount; pid_control_V1_B.ret++) {
      varargout_1[pid_control_V1_B.b_data_n[pid_control_V1_B.ret]] =
        pid_control_V1_B.varargin_3[pid_control_V1_B.ret] *
        obj->MeasurementRange;
    }
  }

  if (obj->Resolution != 0.0) {
    _mm_storeu_pd(&varargout_1[0], _mm_mul_pd(_mm_set_pd(rt_roundd_snf
      (varargout_1[1] / obj->Resolution), rt_roundd_snf(varargout_1[0] /
      obj->Resolution)), _mm_set1_pd(obj->Resolution)));
    varargout_1[2] = rt_roundd_snf(varargout_1[2] / obj->Resolution) *
      obj->Resolution;
  }
}

void pid_control_V1::pid_control_V1_SystemCore_step
  (fusion_simulink_imuSensor_pid_T *obj, const real_T varargin_1[3], const
   real_T varargin_2[3], const real_T varargin_3[4], real_T varargout_1[3],
   real_T varargout_2[3], real_T varargout_3[3])
{
  d_fusion_internal_Acceleromet_T *obj_0;
  e_fusion_internal_GyroscopeSi_T *obj_1;
  e_fusion_internal_Magnetomete_T *obj_2;
  static const char_T b[12] = { 'd', 'o', 'u', 'b', 'l', 'e', '-', 's', 'i', 'd',
    'e', 'd' };

  __m128d tmp;
  static const real_T tmp_0[257] = { 0.0, 0.215241895984875, 0.286174591792068,
    0.335737519214422, 0.375121332878378, 0.408389134611989, 0.43751840220787,
    0.46363433679088, 0.487443966139235, 0.50942332960209, 0.529909720661557,
    0.549151702327164, 0.567338257053817, 0.584616766106378, 0.601104617755991,
    0.61689699000775, 0.63207223638606, 0.646695714894993, 0.660822574244419,
    0.674499822837293, 0.687767892795788, 0.700661841106814, 0.713212285190975,
    0.725446140909999, 0.737387211434295, 0.749056662017815, 0.760473406430107,
    0.771654424224568, 0.782615023307232, 0.793369058840623, 0.80392911698997,
    0.814306670135215, 0.824512208752291, 0.834555354086381, 0.844444954909153,
    0.854189171008163, 0.863795545553308, 0.87327106808886, 0.882622229585165,
    0.891855070732941, 0.900975224461221, 0.909987953496718, 0.91889818364959,
    0.927710533401999, 0.936429340286575, 0.945058684468165, 0.953602409881086,
    0.96206414322304, 0.970447311064224, 0.978755155294224, 0.986990747099062,
    0.99515699963509, 1.00325667954467, 1.01129241744, 1.01926671746548,
    1.02718196603564, 1.03504043983344, 1.04284431314415, 1.05059566459093,
    1.05829648333067, 1.06594867476212, 1.07355406579244, 1.0811144097034,
    1.08863139065398, 1.09610662785202, 1.10354167942464, 1.11093804601357,
    1.11829717411934, 1.12562045921553, 1.13290924865253, 1.14016484436815,
    1.14738850542085, 1.15458145035993, 1.16174485944561, 1.16887987673083,
    1.17598761201545, 1.18306914268269, 1.19012551542669, 1.19715774787944,
    1.20416683014438, 1.2111537262437, 1.21811937548548, 1.22506469375653,
    1.23199057474614, 1.23889789110569, 1.24578749554863, 1.2526602218949,
    1.25951688606371, 1.26635828701823, 1.27318520766536, 1.27999841571382,
    1.28679866449324, 1.29358669373695, 1.30036323033084, 1.30712898903073,
    1.31388467315022, 1.32063097522106, 1.32736857762793, 1.33409815321936,
    1.3408203658964, 1.34753587118059, 1.35424531676263, 1.36094934303328,
    1.36764858359748, 1.37434366577317, 1.38103521107586, 1.38772383568998,
    1.39441015092814, 1.40109476367925, 1.4077782768464, 1.41446128977547,
    1.42114439867531, 1.42782819703026, 1.43451327600589, 1.44120022484872,
    1.44788963128058, 1.45458208188841, 1.46127816251028, 1.46797845861808,
    1.47468355569786, 1.48139403962819, 1.48811049705745, 1.49483351578049,
    1.50156368511546, 1.50830159628131, 1.51504784277671, 1.521803020761,
    1.52856772943771, 1.53534257144151, 1.542128153229, 1.54892508547417,
    1.55573398346918, 1.56255546753104, 1.56939016341512, 1.57623870273591,
    1.58310172339603, 1.58997987002419, 1.59687379442279, 1.60378415602609,
    1.61071162236983, 1.61765686957301, 1.62462058283303, 1.63160345693487,
    1.63860619677555, 1.64562951790478, 1.65267414708306, 1.65974082285818,
    1.66683029616166, 1.67394333092612, 1.68108070472517, 1.68824320943719,
    1.69543165193456, 1.70264685479992, 1.7098896570713, 1.71716091501782,
    1.72446150294804, 1.73179231405296, 1.73915426128591, 1.74654827828172,
    1.75397532031767, 1.76143636531891, 1.76893241491127, 1.77646449552452,
    1.78403365954944, 1.79164098655216, 1.79928758454972, 1.80697459135082,
    1.81470317596628, 1.82247454009388, 1.83028991968276, 1.83815058658281,
    1.84605785028518, 1.8540130597602, 1.86201760539967, 1.87007292107127,
    1.878180486293, 1.88634182853678, 1.8945585256707, 1.90283220855043,
    1.91116456377125, 1.91955733659319, 1.92801233405266, 1.93653142827569,
    1.94511656000868, 1.95376974238465, 1.96249306494436, 1.97128869793366,
    1.98015889690048, 1.98910600761744, 1.99813247135842, 2.00724083056053,
    2.0164337349062, 2.02571394786385, 2.03508435372962, 2.04454796521753,
    2.05410793165065, 2.06376754781173, 2.07353026351874, 2.0833996939983,
    2.09337963113879, 2.10347405571488, 2.11368715068665, 2.12402331568952,
    2.13448718284602, 2.14508363404789, 2.15581781987674, 2.16669518035431,
    2.17772146774029, 2.18890277162636, 2.20024554661128, 2.21175664288416,
    2.22344334009251, 2.23531338492992, 2.24737503294739, 2.25963709517379,
    2.27210899022838, 2.28480080272449, 2.29772334890286, 2.31088825060137,
    2.32430801887113, 2.33799614879653, 2.35196722737914, 2.36623705671729,
    2.38082279517208, 2.39574311978193, 2.41101841390112, 2.42667098493715,
    2.44272531820036, 2.4592083743347, 2.47614993967052, 2.49358304127105,
    2.51154444162669, 2.53007523215985, 2.54922155032478, 2.56903545268184,
    2.58957598670829, 2.61091051848882, 2.63311639363158, 2.65628303757674,
    2.68051464328574, 2.70593365612306, 2.73268535904401, 2.76094400527999,
    2.79092117400193, 2.82287739682644, 2.85713873087322, 2.89412105361341,
    2.93436686720889, 2.97860327988184, 3.02783779176959, 3.08352613200214,
    3.147889289518, 3.2245750520478, 3.32024473383983, 3.44927829856143,
    3.65415288536101, 3.91075795952492 };

  static const real_T tmp_1[257] = { 1.0, 0.977101701267673, 0.959879091800108,
    0.9451989534423, 0.932060075959231, 0.919991505039348, 0.908726440052131,
    0.898095921898344, 0.887984660755834, 0.878309655808918, 0.869008688036857,
    0.860033621196332, 0.851346258458678, 0.842915653112205, 0.834716292986884,
    0.826726833946222, 0.818929191603703, 0.811307874312656, 0.803849483170964,
    0.796542330422959, 0.789376143566025, 0.782341832654803, 0.775431304981187,
    0.768637315798486, 0.761953346836795, 0.755373506507096, 0.748892447219157,
    0.742505296340151, 0.736207598126863, 0.729995264561476, 0.72386453346863,
    0.717811932630722, 0.711834248878248, 0.705928501332754, 0.700091918136512,
    0.694321916126117, 0.688616083004672, 0.682972161644995, 0.677388036218774,
    0.671861719897082, 0.66639134390875, 0.660975147776663, 0.655611470579697,
    0.650298743110817, 0.645035480820822, 0.639820277453057, 0.634651799287624,
    0.629528779924837, 0.624450015547027, 0.619414360605834, 0.614420723888914,
    0.609468064925773, 0.604555390697468, 0.599681752619125, 0.594846243767987,
    0.590047996332826, 0.585286179263371, 0.580559996100791, 0.575868682972354,
    0.571211506735253, 0.566587763256165, 0.561996775814525, 0.557437893618766,
    0.552910490425833, 0.548413963255266, 0.543947731190026, 0.539511234256952,
    0.535103932380458, 0.530725304403662, 0.526374847171684, 0.522052074672322,
    0.517756517229756, 0.513487720747327, 0.509245245995748, 0.505028667943468,
    0.500837575126149, 0.49667156905249, 0.492530263643869, 0.488413284705458,
    0.484320269426683, 0.480250865909047, 0.476204732719506, 0.47218153846773,
    0.468180961405694, 0.464202689048174, 0.460246417812843, 0.456311852678716,
    0.452398706861849, 0.448506701507203, 0.444635565395739, 0.440785034665804,
    0.436954852547985, 0.433144769112652, 0.429354541029442, 0.425583931338022,
    0.421832709229496, 0.418100649837848, 0.414387534040891, 0.410693148270188,
    0.407017284329473, 0.403359739221114, 0.399720314980197, 0.396098818515832,
    0.392495061459315, 0.388908860018789, 0.385340034840077, 0.381788410873393,
    0.378253817245619, 0.374736087137891, 0.371235057668239, 0.367750569779032,
    0.364282468129004, 0.360830600989648, 0.357394820145781, 0.353974980800077,
    0.350570941481406, 0.347182563956794, 0.343809713146851, 0.340452257044522,
    0.337110066637006, 0.333783015830718, 0.330470981379163, 0.327173842813601,
    0.323891482376391, 0.320623784956905, 0.317370638029914, 0.314131931596337,
    0.310907558126286, 0.307697412504292, 0.30450139197665, 0.301319396100803,
    0.298151326696685, 0.294997087799962, 0.291856585617095, 0.288729728482183,
    0.285616426815502, 0.282516593083708, 0.279430141761638, 0.276356989295668,
    0.273297054068577, 0.270250256365875, 0.267216518343561, 0.264195763997261,
    0.261187919132721, 0.258192911337619, 0.255210669954662, 0.252241126055942,
    0.249284212418529, 0.246339863501264, 0.24340801542275, 0.240488605940501,
    0.237581574431238, 0.23468686187233, 0.231804410824339, 0.228934165414681,
    0.226076071322381, 0.223230075763918, 0.220396127480152, 0.217574176724331,
    0.214764175251174, 0.211966076307031, 0.209179834621125, 0.206405406397881,
    0.203642749310335, 0.200891822494657, 0.198152586545776, 0.195425003514135,
    0.192709036903589, 0.190004651670465, 0.187311814223801, 0.1846304924268,
    0.181960655599523, 0.179302274522848, 0.176655321443735, 0.174019770081839,
    0.171395595637506, 0.168782774801212, 0.166181285764482, 0.163591108232366,
    0.161012223437511, 0.158444614155925, 0.15588826472448, 0.153343161060263,
    0.150809290681846, 0.148286642732575, 0.145775208005994, 0.143274978973514,
    0.140785949814445, 0.138308116448551, 0.135841476571254, 0.133386029691669,
    0.130941777173644, 0.12850872228, 0.126086870220186, 0.123676228201597,
    0.12127680548479, 0.11888861344291, 0.116511665625611, 0.114145977827839,
    0.111791568163838, 0.109448457146812, 0.107116667774684, 0.104796225622487,
    0.102487158941935, 0.10018949876881, 0.0979032790388625, 0.095628536713009,
    0.093365311912691, 0.0911136480663738, 0.0888735920682759,
    0.0866451944505581, 0.0844285095703535, 0.082223595813203,
    0.0800305158146631, 0.0778493367020961, 0.0756801303589272,
    0.0735229737139814, 0.0713779490588905, 0.0692451443970068,
    0.0671246538277886, 0.065016577971243, 0.0629210244377582, 0.06083810834954,
    0.0587679529209339, 0.0567106901062031, 0.0546664613248891,
    0.0526354182767924, 0.0506177238609479, 0.0486135532158687,
    0.0466230949019305, 0.0446465522512946, 0.0426841449164746,
    0.0407361106559411, 0.0388027074045262, 0.0368842156885674,
    0.0349809414617162, 0.0330932194585786, 0.0312214171919203,
    0.0293659397581334, 0.0275272356696031, 0.0257058040085489,
    0.0239022033057959, 0.0221170627073089, 0.0203510962300445,
    0.0186051212757247, 0.0168800831525432, 0.0151770883079353,
    0.0134974506017399, 0.0118427578579079, 0.0102149714397015,
    0.00861658276939875, 0.00705087547137324, 0.00552240329925101,
    0.00403797259336304, 0.00260907274610216, 0.0012602859304986,
    0.000477467764609386 };

  static const char_T b_0[12] = { 'd', 'o', 'u', 'b', 'l', 'e', '-', 's', 'i',
    'd', 'e', 'd' };

  static const int32_T tmp_2[2] = { 1, 11 };

  static const int32_T tmp_3[2] = { 1, 12 };

  int32_T exitg1;
  boolean_T guard1;

  /* Start for MATLABSystem: '<Root>/IMU1' */
  if (obj->TunablePropsChanged) {
    obj->TunablePropsChanged = false;
    pid_control_V1_B.ab2 = obj->AccelParamsMeasurementRange;
    pid_control_V1_B.aasq = obj->AccelParamsResolution;
    pid_control_V1_B.ap_ConstantBias[0] = obj->AccelParamsConstantBias[0];
    pid_control_V1_B.ap_ConstantBias[1] = obj->AccelParamsConstantBias[1];
    pid_control_V1_B.ap_ConstantBias[2] = obj->AccelParamsConstantBias[2];
    for (pid_control_V1_B.i_a = 0; pid_control_V1_B.i_a < 9;
         pid_control_V1_B.i_a++) {
      pid_control_V1_B.ap_AxesMisalignment[pid_control_V1_B.i_a] =
        obj->AccelParamsAxesMisalignment[pid_control_V1_B.i_a];
    }

    pid_control_V1_B.B[0] = obj->AccelParamsNoiseDensity[0];
    pid_control_V1_B.B[1] = obj->AccelParamsNoiseDensity[1];
    pid_control_V1_B.B[2] = obj->AccelParamsNoiseDensity[2];
    pid_control_V1_B.ap_BiasInstability[0] = obj->AccelParamsBiasInstability[0];
    pid_control_V1_B.ap_BiasInstability[1] = obj->AccelParamsBiasInstability[1];
    pid_control_V1_B.ap_BiasInstability[2] = obj->AccelParamsBiasInstability[2];
    pid_control_V1_B.whiteNoiseDrift[0] = obj->AccelParamsRandomWalk[0];
    pid_control_V1_B.whiteNoiseDrift[1] = obj->AccelParamsRandomWalk[1];
    pid_control_V1_B.whiteNoiseDrift[2] = obj->AccelParamsRandomWalk[2];
    pid_control_V1_B.ap_TemperatureBias[0] = obj->AccelParamsTemperatureBias[0];
    pid_control_V1_B.ap_TemperatureBias[1] = obj->AccelParamsTemperatureBias[1];
    pid_control_V1_B.ap_TemperatureBias[2] = obj->AccelParamsTemperatureBias[2];
    pid_control_V1_B.ap_TemperatureScaleFactor[0] =
      obj->AccelParamsTemperatureScaleFactor[0];
    pid_control_V1_B.ap_TemperatureScaleFactor[1] =
      obj->AccelParamsTemperatureScaleFactor[1];
    pid_control_V1_B.ap_TemperatureScaleFactor[2] =
      obj->AccelParamsTemperatureScaleFactor[2];
    pid_control_V1_B.ac2 = obj->AccelParamsBiasInstabilityNumerator;
    pid_control_V1_B.coeffs_Denominator[0] =
      obj->AccelParamsBiasInstabilityDenominator[0];
    pid_control_V1_B.coeffs_Denominator[1] =
      obj->AccelParamsBiasInstabilityDenominator[1];
    pid_control_V1_B.n = obj->GyroParamsMeasurementRange;
    pid_control_V1_B.ad2 = obj->GyroParamsResolution;
    pid_control_V1_B.gp_ConstantBias[0] = obj->GyroParamsConstantBias[0];
    pid_control_V1_B.gp_ConstantBias[1] = obj->GyroParamsConstantBias[1];
    pid_control_V1_B.gp_ConstantBias[2] = obj->GyroParamsConstantBias[2];
    for (pid_control_V1_B.i_a = 0; pid_control_V1_B.i_a < 9;
         pid_control_V1_B.i_a++) {
      pid_control_V1_B.gp_AxesMisalignment[pid_control_V1_B.i_a] =
        obj->GyroParamsAxesMisalignment[pid_control_V1_B.i_a];
    }

    pid_control_V1_B.gp_NoiseDensity[0] = obj->GyroParamsNoiseDensity[0];
    pid_control_V1_B.gp_NoiseDensity[1] = obj->GyroParamsNoiseDensity[1];
    pid_control_V1_B.gp_NoiseDensity[2] = obj->GyroParamsNoiseDensity[2];
    pid_control_V1_B.gp_BiasInstability[0] = obj->GyroParamsBiasInstability[0];
    pid_control_V1_B.gp_BiasInstability[1] = obj->GyroParamsBiasInstability[1];
    pid_control_V1_B.gp_BiasInstability[2] = obj->GyroParamsBiasInstability[2];
    pid_control_V1_B.gp_RandomWalk[0] = obj->GyroParamsRandomWalk[0];
    pid_control_V1_B.gp_RandomWalk[1] = obj->GyroParamsRandomWalk[1];
    pid_control_V1_B.gp_RandomWalk[2] = obj->GyroParamsRandomWalk[2];
    pid_control_V1_B.gp_TemperatureBias[0] = obj->GyroParamsTemperatureBias[0];
    pid_control_V1_B.gp_TemperatureBias[1] = obj->GyroParamsTemperatureBias[1];
    pid_control_V1_B.gp_TemperatureBias[2] = obj->GyroParamsTemperatureBias[2];
    pid_control_V1_B.gp_TemperatureScaleFactor[0] =
      obj->GyroParamsTemperatureScaleFactor[0];
    pid_control_V1_B.gp_TemperatureScaleFactor[1] =
      obj->GyroParamsTemperatureScaleFactor[1];
    pid_control_V1_B.gp_TemperatureScaleFactor[2] =
      obj->GyroParamsTemperatureScaleFactor[2];
    pid_control_V1_B.gp_AccelerationBias[0] = obj->GyroParamsAccelerationBias[0];
    pid_control_V1_B.gp_AccelerationBias[1] = obj->GyroParamsAccelerationBias[1];
    pid_control_V1_B.gp_AccelerationBias[2] = obj->GyroParamsAccelerationBias[2];
    pid_control_V1_B.bc2 = obj->GyroParamsBiasInstabilityNumerator;
    pid_control_V1_B.b_Denominator[0] =
      obj->GyroParamsBiasInstabilityDenominator[0];
    pid_control_V1_B.b_Denominator[1] =
      obj->GyroParamsBiasInstabilityDenominator[1];
    pid_control_V1_B.cd2 = obj->MagParamsMeasurementRange;
    pid_control_V1_B.val_h = obj->MagParamsResolution;
    pid_control_V1_B.mp_ConstantBias[0] = obj->MagParamsConstantBias[0];
    pid_control_V1_B.mp_ConstantBias[1] = obj->MagParamsConstantBias[1];
    pid_control_V1_B.mp_ConstantBias[2] = obj->MagParamsConstantBias[2];
    for (pid_control_V1_B.i_a = 0; pid_control_V1_B.i_a < 9;
         pid_control_V1_B.i_a++) {
      pid_control_V1_B.rmat[pid_control_V1_B.i_a] =
        obj->MagParamsAxesMisalignment[pid_control_V1_B.i_a];
    }

    pid_control_V1_B.mp_NoiseDensity[0] = obj->MagParamsNoiseDensity[0];
    pid_control_V1_B.mp_NoiseDensity[1] = obj->MagParamsNoiseDensity[1];
    pid_control_V1_B.mp_NoiseDensity[2] = obj->MagParamsNoiseDensity[2];
    pid_control_V1_B.mp_BiasInstability[0] = obj->MagParamsBiasInstability[0];
    pid_control_V1_B.mp_BiasInstability[1] = obj->MagParamsBiasInstability[1];
    pid_control_V1_B.mp_BiasInstability[2] = obj->MagParamsBiasInstability[2];
    pid_control_V1_B.mp_RandomWalk[0] = obj->MagParamsRandomWalk[0];
    pid_control_V1_B.mp_RandomWalk[1] = obj->MagParamsRandomWalk[1];
    pid_control_V1_B.mp_RandomWalk[2] = obj->MagParamsRandomWalk[2];
    pid_control_V1_B.mp_TemperatureBias[0] = obj->MagParamsTemperatureBias[0];
    pid_control_V1_B.mp_TemperatureBias[1] = obj->MagParamsTemperatureBias[1];
    pid_control_V1_B.mp_TemperatureBias[2] = obj->MagParamsTemperatureBias[2];
    pid_control_V1_B.val[0] = obj->MagParamsTemperatureScaleFactor[0];
    pid_control_V1_B.val[1] = obj->MagParamsTemperatureScaleFactor[1];
    pid_control_V1_B.val[2] = obj->MagParamsTemperatureScaleFactor[2];
    pid_control_V1_B.bd2 = obj->MagParamsBiasInstabilityNumerator;
    pid_control_V1_B.b_Denominator_d[0] =
      obj->MagParamsBiasInstabilityDenominator[0];
    pid_control_V1_B.b_Denominator_d[1] =
      obj->MagParamsBiasInstabilityDenominator[1];
    pid_control_V1_B.flag_m = obj->tunablePropertyChanged[37];
    if (pid_control_V1_B.flag_m) {
      obj_0 = obj->pAccel;
      pid_control_V1_B.flag_m = (obj_0->isInitialized == 1);
      if (pid_control_V1_B.flag_m) {
        obj_0->TunablePropsChanged = true;
        obj_0->tunablePropertyChanged[11] = true;
      }

      obj->pAccel->Temperature = obj->Temperature;
      IMUSensorParameters_updateSyste(pid_control_V1_B.ab2,
        pid_control_V1_B.aasq, pid_control_V1_B.ap_ConstantBias,
        pid_control_V1_B.ap_AxesMisalignment, pid_control_V1_B.B,
        pid_control_V1_B.ap_BiasInstability, pid_control_V1_B.whiteNoiseDrift,
        pid_control_V1_B.ac2, pid_control_V1_B.coeffs_Denominator, b_0,
        pid_control_V1_B.ap_TemperatureBias,
        pid_control_V1_B.ap_TemperatureScaleFactor, obj->pAccel);
      obj_1 = obj->pGyro;
      pid_control_V1_B.flag_m = (obj_1->isInitialized == 1);
      if (pid_control_V1_B.flag_m) {
        obj_1->TunablePropsChanged = true;
        obj_1->tunablePropertyChanged[12] = true;
      }

      obj->pGyro->Temperature = obj->Temperature;
      IMUSensorParameters_updateSys_o(pid_control_V1_B.n, pid_control_V1_B.ad2,
        pid_control_V1_B.gp_ConstantBias, pid_control_V1_B.gp_AxesMisalignment,
        pid_control_V1_B.gp_NoiseDensity, pid_control_V1_B.gp_BiasInstability,
        pid_control_V1_B.gp_RandomWalk, pid_control_V1_B.bc2,
        pid_control_V1_B.b_Denominator, b_0, pid_control_V1_B.gp_TemperatureBias,
        pid_control_V1_B.gp_TemperatureScaleFactor,
        pid_control_V1_B.gp_AccelerationBias, obj->pGyro);
      obj_2 = obj->pMag;
      pid_control_V1_B.flag_m = (obj_2->isInitialized == 1);
      if (pid_control_V1_B.flag_m) {
        obj_2->TunablePropsChanged = true;
        obj_2->tunablePropertyChanged[11] = true;
      }

      obj->pMag->Temperature = obj->Temperature;
      IMUSensorParameters_updateSy_on(pid_control_V1_B.cd2,
        pid_control_V1_B.val_h, pid_control_V1_B.mp_ConstantBias,
        pid_control_V1_B.rmat, pid_control_V1_B.mp_NoiseDensity,
        pid_control_V1_B.mp_BiasInstability, pid_control_V1_B.mp_RandomWalk,
        pid_control_V1_B.bd2, pid_control_V1_B.b_Denominator_d, b_0,
        pid_control_V1_B.mp_TemperatureBias, pid_control_V1_B.val, obj->pMag);
    }

    pid_control_V1_B.flag_b[0] = obj->tunablePropertyChanged[3];
    pid_control_V1_B.flag_b[1] = obj->tunablePropertyChanged[4];
    pid_control_V1_B.flag_b[2] = obj->tunablePropertyChanged[5];
    pid_control_V1_B.flag_b[3] = obj->tunablePropertyChanged[6];
    pid_control_V1_B.flag_b[4] = obj->tunablePropertyChanged[7];
    pid_control_V1_B.flag_b[5] = obj->tunablePropertyChanged[8];
    pid_control_V1_B.flag_b[6] = obj->tunablePropertyChanged[9];
    pid_control_V1_B.flag_b[7] = obj->tunablePropertyChanged[10];
    pid_control_V1_B.flag_b[8] = obj->tunablePropertyChanged[11];
    pid_control_V1_B.flag_b[9] = obj->tunablePropertyChanged[12];
    pid_control_V1_B.flag_b[10] = obj->tunablePropertyChanged[13];
    if (pid_control_V1_vectorAny(pid_control_V1_B.flag_b, tmp_2)) {
      IMUSensorParameters_updateSyste(pid_control_V1_B.ab2,
        pid_control_V1_B.aasq, pid_control_V1_B.ap_ConstantBias,
        pid_control_V1_B.ap_AxesMisalignment, pid_control_V1_B.B,
        pid_control_V1_B.ap_BiasInstability, pid_control_V1_B.whiteNoiseDrift,
        pid_control_V1_B.ac2, pid_control_V1_B.coeffs_Denominator, b_0,
        pid_control_V1_B.ap_TemperatureBias,
        pid_control_V1_B.ap_TemperatureScaleFactor, obj->pAccel);
    }

    pid_control_V1_B.flag[0] = obj->tunablePropertyChanged[14];
    pid_control_V1_B.flag[1] = obj->tunablePropertyChanged[15];
    pid_control_V1_B.flag[2] = obj->tunablePropertyChanged[16];
    pid_control_V1_B.flag[3] = obj->tunablePropertyChanged[17];
    pid_control_V1_B.flag[4] = obj->tunablePropertyChanged[18];
    pid_control_V1_B.flag[5] = obj->tunablePropertyChanged[19];
    pid_control_V1_B.flag[6] = obj->tunablePropertyChanged[20];
    pid_control_V1_B.flag[7] = obj->tunablePropertyChanged[21];
    pid_control_V1_B.flag[8] = obj->tunablePropertyChanged[22];
    pid_control_V1_B.flag[9] = obj->tunablePropertyChanged[23];
    pid_control_V1_B.flag[10] = obj->tunablePropertyChanged[24];
    pid_control_V1_B.flag[11] = obj->tunablePropertyChanged[25];
    if (pid_control_V1_vectorAny(pid_control_V1_B.flag, tmp_3)) {
      IMUSensorParameters_updateSys_o(pid_control_V1_B.n, pid_control_V1_B.ad2,
        pid_control_V1_B.gp_ConstantBias, pid_control_V1_B.gp_AxesMisalignment,
        pid_control_V1_B.gp_NoiseDensity, pid_control_V1_B.gp_BiasInstability,
        pid_control_V1_B.gp_RandomWalk, pid_control_V1_B.bc2,
        pid_control_V1_B.b_Denominator, b_0, pid_control_V1_B.gp_TemperatureBias,
        pid_control_V1_B.gp_TemperatureScaleFactor,
        pid_control_V1_B.gp_AccelerationBias, obj->pGyro);
    }

    pid_control_V1_B.flag_b[0] = obj->tunablePropertyChanged[26];
    pid_control_V1_B.flag_b[1] = obj->tunablePropertyChanged[27];
    pid_control_V1_B.flag_b[2] = obj->tunablePropertyChanged[28];
    pid_control_V1_B.flag_b[3] = obj->tunablePropertyChanged[29];
    pid_control_V1_B.flag_b[4] = obj->tunablePropertyChanged[30];
    pid_control_V1_B.flag_b[5] = obj->tunablePropertyChanged[31];
    pid_control_V1_B.flag_b[6] = obj->tunablePropertyChanged[32];
    pid_control_V1_B.flag_b[7] = obj->tunablePropertyChanged[33];
    pid_control_V1_B.flag_b[8] = obj->tunablePropertyChanged[34];
    pid_control_V1_B.flag_b[9] = obj->tunablePropertyChanged[35];
    pid_control_V1_B.flag_b[10] = obj->tunablePropertyChanged[36];
    if (pid_control_V1_vectorAny(pid_control_V1_B.flag_b, tmp_2)) {
      IMUSensorParameters_updateSy_on(pid_control_V1_B.cd2,
        pid_control_V1_B.val_h, pid_control_V1_B.mp_ConstantBias,
        pid_control_V1_B.rmat, pid_control_V1_B.mp_NoiseDensity,
        pid_control_V1_B.mp_BiasInstability, pid_control_V1_B.mp_RandomWalk,
        pid_control_V1_B.bd2, pid_control_V1_B.b_Denominator_d, b_0,
        pid_control_V1_B.mp_TemperatureBias, pid_control_V1_B.val, obj->pMag);
    }

    for (pid_control_V1_B.i_a = 0; pid_control_V1_B.i_a < 38;
         pid_control_V1_B.i_a++) {
      obj->tunablePropertyChanged[pid_control_V1_B.i_a] = false;
    }
  }

  pid_control_V1_B.n = sqrt(((varargin_3[0] * varargin_3[0] + varargin_3[1] *
    varargin_3[1]) + varargin_3[2] * varargin_3[2]) + varargin_3[3] *
    varargin_3[3]);
  tmp = _mm_set1_pd(pid_control_V1_B.n);

  /* Start for MATLABSystem: '<Root>/IMU1' */
  _mm_storeu_pd(&pid_control_V1_B.dv2[0], _mm_div_pd(_mm_loadu_pd(&varargin_3[0]),
    tmp));
  pid_control_V1_B.aasq = pid_control_V1_B.dv2[0];
  pid_control_V1_B.bd2 = pid_control_V1_B.dv2[1];

  /* Start for MATLABSystem: '<Root>/IMU1' */
  _mm_storeu_pd(&pid_control_V1_B.dv2[0], _mm_div_pd(_mm_loadu_pd(&varargin_3[2]),
    tmp));
  pid_control_V1_B.cd2 = pid_control_V1_B.dv2[0];
  pid_control_V1_B.val_h = pid_control_V1_B.dv2[1];

  /* Start for MATLABSystem: '<Root>/IMU1' */
  _mm_storeu_pd(&pid_control_V1_B.dv2[0], _mm_div_pd(_mm_loadu_pd(&varargin_3[1]),
    tmp));
  pid_control_V1_B.n = varargin_3[3] / pid_control_V1_B.n;
  pid_control_V1_B.ab2 = pid_control_V1_B.aasq * pid_control_V1_B.bd2 * 2.0;
  pid_control_V1_B.ac2 = pid_control_V1_B.aasq * pid_control_V1_B.cd2 * 2.0;
  pid_control_V1_B.ad2 = pid_control_V1_B.aasq * pid_control_V1_B.val_h * 2.0;
  pid_control_V1_B.bc2 = pid_control_V1_B.bd2 * pid_control_V1_B.cd2 * 2.0;
  pid_control_V1_B.bd2 = pid_control_V1_B.bd2 * pid_control_V1_B.val_h * 2.0;
  pid_control_V1_B.cd2 = pid_control_V1_B.cd2 * pid_control_V1_B.val_h * 2.0;
  pid_control_V1_B.aasq = pid_control_V1_B.aasq * pid_control_V1_B.aasq * 2.0 -
    1.0;
  pid_control_V1_B.rmat[0] = pid_control_V1_B.dv2[0] * pid_control_V1_B.dv2[0] *
    2.0 + pid_control_V1_B.aasq;
  pid_control_V1_B.rmat[3] = pid_control_V1_B.bc2 + pid_control_V1_B.ad2;
  pid_control_V1_B.rmat[6] = pid_control_V1_B.bd2 - pid_control_V1_B.ac2;
  pid_control_V1_B.rmat[1] = pid_control_V1_B.bc2 - pid_control_V1_B.ad2;

  /* Start for MATLABSystem: '<Root>/IMU1' */
  pid_control_V1_B.rmat[4] = pid_control_V1_B.dv2[1] * pid_control_V1_B.dv2[1] *
    2.0 + pid_control_V1_B.aasq;
  pid_control_V1_B.rmat[7] = pid_control_V1_B.cd2 + pid_control_V1_B.ab2;
  pid_control_V1_B.rmat[2] = pid_control_V1_B.bd2 + pid_control_V1_B.ac2;
  pid_control_V1_B.rmat[5] = pid_control_V1_B.cd2 - pid_control_V1_B.ab2;

  /* Start for MATLABSystem: '<Root>/IMU1' */
  pid_control_V1_B.rmat[8] = pid_control_V1_B.n * pid_control_V1_B.n * 2.0 +
    pid_control_V1_B.aasq;
  for (pid_control_V1_B.i_a = 0; pid_control_V1_B.i_a < 625;
       pid_control_V1_B.i_a++) {
    /* Start for MATLABSystem: '<Root>/IMU1' */
    pid_control_V1_B.state[pid_control_V1_B.i_a] = obj->
      pStreamState[pid_control_V1_B.i_a];
  }

  for (pid_control_V1_B.i_a = 0; pid_control_V1_B.i_a < 27; pid_control_V1_B.i_a
       ++) {
    do {
      exitg1 = 0;
      pid_contr_genrand_uint32_vector(pid_control_V1_B.state,
        pid_control_V1_B.u32);
      pid_control_V1_B.i_i = (pid_control_V1_B.u32[1] >> 24U) + 1U;
      pid_control_V1_B.aasq = ((static_cast<real_T>(pid_control_V1_B.u32[0] >>
        3U) * 1.6777216E+7 + static_cast<real_T>(pid_control_V1_B.u32[1] &
        16777215U)) * 2.2204460492503131E-16 - 1.0) * tmp_0[static_cast<int32_T>
        (pid_control_V1_B.i_i)];
      if (fabs(pid_control_V1_B.aasq) <= tmp_0[static_cast<int32_T>
          (pid_control_V1_B.i_i) - 1]) {
        exitg1 = 1;
      } else if (pid_control_V1_B.i_i < 256U) {
        pid_control_V1_B.n = pid_control_V1_genrandu(pid_control_V1_B.state);
        pid_control_V1_B.ac2 = tmp_1[static_cast<int32_T>(pid_control_V1_B.i_i)];
        if ((tmp_1[static_cast<int32_T>(pid_control_V1_B.i_i) - 1] -
             pid_control_V1_B.ac2) * pid_control_V1_B.n + pid_control_V1_B.ac2 <
            exp(-0.5 * pid_control_V1_B.aasq * pid_control_V1_B.aasq)) {
          exitg1 = 1;
        }
      } else {
        do {
          pid_control_V1_B.n = pid_control_V1_genrandu(pid_control_V1_B.state);
          pid_control_V1_B.ab2 = log(pid_control_V1_B.n) * 0.273661237329758;
          pid_control_V1_B.n = pid_control_V1_genrandu(pid_control_V1_B.state);
        } while (!(-2.0 * log(pid_control_V1_B.n) > pid_control_V1_B.ab2 *
                   pid_control_V1_B.ab2));

        if (pid_control_V1_B.aasq < 0.0) {
          pid_control_V1_B.aasq = pid_control_V1_B.ab2 - 3.65415288536101;
        } else {
          pid_control_V1_B.aasq = 3.65415288536101 - pid_control_V1_B.ab2;
        }

        exitg1 = 1;
      }
    } while (exitg1 == 0);

    /* Start for MATLABSystem: '<Root>/IMU1' */
    pid_control_V1_B.allRandData[pid_control_V1_B.i_a] = pid_control_V1_B.aasq;
  }

  for (pid_control_V1_B.i_a = 0; pid_control_V1_B.i_a < 625;
       pid_control_V1_B.i_a++) {
    /* Start for MATLABSystem: '<Root>/IMU1' */
    obj->pStreamState[pid_control_V1_B.i_a] =
      pid_control_V1_B.state[pid_control_V1_B.i_a];
  }

  /* Start for MATLABSystem: '<Root>/IMU1' */
  pid_control_V_SystemCore_step_o(obj->pAccel, varargin_1, pid_control_V1_B.rmat,
    &pid_control_V1_B.allRandData[0], varargout_1);
  pid_control__SystemCore_step_on(obj->pGyro, varargin_2, varargin_1,
    pid_control_V1_B.rmat, &pid_control_V1_B.allRandData[9], varargout_2);
  pid_control_V1_B.ap_ConstantBias[0] = obj->MagneticField[0];
  pid_control_V1_B.ap_ConstantBias[1] = obj->MagneticField[1];
  pid_control_V1_B.ap_ConstantBias[2] = obj->MagneticField[2];
  obj_2 = obj->pMag;
  if (obj_2->isInitialized != 1) {
    /* Start for MATLABSystem: '<Root>/IMU1' */
    obj_2->isInitialized = 1;
    for (pid_control_V1_B.i_a = 0; pid_control_V1_B.i_a < 9;
         pid_control_V1_B.i_a++) {
      /* Start for MATLABSystem: '<Root>/IMU1' */
      obj_2->pGain[pid_control_V1_B.i_a] = obj_2->
        AxesMisalignment[pid_control_V1_B.i_a] / 100.0;
    }

    for (pid_control_V1_B.i_a = 0; pid_control_V1_B.i_a < 12;
         pid_control_V1_B.i_a++) {
      /* Start for MATLABSystem: '<Root>/IMU1' */
      pid_control_V1_B.obj1_Value[pid_control_V1_B.i_a] = obj_2->
        NoiseType.Value[pid_control_V1_B.i_a];
    }

    /* Start for MATLABSystem: '<Root>/IMU1' */
    pid_control_V1_B.i_a = std::memcmp(&pid_control_V1_B.obj1_Value[0], &b[0],
      12);
    if (pid_control_V1_B.i_a == 0) {
      obj_2->pBandwidth = 50.0;
    } else {
      obj_2->pBandwidth = 100.0;
    }

    pid_control_V1_B.ac2 = obj_2->BiasInstabilityCoefficients.Numerator;
    obj_2->pBiasInstFilterNum = pid_control_V1_B.ac2;
    pid_control_V1_B.aasq = obj_2->BiasInstabilityCoefficients.Denominator[0];
    obj_2->pBiasInstFilterDen[0] = pid_control_V1_B.aasq;
    pid_control_V1_B.aasq = obj_2->BiasInstabilityCoefficients.Denominator[1];
    obj_2->pBiasInstFilterDen[1] = pid_control_V1_B.aasq;
    obj_2->pCorrelationTime = 0.02;
    pid_control_V1_B.ab2 = 2.0 / (100.0 * obj_2->pCorrelationTime);
    pid_control_V1_B.aasq = sqrt(pid_control_V1_B.ab2);
    pid_control_V1_B.ab2 = obj_2->pBandwidth;
    pid_control_V1_B.n = sqrt(pid_control_V1_B.ab2);
    pid_control_V1_B.ab2 = obj_2->pBandwidth;
    pid_control_V1_B.ab2 = sqrt(pid_control_V1_B.ab2);
    obj_2->TunablePropsChanged = false;
    obj_2->pBiasInstFilterStates[0] = 0.0;
    obj_2->pStdDevBiasInst[0] = pid_control_V1_B.aasq * obj_2->BiasInstability[0];
    obj_2->pStdDevWhiteNoise[0] = pid_control_V1_B.n * obj_2->NoiseDensity[0];
    obj_2->pRandWalkFilterStates[0] = 0.0;
    obj_2->pStdDevRandWalk[0] = obj_2->RandomWalk[0] / pid_control_V1_B.ab2;
    obj_2->pBiasInstFilterStates[0] = 0.0;
    obj_2->pRandWalkFilterStates[0] = 0.0;
    obj_2->pBiasInstFilterStates[1] = 0.0;
    obj_2->pStdDevBiasInst[1] = pid_control_V1_B.aasq * obj_2->BiasInstability[1];
    obj_2->pStdDevWhiteNoise[1] = pid_control_V1_B.n * obj_2->NoiseDensity[1];
    obj_2->pRandWalkFilterStates[1] = 0.0;
    obj_2->pStdDevRandWalk[1] = obj_2->RandomWalk[1] / pid_control_V1_B.ab2;
    obj_2->pBiasInstFilterStates[1] = 0.0;
    obj_2->pRandWalkFilterStates[1] = 0.0;
    obj_2->pBiasInstFilterStates[2] = 0.0;
    obj_2->pStdDevBiasInst[2] = pid_control_V1_B.aasq * obj_2->BiasInstability[2];
    obj_2->pStdDevWhiteNoise[2] = pid_control_V1_B.n * obj_2->NoiseDensity[2];
    obj_2->pRandWalkFilterStates[2] = 0.0;
    obj_2->pStdDevRandWalk[2] = obj_2->RandomWalk[2] / pid_control_V1_B.ab2;
    obj_2->pBiasInstFilterStates[2] = 0.0;
    obj_2->pRandWalkFilterStates[2] = 0.0;
  }

  /* Start for MATLABSystem: '<Root>/IMU1' */
  if (obj_2->TunablePropsChanged) {
    obj_2->TunablePropsChanged = false;
    pid_control_V1_B.flag_m = obj_2->tunablePropertyChanged[3];
    if (pid_control_V1_B.flag_m) {
      for (pid_control_V1_B.i_a = 0; pid_control_V1_B.i_a < 9;
           pid_control_V1_B.i_a++) {
        obj_2->pGain[pid_control_V1_B.i_a] = obj_2->
          AxesMisalignment[pid_control_V1_B.i_a] / 100.0;
      }
    }

    pid_control_V1_B.flag_m = obj_2->tunablePropertyChanged[4];
    guard1 = false;
    if (pid_control_V1_B.flag_m) {
      guard1 = true;
    } else {
      pid_control_V1_B.flag_m = obj_2->tunablePropertyChanged[8];
      if (pid_control_V1_B.flag_m) {
        guard1 = true;
      }
    }

    if (guard1) {
      for (pid_control_V1_B.i_a = 0; pid_control_V1_B.i_a < 12;
           pid_control_V1_B.i_a++) {
        pid_control_V1_B.obj1_Value[pid_control_V1_B.i_a] =
          obj_2->NoiseType.Value[pid_control_V1_B.i_a];
      }

      pid_control_V1_B.i_a = std::memcmp(&pid_control_V1_B.obj1_Value[0], &b[0],
        12);
      if (pid_control_V1_B.i_a == 0) {
        obj_2->pBandwidth = 50.0;
      } else {
        obj_2->pBandwidth = 100.0;
      }

      pid_control_V1_B.ab2 = obj_2->pBandwidth;
      pid_control_V1_B.aasq = sqrt(pid_control_V1_B.ab2);
      obj_2->pStdDevWhiteNoise[0] = pid_control_V1_B.aasq * obj_2->NoiseDensity
        [0];
      obj_2->pStdDevWhiteNoise[1] = pid_control_V1_B.aasq * obj_2->NoiseDensity
        [1];
      obj_2->pStdDevWhiteNoise[2] = pid_control_V1_B.aasq * obj_2->NoiseDensity
        [2];
    }

    pid_control_V1_B.flag_m = obj_2->tunablePropertyChanged[5];
    if (pid_control_V1_B.flag_m) {
      pid_control_V1_B.ab2 = 2.0 / (100.0 * obj_2->pCorrelationTime);
      pid_control_V1_B.aasq = sqrt(pid_control_V1_B.ab2);
      obj_2->pStdDevBiasInst[0] = pid_control_V1_B.aasq * obj_2->
        BiasInstability[0];
      obj_2->pStdDevBiasInst[1] = pid_control_V1_B.aasq * obj_2->
        BiasInstability[1];
      obj_2->pStdDevBiasInst[2] = pid_control_V1_B.aasq * obj_2->
        BiasInstability[2];
    }

    pid_control_V1_B.flag_m = obj_2->tunablePropertyChanged[6];
    guard1 = false;
    if (pid_control_V1_B.flag_m) {
      guard1 = true;
    } else {
      pid_control_V1_B.flag_m = obj_2->tunablePropertyChanged[8];
      if (pid_control_V1_B.flag_m) {
        guard1 = true;
      }
    }

    if (guard1) {
      for (pid_control_V1_B.i_a = 0; pid_control_V1_B.i_a < 12;
           pid_control_V1_B.i_a++) {
        pid_control_V1_B.obj1_Value[pid_control_V1_B.i_a] =
          obj_2->NoiseType.Value[pid_control_V1_B.i_a];
      }

      pid_control_V1_B.i_a = std::memcmp(&pid_control_V1_B.obj1_Value[0], &b[0],
        12);
      if (pid_control_V1_B.i_a == 0) {
        obj_2->pBandwidth = 50.0;
      } else {
        obj_2->pBandwidth = 100.0;
      }

      pid_control_V1_B.ab2 = obj_2->pBandwidth;
      pid_control_V1_B.aasq = sqrt(pid_control_V1_B.ab2);
      obj_2->pStdDevRandWalk[0] = obj_2->RandomWalk[0] / pid_control_V1_B.aasq;
      obj_2->pStdDevRandWalk[1] = obj_2->RandomWalk[1] / pid_control_V1_B.aasq;
      obj_2->pStdDevRandWalk[2] = obj_2->RandomWalk[2] / pid_control_V1_B.aasq;
    }

    pid_control_V1_B.flag_m = obj_2->tunablePropertyChanged[7];
    if (pid_control_V1_B.flag_m) {
      pid_control_V1_B.ac2 = obj_2->BiasInstabilityCoefficients.Numerator;
      obj_2->pBiasInstFilterNum = pid_control_V1_B.ac2;
      pid_control_V1_B.aasq = obj_2->BiasInstabilityCoefficients.Denominator[0];
      obj_2->pBiasInstFilterDen[0] = pid_control_V1_B.aasq;
      pid_control_V1_B.aasq = obj_2->BiasInstabilityCoefficients.Denominator[1];
      obj_2->pBiasInstFilterDen[1] = pid_control_V1_B.aasq;
    }

    for (pid_control_V1_B.i_a = 0; pid_control_V1_B.i_a < 12;
         pid_control_V1_B.i_a++) {
      obj_2->tunablePropertyChanged[pid_control_V1_B.i_a] = false;
    }
  }

  for (pid_control_V1_B.i_a = 0; pid_control_V1_B.i_a < 9; pid_control_V1_B.i_a
       ++) {
    /* Start for MATLABSystem: '<Root>/IMU1' */
    pid_control_V1_B.ap_AxesMisalignment[pid_control_V1_B.i_a] = obj_2->
      pGain[pid_control_V1_B.i_a];
  }

  /* Start for MATLABSystem: '<Root>/IMU1' */
  pid_control_V1_B.aasq = 0.0;
  pid_control_V1_B.n = 0.0;
  pid_control_V1_B.ab2 = 0.0;
  for (pid_control_V1_B.i_a = 0; pid_control_V1_B.i_a < 3; pid_control_V1_B.i_a
       ++) {
    tmp = _mm_add_pd(_mm_mul_pd(_mm_loadu_pd(&pid_control_V1_B.rmat[3 *
      pid_control_V1_B.i_a]), _mm_set1_pd
      (pid_control_V1_B.ap_ConstantBias[pid_control_V1_B.i_a])), _mm_set_pd
                     (pid_control_V1_B.n, pid_control_V1_B.aasq));
    _mm_storeu_pd(&pid_control_V1_B.dv2[0], tmp);
    pid_control_V1_B.aasq = pid_control_V1_B.dv2[0];
    pid_control_V1_B.n = pid_control_V1_B.dv2[1];
    pid_control_V1_B.ab2 += pid_control_V1_B.rmat[3 * pid_control_V1_B.i_a + 2] *
      pid_control_V1_B.ap_ConstantBias[pid_control_V1_B.i_a];
  }

  pid_control_V1_B.B[2] = pid_control_V1_B.ab2;
  pid_control_V1_B.B[1] = pid_control_V1_B.n;
  pid_control_V1_B.B[0] = pid_control_V1_B.aasq;
  pid_control_V1_B.ab2 = 0.0;
  pid_control_V1_B.aasq = 0.0;
  pid_control_V1_B.n = 0.0;
  for (pid_control_V1_B.i_a = 0; pid_control_V1_B.i_a < 3; pid_control_V1_B.i_a
       ++) {
    pid_control_V1_B.ac2 = pid_control_V1_B.B[pid_control_V1_B.i_a];

    /* Start for MATLABSystem: '<Root>/IMU1' */
    tmp = _mm_add_pd(_mm_mul_pd(_mm_loadu_pd
      (&pid_control_V1_B.ap_AxesMisalignment[3 * pid_control_V1_B.i_a]),
      _mm_set1_pd(pid_control_V1_B.ac2)), _mm_set_pd(pid_control_V1_B.aasq,
      pid_control_V1_B.ab2));
    _mm_storeu_pd(&pid_control_V1_B.dv2[0], tmp);

    /* Start for MATLABSystem: '<Root>/IMU1' */
    pid_control_V1_B.ab2 = pid_control_V1_B.dv2[0];
    pid_control_V1_B.aasq = pid_control_V1_B.dv2[1];
    pid_control_V1_B.n += pid_control_V1_B.ap_AxesMisalignment[3 *
      pid_control_V1_B.i_a + 2] * pid_control_V1_B.ac2;
    pid_control_V1_B.ap_ConstantBias[pid_control_V1_B.i_a] = obj_2->
      ConstantBias[pid_control_V1_B.i_a];
  }

  /* Start for MATLABSystem: '<Root>/IMU1' */
  pid_control_V1_B.B[0] = pid_control_V1_B.ab2 +
    pid_control_V1_B.ap_ConstantBias[0];
  pid_control_V1_B.ab2 = obj_2->pStdDevBiasInst[0];
  pid_control_V1_B.val[0] = pid_control_V1_B.allRandData[18] *
    pid_control_V1_B.ab2;
  pid_control_V1_B.B[1] = pid_control_V1_B.aasq +
    pid_control_V1_B.ap_ConstantBias[1];
  pid_control_V1_B.ab2 = obj_2->pStdDevBiasInst[1];
  pid_control_V1_B.val[1] = pid_control_V1_B.allRandData[19] *
    pid_control_V1_B.ab2;
  pid_control_V1_B.B[2] = pid_control_V1_B.n + pid_control_V1_B.ap_ConstantBias
    [2];
  pid_control_V1_B.ab2 = obj_2->pStdDevBiasInst[2];
  pid_control_V1_B.val[2] = pid_control_V1_B.allRandData[20] *
    pid_control_V1_B.ab2;
  for (pid_control_V1_B.i1 = 0; pid_control_V1_B.i1 < 2; pid_control_V1_B.i1++)
  {
    /* Start for MATLABSystem: '<Root>/IMU1' */
    pid_control_V1_B.coeffs_Denominator[pid_control_V1_B.i1] =
      obj_2->pBiasInstFilterDen[pid_control_V1_B.i1];
  }

  /* Start for MATLABSystem: '<Root>/IMU1' */
  pid_control_V1_filter(obj_2->pBiasInstFilterNum,
                        pid_control_V1_B.coeffs_Denominator,
                        pid_control_V1_B.val, obj_2->pBiasInstFilterStates,
                        pid_control_V1_B.ap_BiasInstability,
                        pid_control_V1_B.ap_ConstantBias);
  obj_2->pBiasInstFilterStates[0] = pid_control_V1_B.ap_ConstantBias[0];
  pid_control_V1_B.ab2 = obj_2->pStdDevWhiteNoise[0];
  pid_control_V1_B.whiteNoiseDrift[0] = pid_control_V1_B.allRandData[21] *
    pid_control_V1_B.ab2;
  pid_control_V1_B.ab2 = obj_2->pStdDevRandWalk[0];
  pid_control_V1_B.ac2 = obj_2->pRandWalkFilterStates[0];
  pid_control_V1_B.n = pid_control_V1_B.allRandData[24] * pid_control_V1_B.ab2;
  obj_2->pBiasInstFilterStates[1] = pid_control_V1_B.ap_ConstantBias[1];
  pid_control_V1_B.ab2 = obj_2->pStdDevWhiteNoise[1];
  pid_control_V1_B.whiteNoiseDrift[1] = pid_control_V1_B.allRandData[22] *
    pid_control_V1_B.ab2;
  pid_control_V1_B.ab2 = obj_2->pStdDevRandWalk[1];
  pid_control_V1_B.ad2 = obj_2->pRandWalkFilterStates[1];
  pid_control_V1_B.aasq = pid_control_V1_B.allRandData[25] *
    pid_control_V1_B.ab2;
  obj_2->pBiasInstFilterStates[2] = pid_control_V1_B.ap_ConstantBias[2];
  pid_control_V1_B.ab2 = obj_2->pStdDevWhiteNoise[2];
  pid_control_V1_B.whiteNoiseDrift[2] = pid_control_V1_B.allRandData[23] *
    pid_control_V1_B.ab2;
  pid_control_V1_B.ab2 = obj_2->pStdDevRandWalk[2];
  pid_control_V1_B.bc2 = obj_2->pRandWalkFilterStates[2];
  _mm_storeu_pd(&pid_control_V1_B.dv2[0], _mm_add_pd(_mm_set_pd
    (pid_control_V1_B.ad2, pid_control_V1_B.ac2), _mm_set_pd
    (pid_control_V1_B.aasq, pid_control_V1_B.n)));
  pid_control_V1_B.aasq = pid_control_V1_B.dv2[1];

  /* Start for MATLABSystem: '<Root>/IMU1' */
  pid_control_V1_B.cd2 = pid_control_V1_B.allRandData[26] * pid_control_V1_B.ab2
    + pid_control_V1_B.bc2;
  pid_control_V1_B.ac2 = obj_2->Temperature - 25.0;
  pid_control_V1_B.ad2 = (obj_2->Temperature - 25.0) * 0.01;
  obj_2->pRandWalkFilterStates[0] = pid_control_V1_B.dv2[0];
  pid_control_V1_B.ab2 = pid_control_V1_B.ac2 * obj_2->TemperatureBias[0];
  pid_control_V1_B.bc2 = pid_control_V1_B.ad2 * obj_2->TemperatureScaleFactor[0]
    + 1.0;
  _mm_storeu_pd(&pid_control_V1_B.dv2[0], _mm_mul_pd(_mm_add_pd(_mm_add_pd
    (_mm_add_pd(_mm_add_pd(_mm_set1_pd(pid_control_V1_B.whiteNoiseDrift[0]),
    _mm_set1_pd(pid_control_V1_B.ap_BiasInstability[0])), _mm_set1_pd
                (pid_control_V1_B.dv2[0])), _mm_set1_pd(pid_control_V1_B.ab2)),
    _mm_set1_pd(pid_control_V1_B.B[0])), _mm_set1_pd(pid_control_V1_B.bc2)));
  pid_control_V1_B.n = pid_control_V1_B.dv2[0];
  varargout_3[0] = pid_control_V1_B.dv2[1];
  obj_2->pRandWalkFilterStates[1] = pid_control_V1_B.aasq;
  pid_control_V1_B.ab2 = pid_control_V1_B.ac2 * obj_2->TemperatureBias[1];
  pid_control_V1_B.bc2 = pid_control_V1_B.ad2 * obj_2->TemperatureScaleFactor[1]
    + 1.0;
  _mm_storeu_pd(&pid_control_V1_B.dv2[0], _mm_mul_pd(_mm_add_pd(_mm_add_pd
    (_mm_add_pd(_mm_add_pd(_mm_set1_pd(pid_control_V1_B.whiteNoiseDrift[1]),
    _mm_set1_pd(pid_control_V1_B.ap_BiasInstability[1])), _mm_set1_pd
                (pid_control_V1_B.aasq)), _mm_set1_pd(pid_control_V1_B.ab2)),
    _mm_set1_pd(pid_control_V1_B.B[1])), _mm_set1_pd(pid_control_V1_B.bc2)));
  pid_control_V1_B.aasq = pid_control_V1_B.dv2[0];
  varargout_3[1] = pid_control_V1_B.dv2[1];
  obj_2->pRandWalkFilterStates[2] = pid_control_V1_B.cd2;
  pid_control_V1_B.ab2 = pid_control_V1_B.ac2 * obj_2->TemperatureBias[2];
  pid_control_V1_B.bc2 = pid_control_V1_B.ad2 * obj_2->TemperatureScaleFactor[2]
    + 1.0;
  _mm_storeu_pd(&pid_control_V1_B.dv2[0], _mm_mul_pd(_mm_add_pd(_mm_add_pd
    (_mm_add_pd(_mm_add_pd(_mm_set1_pd(pid_control_V1_B.whiteNoiseDrift[2]),
    _mm_set1_pd(pid_control_V1_B.ap_BiasInstability[2])), _mm_set1_pd
                (pid_control_V1_B.cd2)), _mm_set1_pd(pid_control_V1_B.ab2)),
    _mm_set1_pd(pid_control_V1_B.B[2])), _mm_set1_pd(pid_control_V1_B.bc2)));
  varargout_3[2] = pid_control_V1_B.dv2[1];
  pid_control_V1_B.ab2 = obj_2->MeasurementRange;
  if (!rtIsInf(pid_control_V1_B.ab2)) {
    pid_control_V1_B.ab2 = obj_2->MeasurementRange;
    pid_control_V1_B.n = fabs(pid_control_V1_B.n);
    pid_control_V1_B.val[0] = pid_control_V1_B.n;
    pid_control_V1_B.aasq = fabs(pid_control_V1_B.aasq);
    pid_control_V1_B.val[1] = pid_control_V1_B.aasq;
    pid_control_V1_B.ac2 = fabs(pid_control_V1_B.dv2[0]);
    pid_control_V1_B.val[2] = pid_control_V1_B.ac2;
    pid_control_V1_B.trueCount = 0;
    for (pid_control_V1_B.i_a = 0; pid_control_V1_B.i_a < 3;
         pid_control_V1_B.i_a++) {
      if (pid_control_V1_B.val[pid_control_V1_B.i_a] > pid_control_V1_B.ab2) {
        pid_control_V1_B.b_data[pid_control_V1_B.trueCount] = static_cast<int8_T>
          (pid_control_V1_B.i_a);
        pid_control_V1_B.trueCount++;
      }
    }

    pid_control_V1_B.val[0] = pid_control_V1_B.n;
    pid_control_V1_B.val[1] = pid_control_V1_B.aasq;
    pid_control_V1_B.val[2] = pid_control_V1_B.ac2;
    pid_control_V1_B.trueCount = 0;
    for (pid_control_V1_B.i_a = 0; pid_control_V1_B.i_a < 3;
         pid_control_V1_B.i_a++) {
      if (pid_control_V1_B.val[pid_control_V1_B.i_a] > pid_control_V1_B.ab2) {
        pid_control_V1_B.trueCount++;
      }
    }

    pid_control_V1_B.tmp_size_idx_1 = pid_control_V1_B.trueCount;
    pid_control_V1_B.trueCount = 0;
    for (pid_control_V1_B.i_a = 0; pid_control_V1_B.i_a < 3;
         pid_control_V1_B.i_a++) {
      if (pid_control_V1_B.val[pid_control_V1_B.i_a] > pid_control_V1_B.ab2) {
        pid_control_V1_B.tmp_data[pid_control_V1_B.trueCount] =
          static_cast<int8_T>(pid_control_V1_B.i_a);
        pid_control_V1_B.trueCount++;
      }
    }

    for (pid_control_V1_B.i_a = 0; pid_control_V1_B.i_a <
         pid_control_V1_B.tmp_size_idx_1; pid_control_V1_B.i_a++) {
      pid_control_V1_B.aasq =
        varargout_3[pid_control_V1_B.tmp_data[pid_control_V1_B.i_a]];
      pid_control_V1_B.B[pid_control_V1_B.i_a] = pid_control_V1_B.aasq;
      if (rtIsNaN(pid_control_V1_B.aasq)) {
        pid_control_V1_B.B[pid_control_V1_B.i_a] = (rtNaN);
      } else if (pid_control_V1_B.aasq < 0.0) {
        pid_control_V1_B.B[pid_control_V1_B.i_a] = -1.0;
      } else {
        pid_control_V1_B.B[pid_control_V1_B.i_a] = (pid_control_V1_B.aasq > 0.0);
      }
    }

    for (pid_control_V1_B.i_a = 0; pid_control_V1_B.i_a <
         pid_control_V1_B.tmp_size_idx_1; pid_control_V1_B.i_a++) {
      varargout_3[pid_control_V1_B.b_data[pid_control_V1_B.i_a]] =
        pid_control_V1_B.B[pid_control_V1_B.i_a] * pid_control_V1_B.ab2;
    }
  }

  if (obj_2->Resolution != 0.0) {
    pid_control_V1_B.aasq = obj_2->Resolution;
    _mm_storeu_pd(&varargout_3[0], _mm_mul_pd(_mm_set_pd(rt_roundd_snf
      (varargout_3[1] / pid_control_V1_B.aasq), rt_roundd_snf(varargout_3[0] /
      pid_control_V1_B.aasq)), _mm_set1_pd(pid_control_V1_B.aasq)));
    varargout_3[2] = rt_roundd_snf(varargout_3[2] / pid_control_V1_B.aasq) *
      pid_control_V1_B.aasq;
  }
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
  __m128d tmp;
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

  /* RelationalOperator: '<S281>/Compare' incorporates:
   *  Constant: '<S281>/Constant'
   */
  pid_control_V1_B.Compare = (pid_control_V1_X.Integrator_CSTATE[11] >= 0.05);

  /* Outputs for Enabled SubSystem: '<S290>/Hrgw' incorporates:
   *  EnablePort: '<S303>/Enable'
   */
  /* Outputs for Enabled SubSystem: '<S290>/Hqgw' incorporates:
   *  EnablePort: '<S302>/Enable'
   */
  /* Outputs for Enabled SubSystem: '<S290>/Hpgw' incorporates:
   *  EnablePort: '<S301>/Enable'
   */
  /* Outputs for Enabled SubSystem: '<S291>/Hwgw(s)' incorporates:
   *  EnablePort: '<S306>/Enable'
   */
  /* Outputs for Enabled SubSystem: '<S291>/Hvgw(s)' incorporates:
   *  EnablePort: '<S305>/Enable'
   */
  pid_control_V1_B.b = rtmIsMajorTimeStep((&pid_control_V1_M));

  /* End of Outputs for SubSystem: '<S291>/Hvgw(s)' */
  /* End of Outputs for SubSystem: '<S291>/Hwgw(s)' */
  /* End of Outputs for SubSystem: '<S290>/Hpgw' */
  /* End of Outputs for SubSystem: '<S290>/Hqgw' */
  /* End of Outputs for SubSystem: '<S290>/Hrgw' */
  if (pid_control_V1_B.b) {
    /* MATLAB Function: '<S12>/MATLAB Function-reset' incorporates:
     *  Memory: '<S12>/Memory2'
     */
    memcpy(&pid_control_V1_B.IC[0], &pid_control_V1_DW.Memory2_PreviousInput[0],
           12U * sizeof(real_T));
    pid_control_V1_B.IC[2] = 0.0;
    pid_control_V1_B.IC[11] = 0.0;

    /* InitialCondition: '<S12>/IC' */
    if (pid_control_V1_DW.IC_FirstOutputTime) {
      pid_control_V1_DW.IC_FirstOutputTime = false;

      /* InitialCondition: '<S12>/IC' */
      memcpy(&pid_control_V1_B.IC[0], &pid_control_V1_ConstP.pooled15[0], 12U *
             sizeof(real_T));
    }

    /* End of InitialCondition: '<S12>/IC' */
  }

  /* Outputs for Enabled SubSystem: '<S290>/Hrgw' incorporates:
   *  EnablePort: '<S303>/Enable'
   */
  /* Outputs for Enabled SubSystem: '<S290>/Hqgw' incorporates:
   *  EnablePort: '<S302>/Enable'
   */
  /* Outputs for Enabled SubSystem: '<S290>/Hpgw' incorporates:
   *  EnablePort: '<S301>/Enable'
   */
  /* Outputs for Enabled SubSystem: '<S291>/Hwgw(s)' incorporates:
   *  EnablePort: '<S306>/Enable'
   */
  /* Outputs for Enabled SubSystem: '<S291>/Hvgw(s)' incorporates:
   *  EnablePort: '<S305>/Enable'
   */
  /* Outputs for Enabled SubSystem: '<S291>/Hugw(s)' incorporates:
   *  EnablePort: '<S304>/Enable'
   */
  /* If: '<S295>/if Height < Max low altitude  elseif Height > Min isotropic altitude ' incorporates:
   *  If: '<S296>/if Height < Max low altitude  elseif Height > Min isotropic altitude '
   *  Integrator: '<S12>/Integrator'
   *  RateLimiter: '<Root>/Rate Limiter'
   */
  pid_control_V1_B.b1 = rtsiIsModeUpdateTimeStep(&(&pid_control_V1_M)
    ->solverInfo);

  /* End of Outputs for SubSystem: '<S291>/Hugw(s)' */
  /* End of Outputs for SubSystem: '<S291>/Hvgw(s)' */
  /* End of Outputs for SubSystem: '<S291>/Hwgw(s)' */
  /* End of Outputs for SubSystem: '<S290>/Hpgw' */
  /* End of Outputs for SubSystem: '<S290>/Hqgw' */
  /* End of Outputs for SubSystem: '<S290>/Hrgw' */

  /* Integrator: '<S12>/Integrator' incorporates:
   *  InitialCondition: '<S12>/IC'
   */
  if (pid_control_V1_B.b1) {
    pid_control_V1_B.serverAvailableOnTime =
      (((pid_control_V1_PrevZCX.Integrator_Reset_ZCE == POS_ZCSIG) !=
        pid_control_V1_B.Compare) &&
       (pid_control_V1_PrevZCX.Integrator_Reset_ZCE != UNINITIALIZED_ZCSIG));
    pid_control_V1_PrevZCX.Integrator_Reset_ZCE = pid_control_V1_B.Compare;

    /* evaluate zero-crossings and the level of the reset signal */
    if (pid_control_V1_B.serverAvailableOnTime || pid_control_V1_B.Compare ||
        pid_control_V1_DW.Integrator_DWORK1) {
      memcpy(&pid_control_V1_X.Integrator_CSTATE[0], &pid_control_V1_B.IC[0],
             12U * sizeof(real_T));
      rtsiSetBlockStateForSolverChangedAtMajorStep(&(&pid_control_V1_M)
        ->solverInfo, true);
      rtsiSetContTimeOutputInconsistentWithStateAtMajorStep(&(&pid_control_V1_M
        )->solverInfo, true);
    }
  }

  /* Integrator: '<S12>/Integrator' */
  memcpy(&pid_control_V1_B.x[0], &pid_control_V1_X.Integrator_CSTATE[0], 12U *
         sizeof(real_T));
  if (pid_control_V1_B.b) {
    /* MATLAB Function: '<Root>/MATLAB Function3' */
    pid_control_V1_B.cy = cos(pid_control_V1_B.x[8] * 0.5);
    pid_control_V1_B.sy = sin(pid_control_V1_B.x[8] * 0.5);
    pid_control_V1_B.cp = cos(pid_control_V1_B.x[7] * 0.5);
    pid_control_V1_B.sp = sin(pid_control_V1_B.x[7] * 0.5);
    pid_control_V1_B.cr = cos(pid_control_V1_B.x[6] * 0.5);
    pid_control_V1_B.sr = sin(pid_control_V1_B.x[6] * 0.5);
    pid_control_V1_B.factor_bloqueo = pid_control_V1_B.cr * pid_control_V1_B.cp;
    pid_control_V1_B.phi_lim = pid_control_V1_B.sr * pid_control_V1_B.sp;
    pid_control_V1_B.q[0] = pid_control_V1_B.factor_bloqueo *
      pid_control_V1_B.cy + pid_control_V1_B.phi_lim * pid_control_V1_B.sy;
    pid_control_V1_B.sp *= pid_control_V1_B.cr;
    pid_control_V1_B.cp *= pid_control_V1_B.sr;
    pid_control_V1_B.q[1] = pid_control_V1_B.cp * pid_control_V1_B.cy -
      pid_control_V1_B.sp * pid_control_V1_B.sy;
    pid_control_V1_B.q[2] = pid_control_V1_B.sp * pid_control_V1_B.cy +
      pid_control_V1_B.cp * pid_control_V1_B.sy;
    pid_control_V1_B.q[3] = pid_control_V1_B.factor_bloqueo *
      pid_control_V1_B.sy - pid_control_V1_B.phi_lim * pid_control_V1_B.cy;

    /* MATLABSystem: '<Root>/IMU1' */
    if (pid_control_V1_DW.obj.Temperature != 25.0) {
      pid_control_V1_B.serverAvailableOnTime =
        (pid_control_V1_DW.obj.isInitialized == 1);
      if (pid_control_V1_B.serverAvailableOnTime) {
        pid_control_V1_DW.obj.TunablePropsChanged = true;
        pid_control_V1_DW.obj.tunablePropertyChanged[37] = true;
      }

      pid_control_V1_DW.obj.Temperature = 25.0;
    }

    if (!pid_control_V1_isequal_o(pid_control_V1_DW.obj.MagneticFieldNED,
         pid_control_V1_ConstP.IMU1_MagneticFieldNED)) {
      pid_control_V1_B.serverAvailableOnTime =
        (pid_control_V1_DW.obj.isInitialized == 1);
      if (pid_control_V1_B.serverAvailableOnTime) {
        pid_control_V1_DW.obj.TunablePropsChanged = true;
        pid_control_V1_DW.obj.tunablePropertyChanged[0] = true;
      }

      imuSensor_set_MagneticFieldNED(&pid_control_V1_DW.obj,
        pid_control_V1_ConstP.IMU1_MagneticFieldNED);
    }

    if (pid_control_V1_DW.obj.AccelParamsMeasurementRange != (rtInf)) {
      pid_control_V1_B.serverAvailableOnTime =
        (pid_control_V1_DW.obj.isInitialized == 1);
      if (pid_control_V1_B.serverAvailableOnTime) {
        pid_control_V1_DW.obj.TunablePropsChanged = true;
        pid_control_V1_DW.obj.tunablePropertyChanged[3] = true;
      }

      pid_control_V1_DW.obj.AccelParamsMeasurementRange = (rtInf);
    }

    if (pid_control_V1_DW.obj.AccelParamsResolution != 0.0) {
      pid_control_V1_B.serverAvailableOnTime =
        (pid_control_V1_DW.obj.isInitialized == 1);
      if (pid_control_V1_B.serverAvailableOnTime) {
        pid_control_V1_DW.obj.TunablePropsChanged = true;
        pid_control_V1_DW.obj.tunablePropertyChanged[4] = true;
      }

      pid_control_V1_DW.obj.AccelParamsResolution = 0.0;
    }

    if (!pid_control_V1_isequal_o(pid_control_V1_DW.obj.AccelParamsConstantBias,
         pid_control_V1_ConstP.pooled5)) {
      pid_control_V1_B.serverAvailableOnTime =
        (pid_control_V1_DW.obj.isInitialized == 1);
      if (pid_control_V1_B.serverAvailableOnTime) {
        pid_control_V1_DW.obj.TunablePropsChanged = true;
        pid_control_V1_DW.obj.tunablePropertyChanged[5] = true;
      }

      pid_control_V1_DW.obj.AccelParamsConstantBias[0] = 0.0;
      pid_control_V1_DW.obj.AccelParamsConstantBias[1] = 0.0;
      pid_control_V1_DW.obj.AccelParamsConstantBias[2] = 0.0;
    }

    if (!pid_control_V1_isequal_on
        (pid_control_V1_DW.obj.AccelParamsAxesMisalignment,
         pid_control_V1_ConstP.pooled6)) {
      pid_control_V1_B.serverAvailableOnTime =
        (pid_control_V1_DW.obj.isInitialized == 1);
      if (pid_control_V1_B.serverAvailableOnTime) {
        pid_control_V1_DW.obj.TunablePropsChanged = true;
        pid_control_V1_DW.obj.tunablePropertyChanged[6] = true;
      }

      memcpy(&pid_control_V1_DW.obj.AccelParamsAxesMisalignment[0],
             &pid_control_V1_ConstP.pooled6[0], 9U * sizeof(real_T));
    }

    if (!pid_control_V1_isequal_o(pid_control_V1_DW.obj.AccelParamsNoiseDensity,
         pid_control_V1_ConstP.IMU1_AccelParamsNoiseDensity)) {
      pid_control_V1_B.serverAvailableOnTime =
        (pid_control_V1_DW.obj.isInitialized == 1);
      if (pid_control_V1_B.serverAvailableOnTime) {
        pid_control_V1_DW.obj.TunablePropsChanged = true;
        pid_control_V1_DW.obj.tunablePropertyChanged[7] = true;
      }

      pid_control_V1_DW.obj.AccelParamsNoiseDensity[0] = 0.0004;
      pid_control_V1_DW.obj.AccelParamsNoiseDensity[1] = 0.0004;
      pid_control_V1_DW.obj.AccelParamsNoiseDensity[2] = 0.0004;
    }

    if (!pid_control_V1_isequal_o
        (pid_control_V1_DW.obj.AccelParamsBiasInstability,
         pid_control_V1_ConstP.pooled7)) {
      pid_control_V1_B.serverAvailableOnTime =
        (pid_control_V1_DW.obj.isInitialized == 1);
      if (pid_control_V1_B.serverAvailableOnTime) {
        pid_control_V1_DW.obj.TunablePropsChanged = true;
        pid_control_V1_DW.obj.tunablePropertyChanged[8] = true;
      }

      pid_control_V1_DW.obj.AccelParamsBiasInstability[0] = 0.0001;
      pid_control_V1_DW.obj.AccelParamsBiasInstability[1] = 0.0001;
      pid_control_V1_DW.obj.AccelParamsBiasInstability[2] = 0.0001;
    }

    if (pid_control_V1_DW.obj.AccelParamsBiasInstabilityNumerator != 1.0) {
      pid_control_V1_B.serverAvailableOnTime =
        (pid_control_V1_DW.obj.isInitialized == 1);
      if (pid_control_V1_B.serverAvailableOnTime) {
        pid_control_V1_DW.obj.TunablePropsChanged = true;
        pid_control_V1_DW.obj.tunablePropertyChanged[9] = true;
      }

      pid_control_V1_DW.obj.AccelParamsBiasInstabilityNumerator = 1.0;
    }

    if (!pid_control_V1_isequal
        (pid_control_V1_DW.obj.AccelParamsBiasInstabilityDenominator,
         pid_control_V1_ConstP.pooled9)) {
      pid_control_V1_B.serverAvailableOnTime =
        (pid_control_V1_DW.obj.isInitialized == 1);
      if (pid_control_V1_B.serverAvailableOnTime) {
        pid_control_V1_DW.obj.TunablePropsChanged = true;
        pid_control_V1_DW.obj.tunablePropertyChanged[10] = true;
      }

      pid_control_V1_DW.obj.AccelParamsBiasInstabilityDenominator[0] = 1.0;
      pid_control_V1_DW.obj.AccelParamsBiasInstabilityDenominator[1] = -0.5;
    }

    if (!pid_control_V1_isequal_o(pid_control_V1_DW.obj.AccelParamsRandomWalk,
         pid_control_V1_ConstP.pooled5)) {
      pid_control_V1_B.serverAvailableOnTime =
        (pid_control_V1_DW.obj.isInitialized == 1);
      if (pid_control_V1_B.serverAvailableOnTime) {
        pid_control_V1_DW.obj.TunablePropsChanged = true;
        pid_control_V1_DW.obj.tunablePropertyChanged[11] = true;
      }

      pid_control_V1_DW.obj.AccelParamsRandomWalk[0] = 0.0;
      pid_control_V1_DW.obj.AccelParamsRandomWalk[1] = 0.0;
      pid_control_V1_DW.obj.AccelParamsRandomWalk[2] = 0.0;
    }

    if (!pid_control_V1_isequal_o
        (pid_control_V1_DW.obj.AccelParamsTemperatureBias,
         pid_control_V1_ConstP.pooled5)) {
      pid_control_V1_B.serverAvailableOnTime =
        (pid_control_V1_DW.obj.isInitialized == 1);
      if (pid_control_V1_B.serverAvailableOnTime) {
        pid_control_V1_DW.obj.TunablePropsChanged = true;
        pid_control_V1_DW.obj.tunablePropertyChanged[12] = true;
      }

      pid_control_V1_DW.obj.AccelParamsTemperatureBias[0] = 0.0;
      pid_control_V1_DW.obj.AccelParamsTemperatureBias[1] = 0.0;
      pid_control_V1_DW.obj.AccelParamsTemperatureBias[2] = 0.0;
    }

    if (!pid_control_V1_isequal_o
        (pid_control_V1_DW.obj.AccelParamsTemperatureScaleFactor,
         pid_control_V1_ConstP.pooled5)) {
      pid_control_V1_B.serverAvailableOnTime =
        (pid_control_V1_DW.obj.isInitialized == 1);
      if (pid_control_V1_B.serverAvailableOnTime) {
        pid_control_V1_DW.obj.TunablePropsChanged = true;
        pid_control_V1_DW.obj.tunablePropertyChanged[13] = true;
      }

      pid_control_V1_DW.obj.AccelParamsTemperatureScaleFactor[0] = 0.0;
      pid_control_V1_DW.obj.AccelParamsTemperatureScaleFactor[1] = 0.0;
      pid_control_V1_DW.obj.AccelParamsTemperatureScaleFactor[2] = 0.0;
    }

    if (pid_control_V1_DW.obj.GyroParamsMeasurementRange != (rtInf)) {
      pid_control_V1_B.serverAvailableOnTime =
        (pid_control_V1_DW.obj.isInitialized == 1);
      if (pid_control_V1_B.serverAvailableOnTime) {
        pid_control_V1_DW.obj.TunablePropsChanged = true;
        pid_control_V1_DW.obj.tunablePropertyChanged[14] = true;
      }

      pid_control_V1_DW.obj.GyroParamsMeasurementRange = (rtInf);
    }

    if (pid_control_V1_DW.obj.GyroParamsResolution != 0.0) {
      pid_control_V1_B.serverAvailableOnTime =
        (pid_control_V1_DW.obj.isInitialized == 1);
      if (pid_control_V1_B.serverAvailableOnTime) {
        pid_control_V1_DW.obj.TunablePropsChanged = true;
        pid_control_V1_DW.obj.tunablePropertyChanged[15] = true;
      }

      pid_control_V1_DW.obj.GyroParamsResolution = 0.0;
    }

    if (!pid_control_V1_isequal_o(pid_control_V1_DW.obj.GyroParamsConstantBias,
         pid_control_V1_ConstP.pooled5)) {
      pid_control_V1_B.serverAvailableOnTime =
        (pid_control_V1_DW.obj.isInitialized == 1);
      if (pid_control_V1_B.serverAvailableOnTime) {
        pid_control_V1_DW.obj.TunablePropsChanged = true;
        pid_control_V1_DW.obj.tunablePropertyChanged[16] = true;
      }

      pid_control_V1_DW.obj.GyroParamsConstantBias[0] = 0.0;
      pid_control_V1_DW.obj.GyroParamsConstantBias[1] = 0.0;
      pid_control_V1_DW.obj.GyroParamsConstantBias[2] = 0.0;
    }

    if (!pid_control_V1_isequal_on
        (pid_control_V1_DW.obj.GyroParamsAxesMisalignment,
         pid_control_V1_ConstP.pooled6)) {
      pid_control_V1_B.serverAvailableOnTime =
        (pid_control_V1_DW.obj.isInitialized == 1);
      if (pid_control_V1_B.serverAvailableOnTime) {
        pid_control_V1_DW.obj.TunablePropsChanged = true;
        pid_control_V1_DW.obj.tunablePropertyChanged[17] = true;
      }

      memcpy(&pid_control_V1_DW.obj.GyroParamsAxesMisalignment[0],
             &pid_control_V1_ConstP.pooled6[0], 9U * sizeof(real_T));
    }

    if (!pid_control_V1_isequal_o
        (pid_control_V1_DW.obj.GyroParamsAccelerationBias,
         pid_control_V1_ConstP.pooled5)) {
      pid_control_V1_B.serverAvailableOnTime =
        (pid_control_V1_DW.obj.isInitialized == 1);
      if (pid_control_V1_B.serverAvailableOnTime) {
        pid_control_V1_DW.obj.TunablePropsChanged = true;
        pid_control_V1_DW.obj.tunablePropertyChanged[25] = true;
      }

      pid_control_V1_DW.obj.GyroParamsAccelerationBias[0] = 0.0;
      pid_control_V1_DW.obj.GyroParamsAccelerationBias[1] = 0.0;
      pid_control_V1_DW.obj.GyroParamsAccelerationBias[2] = 0.0;
    }

    if (!pid_control_V1_isequal_o(pid_control_V1_DW.obj.GyroParamsNoiseDensity,
         pid_control_V1_ConstP.pooled7)) {
      pid_control_V1_B.serverAvailableOnTime =
        (pid_control_V1_DW.obj.isInitialized == 1);
      if (pid_control_V1_B.serverAvailableOnTime) {
        pid_control_V1_DW.obj.TunablePropsChanged = true;
        pid_control_V1_DW.obj.tunablePropertyChanged[18] = true;
      }

      pid_control_V1_DW.obj.GyroParamsNoiseDensity[0] = 0.0001;
      pid_control_V1_DW.obj.GyroParamsNoiseDensity[1] = 0.0001;
      pid_control_V1_DW.obj.GyroParamsNoiseDensity[2] = 0.0001;
    }

    if (!pid_control_V1_isequal_o
        (pid_control_V1_DW.obj.GyroParamsBiasInstability,
         pid_control_V1_ConstP.IMU1_GyroParamsBiasInstability)) {
      pid_control_V1_B.serverAvailableOnTime =
        (pid_control_V1_DW.obj.isInitialized == 1);
      if (pid_control_V1_B.serverAvailableOnTime) {
        pid_control_V1_DW.obj.TunablePropsChanged = true;
        pid_control_V1_DW.obj.tunablePropertyChanged[19] = true;
      }

      pid_control_V1_DW.obj.GyroParamsBiasInstability[0] = 1.0E-5;
      pid_control_V1_DW.obj.GyroParamsBiasInstability[1] = 1.0E-5;
      pid_control_V1_DW.obj.GyroParamsBiasInstability[2] = 1.0E-5;
    }

    if (pid_control_V1_DW.obj.GyroParamsBiasInstabilityNumerator != 1.0) {
      pid_control_V1_B.serverAvailableOnTime =
        (pid_control_V1_DW.obj.isInitialized == 1);
      if (pid_control_V1_B.serverAvailableOnTime) {
        pid_control_V1_DW.obj.TunablePropsChanged = true;
        pid_control_V1_DW.obj.tunablePropertyChanged[20] = true;
      }

      pid_control_V1_DW.obj.GyroParamsBiasInstabilityNumerator = 1.0;
    }

    if (!pid_control_V1_isequal
        (pid_control_V1_DW.obj.GyroParamsBiasInstabilityDenominator,
         pid_control_V1_ConstP.pooled9)) {
      pid_control_V1_B.serverAvailableOnTime =
        (pid_control_V1_DW.obj.isInitialized == 1);
      if (pid_control_V1_B.serverAvailableOnTime) {
        pid_control_V1_DW.obj.TunablePropsChanged = true;
        pid_control_V1_DW.obj.tunablePropertyChanged[21] = true;
      }

      pid_control_V1_DW.obj.GyroParamsBiasInstabilityDenominator[0] = 1.0;
      pid_control_V1_DW.obj.GyroParamsBiasInstabilityDenominator[1] = -0.5;
    }

    if (!pid_control_V1_isequal_o(pid_control_V1_DW.obj.GyroParamsRandomWalk,
         pid_control_V1_ConstP.pooled5)) {
      pid_control_V1_B.serverAvailableOnTime =
        (pid_control_V1_DW.obj.isInitialized == 1);
      if (pid_control_V1_B.serverAvailableOnTime) {
        pid_control_V1_DW.obj.TunablePropsChanged = true;
        pid_control_V1_DW.obj.tunablePropertyChanged[22] = true;
      }

      pid_control_V1_DW.obj.GyroParamsRandomWalk[0] = 0.0;
      pid_control_V1_DW.obj.GyroParamsRandomWalk[1] = 0.0;
      pid_control_V1_DW.obj.GyroParamsRandomWalk[2] = 0.0;
    }

    if (!pid_control_V1_isequal_o
        (pid_control_V1_DW.obj.GyroParamsTemperatureBias,
         pid_control_V1_ConstP.pooled5)) {
      pid_control_V1_B.serverAvailableOnTime =
        (pid_control_V1_DW.obj.isInitialized == 1);
      if (pid_control_V1_B.serverAvailableOnTime) {
        pid_control_V1_DW.obj.TunablePropsChanged = true;
        pid_control_V1_DW.obj.tunablePropertyChanged[23] = true;
      }

      pid_control_V1_DW.obj.GyroParamsTemperatureBias[0] = 0.0;
      pid_control_V1_DW.obj.GyroParamsTemperatureBias[1] = 0.0;
      pid_control_V1_DW.obj.GyroParamsTemperatureBias[2] = 0.0;
    }

    if (!pid_control_V1_isequal_o
        (pid_control_V1_DW.obj.GyroParamsTemperatureScaleFactor,
         pid_control_V1_ConstP.pooled5)) {
      pid_control_V1_B.serverAvailableOnTime =
        (pid_control_V1_DW.obj.isInitialized == 1);
      if (pid_control_V1_B.serverAvailableOnTime) {
        pid_control_V1_DW.obj.TunablePropsChanged = true;
        pid_control_V1_DW.obj.tunablePropertyChanged[24] = true;
      }

      pid_control_V1_DW.obj.GyroParamsTemperatureScaleFactor[0] = 0.0;
      pid_control_V1_DW.obj.GyroParamsTemperatureScaleFactor[1] = 0.0;
      pid_control_V1_DW.obj.GyroParamsTemperatureScaleFactor[2] = 0.0;
    }

    if (pid_control_V1_DW.obj.MagParamsMeasurementRange != (rtInf)) {
      pid_control_V1_B.serverAvailableOnTime =
        (pid_control_V1_DW.obj.isInitialized == 1);
      if (pid_control_V1_B.serverAvailableOnTime) {
        pid_control_V1_DW.obj.TunablePropsChanged = true;
        pid_control_V1_DW.obj.tunablePropertyChanged[26] = true;
      }

      pid_control_V1_DW.obj.MagParamsMeasurementRange = (rtInf);
    }

    if (pid_control_V1_DW.obj.MagParamsResolution != 0.0) {
      pid_control_V1_B.serverAvailableOnTime =
        (pid_control_V1_DW.obj.isInitialized == 1);
      if (pid_control_V1_B.serverAvailableOnTime) {
        pid_control_V1_DW.obj.TunablePropsChanged = true;
        pid_control_V1_DW.obj.tunablePropertyChanged[27] = true;
      }

      pid_control_V1_DW.obj.MagParamsResolution = 0.0;
    }

    if (!pid_control_V1_isequal_o(pid_control_V1_DW.obj.MagParamsConstantBias,
         pid_control_V1_ConstP.pooled5)) {
      pid_control_V1_B.serverAvailableOnTime =
        (pid_control_V1_DW.obj.isInitialized == 1);
      if (pid_control_V1_B.serverAvailableOnTime) {
        pid_control_V1_DW.obj.TunablePropsChanged = true;
        pid_control_V1_DW.obj.tunablePropertyChanged[28] = true;
      }

      pid_control_V1_DW.obj.MagParamsConstantBias[0] = 0.0;
      pid_control_V1_DW.obj.MagParamsConstantBias[1] = 0.0;
      pid_control_V1_DW.obj.MagParamsConstantBias[2] = 0.0;
    }

    if (!pid_control_V1_isequal_on
        (pid_control_V1_DW.obj.MagParamsAxesMisalignment,
         pid_control_V1_ConstP.pooled6)) {
      pid_control_V1_B.serverAvailableOnTime =
        (pid_control_V1_DW.obj.isInitialized == 1);
      if (pid_control_V1_B.serverAvailableOnTime) {
        pid_control_V1_DW.obj.TunablePropsChanged = true;
        pid_control_V1_DW.obj.tunablePropertyChanged[29] = true;
      }

      memcpy(&pid_control_V1_DW.obj.MagParamsAxesMisalignment[0],
             &pid_control_V1_ConstP.pooled6[0], 9U * sizeof(real_T));
    }

    if (!pid_control_V1_isequal_o(pid_control_V1_DW.obj.MagParamsNoiseDensity,
         pid_control_V1_ConstP.pooled5)) {
      pid_control_V1_B.serverAvailableOnTime =
        (pid_control_V1_DW.obj.isInitialized == 1);
      if (pid_control_V1_B.serverAvailableOnTime) {
        pid_control_V1_DW.obj.TunablePropsChanged = true;
        pid_control_V1_DW.obj.tunablePropertyChanged[30] = true;
      }

      pid_control_V1_DW.obj.MagParamsNoiseDensity[0] = 0.0;
      pid_control_V1_DW.obj.MagParamsNoiseDensity[1] = 0.0;
      pid_control_V1_DW.obj.MagParamsNoiseDensity[2] = 0.0;
    }

    if (!pid_control_V1_isequal_o(pid_control_V1_DW.obj.MagParamsBiasInstability,
         pid_control_V1_ConstP.pooled5)) {
      pid_control_V1_B.serverAvailableOnTime =
        (pid_control_V1_DW.obj.isInitialized == 1);
      if (pid_control_V1_B.serverAvailableOnTime) {
        pid_control_V1_DW.obj.TunablePropsChanged = true;
        pid_control_V1_DW.obj.tunablePropertyChanged[31] = true;
      }

      pid_control_V1_DW.obj.MagParamsBiasInstability[0] = 0.0;
      pid_control_V1_DW.obj.MagParamsBiasInstability[1] = 0.0;
      pid_control_V1_DW.obj.MagParamsBiasInstability[2] = 0.0;
    }

    if (pid_control_V1_DW.obj.MagParamsBiasInstabilityNumerator != 1.0) {
      pid_control_V1_B.serverAvailableOnTime =
        (pid_control_V1_DW.obj.isInitialized == 1);
      if (pid_control_V1_B.serverAvailableOnTime) {
        pid_control_V1_DW.obj.TunablePropsChanged = true;
        pid_control_V1_DW.obj.tunablePropertyChanged[32] = true;
      }

      pid_control_V1_DW.obj.MagParamsBiasInstabilityNumerator = 1.0;
    }

    if (!pid_control_V1_isequal
        (pid_control_V1_DW.obj.MagParamsBiasInstabilityDenominator,
         pid_control_V1_ConstP.pooled9)) {
      pid_control_V1_B.serverAvailableOnTime =
        (pid_control_V1_DW.obj.isInitialized == 1);
      if (pid_control_V1_B.serverAvailableOnTime) {
        pid_control_V1_DW.obj.TunablePropsChanged = true;
        pid_control_V1_DW.obj.tunablePropertyChanged[33] = true;
      }

      pid_control_V1_DW.obj.MagParamsBiasInstabilityDenominator[0] = 1.0;
      pid_control_V1_DW.obj.MagParamsBiasInstabilityDenominator[1] = -0.5;
    }

    if (!pid_control_V1_isequal_o(pid_control_V1_DW.obj.MagParamsRandomWalk,
         pid_control_V1_ConstP.pooled5)) {
      pid_control_V1_B.serverAvailableOnTime =
        (pid_control_V1_DW.obj.isInitialized == 1);
      if (pid_control_V1_B.serverAvailableOnTime) {
        pid_control_V1_DW.obj.TunablePropsChanged = true;
        pid_control_V1_DW.obj.tunablePropertyChanged[34] = true;
      }

      pid_control_V1_DW.obj.MagParamsRandomWalk[0] = 0.0;
      pid_control_V1_DW.obj.MagParamsRandomWalk[1] = 0.0;
      pid_control_V1_DW.obj.MagParamsRandomWalk[2] = 0.0;
    }

    if (!pid_control_V1_isequal_o(pid_control_V1_DW.obj.MagParamsTemperatureBias,
         pid_control_V1_ConstP.pooled5)) {
      pid_control_V1_B.serverAvailableOnTime =
        (pid_control_V1_DW.obj.isInitialized == 1);
      if (pid_control_V1_B.serverAvailableOnTime) {
        pid_control_V1_DW.obj.TunablePropsChanged = true;
        pid_control_V1_DW.obj.tunablePropertyChanged[35] = true;
      }

      pid_control_V1_DW.obj.MagParamsTemperatureBias[0] = 0.0;
      pid_control_V1_DW.obj.MagParamsTemperatureBias[1] = 0.0;
      pid_control_V1_DW.obj.MagParamsTemperatureBias[2] = 0.0;
    }

    if (!pid_control_V1_isequal_o
        (pid_control_V1_DW.obj.MagParamsTemperatureScaleFactor,
         pid_control_V1_ConstP.pooled5)) {
      pid_control_V1_B.serverAvailableOnTime =
        (pid_control_V1_DW.obj.isInitialized == 1);
      if (pid_control_V1_B.serverAvailableOnTime) {
        pid_control_V1_DW.obj.TunablePropsChanged = true;
        pid_control_V1_DW.obj.tunablePropertyChanged[36] = true;
      }

      pid_control_V1_DW.obj.MagParamsTemperatureScaleFactor[0] = 0.0;
      pid_control_V1_DW.obj.MagParamsTemperatureScaleFactor[1] = 0.0;
      pid_control_V1_DW.obj.MagParamsTemperatureScaleFactor[2] = 0.0;
    }

    pid_control_V1_SystemCore_step(&pid_control_V1_DW.obj, &pid_control_V1_B.x[0],
      &pid_control_V1_B.x[3], pid_control_V1_B.q, pid_control_V1_B.IMU1_o1,
      pid_control_V1_B.IMU1_o2, pid_control_V1_B.FA_b);

    /* End of MATLABSystem: '<Root>/IMU1' */
    /* MATLABSystem: '<S13>/SourceBlock' */
    pid_control_V1_B.SourceBlock_o1_o = Sub_pid_control_V1_435.getLatestMessage(
      &pid_control_V1_B.SourceBlock_o2_d);

    /* Outputs for Enabled SubSystem: '<S13>/Enabled Subsystem' */
    pid_control__EnabledSubsystem_k(pid_control_V1_B.SourceBlock_o1_o,
      &pid_control_V1_B.SourceBlock_o2_d, &pid_control_V1_B.EnabledSubsystem_b);

    /* End of Outputs for SubSystem: '<S13>/Enabled Subsystem' */

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
  pid_control_V1_B.cy = pid_control_V1_B.Switch3 - (-pid_control_V1_B.x[11]);

  /* Gain: '<S107>/Filter Coefficient' incorporates:
   *  Gain: '<S97>/Derivative Gain'
   *  Integrator: '<S99>/Filter'
   *  Sum: '<S99>/SumD'
   */
  pid_control_V1_B.FilterCoefficient = (4.5 * pid_control_V1_B.cy -
    pid_control_V1_X.Filter_CSTATE) * 100.0;

  /* Sum: '<S113>/Sum' incorporates:
   *  Gain: '<S109>/Proportional Gain'
   *  Integrator: '<S104>/Integrator'
   */
  pid_control_V1_B.sy = (2.0 * pid_control_V1_B.cy +
    pid_control_V1_X.Integrator_CSTATE_n) + pid_control_V1_B.FilterCoefficient;

  /* Saturate: '<S111>/Saturation' */
  if (pid_control_V1_B.sy > 20.0) {
    /* Saturate: '<S111>/Saturation' */
    pid_control_V1_B.Saturation = 20.0;
  } else if (pid_control_V1_B.sy < 0.0) {
    /* Saturate: '<S111>/Saturation' */
    pid_control_V1_B.Saturation = 0.0;
  } else {
    /* Saturate: '<S111>/Saturation' */
    pid_control_V1_B.Saturation = pid_control_V1_B.sy;
  }

  /* End of Saturate: '<S111>/Saturation' */
  if (pid_control_V1_B.b) {
    /* MATLABSystem: '<S15>/SourceBlock' */
    pid_control_V1_B.SourceBlock_o1 = Sub_pid_control_V1_377.getLatestMessage
      (&pid_control_V1_B.SourceBlock_o2);

    /* Outputs for Enabled SubSystem: '<S15>/Enabled Subsystem' */
    pid_control__EnabledSubsystem_k(pid_control_V1_B.SourceBlock_o1,
      &pid_control_V1_B.SourceBlock_o2, &pid_control_V1_B.EnabledSubsystem_a);

    /* End of Outputs for SubSystem: '<S15>/Enabled Subsystem' */

    /* Switch: '<Root>/Switch2' */
    if (pid_control_V1_B.SourceBlock_o1) {
      /* Switch: '<Root>/Switch2' */
      pid_control_V1_B.Switch2 = pid_control_V1_B.EnabledSubsystem_a.In1.data;
    } else {
      /* Switch: '<Root>/Switch2' incorporates:
       *  UnitDelay: '<Root>/Unit Delay2'
       */
      pid_control_V1_B.Switch2 = pid_control_V1_DW.UnitDelay2_DSTATE;
    }

    /* End of Switch: '<Root>/Switch2' */

    /* MATLAB Function: '<Root>/MATLAB Function2' */
    pid_control_V1_B.yaw_rate_sp = tanh(rt_atan2d_snf(sin
      (pid_control_V1_B.Switch2 - pid_control_V1_DW.UnitDelay_DSTATE), cos
      (pid_control_V1_B.Switch2 - pid_control_V1_DW.UnitDelay_DSTATE)) * 0.15 /
      0.1) * 0.1;
    if ((pid_control_V1_DW.UnitDelay6_DSTATE - 0.25 <= 0.001) || rtIsNaN
        (pid_control_V1_DW.UnitDelay6_DSTATE - 0.25)) {
      pid_control_V1_B.cr = 0.001;
    } else {
      pid_control_V1_B.cr = pid_control_V1_DW.UnitDelay6_DSTATE - 0.25;
    }

    if (pid_control_V1_B.cr - 0.0505 <= 0.001) {
      pid_control_V1_B.cr = 0.001;
    } else {
      pid_control_V1_B.cr -= 0.0505;
    }

    pid_control_V1_B.beta = pid_control_V1_B.cr / 0.3489;
    if (pid_control_V1_B.beta >= 0.9999) {
      pid_control_V1_B.beta = 0.9999;
    }

    pid_control_V1_B.phi_lim = asin(pid_control_V1_B.beta) * 0.6;
    if (!(pid_control_V1_B.phi_lim <= 0.17453292519943295)) {
      pid_control_V1_B.phi_lim = 0.17453292519943295;
    }

    if (!(pid_control_V1_B.phi_lim >= 0.001)) {
      pid_control_V1_B.phi_lim = 0.001;
    }

    if ((pid_control_V1_DW.UnitDelay6_DSTATE - 0.35 <= 0.0) || rtIsNaN
        (pid_control_V1_DW.UnitDelay6_DSTATE - 0.35)) {
      pid_control_V1_B.cr = 0.0;
    } else {
      pid_control_V1_B.cr = pid_control_V1_DW.UnitDelay6_DSTATE - 0.35;
    }

    pid_control_V1_B.factor_bloqueo = tanh(pid_control_V1_B.cr * 20.0);
    if (pid_control_V1_DW.UnitDelay5_DSTATE >= 0.5) {
      pid_control_V1_B.cr = pid_control_V1_DW.UnitDelay5_DSTATE;
    } else {
      pid_control_V1_B.cr = 0.5;
    }

    pid_control_V1_B.roll_sp = -(tanh((1.0 / (exp
      ((pid_control_V1_DW.UnitDelay6_DSTATE - 0.525) * -10.0) + 1.0) * 0.8 + 0.2)
      * atan(pid_control_V1_B.cr * pid_control_V1_B.yaw_rate_sp / 9.81) /
      pid_control_V1_B.phi_lim) * pid_control_V1_B.phi_lim) *
      pid_control_V1_B.factor_bloqueo;
    pid_control_V1_B.yaw_rate_sp *= pid_control_V1_B.factor_bloqueo;

    /* End of MATLAB Function: '<Root>/MATLAB Function2' */

    /* MATLABSystem: '<S14>/SourceBlock' */
    pid_control_V1_B.SourceBlock_o1_e = Sub_pid_control_V1_538.getLatestMessage(
      &pid_control_V1_B.SourceBlock_o2_o);

    /* Switch: '<Root>/Switch1' */
    if (pid_control_V1_B.SourceBlock_o1_e) {
      /* Switch: '<Root>/Switch1' */
      pid_control_V1_B.Switch1 = pid_control_V1_B.roll_sp;
    } else {
      /* Switch: '<Root>/Switch1' incorporates:
       *  UnitDelay: '<Root>/Unit Delay1'
       */
      pid_control_V1_B.Switch1 = pid_control_V1_DW.UnitDelay1_DSTATE;
    }

    /* End of Switch: '<Root>/Switch1' */
  }

  /* Gain: '<S49>/Integral Gain' incorporates:
   *  Sum: '<Root>/Sum4'
   */
  pid_control_V1_B.Switch = pid_control_V1_B.Switch1 - pid_control_V1_B.x[6];

  /* Gain: '<S55>/Filter Coefficient' incorporates:
   *  Gain: '<S45>/Derivative Gain'
   *  Integrator: '<S47>/Filter'
   *  Sum: '<S47>/SumD'
   */
  pid_control_V1_B.FilterCoefficient_c = (-0.5 * pid_control_V1_B.Switch -
    pid_control_V1_X.Filter_CSTATE_g) * 100.0;

  /* Sum: '<S61>/Sum' incorporates:
   *  Gain: '<S57>/Proportional Gain'
   *  Integrator: '<S52>/Integrator'
   */
  pid_control_V1_B.factor_bloqueo = (-1.2 * pid_control_V1_B.Switch +
    pid_control_V1_X.Integrator_CSTATE_m) + pid_control_V1_B.FilterCoefficient_c;

  /* Saturate: '<S59>/Saturation' */
  if (pid_control_V1_B.factor_bloqueo > 0.087266462599716474) {
    /* Saturate: '<S59>/Saturation' */
    pid_control_V1_B.Saturation_k = 0.087266462599716474;
  } else if (pid_control_V1_B.factor_bloqueo < -0.087266462599716474) {
    /* Saturate: '<S59>/Saturation' */
    pid_control_V1_B.Saturation_k = -0.087266462599716474;
  } else {
    /* Saturate: '<S59>/Saturation' */
    pid_control_V1_B.Saturation_k = pid_control_V1_B.factor_bloqueo;
  }

  /* End of Saturate: '<S59>/Saturation' */

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
    pid_control_V1_B.cp = (&pid_control_V1_M)->Timing.t[0];
    pid_control_V1_B.phi_lim = pid_control_V1_B.cp -
      pid_control_V1_DW.LastMajorTime;
    if (pid_control_V1_DW.LastMajorTime == pid_control_V1_B.cp) {
      if (pid_control_V1_DW.PrevLimited) {
        /* RateLimiter: '<Root>/Rate Limiter' */
        pid_control_V1_B.RateLimiter = pid_control_V1_DW.PrevY;
      } else {
        /* RateLimiter: '<Root>/Rate Limiter' */
        pid_control_V1_B.RateLimiter = pid_control_V1_B.Saturation_i;
      }
    } else {
      pid_control_V1_B.sr = pid_control_V1_B.phi_lim * 0.06;
      pid_control_V1_B.cp = pid_control_V1_B.Saturation_i -
        pid_control_V1_DW.PrevY;
      if (pid_control_V1_B.cp > pid_control_V1_B.sr) {
        /* RateLimiter: '<Root>/Rate Limiter' */
        pid_control_V1_B.RateLimiter = pid_control_V1_DW.PrevY +
          pid_control_V1_B.sr;
        pid_control_V1_B.serverAvailableOnTime = true;
      } else {
        pid_control_V1_B.phi_lim *= -0.06;
        if (pid_control_V1_B.cp < pid_control_V1_B.phi_lim) {
          /* RateLimiter: '<Root>/Rate Limiter' */
          pid_control_V1_B.RateLimiter = pid_control_V1_DW.PrevY +
            pid_control_V1_B.phi_lim;
          pid_control_V1_B.serverAvailableOnTime = true;
        } else {
          /* RateLimiter: '<Root>/Rate Limiter' */
          pid_control_V1_B.RateLimiter = pid_control_V1_B.Saturation_i;
          pid_control_V1_B.serverAvailableOnTime = false;
        }
      }

      if (pid_control_V1_B.b1) {
        pid_control_V1_DW.PrevLimited = pid_control_V1_B.serverAvailableOnTime;
      }
    }
  }

  /* Sum: '<Root>/Sum1' */
  pid_control_V1_B.sr = pid_control_V1_B.RateLimiter - pid_control_V1_B.x[7];

  /* Gain: '<S159>/Filter Coefficient' incorporates:
   *  Gain: '<S149>/Derivative Gain'
   *  Integrator: '<S151>/Filter'
   *  Sum: '<S151>/SumD'
   */
  pid_control_V1_B.FilterCoefficient_m = (-0.2 * pid_control_V1_B.sr -
    pid_control_V1_X.Filter_CSTATE_m) * 15.0;

  /* Sum: '<S165>/Sum' incorporates:
   *  Gain: '<S161>/Proportional Gain'
   *  Integrator: '<S156>/Integrator'
   */
  pid_control_V1_B.sp = (-0.35 * pid_control_V1_B.sr +
    pid_control_V1_X.Integrator_CSTATE_p) + pid_control_V1_B.FilterCoefficient_m;

  /* Saturate: '<S163>/Saturation' */
  if (pid_control_V1_B.sp > 0.3490658503988659) {
    /* Saturate: '<S163>/Saturation' */
    pid_control_V1_B.Saturation_f = 0.3490658503988659;
  } else if (pid_control_V1_B.sp < -0.3490658503988659) {
    /* Saturate: '<S163>/Saturation' */
    pid_control_V1_B.Saturation_f = -0.3490658503988659;
  } else {
    /* Saturate: '<S163>/Saturation' */
    pid_control_V1_B.Saturation_f = pid_control_V1_B.sp;
  }

  /* End of Saturate: '<S163>/Saturation' */

  /* Sum: '<Root>/Sum5' */
  pid_control_V1_B.cp = pid_control_V1_B.yaw_rate_sp - pid_control_V1_B.x[8];

  /* Gain: '<S201>/Derivative Gain' incorporates:
   *  Gain: '<S213>/Proportional Gain'
   */
  pid_control_V1_B.phi_lim = -0.4 * pid_control_V1_B.cp;

  /* Gain: '<S211>/Filter Coefficient' incorporates:
   *  Gain: '<S201>/Derivative Gain'
   *  Integrator: '<S203>/Filter'
   *  Sum: '<S203>/SumD'
   */
  pid_control_V1_B.FilterCoefficient_p = (pid_control_V1_B.phi_lim -
    pid_control_V1_X.Filter_CSTATE_f) * 100.0;

  /* Sum: '<S217>/Sum' incorporates:
   *  Integrator: '<S208>/Integrator'
   */
  pid_control_V1_B.Saturation_m = (pid_control_V1_B.phi_lim +
    pid_control_V1_X.Integrator_CSTATE_d) + pid_control_V1_B.FilterCoefficient_p;

  /* Saturate: '<S215>/Saturation' */
  if (pid_control_V1_B.Saturation_m > 0.26179938779914941) {
    /* Sum: '<S217>/Sum' incorporates:
     *  Saturate: '<S215>/Saturation'
     */
    pid_control_V1_B.Saturation_m = 0.26179938779914941;
  } else if (pid_control_V1_B.Saturation_m < -0.26179938779914941) {
    /* Sum: '<S217>/Sum' incorporates:
     *  Saturate: '<S215>/Saturation'
     */
    pid_control_V1_B.Saturation_m = -0.26179938779914941;
  }

  /* End of Saturate: '<S215>/Saturation' */

  /* Gain: '<S265>/Filter Coefficient' incorporates:
   *  Constant: '<Root>/Constant_U'
   *  Gain: '<S255>/Derivative Gain'
   *  Integrator: '<S257>/Filter'
   *  Sum: '<Root>/Sum3'
   *  Sum: '<S257>/SumD'
   */
  pid_control_V1_B.FilterCoefficient_cv = ((20.2 - pid_control_V1_B.x[0]) *
    0.005 - pid_control_V1_X.Filter_CSTATE_l) * 100.0;

  /* Sum: '<S271>/Sum' incorporates:
   *  Constant: '<Root>/Constant_U'
   *  Gain: '<S267>/Proportional Gain'
   *  Integrator: '<S262>/Integrator'
   *  Sum: '<Root>/Sum3'
   */
  pid_control_V1_B.phi_lim = ((20.2 - pid_control_V1_B.x[0]) * 0.08 +
    pid_control_V1_X.Integrator_CSTATE_f) +
    pid_control_V1_B.FilterCoefficient_cv;

  /* Saturate: '<S269>/Saturation' */
  if (pid_control_V1_B.phi_lim > 1.0) {
    /* Saturate: '<S269>/Saturation' */
    pid_control_V1_B.Saturation_o = 1.0;
  } else if (pid_control_V1_B.phi_lim < 0.0) {
    /* Saturate: '<S269>/Saturation' */
    pid_control_V1_B.Saturation_o = 0.0;
  } else {
    /* Saturate: '<S269>/Saturation' */
    pid_control_V1_B.Saturation_o = pid_control_V1_B.phi_lim;
  }

  /* End of Saturate: '<S269>/Saturation' */
  if (pid_control_V1_B.b) {
    /* Memory: '<S12>/Memory' */
    pid_control_V1_B.Memory[0] = pid_control_V1_DW.Memory_PreviousInput[0];

    /* Memory: '<S12>/Memory1' */
    pid_control_V1_B.Memory1[0] = pid_control_V1_DW.Memory1_PreviousInput[0];

    /* Memory: '<S12>/Memory' */
    pid_control_V1_B.Memory[1] = pid_control_V1_DW.Memory_PreviousInput[1];

    /* Memory: '<S12>/Memory1' */
    pid_control_V1_B.Memory1[1] = pid_control_V1_DW.Memory1_PreviousInput[1];

    /* Memory: '<S12>/Memory' */
    pid_control_V1_B.Memory[2] = pid_control_V1_DW.Memory_PreviousInput[2];

    /* Memory: '<S12>/Memory1' */
    pid_control_V1_B.Memory1[2] = pid_control_V1_DW.Memory1_PreviousInput[2];
  }

  /* SignalConversion generated from: '<S283>/ SFunction ' incorporates:
   *  MATLAB Function: '<S12>/MATLAB Function - MODEL'
   */
  pid_control_V1_B.TmpSignalConversionAtSFunct[0] =
    pid_control_V1_B.Saturation_k;
  pid_control_V1_B.TmpSignalConversionAtSFunct[1] =
    pid_control_V1_B.Saturation_f;
  pid_control_V1_B.TmpSignalConversionAtSFunct[2] =
    pid_control_V1_B.Saturation_m;
  pid_control_V1_B.TmpSignalConversionAtSFunct[3] =
    pid_control_V1_B.Saturation_o;
  pid_control_V1_B.TmpSignalConversionAtSFunct[4] =
    pid_control_V1_B.Saturation_o;

  /* MATLAB Function: '<S12>/MATLAB Function - MODEL' incorporates:
   *  Memory: '<S12>/Memory'
   */
  if (pid_control_V1_B.TmpSignalConversionAtSFunct[1] <= 0.3490658503988659) {
    pid_control_V1_B.u2 = pid_control_V1_B.TmpSignalConversionAtSFunct[1];
  } else {
    pid_control_V1_B.u2 = 0.3490658503988659;
  }

  if (!(pid_control_V1_B.u2 >= -0.3490658503988659)) {
    pid_control_V1_B.u2 = -0.3490658503988659;
  }

  tmp = _mm_add_pd(_mm_loadu_pd(&pid_control_V1_B.x[0]), _mm_loadu_pd
                   (&pid_control_V1_B.Memory[0]));
  _mm_storeu_pd(&pid_control_V1_B.dv1[0], tmp);

  /* MATLAB Function: '<S12>/MATLAB Function - MODEL' incorporates:
   *  Memory: '<S12>/Memory'
   *  Memory: '<S12>/Memory1'
   */
  pid_control_V1_B.cr = pid_control_V1_B.x[2] + pid_control_V1_B.Memory[2];
  pid_control_V1_B.Va_out = sqrt((pid_control_V1_B.dv1[0] *
    pid_control_V1_B.dv1[0] + pid_control_V1_B.dv1[1] * pid_control_V1_B.dv1[1])
    + pid_control_V1_B.cr * pid_control_V1_B.cr);
  if (pid_control_V1_B.Va_out < 0.5) {
    pid_control_V1_B.Va_out = 0.5;
    pid_control_V1_B.cr = 0.0;
    pid_control_V1_B.beta = 0.0;
  } else {
    pid_control_V1_B.cr = rt_atan2d_snf(pid_control_V1_B.cr,
      pid_control_V1_B.dv1[0]);
    pid_control_V1_B.beta = pid_control_V1_B.dv1[1] / pid_control_V1_B.Va_out;
    if ((pid_control_V1_B.beta >= 1.0) || rtIsNaN(pid_control_V1_B.beta)) {
      pid_control_V1_B.beta = 1.0;
    }

    if (pid_control_V1_B.beta <= -1.0) {
      pid_control_V1_B.beta = -1.0;
    }

    pid_control_V1_B.beta = asin(pid_control_V1_B.beta);
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
  pid_control_V1_B.IMU1_o1[0] = pid_control_V1_B.x[3];
  pid_control_V1_B.IMU1_o1[1] = pid_control_V1_B.x[4];
  pid_control_V1_B.IMU1_o1[2] = pid_control_V1_B.x[5];
  pid_control_V1_B.IMU1_o2[0] = pid_control_V1_B.x[0];
  pid_control_V1_B.IMU1_o2[1] = pid_control_V1_B.x[1];
  pid_control_V1_B.IMU1_o2[2] = pid_control_V1_B.x[2];
  pid_control_V1_B.Vd1 = pid_control_V1_B.hw / 0.6977;
  pid_control_V1_B.mu_Lw_out = rt_powd_snf(pid_control_V1_B.Vd1, 0.787) * 288.0 *
    exp(rt_powd_snf(pid_control_V1_B.Vd1, 0.327) * -9.14) * 0.97986308862072491 /
    5.9129476540958859 + 1.0;
  pid_control_V1_B.mu_Dw_out = (1.0 - exp(rt_powd_snf(pid_control_V1_B.Vd1,
    0.814) * -4.74) * 0.97916641726789588) - exp(rt_powd_snf
    (pid_control_V1_B.Vd1, 0.758) * -3.88) * (pid_control_V1_B.Vd1 *
    pid_control_V1_B.Vd1);
  pid_control_V1_B.CD_ih_IGE = pid_control_V1_B.hh / 0.3808;
  pid_control_V1_B.hh = ((pid_control_V1_B.cr - -0.065449846949787352) +
    0.026179938779914941) * 4.9604094530365153;
  pid_control_V1_B.hw = (((pid_control_V1_B.cr - -0.043633231299858237) +
    0.0087266462599716477) - (0.56 / pid_control_V1_B.Va_out * 0.35 *
    pid_control_V1_B.q_aero + (pid_control_V1_B.cr - -0.065449846949787352) *
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
  pid_control_V1_B.Dtot = ((pid_control_V1_B.u2 * pid_control_V1_B.u2 * -1.08E-5
    + 0.000715 * pid_control_V1_B.u2) * 0.02164 + ((pid_control_V1_B.CD_iw_IGE *
    0.0649 + 0.0027258) + pid_control_V1_B.CD_ih_IGE * 0.02164)) *
    pid_control_V1_B.Q;
  pid_control_V1_B.Ltot_tmp = pid_control_V1_B.CL_w_IGE * 0.0649 +
    pid_control_V1_B.CL_h_IGE * 0.02164;
  pid_control_V1_B.Ltot = pid_control_V1_B.Ltot_tmp * pid_control_V1_B.Q;
  pid_control_V1_B.CQ = -0.019 * pid_control_V1_B.beta * 180.0 /
    3.1415926535897931;
  pid_control_V1_B.FA_b_tmp = sin(pid_control_V1_B.cr);
  pid_control_V1_B.FA_b_tmp_n = cos(pid_control_V1_B.cr);
  pid_control_V1_B.R[0] = pid_control_V1_B.FA_b_tmp_n;
  pid_control_V1_B.R[3] = 0.0;
  pid_control_V1_B.R[6] = -pid_control_V1_B.FA_b_tmp;
  pid_control_V1_B.R[2] = pid_control_V1_B.FA_b_tmp;
  pid_control_V1_B.R[5] = 0.0;
  pid_control_V1_B.R[8] = pid_control_V1_B.FA_b_tmp_n;
  pid_control_V1_B.FA_b[0] = -pid_control_V1_B.Dtot;
  pid_control_V1_B.FA_b[1] = pid_control_V1_B.CQ * pid_control_V1_B.Q * 0.0649;
  pid_control_V1_B.FA_b[2] = -pid_control_V1_B.Ltot;
  pid_control_V1_B.R[1] = 0.0;
  pid_control_V1_B.R[4] = 1.0;
  pid_control_V1_B.R[7] = 0.0;
  pid_control_V1_B.FA_b_tmp = 0.0;
  pid_control_V1_B.FA_b_tmp_n = 0.0;
  pid_control_V1_B.FA_b_b = 0.0;
  for (pid_control_V1_B.i = 0; pid_control_V1_B.i < 3; pid_control_V1_B.i++) {
    tmp = _mm_add_pd(_mm_mul_pd(_mm_loadu_pd(&pid_control_V1_B.R[3 *
      pid_control_V1_B.i]), _mm_set1_pd(pid_control_V1_B.FA_b[pid_control_V1_B.i])),
                     _mm_set_pd(pid_control_V1_B.FA_b_tmp_n,
      pid_control_V1_B.FA_b_tmp));
    _mm_storeu_pd(&pid_control_V1_B.dv1[0], tmp);
    pid_control_V1_B.FA_b_tmp = pid_control_V1_B.dv1[0];
    pid_control_V1_B.FA_b_tmp_n = pid_control_V1_B.dv1[1];
    pid_control_V1_B.FA_b_b += pid_control_V1_B.R[3 * pid_control_V1_B.i + 2] *
      pid_control_V1_B.FA_b[pid_control_V1_B.i];
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
    pid_control_V1_B.beta) + -0.5 * pid_control_V1_B.Vd2;
  pid_control_V1_B.u2 = ((exp(pid_control_V1_B.Vd1 * -4.0) * -0.05 + -1.14 *
    pid_control_V1_B.cr) + pid_control_V1_B.q_aero * 0.093 /
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
    pid_control_V1_B.beta) + -0.3 * pid_control_V1_B.Vd2;
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
    pid_control_V1_B.FA_b_tmp;
  _mm_storeu_pd(&pid_control_V1_B.dv1[0], _mm_add_pd(_mm_set_pd
    (pid_control_V1_B.FE1_b[2], pid_control_V1_B.dv1[0]), _mm_set_pd
    (pid_control_V1_B.FE2_b_idx_2, pid_control_V1_B.FA_b_tmp_n)));
  pid_control_V1_B.F_b[1] = pid_control_V1_B.dv1[0];
  pid_control_V1_B.FE_b = pid_control_V1_B.dv1[1];
  pid_control_V1_B.F_b[2] = (pid_control_V1_B.Fg_b_idx_2 + pid_control_V1_B.dv1
    [1]) + pid_control_V1_B.FA_b_b;
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
  pid_control_V1_B.R_tmp_l = pid_control_V1_B.c_phi * pid_control_V1_B.s_the;
  pid_control_V1_B.R[6] = pid_control_V1_B.R_tmp_l * pid_control_V1_B.c_psi +
    pid_control_V1_B.FE2_b_idx_2 * pid_control_V1_B.s_psi;
  pid_control_V1_B.R[1] = pid_control_V1_B.c_the * pid_control_V1_B.s_psi;
  pid_control_V1_B.R[4] = pid_control_V1_B.R_tmp * pid_control_V1_B.s_psi +
    pid_control_V1_B.c_phi * pid_control_V1_B.c_psi;
  pid_control_V1_B.R[7] = pid_control_V1_B.R_tmp_l * pid_control_V1_B.s_psi -
    pid_control_V1_B.FE2_b_idx_2 * pid_control_V1_B.c_psi;
  pid_control_V1_B.R[2] = -pid_control_V1_B.s_the;
  pid_control_V1_B.R[5] = pid_control_V1_B.FE2_b_idx_2 * pid_control_V1_B.c_the;
  pid_control_V1_B.R[8] = pid_control_V1_B.c_phi * pid_control_V1_B.c_the;
  _mm_storeu_pd(&pid_control_V1_B.FA_b[0], _mm_sub_pd(_mm_mul_pd(_mm_set_pd
    (pid_control_V1_B.IMU1_o2[0], pid_control_V1_B.IMU1_o1[1]), _mm_set_pd
    (pid_control_V1_B.IMU1_o1[2], pid_control_V1_B.IMU1_o2[2])), _mm_mul_pd
    (_mm_set_pd(pid_control_V1_B.IMU1_o1[0], pid_control_V1_B.IMU1_o2[1]),
     _mm_set_pd(pid_control_V1_B.IMU1_o2[2], pid_control_V1_B.IMU1_o1[2]))));
  pid_control_V1_B.FA_b[2] = pid_control_V1_B.IMU1_o1[0] *
    pid_control_V1_B.IMU1_o2[1] - pid_control_V1_B.IMU1_o2[0] *
    pid_control_V1_B.IMU1_o1[1];
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
  for (pid_control_V1_B.i = 0; pid_control_V1_B.i < 3; pid_control_V1_B.i++) {
    tmp = _mm_add_pd(_mm_mul_pd(_mm_loadu_pd(&pid_control_V1_B.dv[3 *
      pid_control_V1_B.i]), _mm_set1_pd
      (pid_control_V1_B.IMU1_o1[pid_control_V1_B.i])), _mm_set_pd
                     (pid_control_V1_B.FE2_b_idx_2, pid_control_V1_B.c_phi));
    _mm_storeu_pd(&pid_control_V1_B.dv1[0], tmp);
    pid_control_V1_B.c_phi = pid_control_V1_B.dv1[0];
    pid_control_V1_B.FE2_b_idx_2 = pid_control_V1_B.dv1[1];
    _mm_storeu_pd(&pid_control_V1_B.dv1[0], _mm_add_pd(_mm_mul_pd(_mm_set_pd
      (pid_control_V1_B.R[3 * pid_control_V1_B.i], pid_control_V1_B.dv[3 *
       pid_control_V1_B.i + 2]), _mm_set_pd
      (pid_control_V1_B.FE1_b[pid_control_V1_B.i],
       pid_control_V1_B.IMU1_o1[pid_control_V1_B.i])), _mm_set_pd
      (pid_control_V1_B.s_the, pid_control_V1_B.c_the)));
    pid_control_V1_B.c_the = pid_control_V1_B.dv1[0];
    pid_control_V1_B.s_the = pid_control_V1_B.dv1[1];
    tmp = _mm_add_pd(_mm_mul_pd(_mm_loadu_pd(&pid_control_V1_B.R[3 *
      pid_control_V1_B.i + 1]), _mm_set1_pd
      (pid_control_V1_B.FE1_b[pid_control_V1_B.i])), _mm_set_pd
                     (pid_control_V1_B.s_psi, pid_control_V1_B.c_psi));
    _mm_storeu_pd(&pid_control_V1_B.dv1[0], tmp);
    pid_control_V1_B.c_psi = pid_control_V1_B.dv1[0];
    pid_control_V1_B.s_psi = pid_control_V1_B.dv1[1];
    pid_control_V1_B.XDOT[pid_control_V1_B.i] = 0.83333333333333337 *
      pid_control_V1_B.F_b[pid_control_V1_B.i] -
      pid_control_V1_B.FA_b[pid_control_V1_B.i];
  }

  pid_control_V1_B.XDOT[3] = ((0.023669 * pid_control_V1_B.Mcg_b_idx_0 +
    0.001856 * pid_control_V1_B.FE2_b_idx_0) - (-2.6338496E-5 *
    pid_control_V1_B.IMU1_o1[0] + 0.000144843342 * pid_control_V1_B.IMU1_o1[2]) *
    pid_control_V1_B.IMU1_o1[1]) / 0.000191043437;
  pid_control_V1_B.XDOT[4] = ((pid_control_V1_B.Q - -0.015451999999999999 *
    pid_control_V1_B.IMU1_o1[0] * pid_control_V1_B.IMU1_o1[2]) -
    (pid_control_V1_B.IMU1_o1[0] * pid_control_V1_B.IMU1_o1[0] -
     pid_control_V1_B.IMU1_o1[2] * pid_control_V1_B.IMU1_o1[2]) * 0.001856) /
    0.017695;
  pid_control_V1_B.XDOT[5] = ((-2.6338496E-5 * pid_control_V1_B.IMU1_o1[2] +
    -7.4435989999999989E-5 * pid_control_V1_B.IMU1_o1[0]) *
    pid_control_V1_B.IMU1_o1[1] + (0.001856 * pid_control_V1_B.Mcg_b_idx_0 +
    0.008217 * pid_control_V1_B.FE2_b_idx_0)) / 0.000191043437;
  pid_control_V1_B.XDOT[9] = pid_control_V1_B.s_the;
  pid_control_V1_B.XDOT[10] = pid_control_V1_B.c_psi;
  pid_control_V1_B.XDOT[11] = pid_control_V1_B.s_psi;
  if (!(pid_control_V1_B.Dtot >= 0.001)) {
    pid_control_V1_B.Dtot = 0.001;
  }

  pid_control_V1_B.XDOT[12] = pid_control_V1_B.Ltot / pid_control_V1_B.Dtot;
  pid_control_V1_B.XDOT[19] = pid_control_V1_B.CQ;
  pid_control_V1_B.XDOT[20] = pid_control_V1_B.Cl;
  pid_control_V1_B.XDOT[21] = pid_control_V1_B.u2;
  pid_control_V1_B.XDOT[22] = pid_control_V1_B.q_aero;
  pid_control_V1_B.XDOT[23] = pid_control_V1_B.cr;
  pid_control_V1_B.XDOT[24] = pid_control_V1_B.beta;
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
  pid_control_V1_B.XDOT[37] = pid_control_V1_B.FA_b_tmp;
  pid_control_V1_B.XDOT[7] = pid_control_V1_B.FE2_b_idx_2;
  pid_control_V1_B.XDOT[14] = pid_control_V1_B.F_b[1];
  pid_control_V1_B.XDOT[17] = pid_control_V1_B.Q;
  pid_control_V1_B.XDOT[32] = pid_control_V1_B.Vd2;
  pid_control_V1_B.XDOT[35] = 0.0;
  pid_control_V1_B.XDOT[38] = pid_control_V1_B.FA_b_tmp_n;
  pid_control_V1_B.XDOT[8] = pid_control_V1_B.c_the;
  pid_control_V1_B.XDOT[15] = pid_control_V1_B.F_b[2];
  pid_control_V1_B.XDOT[18] = pid_control_V1_B.FE2_b_idx_0;
  pid_control_V1_B.XDOT[33] = pid_control_V1_B.Fg_b_idx_2;
  pid_control_V1_B.XDOT[36] = pid_control_V1_B.FE_b;
  pid_control_V1_B.XDOT[39] = pid_control_V1_B.FA_b_b;
  pid_control_V1_B.CL_total = pid_control_V1_B.Ltot_tmp / 0.08654;
  pid_control_V1_B.h_out = -pid_control_V1_B.x[11];
  if (pid_control_V1_B.b) {
    /* MATLAB Function: '<Root>/MATLAB Function' */
    memset(&pid_control_V1_B.stringOut_l[0], 0, sizeof(uint8_T) << 7U);
    for (pid_control_V1_B.i = 0; pid_control_V1_B.i < 11; pid_control_V1_B.i++)
    {
      pid_control_V1_B.stringOut_l[pid_control_V1_B.i] = b[pid_control_V1_B.i];
    }

    pid_control_V1_B.lengthOut_e = 11U;

    /* End of MATLAB Function: '<Root>/MATLAB Function' */

    /* MATLAB Function: '<Root>/MATLAB Function1' */
    memset(&pid_control_V1_B.stringOut[0], 0, sizeof(uint8_T) << 7U);
    for (pid_control_V1_B.i = 0; pid_control_V1_B.i < 5; pid_control_V1_B.i++) {
      pid_control_V1_B.stringOut[pid_control_V1_B.i] = b_0[pid_control_V1_B.i];
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
  _mm_storeu_pd(&pid_control_V1_B.IMU1_o1[0], _mm_div_pd(_mm_set_pd
    (-pid_control_V1_B.x[7], -pid_control_V1_B.x[8]), _mm_set1_pd(2.0)));

  /* MATLABSystem: '<Root>/Coordinate Transformation Conversion' incorporates:
   *  Constant: '<Root>/Constant'
   *  Sum: '<Root>/Sum'
   */
  pid_control_V1_B.IMU1_o1[2] = (pid_control_V1_B.x[6] + 1.57) / 2.0;
  pid_control_V1_B.beta = sin(pid_control_V1_B.IMU1_o1[0]);
  pid_control_V1_B.hh = sin(pid_control_V1_B.IMU1_o1[1]);
  pid_control_V1_B.hw = sin(pid_control_V1_B.IMU1_o1[2]);
  pid_control_V1_B.CL_w_IGE = cos(pid_control_V1_B.IMU1_o1[0]);
  pid_control_V1_B.CL_h_IGE = cos(pid_control_V1_B.IMU1_o1[1]);
  pid_control_V1_B.CD_iw_IGE = cos(pid_control_V1_B.IMU1_o1[2]);

  /* BusAssignment: '<Root>/Bus Assignment' incorporates:
   *  Gain: '<Root>/Gain3'
   */
  pid_control_V1_B.BusAssignment.state.pose.position.x = pid_control_V1_B.x[9];
  pid_control_V1_B.BusAssignment.state.pose.position.y = -pid_control_V1_B.x[10];
  pid_control_V1_B.BusAssignment.state.pose.position.z = pid_control_V1_B.Gain;

  /* Start for MATLABSystem: '<Root>/Coordinate Transformation Conversion' */
  pid_control_V1_B.cr = pid_control_V1_B.CL_w_IGE * pid_control_V1_B.CL_h_IGE;

  /* BusAssignment: '<Root>/Bus Assignment' incorporates:
   *  MATLABSystem: '<Root>/Coordinate Transformation Conversion'
   * */
  pid_control_V1_B.BusAssignment.state.pose.orientation.w =
    pid_control_V1_B.beta * pid_control_V1_B.hh * pid_control_V1_B.hw +
    pid_control_V1_B.cr * pid_control_V1_B.CD_iw_IGE;
  pid_control_V1_B.BusAssignment.state.pose.orientation.z = pid_control_V1_B.cr *
    pid_control_V1_B.hw - pid_control_V1_B.CD_iw_IGE * pid_control_V1_B.beta *
    pid_control_V1_B.hh;
  pid_control_V1_B.BusAssignment.state.pose.orientation.y =
    pid_control_V1_B.CL_w_IGE * pid_control_V1_B.CD_iw_IGE * pid_control_V1_B.hh
    + pid_control_V1_B.CL_h_IGE * pid_control_V1_B.beta * pid_control_V1_B.hw;
  pid_control_V1_B.BusAssignment.state.pose.orientation.x =
    pid_control_V1_B.CL_h_IGE * pid_control_V1_B.CD_iw_IGE *
    pid_control_V1_B.beta - pid_control_V1_B.CL_w_IGE * pid_control_V1_B.hh *
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
  pid_control_V1_B.serverAvailableOnTime =
    ServCall_pid_control_V1_326.waitForServer(5.0);
  if (pid_control_V1_B.serverAvailableOnTime) {
    ServCall_pid_control_V1_326.call(&pid_control_V1_B.BusAssignment,
      &pid_control_V1_B.r);
  }

  /* End of MATLABSystem: '<S2>/ServiceCaller' */
  /* End of Outputs for SubSystem: '<Root>/Call Service' */

  /* Gain: '<S42>/ZeroGain' */
  pid_control_V1_B.beta = 0.0 * pid_control_V1_B.factor_bloqueo;

  /* DeadZone: '<S44>/DeadZone' */
  if (pid_control_V1_B.factor_bloqueo > 0.087266462599716474) {
    pid_control_V1_B.factor_bloqueo -= 0.087266462599716474;
  } else if (pid_control_V1_B.factor_bloqueo >= -0.087266462599716474) {
    pid_control_V1_B.factor_bloqueo = 0.0;
  } else {
    pid_control_V1_B.factor_bloqueo -= -0.087266462599716474;
  }

  /* End of DeadZone: '<S44>/DeadZone' */

  /* Gain: '<S49>/Integral Gain' */
  pid_control_V1_B.Switch *= -0.08;

  /* Signum: '<S42>/SignPreSat' */
  if (rtIsNaN(pid_control_V1_B.factor_bloqueo)) {
    /* DataTypeConversion: '<S42>/DataTypeConv1' */
    pid_control_V1_B.i = 0;
  } else {
    if (pid_control_V1_B.factor_bloqueo < 0.0) {
      /* DataTypeConversion: '<S42>/DataTypeConv1' */
      pid_control_V1_B.cr = -1.0;
    } else {
      /* DataTypeConversion: '<S42>/DataTypeConv1' */
      pid_control_V1_B.cr = (pid_control_V1_B.factor_bloqueo > 0.0);
    }

    /* DataTypeConversion: '<S42>/DataTypeConv1' */
    pid_control_V1_B.i = static_cast<int32_T>(fmod(pid_control_V1_B.cr, 256.0));
  }

  /* End of Signum: '<S42>/SignPreSat' */

  /* Signum: '<S42>/SignPreIntegrator' */
  if (rtIsNaN(pid_control_V1_B.Switch)) {
    /* DataTypeConversion: '<S42>/DataTypeConv2' */
    pid_control_V1_B.i_f = 0;
  } else {
    if (pid_control_V1_B.Switch < 0.0) {
      /* DataTypeConversion: '<S42>/DataTypeConv2' */
      pid_control_V1_B.cr = -1.0;
    } else {
      /* DataTypeConversion: '<S42>/DataTypeConv2' */
      pid_control_V1_B.cr = (pid_control_V1_B.Switch > 0.0);
    }

    /* DataTypeConversion: '<S42>/DataTypeConv2' */
    pid_control_V1_B.i_f = static_cast<int32_T>(fmod(pid_control_V1_B.cr, 256.0));
  }

  /* End of Signum: '<S42>/SignPreIntegrator' */

  /* DataTypeConversion: '<S42>/DataTypeConv1' */
  if (pid_control_V1_B.i < 0) {
    pid_control_V1_B.i = static_cast<int8_T>(-static_cast<int8_T>
      (static_cast<uint8_T>(-static_cast<real_T>(pid_control_V1_B.i))));
  }

  /* DataTypeConversion: '<S42>/DataTypeConv2' */
  if (pid_control_V1_B.i_f < 0) {
    pid_control_V1_B.i_f = static_cast<int8_T>(-static_cast<int8_T>(static_cast<
      uint8_T>(-static_cast<real_T>(pid_control_V1_B.i_f))));
  }

  /* Logic: '<S42>/AND3' incorporates:
   *  DataTypeConversion: '<S42>/DataTypeConv1'
   *  DataTypeConversion: '<S42>/DataTypeConv2'
   *  RelationalOperator: '<S42>/Equal1'
   *  RelationalOperator: '<S42>/NotEqual'
   */
  pid_control_V1_B.AND3 = ((pid_control_V1_B.beta !=
    pid_control_V1_B.factor_bloqueo) && (pid_control_V1_B.i ==
    pid_control_V1_B.i_f));
  if (pid_control_V1_B.b) {
    /* Memory: '<S42>/Memory' */
    pid_control_V1_B.Memory_a = pid_control_V1_DW.Memory_PreviousInput_o;
  }

  /* Switch: '<S42>/Switch' */
  if (pid_control_V1_B.Memory_a) {
    /* Gain: '<S49>/Integral Gain' incorporates:
     *  Constant: '<S42>/Constant1'
     *  Switch: '<S42>/Switch'
     */
    pid_control_V1_B.Switch = 0.0;
  }

  /* End of Switch: '<S42>/Switch' */

  /* Sum: '<S96>/SumI4' incorporates:
   *  Gain: '<S101>/Integral Gain'
   *  Gain: '<S96>/Kb'
   *  Sum: '<S96>/SumI2'
   */
  pid_control_V1_B.SumI4 = (pid_control_V1_B.Saturation - pid_control_V1_B.sy) *
    0.1 + 0.25 * pid_control_V1_B.cy;

  /* Sum: '<S148>/SumI4' incorporates:
   *  Gain: '<S153>/Integral Gain'
   *  Sum: '<S148>/SumI2'
   */
  pid_control_V1_B.SumI4_i = (pid_control_V1_B.Saturation_f -
    pid_control_V1_B.sp) + -0.05 * pid_control_V1_B.sr;

  /* Gain: '<S205>/Integral Gain' */
  pid_control_V1_B.IntegralGain = -0.005 * pid_control_V1_B.cp;

  /* Gain: '<S252>/ZeroGain' */
  pid_control_V1_B.cy = 0.0 * pid_control_V1_B.phi_lim;

  /* DeadZone: '<S254>/DeadZone' */
  if (pid_control_V1_B.phi_lim > 1.0) {
    pid_control_V1_B.phi_lim--;
  } else if (pid_control_V1_B.phi_lim >= 0.0) {
    pid_control_V1_B.phi_lim = 0.0;
  }

  /* End of DeadZone: '<S254>/DeadZone' */

  /* Gain: '<S259>/Integral Gain' incorporates:
   *  Constant: '<Root>/Constant_U'
   *  Sum: '<Root>/Sum3'
   */
  pid_control_V1_B.Switch_j = (20.2 - pid_control_V1_B.x[0]) * 0.015;

  /* Signum: '<S252>/SignPreSat' */
  if (rtIsNaN(pid_control_V1_B.phi_lim)) {
    /* DataTypeConversion: '<S252>/DataTypeConv1' */
    pid_control_V1_B.i = 0;
  } else {
    if (pid_control_V1_B.phi_lim < 0.0) {
      /* DataTypeConversion: '<S252>/DataTypeConv1' */
      pid_control_V1_B.cr = -1.0;
    } else {
      /* DataTypeConversion: '<S252>/DataTypeConv1' */
      pid_control_V1_B.cr = (pid_control_V1_B.phi_lim > 0.0);
    }

    /* DataTypeConversion: '<S252>/DataTypeConv1' */
    pid_control_V1_B.i = static_cast<int32_T>(fmod(pid_control_V1_B.cr, 256.0));
  }

  /* End of Signum: '<S252>/SignPreSat' */

  /* Signum: '<S252>/SignPreIntegrator' */
  if (rtIsNaN(pid_control_V1_B.Switch_j)) {
    /* DataTypeConversion: '<S252>/DataTypeConv2' */
    pid_control_V1_B.i_f = 0;
  } else {
    if (pid_control_V1_B.Switch_j < 0.0) {
      /* DataTypeConversion: '<S252>/DataTypeConv2' */
      pid_control_V1_B.cr = -1.0;
    } else {
      /* DataTypeConversion: '<S252>/DataTypeConv2' */
      pid_control_V1_B.cr = (pid_control_V1_B.Switch_j > 0.0);
    }

    /* DataTypeConversion: '<S252>/DataTypeConv2' */
    pid_control_V1_B.i_f = static_cast<int32_T>(fmod(pid_control_V1_B.cr, 256.0));
  }

  /* End of Signum: '<S252>/SignPreIntegrator' */

  /* DataTypeConversion: '<S252>/DataTypeConv1' */
  if (pid_control_V1_B.i < 0) {
    pid_control_V1_B.i = static_cast<int8_T>(-static_cast<int8_T>
      (static_cast<uint8_T>(-static_cast<real_T>(pid_control_V1_B.i))));
  }

  /* DataTypeConversion: '<S252>/DataTypeConv2' */
  if (pid_control_V1_B.i_f < 0) {
    pid_control_V1_B.i_f = static_cast<int8_T>(-static_cast<int8_T>(static_cast<
      uint8_T>(-static_cast<real_T>(pid_control_V1_B.i_f))));
  }

  /* Logic: '<S252>/AND3' incorporates:
   *  DataTypeConversion: '<S252>/DataTypeConv1'
   *  DataTypeConversion: '<S252>/DataTypeConv2'
   *  RelationalOperator: '<S252>/Equal1'
   *  RelationalOperator: '<S252>/NotEqual'
   */
  pid_control_V1_B.AND3_c = ((pid_control_V1_B.cy != pid_control_V1_B.phi_lim) &&
    (pid_control_V1_B.i == pid_control_V1_B.i_f));
  if (pid_control_V1_B.b) {
    /* Memory: '<S252>/Memory' */
    pid_control_V1_B.Memory_h = pid_control_V1_DW.Memory_PreviousInput_a;
  }

  /* Switch: '<S252>/Switch' */
  if (pid_control_V1_B.Memory_h) {
    /* Gain: '<S259>/Integral Gain' incorporates:
     *  Constant: '<S252>/Constant1'
     *  Switch: '<S252>/Switch'
     */
    pid_control_V1_B.Switch_j = 0.0;
  }

  /* End of Switch: '<S252>/Switch' */

  /* UnitConversion: '<S292>/Unit Conversion' incorporates:
   *  Gain: '<S12>/Gain4'
   */
  /* Unit Conversion - from: m to: ft
     Expression: output = (3.28084*input) + (0) */
  pid_control_V1_B.cy = 3.280839895013123 * -pid_control_V1_B.x[11];

  /* UnitConversion: '<S298>/Unit Conversion' */
  /* Unit Conversion - from: m/s to: ft/s
     Expression: output = (3.28084*input) + (0) */
  pid_control_V1_B.sy = 3.280839895013123 * pid_control_V1_B.Va_out;

  /* Saturate: '<S325>/Limit Function 10ft to 1000ft' incorporates:
   *  Saturate: '<S308>/Limit Height h<1000ft'
   */
  if (pid_control_V1_B.cy > 1000.0) {
    pid_control_V1_B.factor_bloqueo = 1000.0;
    pid_control_V1_B.phi_lim = 1000.0;
  } else {
    if (pid_control_V1_B.cy < 10.0) {
      pid_control_V1_B.factor_bloqueo = 10.0;
    } else {
      pid_control_V1_B.factor_bloqueo = pid_control_V1_B.cy;
    }

    if (pid_control_V1_B.cy < 0.0) {
      pid_control_V1_B.phi_lim = 0.0;
    } else {
      pid_control_V1_B.phi_lim = pid_control_V1_B.cy;
    }
  }

  /* End of Saturate: '<S325>/Limit Function 10ft to 1000ft' */

  /* Fcn: '<S325>/Low Altitude Scale Length' */
  pid_control_V1_B.sr = pid_control_V1_B.factor_bloqueo / rt_powd_snf(0.000823 *
    pid_control_V1_B.factor_bloqueo + 0.177, 1.2);

  /* Product: '<S308>/sigma_ug, sigma_vg' incorporates:
   *  Fcn: '<S308>/Low Altitude Intensity'
   */
  pid_control_V1_B.cp = 1.0 / rt_powd_snf(0.000823 * pid_control_V1_B.phi_lim +
    0.177, 0.4) * pid_control_V1_ConstB.sigma_wg;

  /* Interpolation_n-D: '<S307>/Medium//High Altitude Intensity' incorporates:
   *  PreLookup: '<S307>/PreLook-Up Index Search  (altitude)'
   */
  pid_control_V1_B.bpIndex[0] = plook_bincpa(pid_control_V1_B.cy,
    pid_control_V1_ConstP.PreLookUpIndexSearchaltitude_Br, 11U,
    &pid_control_V1_B.phi_lim,
    &pid_control_V1_DW.PreLookUpIndexSearchaltitude_DW);
  pid_control_V1_B.frac[0] = pid_control_V1_B.phi_lim;
  pid_control_V1_B.frac[1] = pid_control_V1_ConstB.PreLookUpIndexSearchprobofe;
  pid_control_V1_B.bpIndex[1] =
    pid_control_V1_ConstB.PreLookUpIndexSearchprobo_g;
  pid_control_V1_B.phi_lim = intrp2d_la_pw(pid_control_V1_B.bpIndex,
    pid_control_V1_B.frac, pid_control_V1_ConstP.MediumHighAltitudeIntensity_Tab,
    12U, pid_control_V1_ConstP.MediumHighAltitudeIntensity_max);
  if (pid_control_V1_B.b) {
    /* Product: '<S300>/Divide' incorporates:
     *  Product: '<S300>/Product'
     *  RandomNumber: '<S300>/White Noise'
     */
    tmp = _mm_mul_pd(_mm_loadu_pd(&pid_control_V1_ConstB.Divide[0]),
                     _mm_loadu_pd(&pid_control_V1_DW.NextOutput[0]));

    /* Product: '<S300>/Product' */
    _mm_storeu_pd(&pid_control_V1_B.Product[0], tmp);

    /* Product: '<S300>/Divide' incorporates:
     *  Product: '<S300>/Product'
     *  RandomNumber: '<S300>/White Noise'
     */
    tmp = _mm_mul_pd(_mm_loadu_pd(&pid_control_V1_ConstB.Divide[2]),
                     _mm_loadu_pd(&pid_control_V1_DW.NextOutput[2]));

    /* Product: '<S300>/Product' */
    _mm_storeu_pd(&pid_control_V1_B.Product[2], tmp);

    /* Outputs for Enabled SubSystem: '<S291>/Hugw(s)' incorporates:
     *  EnablePort: '<S304>/Enable'
     */
    if (pid_control_V1_B.b1 && (!pid_control_V1_DW.Hugws_MODE)) {
      (void) memset(&(pid_control_V1_XDis.ug_p_CSTATE), 0,
                    2*sizeof(boolean_T));

      /* InitializeConditions for Integrator: '<S304>/ug_p' */
      pid_control_V1_X.ug_p_CSTATE[0] = 0.0;
      pid_control_V1_X.ug_p_CSTATE[1] = 0.0;
      pid_control_V1_DW.Hugws_MODE = true;
    }

    /* End of Outputs for SubSystem: '<S291>/Hugw(s)' */
  }

  /* Outputs for Enabled SubSystem: '<S291>/Hugw(s)' incorporates:
   *  EnablePort: '<S304>/Enable'
   */
  if (pid_control_V1_DW.Hugws_MODE) {
    /* Product: '<S304>/Lug//V' */
    pid_control_V1_B.frac[0] = pid_control_V1_B.sr / pid_control_V1_B.sy;
    pid_control_V1_B.frac[1] = pid_control_V1_ConstB.UnitConversion_c /
      pid_control_V1_B.sy;

    /* Sqrt: '<S304>/sqrt' incorporates:
     *  Gain: '<S304>/(2//pi)'
     *  Integrator: '<S304>/ug_p'
     *  Product: '<S304>/Lug//V1'
     */
    tmp = _mm_div_pd(_mm_sub_pd(_mm_mul_pd(_mm_set_pd(sqrt(0.63661977236758138 *
      pid_control_V1_B.frac[1]), sqrt(0.63661977236758138 *
      pid_control_V1_B.frac[0])), _mm_set1_pd(pid_control_V1_B.Product[0])),
      _mm_loadu_pd(&pid_control_V1_X.ug_p_CSTATE[0])), _mm_loadu_pd
                     (&pid_control_V1_B.frac[0]));

    /* Product: '<S304>/w' */
    _mm_storeu_pd(&pid_control_V1_B.w_n[0], tmp);

    /* Integrator: '<S304>/ug_p' incorporates:
     *  Product: '<S304>/w1'
     */
    tmp = _mm_mul_pd(_mm_loadu_pd(&pid_control_V1_X.ug_p_CSTATE[0]), _mm_set_pd
                     (pid_control_V1_B.phi_lim, pid_control_V1_B.cp));

    /* Product: '<S304>/w1' */
    _mm_storeu_pd(&pid_control_V1_B.w1_c[0], tmp);
  }

  /* End of Outputs for SubSystem: '<S291>/Hugw(s)' */

  /* Gain: '<S297>/Lv' */
  pid_control_V1_B.frac[1] = pid_control_V1_ConstB.UnitConversion_c;

  /* Outputs for Enabled SubSystem: '<S291>/Hvgw(s)' incorporates:
   *  EnablePort: '<S305>/Enable'
   */
  if (pid_control_V1_B.b && pid_control_V1_B.b1 &&
      (!pid_control_V1_DW.Hvgws_MODE)) {
    (void) memset(&(pid_control_V1_XDis.vg_p1_CSTATE), 0,
                  4*sizeof(boolean_T));

    /* InitializeConditions for Integrator: '<S305>/vg_p1' */
    pid_control_V1_X.vg_p1_CSTATE[0] = 0.0;

    /* InitializeConditions for Integrator: '<S305>/vgw_p2' */
    pid_control_V1_X.vgw_p2_CSTATE[0] = 0.0;

    /* InitializeConditions for Integrator: '<S305>/vg_p1' */
    pid_control_V1_X.vg_p1_CSTATE[1] = 0.0;

    /* InitializeConditions for Integrator: '<S305>/vgw_p2' */
    pid_control_V1_X.vgw_p2_CSTATE[1] = 0.0;
    pid_control_V1_DW.Hvgws_MODE = true;
  }

  if (pid_control_V1_DW.Hvgws_MODE) {
    /* Product: '<S305>/Lvg//V' incorporates:
     *  Gain: '<S297>/Lv'
     */
    pid_control_V1_B.sr /= pid_control_V1_B.sy;

    /* Product: '<S305>/w' incorporates:
     *  Gain: '<S305>/(1//pi)'
     *  Integrator: '<S305>/vg_p1'
     *  Product: '<S305>/Lug//V1'
     *  Sqrt: '<S305>/sqrt'
     *  Sum: '<S305>/Sum'
     */
    pid_control_V1_B.sp = (sqrt(0.31830988618379069 * pid_control_V1_B.sr) *
      pid_control_V1_B.Product[1] - pid_control_V1_X.vg_p1_CSTATE[0]) /
      pid_control_V1_B.sr;
    pid_control_V1_B.w_g[0] = pid_control_V1_B.sp;

    /* Product: '<S305>/w ' incorporates:
     *  Gain: '<S305>/sqrt(3)'
     *  Integrator: '<S305>/vg_p1'
     *  Integrator: '<S305>/vgw_p2'
     *  Product: '<S305>/Lvg//V '
     *  Sum: '<S305>/Sum1'
     */
    pid_control_V1_B.w_e[0] = (pid_control_V1_B.sp * pid_control_V1_B.sr *
      1.7320508075688772 + (pid_control_V1_X.vg_p1_CSTATE[0] -
      pid_control_V1_X.vgw_p2_CSTATE[0])) / pid_control_V1_B.sr;

    /* Product: '<S305>/Lvg//V' */
    pid_control_V1_B.sr = pid_control_V1_B.frac[1] / pid_control_V1_B.sy;

    /* Product: '<S305>/w' incorporates:
     *  Gain: '<S305>/(1//pi)'
     *  Integrator: '<S305>/vg_p1'
     *  Product: '<S305>/Lug//V1'
     *  Sqrt: '<S305>/sqrt'
     *  Sum: '<S305>/Sum'
     */
    pid_control_V1_B.sp = (sqrt(0.31830988618379069 * pid_control_V1_B.sr) *
      pid_control_V1_B.Product[1] - pid_control_V1_X.vg_p1_CSTATE[1]) /
      pid_control_V1_B.sr;
    pid_control_V1_B.w_g[1] = pid_control_V1_B.sp;

    /* Product: '<S305>/w ' incorporates:
     *  Gain: '<S305>/sqrt(3)'
     *  Integrator: '<S305>/vg_p1'
     *  Integrator: '<S305>/vgw_p2'
     *  Product: '<S305>/Lvg//V '
     *  Sum: '<S305>/Sum1'
     */
    pid_control_V1_B.w_e[1] = (pid_control_V1_B.sp * pid_control_V1_B.sr *
      1.7320508075688772 + (pid_control_V1_X.vg_p1_CSTATE[1] -
      pid_control_V1_X.vgw_p2_CSTATE[1])) / pid_control_V1_B.sr;

    /* Product: '<S305>/w 1' incorporates:
     *  Integrator: '<S305>/vgw_p2'
     */
    tmp = _mm_mul_pd(_mm_set_pd(pid_control_V1_B.phi_lim, pid_control_V1_B.cp),
                     _mm_loadu_pd(&pid_control_V1_X.vgw_p2_CSTATE[0]));

    /* Product: '<S305>/w 1' */
    _mm_storeu_pd(&pid_control_V1_B.w1[0], tmp);
  }

  /* End of Outputs for SubSystem: '<S291>/Hvgw(s)' */

  /* Gain: '<S297>/Lw' */
  pid_control_V1_B.frac[1] = pid_control_V1_ConstB.UnitConversion_c;

  /* Outputs for Enabled SubSystem: '<S291>/Hwgw(s)' incorporates:
   *  EnablePort: '<S306>/Enable'
   */
  if (pid_control_V1_B.b && pid_control_V1_B.b1 &&
      (!pid_control_V1_DW.Hwgws_MODE)) {
    (void) memset(&(pid_control_V1_XDis.wg_p1_CSTATE), 0,
                  4*sizeof(boolean_T));

    /* InitializeConditions for Integrator: '<S306>/wg_p1' */
    pid_control_V1_X.wg_p1_CSTATE[0] = 0.0;

    /* InitializeConditions for Integrator: '<S306>/wg_p2' */
    pid_control_V1_X.wg_p2_CSTATE[0] = 0.0;

    /* InitializeConditions for Integrator: '<S306>/wg_p1' */
    pid_control_V1_X.wg_p1_CSTATE[1] = 0.0;

    /* InitializeConditions for Integrator: '<S306>/wg_p2' */
    pid_control_V1_X.wg_p2_CSTATE[1] = 0.0;
    pid_control_V1_DW.Hwgws_MODE = true;
  }

  if (pid_control_V1_DW.Hwgws_MODE) {
    /* Product: '<S306>/Lwg//V' incorporates:
     *  Gain: '<S297>/Lw'
     */
    pid_control_V1_B.cp = pid_control_V1_B.factor_bloqueo / pid_control_V1_B.sy;

    /* Product: '<S306>/w' incorporates:
     *  Gain: '<S306>/1//pi'
     *  Integrator: '<S306>/wg_p1'
     *  Product: '<S306>/Lug//V1'
     *  Sqrt: '<S306>/sqrt1'
     *  Sum: '<S306>/Sum'
     */
    pid_control_V1_B.sr = (sqrt(0.31830988618379069 * pid_control_V1_B.cp) *
      pid_control_V1_B.Product[2] - pid_control_V1_X.wg_p1_CSTATE[0]) /
      pid_control_V1_B.cp;
    pid_control_V1_B.w[0] = pid_control_V1_B.sr;

    /* Product: '<S306>/w ' incorporates:
     *  Integrator: '<S306>/wg_p1'
     *  Integrator: '<S306>/wg_p2'
     *  Product: '<S306>/Lwg//V'
     *  Product: '<S306>/Lwg//V '
     *  Sum: '<S306>/Sum1'
     */
    pid_control_V1_B.w_a[0] = (pid_control_V1_B.sr *
      pid_control_V1_ConstB.sqrt_a * pid_control_V1_B.cp +
      (pid_control_V1_X.wg_p1_CSTATE[0] - pid_control_V1_X.wg_p2_CSTATE[0])) /
      pid_control_V1_B.cp;

    /* Product: '<S306>/Lwg//V' */
    pid_control_V1_B.cp = pid_control_V1_B.frac[1] / pid_control_V1_B.sy;

    /* Product: '<S306>/w' incorporates:
     *  Gain: '<S306>/1//pi'
     *  Integrator: '<S306>/wg_p1'
     *  Product: '<S306>/Lug//V1'
     *  Sqrt: '<S306>/sqrt1'
     *  Sum: '<S306>/Sum'
     */
    pid_control_V1_B.sr = (sqrt(0.31830988618379069 * pid_control_V1_B.cp) *
      pid_control_V1_B.Product[2] - pid_control_V1_X.wg_p1_CSTATE[1]) /
      pid_control_V1_B.cp;
    pid_control_V1_B.w[1] = pid_control_V1_B.sr;

    /* Product: '<S306>/w ' incorporates:
     *  Integrator: '<S306>/wg_p1'
     *  Integrator: '<S306>/wg_p2'
     *  Product: '<S306>/Lwg//V'
     *  Product: '<S306>/Lwg//V '
     *  Sum: '<S306>/Sum1'
     */
    pid_control_V1_B.w_a[1] = (pid_control_V1_B.sr *
      pid_control_V1_ConstB.sqrt_a * pid_control_V1_B.cp +
      (pid_control_V1_X.wg_p1_CSTATE[1] - pid_control_V1_X.wg_p2_CSTATE[1])) /
      pid_control_V1_B.cp;

    /* Product: '<S306>/Lwg//V 1' incorporates:
     *  Integrator: '<S306>/wg_p2'
     */
    tmp = _mm_mul_pd(_mm_set_pd(pid_control_V1_B.phi_lim,
      pid_control_V1_ConstB.sigma_wg), _mm_loadu_pd
                     (&pid_control_V1_X.wg_p2_CSTATE[0]));

    /* Product: '<S306>/Lwg//V 1' */
    _mm_storeu_pd(&pid_control_V1_B.LwgV1[0], tmp);
  }

  /* End of Outputs for SubSystem: '<S291>/Hwgw(s)' */

  /* Angle2Dcm: '<S12>/Rotation Angles to Direction Cosine Matrix' */
  pid_control_V1_B.cp = cos(pid_control_V1_B.x[6]);
  pid_control_V1_B.sr = sin(pid_control_V1_B.x[6]);
  pid_control_V1_B.sp = -sin(pid_control_V1_B.x[6]);
  pid_control_V1_B.cr = cos(pid_control_V1_B.x[6]);
  pid_control_V1_B.Dtot = cos(pid_control_V1_B.x[7]);
  pid_control_V1_B.FA_b_tmp = -sin(pid_control_V1_B.x[7]);
  pid_control_V1_B.beta = sin(pid_control_V1_B.x[7]);
  pid_control_V1_B.hh = cos(pid_control_V1_B.x[7]);
  pid_control_V1_B.hw = cos(pid_control_V1_B.x[8]);
  pid_control_V1_B.CL_w_IGE = sin(pid_control_V1_B.x[8]);
  pid_control_V1_B.CL_h_IGE = -sin(pid_control_V1_B.x[8]);
  pid_control_V1_B.Ltot_tmp = cos(pid_control_V1_B.x[8]);
  pid_control_V1_B.CD_iw_IGE = 0.0 * pid_control_V1_B.beta +
    pid_control_V1_B.Dtot;
  pid_control_V1_B.CD_ih_IGE = 0.0 * pid_control_V1_B.hh +
    pid_control_V1_B.FA_b_tmp;
  pid_control_V1_B.Ltot = pid_control_V1_B.hw * 0.0;
  pid_control_V1_B.CQ = 0.0 * pid_control_V1_B.Dtot;
  pid_control_V1_B.Dtot = (pid_control_V1_B.CQ + pid_control_V1_B.Ltot) +
    pid_control_V1_B.CL_w_IGE * pid_control_V1_B.beta;
  pid_control_V1_B.hw += pid_control_V1_B.CL_w_IGE * 0.0;
  pid_control_V1_B.FA_b_tmp *= 0.0;
  pid_control_V1_B.CL_w_IGE = (pid_control_V1_B.FA_b_tmp + pid_control_V1_B.Ltot)
    + pid_control_V1_B.CL_w_IGE * pid_control_V1_B.hh;
  pid_control_V1_B.Ltot = pid_control_V1_B.CL_h_IGE * 0.0;
  pid_control_V1_B.beta = (pid_control_V1_B.CQ + pid_control_V1_B.Ltot) +
    pid_control_V1_B.beta * pid_control_V1_B.Ltot_tmp;
  pid_control_V1_B.CL_h_IGE += pid_control_V1_B.Ltot_tmp * 0.0;
  pid_control_V1_B.hh = (pid_control_V1_B.FA_b_tmp + pid_control_V1_B.Ltot) +
    pid_control_V1_B.Ltot_tmp * pid_control_V1_B.hh;
  pid_control_V1_B.Ltot_tmp = pid_control_V1_B.CD_ih_IGE * 0.0;
  pid_control_V1_B.RotationAnglestoDirectionCo[0] = (pid_control_V1_B.CD_iw_IGE *
    pid_control_V1_B.cp + 0.0 * pid_control_V1_B.sp) + pid_control_V1_B.Ltot_tmp;
  pid_control_V1_B.Ltot = pid_control_V1_B.CL_w_IGE * 0.0;
  pid_control_V1_B.RotationAnglestoDirectionCo[1] = (pid_control_V1_B.cp *
    pid_control_V1_B.Dtot + pid_control_V1_B.sp * pid_control_V1_B.hw) +
    pid_control_V1_B.Ltot;
  pid_control_V1_B.CQ = pid_control_V1_B.hh * 0.0;
  pid_control_V1_B.RotationAnglestoDirectionCo[2] = (pid_control_V1_B.cp *
    pid_control_V1_B.beta + pid_control_V1_B.sp * pid_control_V1_B.CL_h_IGE) +
    pid_control_V1_B.CQ;
  pid_control_V1_B.RotationAnglestoDirectionCo[3] = (pid_control_V1_B.CD_iw_IGE *
    pid_control_V1_B.sr + 0.0 * pid_control_V1_B.cr) + pid_control_V1_B.Ltot_tmp;
  pid_control_V1_B.RotationAnglestoDirectionCo[4] = (pid_control_V1_B.sr *
    pid_control_V1_B.Dtot + pid_control_V1_B.hw * pid_control_V1_B.cr) +
    pid_control_V1_B.Ltot;
  pid_control_V1_B.RotationAnglestoDirectionCo[5] = (pid_control_V1_B.sr *
    pid_control_V1_B.beta + pid_control_V1_B.cr * pid_control_V1_B.CL_h_IGE) +
    pid_control_V1_B.CQ;
  pid_control_V1_B.RotationAnglestoDirectionCo[6] = pid_control_V1_B.CD_iw_IGE *
    0.0 + pid_control_V1_B.CD_ih_IGE;
  pid_control_V1_B.RotationAnglestoDirectionCo[7] = (pid_control_V1_B.Dtot * 0.0
    + pid_control_V1_B.hw * 0.0) + pid_control_V1_B.CL_w_IGE;
  pid_control_V1_B.RotationAnglestoDirectionCo[8] = (pid_control_V1_B.beta * 0.0
    + pid_control_V1_B.CL_h_IGE * 0.0) + pid_control_V1_B.hh;

  /* If: '<S296>/if Height < Max low altitude  elseif Height > Min isotropic altitude ' incorporates:
   *  Constant: '<S317>/max_height_low'
   *  Product: '<S317>/Product1'
   *  Product: '<S322>/Product1'
   *  Product: '<S322>/Product2'
   *  Product: '<S324>/Product1'
   *  Product: '<S324>/Product2'
   *  Sum: '<S317>/Sum1'
   *  Sum: '<S317>/Sum2'
   *  Sum: '<S317>/Sum3'
   *  Sum: '<S322>/Sum'
   *  Sum: '<S324>/Sum'
   */
  pid_control_V1_B.rtPrevAction =
    pid_control_V1_DW.ifHeightMaxlowaltitudeelseifHei;
  if (pid_control_V1_B.b1) {
    if (pid_control_V1_B.cy <= 1000.0) {
      pid_control_V1_B.rtAction = 0;
    } else if (pid_control_V1_B.cy >= 2000.0) {
      pid_control_V1_B.rtAction = 1;
    } else {
      pid_control_V1_B.rtAction = 2;
    }

    pid_control_V1_DW.ifHeightMaxlowaltitudeelseifHei =
      pid_control_V1_B.rtAction;
  } else {
    pid_control_V1_B.rtAction =
      pid_control_V1_DW.ifHeightMaxlowaltitudeelseifHei;
  }

  if (pid_control_V1_B.rtPrevAction != pid_control_V1_B.rtAction) {
    rtsiSetBlockStateForSolverChangedAtMajorStep(&(&pid_control_V1_M)
      ->solverInfo, true);
  }

  switch (pid_control_V1_B.rtAction) {
   case 0:
    /* Outputs for IfAction SubSystem: '<S296>/Low altitude  velocities' incorporates:
     *  ActionPort: '<S318>/Action Port'
     */
    /* SignalConversion generated from: '<S323>/Vector Concatenate' */
    pid_control_V1_B.IMU1_o2[2] = pid_control_V1_B.LwgV1[0];

    /* Trigonometry: '<S324>/Trigonometric Function' incorporates:
     *  UnitConversion: '<S289>/Unit Conversion'
     */
    pid_control_V1_B.cp = sin(pid_control_V1_ConstB.UnitConversion);
    pid_control_V1_B.sr = cos(pid_control_V1_ConstB.UnitConversion);
    _mm_storeu_pd(&pid_control_V1_B.IMU1_o2[0], _mm_add_pd(_mm_mul_pd(_mm_set_pd
      (pid_control_V1_B.cp, pid_control_V1_B.w1_c[0]), _mm_set_pd
      (pid_control_V1_B.w1_c[0], pid_control_V1_B.sr)), _mm_mul_pd(_mm_mul_pd
      (_mm_set_pd(pid_control_V1_B.w1[0], pid_control_V1_B.cp), _mm_set_pd
       (pid_control_V1_B.sr, pid_control_V1_B.w1[0])), _mm_set_pd(1.0, -1.0))));

    /* Product: '<S323>/Product' incorporates:
     *  Angle2Dcm: '<S12>/Rotation Angles to Direction Cosine Matrix'
     *  Concatenate: '<S323>/Vector Concatenate'
     *  Product: '<S324>/Product1'
     *  Product: '<S324>/Product2'
     *  Reshape: '<S323>/Reshape1'
     *  Sum: '<S324>/Sum'
     */
    pid_control_V1_B.cp = 0.0;
    pid_control_V1_B.sr = 0.0;
    pid_control_V1_B.sp = 0.0;
    for (pid_control_V1_B.i = 0; pid_control_V1_B.i < 3; pid_control_V1_B.i++) {
      tmp = _mm_add_pd(_mm_mul_pd(_mm_loadu_pd
        (&pid_control_V1_B.RotationAnglestoDirectionCo[3 * pid_control_V1_B.i]),
        _mm_set1_pd(pid_control_V1_B.IMU1_o2[pid_control_V1_B.i])), _mm_set_pd
                       (pid_control_V1_B.sr, pid_control_V1_B.cp));
      _mm_storeu_pd(&pid_control_V1_B.dv1[0], tmp);
      pid_control_V1_B.cp = pid_control_V1_B.dv1[0];
      pid_control_V1_B.sr = pid_control_V1_B.dv1[1];
      pid_control_V1_B.sp += pid_control_V1_B.RotationAnglestoDirectionCo[3 *
        pid_control_V1_B.i + 2] * pid_control_V1_B.IMU1_o2[pid_control_V1_B.i];
    }

    pid_control_V1_B.IMU1_o1[2] = pid_control_V1_B.sp;
    pid_control_V1_B.IMU1_o1[1] = pid_control_V1_B.sr;
    pid_control_V1_B.IMU1_o1[0] = pid_control_V1_B.cp;

    /* End of Product: '<S323>/Product' */
    /* End of Outputs for SubSystem: '<S296>/Low altitude  velocities' */
    break;

   case 1:
    /* Outputs for IfAction SubSystem: '<S296>/Medium//High  altitude velocities' incorporates:
     *  ActionPort: '<S319>/Action Port'
     */
    /* Gain: '<S319>/Gain' */
    pid_control_V1_B.IMU1_o1[0] = pid_control_V1_B.w1_c[1];
    pid_control_V1_B.IMU1_o1[1] = pid_control_V1_B.w1[1];
    pid_control_V1_B.IMU1_o1[2] = pid_control_V1_B.LwgV1[1];

    /* End of Outputs for SubSystem: '<S296>/Medium//High  altitude velocities' */
    break;

   default:
    /* Outputs for IfAction SubSystem: '<S296>/Interpolate  velocities' incorporates:
     *  ActionPort: '<S317>/Action Port'
     */
    /* Trigonometry: '<S322>/Trigonometric Function' incorporates:
     *  UnitConversion: '<S289>/Unit Conversion'
     */
    pid_control_V1_B.cp = sin(pid_control_V1_ConstB.UnitConversion);
    pid_control_V1_B.sr = cos(pid_control_V1_ConstB.UnitConversion);
    _mm_storeu_pd(&pid_control_V1_B.IMU1_o1[0], _mm_add_pd(_mm_mul_pd(_mm_set_pd
      (pid_control_V1_B.cp, pid_control_V1_B.w1_c[0]), _mm_set_pd
      (pid_control_V1_B.w1_c[0], pid_control_V1_B.sr)), _mm_mul_pd(_mm_mul_pd
      (_mm_set_pd(pid_control_V1_B.w1[0], pid_control_V1_B.cp), _mm_set_pd
       (pid_control_V1_B.sr, pid_control_V1_B.w1[0])), _mm_set_pd(1.0, -1.0))));

    /* SignalConversion generated from: '<S321>/Vector Concatenate' incorporates:
     *  Product: '<S322>/Product1'
     *  Product: '<S322>/Product2'
     *  Sum: '<S322>/Sum'
     */
    pid_control_V1_B.IMU1_o1[2] = pid_control_V1_B.LwgV1[0];

    /* Product: '<S321>/Product' incorporates:
     *  Angle2Dcm: '<S12>/Rotation Angles to Direction Cosine Matrix'
     *  Concatenate: '<S321>/Vector Concatenate'
     */
    pid_control_V1_B.cp = 0.0;
    pid_control_V1_B.sr = 0.0;
    pid_control_V1_B.sp = 0.0;
    for (pid_control_V1_B.i = 0; pid_control_V1_B.i < 3; pid_control_V1_B.i++) {
      tmp = _mm_add_pd(_mm_mul_pd(_mm_loadu_pd
        (&pid_control_V1_B.RotationAnglestoDirectionCo[3 * pid_control_V1_B.i]),
        _mm_set1_pd(pid_control_V1_B.IMU1_o1[pid_control_V1_B.i])), _mm_set_pd
                       (pid_control_V1_B.sr, pid_control_V1_B.cp));
      _mm_storeu_pd(&pid_control_V1_B.dv1[0], tmp);
      pid_control_V1_B.cp = pid_control_V1_B.dv1[0];
      pid_control_V1_B.sr = pid_control_V1_B.dv1[1];
      pid_control_V1_B.sp += pid_control_V1_B.RotationAnglestoDirectionCo[3 *
        pid_control_V1_B.i + 2] * pid_control_V1_B.IMU1_o1[pid_control_V1_B.i];
    }

    pid_control_V1_B.IMU1_o2[2] = pid_control_V1_B.sp;
    pid_control_V1_B.IMU1_o2[1] = pid_control_V1_B.sr;
    pid_control_V1_B.IMU1_o2[0] = pid_control_V1_B.cp;

    /* End of Product: '<S321>/Product' */
    tmp = _mm_add_pd(_mm_div_pd(_mm_mul_pd(_mm_sub_pd(_mm_set_pd
      (pid_control_V1_B.w1[1], pid_control_V1_B.w1_c[1]), _mm_loadu_pd
      (&pid_control_V1_B.IMU1_o2[0])), _mm_sub_pd(_mm_set1_pd
      (pid_control_V1_B.cy), _mm_set1_pd(1000.0))), _mm_set1_pd
      (pid_control_V1_ConstB.Sum)), _mm_loadu_pd(&pid_control_V1_B.IMU1_o2[0]));
    _mm_storeu_pd(&pid_control_V1_B.IMU1_o1[0], tmp);

    /* Sum: '<S317>/Sum3' incorporates:
     *  Constant: '<S317>/max_height_low'
     *  Product: '<S317>/Product1'
     *  Sum: '<S317>/Sum1'
     *  Sum: '<S317>/Sum2'
     */
    pid_control_V1_B.IMU1_o1[2] = (pid_control_V1_B.LwgV1[1] -
      pid_control_V1_B.IMU1_o2[2]) * (pid_control_V1_B.cy - 1000.0) /
      pid_control_V1_ConstB.Sum + pid_control_V1_B.IMU1_o2[2];

    /* End of Outputs for SubSystem: '<S296>/Interpolate  velocities' */
    break;
  }

  /* UnitConversion: '<S282>/Unit Conversion' */
  /* Unit Conversion - from: ft/s to: m/s
     Expression: output = (0.3048*input) + (0) */
  tmp = _mm_mul_pd(_mm_set1_pd(0.3048), _mm_loadu_pd(&pid_control_V1_B.IMU1_o1[0]));
  _mm_storeu_pd(&pid_control_V1_B.IMU1_o1[0], tmp);
  pid_control_V1_B.IMU1_o1[2] *= 0.3048;
  if (pid_control_V1_B.b) {
    /* MATLABSystem: '<S288>/SourceBlock' */
    pid_control_V1_B.SourceBlock_o1_h = Sub_pid_control_V1_417.getLatestMessage(
      &pid_control_V1_B.SourceBlock_o2_j);

    /* Outputs for Enabled SubSystem: '<S288>/Enabled Subsystem' */
    pid_control_V1_EnabledSubsystem(pid_control_V1_B.SourceBlock_o1_h,
      &pid_control_V1_B.SourceBlock_o2_j, &pid_control_V1_B.EnabledSubsystem_pt);

    /* End of Outputs for SubSystem: '<S288>/Enabled Subsystem' */
  }

  /* Switch: '<S12>/Switch' */
  if (pid_control_V1_B.EnabledSubsystem_pt.In1.data) {
    /* Switch: '<S12>/Switch' */
    pid_control_V1_B.Switch_p[0] = pid_control_V1_B.IMU1_o1[0];
    pid_control_V1_B.Switch_p[1] = pid_control_V1_B.IMU1_o1[1];
    pid_control_V1_B.Switch_p[2] = pid_control_V1_B.IMU1_o1[2];
  } else {
    /* Switch: '<S12>/Switch' incorporates:
     *  Constant: '<S12>/Constant'
     */
    pid_control_V1_B.Switch_p[0] = 0.0;
    pid_control_V1_B.Switch_p[1] = 0.0;
    pid_control_V1_B.Switch_p[2] = 0.0;
  }

  /* End of Switch: '<S12>/Switch' */

  /* Outputs for Enabled SubSystem: '<S290>/Hpgw' incorporates:
   *  EnablePort: '<S301>/Enable'
   */
  if (pid_control_V1_B.b && pid_control_V1_B.b1 && (!pid_control_V1_DW.Hpgw_MODE))
  {
    (void) memset(&(pid_control_V1_XDis.pgw_p_CSTATE), 0,
                  2*sizeof(boolean_T));

    /* InitializeConditions for Integrator: '<S301>/pgw_p' */
    pid_control_V1_X.pgw_p_CSTATE[0] = 0.0;
    pid_control_V1_X.pgw_p_CSTATE[1] = 0.0;
    pid_control_V1_DW.Hpgw_MODE = true;
  }

  if (pid_control_V1_DW.Hpgw_MODE) {
    /* Fcn: '<S301>/sqrt(0.8//V)' */
    pid_control_V1_B.cr = 0.8 / pid_control_V1_B.sy;
    if (pid_control_V1_B.cr < 0.0) {
      pid_control_V1_B.cr = -sqrt(-pid_control_V1_B.cr);
    } else {
      pid_control_V1_B.cr = sqrt(pid_control_V1_B.cr);
    }

    /* Product: '<S301>/w3' */
    pid_control_V1_B.cp = pid_control_V1_B.sy * pid_control_V1_ConstB.w4;

    /* Product: '<S301>/w' incorporates:
     *  Fcn: '<S301>/sqrt(0.8//V)'
     *  Gain: '<S297>/Lw'
     *  Integrator: '<S301>/pgw_p'
     *  Math: '<S301>/L^1//3'
     *  Product: '<S301>/Lug//V1'
     *  Product: '<S301>/w1'
     *  Product: '<S301>/w2'
     *  Sum: '<S301>/Sum'
     */
    pid_control_V1_B.w_o[0] = (pid_control_V1_B.cr / rt_powd_snf
      (pid_control_V1_B.factor_bloqueo, 0.33333333333333331) *
      pid_control_V1_ConstB.u16 * pid_control_V1_B.Product[3] -
      pid_control_V1_X.pgw_p_CSTATE[0]) * pid_control_V1_B.cp;

    /* Math: '<S301>/L^1//3' */
    if (pid_control_V1_B.frac[1] < 0.0) {
      pid_control_V1_B.factor_bloqueo = -rt_powd_snf(-pid_control_V1_B.frac[1],
        0.33333333333333331);
    } else {
      pid_control_V1_B.factor_bloqueo = rt_powd_snf(pid_control_V1_B.frac[1],
        0.33333333333333331);
    }

    /* Product: '<S301>/w' incorporates:
     *  Fcn: '<S301>/sqrt(0.8//V)'
     *  Integrator: '<S301>/pgw_p'
     *  Math: '<S301>/L^1//3'
     *  Product: '<S301>/Lug//V1'
     *  Product: '<S301>/w1'
     *  Product: '<S301>/w2'
     *  Sum: '<S301>/Sum'
     */
    pid_control_V1_B.w_o[1] = (pid_control_V1_B.cr /
      pid_control_V1_B.factor_bloqueo * pid_control_V1_ConstB.u16 *
      pid_control_V1_B.Product[3] - pid_control_V1_X.pgw_p_CSTATE[1]) *
      pid_control_V1_B.cp;

    /* Product: '<S301>/sigma_w' incorporates:
     *  Integrator: '<S301>/pgw_p'
     */
    tmp = _mm_mul_pd(_mm_set_pd(pid_control_V1_B.phi_lim,
      pid_control_V1_ConstB.sigma_wg), _mm_loadu_pd
                     (&pid_control_V1_X.pgw_p_CSTATE[0]));

    /* Product: '<S301>/sigma_w' */
    _mm_storeu_pd(&pid_control_V1_B.sigma_w[0], tmp);
  }

  /* End of Outputs for SubSystem: '<S290>/Hpgw' */

  /* Outputs for Enabled SubSystem: '<S290>/Hqgw' incorporates:
   *  EnablePort: '<S302>/Enable'
   */
  if (pid_control_V1_B.b && pid_control_V1_B.b1 && (!pid_control_V1_DW.Hqgw_MODE))
  {
    (void) memset(&(pid_control_V1_XDis.qgw_p_CSTATE), 0,
                  2*sizeof(boolean_T));

    /* InitializeConditions for Integrator: '<S302>/qgw_p' */
    pid_control_V1_X.qgw_p_CSTATE[0] = 0.0;
    pid_control_V1_X.qgw_p_CSTATE[1] = 0.0;
    pid_control_V1_DW.Hqgw_MODE = true;
  }

  if (pid_control_V1_DW.Hqgw_MODE) {
    /* Gain: '<S302>/pi//4' */
    pid_control_V1_B.factor_bloqueo = 0.78539816339744828 * pid_control_V1_B.sy;

    /* Product: '<S302>/w' incorporates:
     *  Integrator: '<S302>/qgw_p'
     *  Product: '<S302>/wg//V'
     *  Sum: '<S302>/Sum'
     */
    pid_control_V1_B.phi_lim = (pid_control_V1_B.LwgV1[0] / pid_control_V1_B.sy
      - pid_control_V1_X.qgw_p_CSTATE[0]) * (pid_control_V1_B.factor_bloqueo /
      pid_control_V1_ConstB.UnitConversion_n);
    pid_control_V1_B.w_e0[0] = pid_control_V1_B.phi_lim;

    /* UnaryMinus: '<S302>/Unary Minus' */
    pid_control_V1_B.UnaryMinus[0] = -pid_control_V1_B.phi_lim;

    /* Product: '<S302>/w' incorporates:
     *  Integrator: '<S302>/qgw_p'
     *  Product: '<S302>/wg//V'
     *  Sum: '<S302>/Sum'
     */
    pid_control_V1_B.phi_lim = (pid_control_V1_B.LwgV1[1] / pid_control_V1_B.sy
      - pid_control_V1_X.qgw_p_CSTATE[1]) * (pid_control_V1_B.factor_bloqueo /
      pid_control_V1_ConstB.UnitConversion_n);
    pid_control_V1_B.w_e0[1] = pid_control_V1_B.phi_lim;

    /* UnaryMinus: '<S302>/Unary Minus' */
    pid_control_V1_B.UnaryMinus[1] = -pid_control_V1_B.phi_lim;
  }

  /* End of Outputs for SubSystem: '<S290>/Hqgw' */

  /* Outputs for Enabled SubSystem: '<S290>/Hrgw' incorporates:
   *  EnablePort: '<S303>/Enable'
   */
  if (pid_control_V1_B.b && pid_control_V1_B.b1 && (!pid_control_V1_DW.Hrgw_MODE))
  {
    (void) memset(&(pid_control_V1_XDis.rgw_p_CSTATE), 0,
                  2*sizeof(boolean_T));

    /* InitializeConditions for Integrator: '<S303>/rgw_p' */
    pid_control_V1_X.rgw_p_CSTATE[0] = 0.0;
    pid_control_V1_X.rgw_p_CSTATE[1] = 0.0;
    pid_control_V1_DW.Hrgw_MODE = true;
  }

  if (pid_control_V1_DW.Hrgw_MODE) {
    /* Product: '<S303>/vg//V' incorporates:
     *  Gain: '<S303>/pi//3'
     *  Integrator: '<S303>/rgw_p'
     *  Product: '<S303>/w'
     */
    tmp = _mm_mul_pd(_mm_sub_pd(_mm_div_pd(_mm_loadu_pd(&pid_control_V1_B.w1[0]),
      _mm_set1_pd(pid_control_V1_B.sy)), _mm_loadu_pd
      (&pid_control_V1_X.rgw_p_CSTATE[0])), _mm_div_pd(_mm_set1_pd
      (1.0471975511965976 * pid_control_V1_B.sy), _mm_set1_pd
      (pid_control_V1_ConstB.UnitConversion_n)));

    /* Product: '<S303>/w' */
    _mm_storeu_pd(&pid_control_V1_B.w_d[0], tmp);
  }

  /* End of Outputs for SubSystem: '<S290>/Hrgw' */

  /* If: '<S295>/if Height < Max low altitude  elseif Height > Min isotropic altitude ' incorporates:
   *  Constant: '<S309>/max_height_low'
   *  Product: '<S309>/Product1'
   *  Product: '<S314>/Product1'
   *  Product: '<S314>/Product2'
   *  Product: '<S316>/Product1'
   *  Product: '<S316>/Product2'
   *  Sum: '<S309>/Sum1'
   *  Sum: '<S309>/Sum2'
   *  Sum: '<S309>/Sum3'
   *  Sum: '<S314>/Sum'
   *  Sum: '<S316>/Sum'
   */
  pid_control_V1_B.rtPrevAction =
    pid_control_V1_DW.ifHeightMaxlowaltitudeelseifH_a;
  if (pid_control_V1_B.b1) {
    if (pid_control_V1_B.cy <= 1000.0) {
      pid_control_V1_B.rtAction = 0;
    } else if (pid_control_V1_B.cy >= 2000.0) {
      pid_control_V1_B.rtAction = 1;
    } else {
      pid_control_V1_B.rtAction = 2;
    }

    pid_control_V1_DW.ifHeightMaxlowaltitudeelseifH_a =
      pid_control_V1_B.rtAction;
  } else {
    pid_control_V1_B.rtAction =
      pid_control_V1_DW.ifHeightMaxlowaltitudeelseifH_a;
  }

  if (pid_control_V1_B.rtPrevAction != pid_control_V1_B.rtAction) {
    rtsiSetBlockStateForSolverChangedAtMajorStep(&(&pid_control_V1_M)
      ->solverInfo, true);
  }

  switch (pid_control_V1_B.rtAction) {
   case 0:
    /* Outputs for IfAction SubSystem: '<S295>/Low altitude  rates' incorporates:
     *  ActionPort: '<S310>/Action Port'
     */
    /* SignalConversion generated from: '<S315>/Vector Concatenate' */
    pid_control_V1_B.IMU1_o2[2] = pid_control_V1_B.w_d[0];

    /* Trigonometry: '<S316>/Trigonometric Function1' incorporates:
     *  UnitConversion: '<S289>/Unit Conversion'
     */
    pid_control_V1_B.cy = sin(pid_control_V1_ConstB.UnitConversion);
    pid_control_V1_B.sy = cos(pid_control_V1_ConstB.UnitConversion);
    _mm_storeu_pd(&pid_control_V1_B.IMU1_o2[0], _mm_add_pd(_mm_mul_pd(_mm_set_pd
      (pid_control_V1_B.cy, pid_control_V1_B.sigma_w[0]), _mm_set_pd
      (pid_control_V1_B.sigma_w[0], pid_control_V1_B.sy)), _mm_mul_pd(_mm_mul_pd
      (_mm_set_pd(pid_control_V1_B.UnaryMinus[0], pid_control_V1_B.cy),
       _mm_set_pd(pid_control_V1_B.sy, pid_control_V1_B.UnaryMinus[0])),
      _mm_set_pd(1.0, -1.0))));

    /* Product: '<S315>/Product' incorporates:
     *  Angle2Dcm: '<S12>/Rotation Angles to Direction Cosine Matrix'
     *  Concatenate: '<S315>/Vector Concatenate'
     *  Product: '<S316>/Product1'
     *  Product: '<S316>/Product2'
     *  Reshape: '<S315>/Reshape1'
     *  Sum: '<S316>/Sum'
     */
    pid_control_V1_B.cp = 0.0;
    pid_control_V1_B.sr = 0.0;
    pid_control_V1_B.sp = 0.0;
    for (pid_control_V1_B.i = 0; pid_control_V1_B.i < 3; pid_control_V1_B.i++) {
      tmp = _mm_add_pd(_mm_mul_pd(_mm_loadu_pd
        (&pid_control_V1_B.RotationAnglestoDirectionCo[3 * pid_control_V1_B.i]),
        _mm_set1_pd(pid_control_V1_B.IMU1_o2[pid_control_V1_B.i])), _mm_set_pd
                       (pid_control_V1_B.sr, pid_control_V1_B.cp));
      _mm_storeu_pd(&pid_control_V1_B.dv1[0], tmp);
      pid_control_V1_B.cp = pid_control_V1_B.dv1[0];
      pid_control_V1_B.sr = pid_control_V1_B.dv1[1];
      pid_control_V1_B.sp += pid_control_V1_B.RotationAnglestoDirectionCo[3 *
        pid_control_V1_B.i + 2] * pid_control_V1_B.IMU1_o2[pid_control_V1_B.i];
    }

    pid_control_V1_B.IMU1_o1[2] = pid_control_V1_B.sp;
    pid_control_V1_B.IMU1_o1[1] = pid_control_V1_B.sr;
    pid_control_V1_B.IMU1_o1[0] = pid_control_V1_B.cp;

    /* End of Product: '<S315>/Product' */
    /* End of Outputs for SubSystem: '<S295>/Low altitude  rates' */
    break;

   case 1:
    /* Outputs for IfAction SubSystem: '<S295>/Medium//High  altitude rates' incorporates:
     *  ActionPort: '<S311>/Action Port'
     */
    /* Gain: '<S311>/Gain' */
    pid_control_V1_B.IMU1_o1[0] = pid_control_V1_B.sigma_w[1];
    pid_control_V1_B.IMU1_o1[1] = pid_control_V1_B.UnaryMinus[1];
    pid_control_V1_B.IMU1_o1[2] = pid_control_V1_B.w_d[1];

    /* End of Outputs for SubSystem: '<S295>/Medium//High  altitude rates' */
    break;

   default:
    /* Outputs for IfAction SubSystem: '<S295>/Interpolate  rates' incorporates:
     *  ActionPort: '<S309>/Action Port'
     */
    /* Trigonometry: '<S314>/Trigonometric Function' incorporates:
     *  UnitConversion: '<S289>/Unit Conversion'
     */
    pid_control_V1_B.sy = sin(pid_control_V1_ConstB.UnitConversion);
    pid_control_V1_B.factor_bloqueo = cos(pid_control_V1_ConstB.UnitConversion);
    _mm_storeu_pd(&pid_control_V1_B.IMU1_o1[0], _mm_add_pd(_mm_mul_pd(_mm_set_pd
      (pid_control_V1_B.sy, pid_control_V1_B.sigma_w[0]), _mm_set_pd
      (pid_control_V1_B.sigma_w[0], pid_control_V1_B.factor_bloqueo)),
      _mm_mul_pd(_mm_mul_pd(_mm_set_pd(pid_control_V1_B.UnaryMinus[0],
      pid_control_V1_B.sy), _mm_set_pd(pid_control_V1_B.factor_bloqueo,
      pid_control_V1_B.UnaryMinus[0])), _mm_set_pd(1.0, -1.0))));

    /* SignalConversion generated from: '<S313>/Vector Concatenate' incorporates:
     *  Product: '<S314>/Product1'
     *  Product: '<S314>/Product2'
     *  Sum: '<S314>/Sum'
     */
    pid_control_V1_B.IMU1_o1[2] = pid_control_V1_B.w_d[0];

    /* Product: '<S313>/Product' incorporates:
     *  Angle2Dcm: '<S12>/Rotation Angles to Direction Cosine Matrix'
     *  Concatenate: '<S313>/Vector Concatenate'
     */
    pid_control_V1_B.cp = 0.0;
    pid_control_V1_B.sr = 0.0;
    pid_control_V1_B.sp = 0.0;
    for (pid_control_V1_B.i = 0; pid_control_V1_B.i < 3; pid_control_V1_B.i++) {
      tmp = _mm_add_pd(_mm_mul_pd(_mm_loadu_pd
        (&pid_control_V1_B.RotationAnglestoDirectionCo[3 * pid_control_V1_B.i]),
        _mm_set1_pd(pid_control_V1_B.IMU1_o1[pid_control_V1_B.i])), _mm_set_pd
                       (pid_control_V1_B.sr, pid_control_V1_B.cp));
      _mm_storeu_pd(&pid_control_V1_B.dv1[0], tmp);
      pid_control_V1_B.cp = pid_control_V1_B.dv1[0];
      pid_control_V1_B.sr = pid_control_V1_B.dv1[1];
      pid_control_V1_B.sp += pid_control_V1_B.RotationAnglestoDirectionCo[3 *
        pid_control_V1_B.i + 2] * pid_control_V1_B.IMU1_o1[pid_control_V1_B.i];
    }

    pid_control_V1_B.IMU1_o2[2] = pid_control_V1_B.sp;
    pid_control_V1_B.IMU1_o2[1] = pid_control_V1_B.sr;
    pid_control_V1_B.IMU1_o2[0] = pid_control_V1_B.cp;

    /* End of Product: '<S313>/Product' */
    tmp = _mm_add_pd(_mm_div_pd(_mm_mul_pd(_mm_sub_pd(_mm_set_pd
      (pid_control_V1_B.UnaryMinus[1], pid_control_V1_B.sigma_w[1]),
      _mm_loadu_pd(&pid_control_V1_B.IMU1_o2[0])), _mm_sub_pd(_mm_set1_pd
      (pid_control_V1_B.cy), _mm_set1_pd(1000.0))), _mm_set1_pd
      (pid_control_V1_ConstB.Sum_a)), _mm_loadu_pd(&pid_control_V1_B.IMU1_o2[0]));
    _mm_storeu_pd(&pid_control_V1_B.IMU1_o1[0], tmp);

    /* Sum: '<S309>/Sum3' incorporates:
     *  Constant: '<S309>/max_height_low'
     *  Product: '<S309>/Product1'
     *  Sum: '<S309>/Sum1'
     *  Sum: '<S309>/Sum2'
     */
    pid_control_V1_B.IMU1_o1[2] = (pid_control_V1_B.w_d[1] -
      pid_control_V1_B.IMU1_o2[2]) * (pid_control_V1_B.cy - 1000.0) /
      pid_control_V1_ConstB.Sum_a + pid_control_V1_B.IMU1_o2[2];

    /* End of Outputs for SubSystem: '<S295>/Interpolate  rates' */
    break;
  }

  if (pid_control_V1_B.b) {
    /* MATLABSystem: '<S285>/SourceBlock' */
    pid_control_V1_B.SourceBlock_o1_k = Sub_pid_control_V1_423.getLatestMessage(
      &pid_control_V1_B.SourceBlock_o2_dd);

    /* Outputs for Enabled SubSystem: '<S285>/Enabled Subsystem' */
    pid_control_V1_EnabledSubsystem(pid_control_V1_B.SourceBlock_o1_k,
      &pid_control_V1_B.SourceBlock_o2_dd, &pid_control_V1_B.EnabledSubsystem);

    /* End of Outputs for SubSystem: '<S285>/Enabled Subsystem' */
  }

  /* Switch: '<S12>/Switch1' */
  if (pid_control_V1_B.EnabledSubsystem.In1.data) {
    /* Switch: '<S12>/Switch1' */
    pid_control_V1_B.Switch1_p[0] = pid_control_V1_B.IMU1_o1[0];
    pid_control_V1_B.Switch1_p[1] = pid_control_V1_B.IMU1_o1[1];
    pid_control_V1_B.Switch1_p[2] = pid_control_V1_B.IMU1_o1[2];
  } else {
    /* Switch: '<S12>/Switch1' incorporates:
     *  Constant: '<S12>/Constant2'
     */
    pid_control_V1_B.Switch1_p[0] = 0.0;
    pid_control_V1_B.Switch1_p[1] = 0.0;
    pid_control_V1_B.Switch1_p[2] = 0.0;
  }

  /* End of Switch: '<S12>/Switch1' */
  if (pid_control_V1_B.b) {
    /* MATLABSystem: '<S286>/SourceBlock' */
    pid_control_V1_B.SourceBlock_o1_c = Sub_pid_control_V1_443.getLatestMessage(
      &pid_control_V1_B.SourceBlock_o2_p);

    /* Outputs for Enabled SubSystem: '<S286>/Enabled Subsystem' */
    pid_control__EnabledSubsystem_k(pid_control_V1_B.SourceBlock_o1_c,
      &pid_control_V1_B.SourceBlock_o2_p, &pid_control_V1_B.EnabledSubsystem_k);

    /* End of Outputs for SubSystem: '<S286>/Enabled Subsystem' */

    /* SignalConversion generated from: '<S12>/Bus Selector2' */
    pid_control_V1_B.data = pid_control_V1_B.EnabledSubsystem_k.In1.data;

    /* MATLABSystem: '<S287>/SourceBlock' */
    pid_control_V1_B.SourceBlock_o1_d = Sub_pid_control_V1_445.getLatestMessage(
      &pid_control_V1_B.SourceBlock_o2_k);

    /* Outputs for Enabled SubSystem: '<S287>/Enabled Subsystem' */
    pid_control__EnabledSubsystem_k(pid_control_V1_B.SourceBlock_o1_d,
      &pid_control_V1_B.SourceBlock_o2_k, &pid_control_V1_B.EnabledSubsystem_p);

    /* End of Outputs for SubSystem: '<S287>/Enabled Subsystem' */

    /* SignalConversion generated from: '<S12>/Bus Selector3' */
    pid_control_V1_B.data_n = pid_control_V1_B.EnabledSubsystem_p.In1.data;
  }

  /* Product: '<S12>/Product2' incorporates:
   *  Math: '<S12>/Square'
   *  Math: '<S12>/Square1'
   *  Math: '<S12>/Square2'
   *  Sqrt: '<S12>/Sqrt'
   *  Sum: '<S12>/Sum2'
   */
  pid_control_V1_B.Power = sqrt((pid_control_V1_B.x[0] * pid_control_V1_B.x[0] +
    pid_control_V1_B.x[1] * pid_control_V1_B.x[1]) + pid_control_V1_B.x[2] *
    pid_control_V1_B.x[2]) * pid_control_V1_B.XDOT[34];

  /* Gain: '<S12>/Gain3' */
  pid_control_V1_B.Gain3 = 0.001 * pid_control_V1_B.Power;
  if (pid_control_V1_B.b) {
  }

  /* Gain: '<S12>/Gain1' incorporates:
   *  Integrator: '<S12>/Integrator1'
   */
  pid_control_V1_B.EnergykWh = 2.7777777777777776E-7 *
    pid_control_V1_X.Integrator1_CSTATE;
  if (pid_control_V1_B.b) {
    /* SignalConversion generated from: '<S12>/To Workspace1' */
    pid_control_V1_B.TmpSignalConversionAtSFunct[0] =
      pid_control_V1_B.Saturation_k;
    pid_control_V1_B.TmpSignalConversionAtSFunct[1] =
      pid_control_V1_B.Saturation_f;
    pid_control_V1_B.TmpSignalConversionAtSFunct[2] =
      pid_control_V1_B.Saturation_m;
    pid_control_V1_B.TmpSignalConversionAtSFunct[3] =
      pid_control_V1_B.Saturation_o;
    pid_control_V1_B.TmpSignalConversionAtSFunct[4] =
      pid_control_V1_B.Saturation_o;
  }

  /* Product: '<S12>/Divide' incorporates:
   *  Constant: '<S12>/thrust efficiency Cp?'
   */
  pid_control_V1_B.powerdemand = pid_control_V1_B.Gain3 / 0.57;
  if (pid_control_V1_B.b) {
  }

  /* Product: '<S12>/Divide1' */
  pid_control_V1_B.loadtorque = pid_control_V1_B.powerdemand /
    pid_control_V1_ConstB.motorspeed;
  if (pid_control_V1_B.b) {
    /* Gain: '<S280>/Output' incorporates:
     *  RandomNumber: '<S280>/White Noise'
     */
    pid_control_V1_B.Output = 10.0 * pid_control_V1_DW.NextOutput_k;
  }

  /* TransferFcn: '<S12>/Transfer Fcn' */
  pid_control_V1_B.factor_bloqueo = 0.5303 *
    pid_control_V1_X.TransferFcn_CSTATE[0] + 0.0 *
    pid_control_V1_X.TransferFcn_CSTATE[1];

  /* Switch: '<S12>/Switch2' incorporates:
   *  Constant: '<S12>/Constant3'
   */
  if (!(pid_control_V1_B.data != 0.0)) {
    pid_control_V1_B.factor_bloqueo = 0.0;
  }

  /* End of Switch: '<S12>/Switch2' */

  /* Sum: '<S12>/Sum' */
  pid_control_V1_B.Sum[0] = pid_control_V1_B.Switch_p[0];
  pid_control_V1_B.Sum[1] = pid_control_V1_B.Switch_p[1] +
    pid_control_V1_B.factor_bloqueo;
  pid_control_V1_B.Sum[2] = pid_control_V1_B.Switch_p[2];

  /* Sum: '<S12>/Sum1' */
  pid_control_V1_B.Sum1[0] = pid_control_V1_B.Switch1_p[0];

  /* Switch: '<S12>/Switch3' incorporates:
   *  Constant: '<S12>/Constant4'
   *  TransferFcn: '<S12>/Transfer Fcn1'
   */
  if (pid_control_V1_B.data_n != 0.0) {
    pid_control_V1_B.cr = -0.0003571 * pid_control_V1_X.TransferFcn1_CSTATE +
      0.03571 * pid_control_V1_B.Output;
  } else {
    pid_control_V1_B.cr = 0.0;
  }

  /* Sum: '<S12>/Sum1' incorporates:
   *  Switch: '<S12>/Switch3'
   */
  pid_control_V1_B.Sum1[1] = pid_control_V1_B.Switch1_p[1] + pid_control_V1_B.cr;
  pid_control_V1_B.Sum1[2] = pid_control_V1_B.Switch1_p[2];
  if (pid_control_V1_B.b) {
    /* Outputs for Enabled SubSystem: '<S14>/Enabled Subsystem' */
    pid_control__EnabledSubsystem_k(pid_control_V1_B.SourceBlock_o1_e,
      &pid_control_V1_B.SourceBlock_o2_o, &pid_control_V1_B.EnabledSubsystem_ps);

    /* End of Outputs for SubSystem: '<S14>/Enabled Subsystem' */
  }

  if (rtmIsMajorTimeStep((&pid_control_V1_M))) {
    if (rtmIsMajorTimeStep((&pid_control_V1_M))) {
      /* Update for Memory: '<S12>/Memory2' incorporates:
       *  Integrator: '<S12>/Integrator'
       */
      memcpy(&pid_control_V1_DW.Memory2_PreviousInput[0], &pid_control_V1_B.x[0],
             12U * sizeof(real_T));

      /* Update for UnitDelay: '<Root>/Unit Delay3' */
      pid_control_V1_DW.UnitDelay3_DSTATE = pid_control_V1_B.Switch3;

      /* Update for UnitDelay: '<Root>/Unit Delay2' */
      pid_control_V1_DW.UnitDelay2_DSTATE = pid_control_V1_B.Switch2;

      /* Update for UnitDelay: '<Root>/Unit Delay' */
      pid_control_V1_DW.UnitDelay_DSTATE = pid_control_V1_B.x[8];

      /* Update for UnitDelay: '<Root>/Unit Delay5' */
      pid_control_V1_DW.UnitDelay5_DSTATE = pid_control_V1_B.Va_out;

      /* Update for UnitDelay: '<Root>/Unit Delay6' */
      pid_control_V1_DW.UnitDelay6_DSTATE = pid_control_V1_B.h_out;

      /* Update for UnitDelay: '<Root>/Unit Delay1' */
      pid_control_V1_DW.UnitDelay1_DSTATE = pid_control_V1_B.Switch1;

      /* Update for Memory: '<S12>/Memory' incorporates:
       *  Sum: '<S12>/Sum'
       */
      pid_control_V1_DW.Memory_PreviousInput[0] = pid_control_V1_B.Sum[0];

      /* Update for Memory: '<S12>/Memory1' incorporates:
       *  Sum: '<S12>/Sum1'
       */
      pid_control_V1_DW.Memory1_PreviousInput[0] = pid_control_V1_B.Sum1[0];

      /* Update for Memory: '<S12>/Memory' incorporates:
       *  Sum: '<S12>/Sum'
       */
      pid_control_V1_DW.Memory_PreviousInput[1] = pid_control_V1_B.Sum[1];

      /* Update for Memory: '<S12>/Memory1' incorporates:
       *  Sum: '<S12>/Sum1'
       */
      pid_control_V1_DW.Memory1_PreviousInput[1] = pid_control_V1_B.Sum1[1];

      /* Update for Memory: '<S12>/Memory' incorporates:
       *  Sum: '<S12>/Sum'
       */
      pid_control_V1_DW.Memory_PreviousInput[2] = pid_control_V1_B.Sum[2];

      /* Update for Memory: '<S12>/Memory1' incorporates:
       *  Sum: '<S12>/Sum1'
       */
      pid_control_V1_DW.Memory1_PreviousInput[2] = pid_control_V1_B.Sum1[2];

      /* Update for Memory: '<S42>/Memory' */
      pid_control_V1_DW.Memory_PreviousInput_o = pid_control_V1_B.AND3;

      /* Update for Memory: '<S252>/Memory' */
      pid_control_V1_DW.Memory_PreviousInput_a = pid_control_V1_B.AND3_c;

      /* Update for RandomNumber: '<S300>/White Noise' */
      pid_control_V1_DW.NextOutput[0] = rt_nrand_Upu32_Yd_f_pw_snf
        (&pid_control_V1_DW.RandSeed[0]);
      pid_control_V1_DW.NextOutput[1] = rt_nrand_Upu32_Yd_f_pw_snf
        (&pid_control_V1_DW.RandSeed[1]);
      pid_control_V1_DW.NextOutput[2] = rt_nrand_Upu32_Yd_f_pw_snf
        (&pid_control_V1_DW.RandSeed[2]);
      pid_control_V1_DW.NextOutput[3] = rt_nrand_Upu32_Yd_f_pw_snf
        (&pid_control_V1_DW.RandSeed[3]);

      /* Update for RandomNumber: '<S280>/White Noise' */
      pid_control_V1_DW.NextOutput_k = rt_nrand_Upu32_Yd_f_pw_snf
        (&pid_control_V1_DW.RandSeed_a);
    }

    /* Update for Integrator: '<S12>/Integrator' */
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

  /* Derivatives for Integrator: '<S12>/Integrator' */
  if (!pid_control_V1_B.Compare) {
    memcpy(&_rtXdot->Integrator_CSTATE[0], &pid_control_V1_B.XDOT[0], 12U *
           sizeof(real_T));
  } else {
    /* level reset is active */
    memset(&_rtXdot->Integrator_CSTATE[0], 0, 12U * sizeof(real_T));
  }

  /* End of Derivatives for Integrator: '<S12>/Integrator' */

  /* Derivatives for Integrator: '<S104>/Integrator' */
  _rtXdot->Integrator_CSTATE_n = pid_control_V1_B.SumI4;

  /* Derivatives for Integrator: '<S99>/Filter' */
  _rtXdot->Filter_CSTATE = pid_control_V1_B.FilterCoefficient;

  /* Derivatives for Integrator: '<S52>/Integrator' */
  _rtXdot->Integrator_CSTATE_m = pid_control_V1_B.Switch;

  /* Derivatives for Integrator: '<S47>/Filter' */
  _rtXdot->Filter_CSTATE_g = pid_control_V1_B.FilterCoefficient_c;

  /* Derivatives for Integrator: '<S156>/Integrator' */
  _rtXdot->Integrator_CSTATE_p = pid_control_V1_B.SumI4_i;

  /* Derivatives for Integrator: '<S151>/Filter' */
  _rtXdot->Filter_CSTATE_m = pid_control_V1_B.FilterCoefficient_m;

  /* Derivatives for Integrator: '<S208>/Integrator' */
  _rtXdot->Integrator_CSTATE_d = pid_control_V1_B.IntegralGain;

  /* Derivatives for Integrator: '<S203>/Filter' */
  _rtXdot->Filter_CSTATE_f = pid_control_V1_B.FilterCoefficient_p;

  /* Derivatives for Integrator: '<S262>/Integrator' */
  _rtXdot->Integrator_CSTATE_f = pid_control_V1_B.Switch_j;

  /* Derivatives for Integrator: '<S257>/Filter' */
  _rtXdot->Filter_CSTATE_l = pid_control_V1_B.FilterCoefficient_cv;

  /* Derivatives for Enabled SubSystem: '<S291>/Hugw(s)' */
  if (pid_control_V1_DW.Hugws_MODE) {
    /* Derivatives for Integrator: '<S304>/ug_p' */
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

  /* End of Derivatives for SubSystem: '<S291>/Hugw(s)' */

  /* Derivatives for Enabled SubSystem: '<S291>/Hvgw(s)' */
  if (pid_control_V1_DW.Hvgws_MODE) {
    /* Derivatives for Integrator: '<S305>/vg_p1' */
    _rtXdot->vg_p1_CSTATE[0] = pid_control_V1_B.w_g[0];

    /* Derivatives for Integrator: '<S305>/vgw_p2' */
    _rtXdot->vgw_p2_CSTATE[0] = pid_control_V1_B.w_e[0];

    /* Derivatives for Integrator: '<S305>/vg_p1' */
    _rtXdot->vg_p1_CSTATE[1] = pid_control_V1_B.w_g[1];

    /* Derivatives for Integrator: '<S305>/vgw_p2' */
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

  /* End of Derivatives for SubSystem: '<S291>/Hvgw(s)' */

  /* Derivatives for Enabled SubSystem: '<S291>/Hwgw(s)' */
  if (pid_control_V1_DW.Hwgws_MODE) {
    /* Derivatives for Integrator: '<S306>/wg_p1' */
    _rtXdot->wg_p1_CSTATE[0] = pid_control_V1_B.w[0];

    /* Derivatives for Integrator: '<S306>/wg_p2' */
    _rtXdot->wg_p2_CSTATE[0] = pid_control_V1_B.w_a[0];

    /* Derivatives for Integrator: '<S306>/wg_p1' */
    _rtXdot->wg_p1_CSTATE[1] = pid_control_V1_B.w[1];

    /* Derivatives for Integrator: '<S306>/wg_p2' */
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

  /* End of Derivatives for SubSystem: '<S291>/Hwgw(s)' */

  /* Derivatives for Enabled SubSystem: '<S290>/Hpgw' */
  if (pid_control_V1_DW.Hpgw_MODE) {
    /* Derivatives for Integrator: '<S301>/pgw_p' */
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

  /* End of Derivatives for SubSystem: '<S290>/Hpgw' */

  /* Derivatives for Enabled SubSystem: '<S290>/Hqgw' */
  if (pid_control_V1_DW.Hqgw_MODE) {
    /* Derivatives for Integrator: '<S302>/qgw_p' */
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

  /* End of Derivatives for SubSystem: '<S290>/Hqgw' */

  /* Derivatives for Enabled SubSystem: '<S290>/Hrgw' */
  if (pid_control_V1_DW.Hrgw_MODE) {
    /* Derivatives for Integrator: '<S303>/rgw_p' */
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

  /* End of Derivatives for SubSystem: '<S290>/Hrgw' */

  /* Derivatives for Integrator: '<S12>/Integrator1' */
  _rtXdot->Integrator1_CSTATE = pid_control_V1_B.Power;

  /* Derivatives for TransferFcn: '<S12>/Transfer Fcn' */
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

  /* Derivatives for TransferFcn: '<S12>/Transfer Fcn1' */
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

  {
    int32_T i;
    boolean_T flag;

    /* Start for InitialCondition: '<S12>/IC' */
    memcpy(&pid_control_V1_B.IC[0], &pid_control_V1_ConstP.pooled15[0], 12U *
           sizeof(real_T));

    /* Start for InitialCondition: '<S12>/IC' */
    pid_control_V1_DW.IC_FirstOutputTime = true;

    /* Start for MATLABSystem: '<Root>/IMU1' */
    pid_control_V1_DW.obj.MagneticFieldNED[0] = 27.555;
    pid_control_V1_DW.obj.MagneticFieldNED[1] = -2.4169;
    pid_control_V1_DW.obj.MagneticFieldNED[2] = -16.0849;
    pid_control_V1_DW.obj.isInitialized = 0;
    for (i = 0; i < 38; i++) {
      pid_control_V1_DW.obj.tunablePropertyChanged[i] = false;
    }

    flag = (pid_control_V1_DW.obj.isInitialized == 1);
    if (flag) {
      pid_control_V1_DW.obj.TunablePropsChanged = true;
      pid_control_V1_DW.obj.tunablePropertyChanged[2] = true;
    }

    pid_control_V1_DW.objisempty_d = true;
    flag = (pid_control_V1_DW.obj.isInitialized == 1);
    if (flag) {
      pid_control_V1_DW.obj.TunablePropsChanged = true;
      pid_control_V1_DW.obj.tunablePropertyChanged[37] = true;
    }

    pid_control_V1_DW.obj.Temperature = 25.0;
    flag = (pid_control_V1_DW.obj.isInitialized == 1);
    if (flag) {
      pid_control_V1_DW.obj.TunablePropsChanged = true;
      pid_control_V1_DW.obj.tunablePropertyChanged[0] = true;
    }

    flag = (pid_control_V1_DW.obj.isInitialized == 1);
    if (flag) {
      pid_control_V1_DW.obj.TunablePropsChanged = true;
      pid_control_V1_DW.obj.tunablePropertyChanged[2] = true;
    }

    pid_control_V1_DW.obj.MagneticField[0] = 27.555;
    pid_control_V1_DW.obj.MagneticField[1] = -2.4169;
    pid_control_V1_DW.obj.MagneticField[2] = -16.0849;
    flag = (pid_control_V1_DW.obj.isInitialized == 1);
    if (flag) {
      pid_control_V1_DW.obj.TunablePropsChanged = true;
      pid_control_V1_DW.obj.tunablePropertyChanged[3] = true;
    }

    pid_control_V1_DW.obj.AccelParamsMeasurementRange = (rtInf);
    flag = (pid_control_V1_DW.obj.isInitialized == 1);
    if (flag) {
      pid_control_V1_DW.obj.TunablePropsChanged = true;
      pid_control_V1_DW.obj.tunablePropertyChanged[4] = true;
    }

    pid_control_V1_DW.obj.AccelParamsResolution = 0.0;
    flag = (pid_control_V1_DW.obj.isInitialized == 1);
    if (flag) {
      pid_control_V1_DW.obj.TunablePropsChanged = true;
      pid_control_V1_DW.obj.tunablePropertyChanged[5] = true;
    }

    pid_control_V1_DW.obj.AccelParamsConstantBias[0] = 0.0;
    pid_control_V1_DW.obj.AccelParamsConstantBias[1] = 0.0;
    pid_control_V1_DW.obj.AccelParamsConstantBias[2] = 0.0;
    flag = (pid_control_V1_DW.obj.isInitialized == 1);
    if (flag) {
      pid_control_V1_DW.obj.TunablePropsChanged = true;
      pid_control_V1_DW.obj.tunablePropertyChanged[6] = true;
    }

    memcpy(&pid_control_V1_DW.obj.AccelParamsAxesMisalignment[0],
           &pid_control_V1_ConstP.pooled6[0], 9U * sizeof(real_T));
    flag = (pid_control_V1_DW.obj.isInitialized == 1);
    if (flag) {
      pid_control_V1_DW.obj.TunablePropsChanged = true;
      pid_control_V1_DW.obj.tunablePropertyChanged[7] = true;
    }

    pid_control_V1_DW.obj.AccelParamsNoiseDensity[0] = 0.0004;
    pid_control_V1_DW.obj.AccelParamsNoiseDensity[1] = 0.0004;
    pid_control_V1_DW.obj.AccelParamsNoiseDensity[2] = 0.0004;
    flag = (pid_control_V1_DW.obj.isInitialized == 1);
    if (flag) {
      pid_control_V1_DW.obj.TunablePropsChanged = true;
      pid_control_V1_DW.obj.tunablePropertyChanged[8] = true;
    }

    pid_control_V1_DW.obj.AccelParamsBiasInstability[0] = 0.0001;
    pid_control_V1_DW.obj.AccelParamsBiasInstability[1] = 0.0001;
    pid_control_V1_DW.obj.AccelParamsBiasInstability[2] = 0.0001;
    flag = (pid_control_V1_DW.obj.isInitialized == 1);
    if (flag) {
      pid_control_V1_DW.obj.TunablePropsChanged = true;
      pid_control_V1_DW.obj.tunablePropertyChanged[9] = true;
    }

    pid_control_V1_DW.obj.AccelParamsBiasInstabilityNumerator = 1.0;
    flag = (pid_control_V1_DW.obj.isInitialized == 1);
    if (flag) {
      pid_control_V1_DW.obj.TunablePropsChanged = true;
      pid_control_V1_DW.obj.tunablePropertyChanged[10] = true;
    }

    pid_control_V1_DW.obj.AccelParamsBiasInstabilityDenominator[0] = 1.0;
    pid_control_V1_DW.obj.AccelParamsBiasInstabilityDenominator[1] = -0.5;
    flag = (pid_control_V1_DW.obj.isInitialized == 1);
    if (flag) {
      pid_control_V1_DW.obj.TunablePropsChanged = true;
      pid_control_V1_DW.obj.tunablePropertyChanged[11] = true;
    }

    pid_control_V1_DW.obj.AccelParamsRandomWalk[0] = 0.0;
    pid_control_V1_DW.obj.AccelParamsRandomWalk[1] = 0.0;
    pid_control_V1_DW.obj.AccelParamsRandomWalk[2] = 0.0;
    flag = (pid_control_V1_DW.obj.isInitialized == 1);
    if (flag) {
      pid_control_V1_DW.obj.TunablePropsChanged = true;
      pid_control_V1_DW.obj.tunablePropertyChanged[12] = true;
    }

    pid_control_V1_DW.obj.AccelParamsTemperatureBias[0] = 0.0;
    pid_control_V1_DW.obj.AccelParamsTemperatureBias[1] = 0.0;
    pid_control_V1_DW.obj.AccelParamsTemperatureBias[2] = 0.0;
    flag = (pid_control_V1_DW.obj.isInitialized == 1);
    if (flag) {
      pid_control_V1_DW.obj.TunablePropsChanged = true;
      pid_control_V1_DW.obj.tunablePropertyChanged[13] = true;
    }

    pid_control_V1_DW.obj.AccelParamsTemperatureScaleFactor[0] = 0.0;
    pid_control_V1_DW.obj.AccelParamsTemperatureScaleFactor[1] = 0.0;
    pid_control_V1_DW.obj.AccelParamsTemperatureScaleFactor[2] = 0.0;
    flag = (pid_control_V1_DW.obj.isInitialized == 1);
    if (flag) {
      pid_control_V1_DW.obj.TunablePropsChanged = true;
      pid_control_V1_DW.obj.tunablePropertyChanged[14] = true;
    }

    pid_control_V1_DW.obj.GyroParamsMeasurementRange = (rtInf);
    flag = (pid_control_V1_DW.obj.isInitialized == 1);
    if (flag) {
      pid_control_V1_DW.obj.TunablePropsChanged = true;
      pid_control_V1_DW.obj.tunablePropertyChanged[15] = true;
    }

    pid_control_V1_DW.obj.GyroParamsResolution = 0.0;
    flag = (pid_control_V1_DW.obj.isInitialized == 1);
    if (flag) {
      pid_control_V1_DW.obj.TunablePropsChanged = true;
      pid_control_V1_DW.obj.tunablePropertyChanged[16] = true;
    }

    pid_control_V1_DW.obj.GyroParamsConstantBias[0] = 0.0;
    pid_control_V1_DW.obj.GyroParamsConstantBias[1] = 0.0;
    pid_control_V1_DW.obj.GyroParamsConstantBias[2] = 0.0;
    flag = (pid_control_V1_DW.obj.isInitialized == 1);
    if (flag) {
      pid_control_V1_DW.obj.TunablePropsChanged = true;
      pid_control_V1_DW.obj.tunablePropertyChanged[17] = true;
    }

    memcpy(&pid_control_V1_DW.obj.GyroParamsAxesMisalignment[0],
           &pid_control_V1_ConstP.pooled6[0], 9U * sizeof(real_T));
    flag = (pid_control_V1_DW.obj.isInitialized == 1);
    if (flag) {
      pid_control_V1_DW.obj.TunablePropsChanged = true;
      pid_control_V1_DW.obj.tunablePropertyChanged[25] = true;
    }

    pid_control_V1_DW.obj.GyroParamsAccelerationBias[0] = 0.0;
    pid_control_V1_DW.obj.GyroParamsAccelerationBias[1] = 0.0;
    pid_control_V1_DW.obj.GyroParamsAccelerationBias[2] = 0.0;
    flag = (pid_control_V1_DW.obj.isInitialized == 1);
    if (flag) {
      pid_control_V1_DW.obj.TunablePropsChanged = true;
      pid_control_V1_DW.obj.tunablePropertyChanged[18] = true;
    }

    pid_control_V1_DW.obj.GyroParamsNoiseDensity[0] = 0.0001;
    pid_control_V1_DW.obj.GyroParamsNoiseDensity[1] = 0.0001;
    pid_control_V1_DW.obj.GyroParamsNoiseDensity[2] = 0.0001;
    flag = (pid_control_V1_DW.obj.isInitialized == 1);
    if (flag) {
      pid_control_V1_DW.obj.TunablePropsChanged = true;
      pid_control_V1_DW.obj.tunablePropertyChanged[19] = true;
    }

    pid_control_V1_DW.obj.GyroParamsBiasInstability[0] = 1.0E-5;
    pid_control_V1_DW.obj.GyroParamsBiasInstability[1] = 1.0E-5;
    pid_control_V1_DW.obj.GyroParamsBiasInstability[2] = 1.0E-5;
    flag = (pid_control_V1_DW.obj.isInitialized == 1);
    if (flag) {
      pid_control_V1_DW.obj.TunablePropsChanged = true;
      pid_control_V1_DW.obj.tunablePropertyChanged[20] = true;
    }

    pid_control_V1_DW.obj.GyroParamsBiasInstabilityNumerator = 1.0;
    flag = (pid_control_V1_DW.obj.isInitialized == 1);
    if (flag) {
      pid_control_V1_DW.obj.TunablePropsChanged = true;
      pid_control_V1_DW.obj.tunablePropertyChanged[21] = true;
    }

    pid_control_V1_DW.obj.GyroParamsBiasInstabilityDenominator[0] = 1.0;
    pid_control_V1_DW.obj.GyroParamsBiasInstabilityDenominator[1] = -0.5;
    flag = (pid_control_V1_DW.obj.isInitialized == 1);
    if (flag) {
      pid_control_V1_DW.obj.TunablePropsChanged = true;
      pid_control_V1_DW.obj.tunablePropertyChanged[22] = true;
    }

    pid_control_V1_DW.obj.GyroParamsRandomWalk[0] = 0.0;
    pid_control_V1_DW.obj.GyroParamsRandomWalk[1] = 0.0;
    pid_control_V1_DW.obj.GyroParamsRandomWalk[2] = 0.0;
    flag = (pid_control_V1_DW.obj.isInitialized == 1);
    if (flag) {
      pid_control_V1_DW.obj.TunablePropsChanged = true;
      pid_control_V1_DW.obj.tunablePropertyChanged[23] = true;
    }

    pid_control_V1_DW.obj.GyroParamsTemperatureBias[0] = 0.0;
    pid_control_V1_DW.obj.GyroParamsTemperatureBias[1] = 0.0;
    pid_control_V1_DW.obj.GyroParamsTemperatureBias[2] = 0.0;
    flag = (pid_control_V1_DW.obj.isInitialized == 1);
    if (flag) {
      pid_control_V1_DW.obj.TunablePropsChanged = true;
      pid_control_V1_DW.obj.tunablePropertyChanged[24] = true;
    }

    pid_control_V1_DW.obj.GyroParamsTemperatureScaleFactor[0] = 0.0;
    pid_control_V1_DW.obj.GyroParamsTemperatureScaleFactor[1] = 0.0;
    pid_control_V1_DW.obj.GyroParamsTemperatureScaleFactor[2] = 0.0;
    flag = (pid_control_V1_DW.obj.isInitialized == 1);
    if (flag) {
      pid_control_V1_DW.obj.TunablePropsChanged = true;
      pid_control_V1_DW.obj.tunablePropertyChanged[26] = true;
    }

    pid_control_V1_DW.obj.MagParamsMeasurementRange = (rtInf);
    flag = (pid_control_V1_DW.obj.isInitialized == 1);
    if (flag) {
      pid_control_V1_DW.obj.TunablePropsChanged = true;
      pid_control_V1_DW.obj.tunablePropertyChanged[27] = true;
    }

    pid_control_V1_DW.obj.MagParamsResolution = 0.0;
    flag = (pid_control_V1_DW.obj.isInitialized == 1);
    if (flag) {
      pid_control_V1_DW.obj.TunablePropsChanged = true;
      pid_control_V1_DW.obj.tunablePropertyChanged[28] = true;
    }

    pid_control_V1_DW.obj.MagParamsConstantBias[0] = 0.0;
    pid_control_V1_DW.obj.MagParamsConstantBias[1] = 0.0;
    pid_control_V1_DW.obj.MagParamsConstantBias[2] = 0.0;
    flag = (pid_control_V1_DW.obj.isInitialized == 1);
    if (flag) {
      pid_control_V1_DW.obj.TunablePropsChanged = true;
      pid_control_V1_DW.obj.tunablePropertyChanged[29] = true;
    }

    memcpy(&pid_control_V1_DW.obj.MagParamsAxesMisalignment[0],
           &pid_control_V1_ConstP.pooled6[0], 9U * sizeof(real_T));
    flag = (pid_control_V1_DW.obj.isInitialized == 1);
    if (flag) {
      pid_control_V1_DW.obj.TunablePropsChanged = true;
      pid_control_V1_DW.obj.tunablePropertyChanged[30] = true;
    }

    pid_control_V1_DW.obj.MagParamsNoiseDensity[0] = 0.0;
    pid_control_V1_DW.obj.MagParamsNoiseDensity[1] = 0.0;
    pid_control_V1_DW.obj.MagParamsNoiseDensity[2] = 0.0;
    flag = (pid_control_V1_DW.obj.isInitialized == 1);
    if (flag) {
      pid_control_V1_DW.obj.TunablePropsChanged = true;
      pid_control_V1_DW.obj.tunablePropertyChanged[31] = true;
    }

    pid_control_V1_DW.obj.MagParamsBiasInstability[0] = 0.0;
    pid_control_V1_DW.obj.MagParamsBiasInstability[1] = 0.0;
    pid_control_V1_DW.obj.MagParamsBiasInstability[2] = 0.0;
    flag = (pid_control_V1_DW.obj.isInitialized == 1);
    if (flag) {
      pid_control_V1_DW.obj.TunablePropsChanged = true;
      pid_control_V1_DW.obj.tunablePropertyChanged[32] = true;
    }

    pid_control_V1_DW.obj.MagParamsBiasInstabilityNumerator = 1.0;
    flag = (pid_control_V1_DW.obj.isInitialized == 1);
    if (flag) {
      pid_control_V1_DW.obj.TunablePropsChanged = true;
      pid_control_V1_DW.obj.tunablePropertyChanged[33] = true;
    }

    pid_control_V1_DW.obj.MagParamsBiasInstabilityDenominator[0] = 1.0;
    pid_control_V1_DW.obj.MagParamsBiasInstabilityDenominator[1] = -0.5;
    flag = (pid_control_V1_DW.obj.isInitialized == 1);
    if (flag) {
      pid_control_V1_DW.obj.TunablePropsChanged = true;
      pid_control_V1_DW.obj.tunablePropertyChanged[34] = true;
    }

    pid_control_V1_DW.obj.MagParamsRandomWalk[0] = 0.0;
    pid_control_V1_DW.obj.MagParamsRandomWalk[1] = 0.0;
    pid_control_V1_DW.obj.MagParamsRandomWalk[2] = 0.0;
    flag = (pid_control_V1_DW.obj.isInitialized == 1);
    if (flag) {
      pid_control_V1_DW.obj.TunablePropsChanged = true;
      pid_control_V1_DW.obj.tunablePropertyChanged[35] = true;
    }

    pid_control_V1_DW.obj.MagParamsTemperatureBias[0] = 0.0;
    pid_control_V1_DW.obj.MagParamsTemperatureBias[1] = 0.0;
    pid_control_V1_DW.obj.MagParamsTemperatureBias[2] = 0.0;
    flag = (pid_control_V1_DW.obj.isInitialized == 1);
    if (flag) {
      pid_control_V1_DW.obj.TunablePropsChanged = true;
      pid_control_V1_DW.obj.tunablePropertyChanged[36] = true;
    }

    pid_control_V1_DW.obj.MagParamsTemperatureScaleFactor[0] = 0.0;
    pid_control_V1_DW.obj.MagParamsTemperatureScaleFactor[1] = 0.0;
    pid_control_V1_DW.obj.MagParamsTemperatureScaleFactor[2] = 0.0;
    pid_control_V1_DW.obj.isInitialized = 1;
    pid_control_imuSensor_setupImpl(&pid_control_V1_DW.obj);
    pid_control_V1_DW.obj.TunablePropsChanged = false;

    /* End of Start for MATLABSystem: '<Root>/IMU1' */

    /* Start for MATLABSystem: '<S13>/SourceBlock' */
    pid_control_V1_DW.obj_m.QOSAvoidROSNamespaceConventions = false;
    pid_control_V1_DW.obj_m.matlabCodegenIsDeleted = false;
    pid_control_V1_DW.objisempty_g = true;
    pid_control_V1_DW.obj_m.isSetupComplete = false;
    pid_control_V1_DW.obj_m.isInitialized = 1;
    pid_c_Subscriber_setupImpl_onhg(&pid_control_V1_DW.obj_m);
    pid_control_V1_DW.obj_m.isSetupComplete = true;

    /* Start for MATLABSystem: '<S15>/SourceBlock' */
    pid_control_V1_DW.obj_k.QOSAvoidROSNamespaceConventions = false;
    pid_control_V1_DW.obj_k.matlabCodegenIsDeleted = false;
    pid_control_V1_DW.objisempty = true;
    pid_control_V1_DW.obj_k.isSetupComplete = false;
    pid_control_V1_DW.obj_k.isInitialized = 1;
    pid_Subscriber_setupImpl_onhgd0(&pid_control_V1_DW.obj_k);
    pid_control_V1_DW.obj_k.isSetupComplete = true;

    /* Start for MATLABSystem: '<S14>/SourceBlock' */
    pid_control_V1_DW.obj_i.QOSAvoidROSNamespaceConventions = false;
    pid_control_V1_DW.obj_i.matlabCodegenIsDeleted = false;
    pid_control_V1_DW.objisempty_f = true;
    pid_control_V1_DW.obj_i.isSetupComplete = false;
    pid_control_V1_DW.obj_i.isInitialized = 1;
    pid__Subscriber_setupImpl_onhgd(&pid_control_V1_DW.obj_i);
    pid_control_V1_DW.obj_i.isSetupComplete = true;

    /* Start for MATLABSystem: '<Root>/Coordinate Transformation Conversion' */
    pid_control_V1_DW.objisempty_dc = true;
    pid_control_V1_DW.obj_c.isInitialized = 1;

    /* Start for Atomic SubSystem: '<Root>/Call Service' */
    /* Start for MATLABSystem: '<S2>/ServiceCaller' */
    pid_control_V1_DW.obj_j.QOSAvoidROSNamespaceConventions = false;
    pid_control_V1_DW.obj_j.matlabCodegenIsDeleted = false;
    pid_control_V1_DW.objisempty_ft = true;
    pid_control_V1_DW.obj_j.isSetupComplete = false;
    pid_control_V1_DW.obj_j.isInitialized = 1;
    pid_con_ServiceCaller_setupImpl(&pid_control_V1_DW.obj_j);
    pid_control_V1_DW.obj_j.isSetupComplete = true;

    /* End of Start for SubSystem: '<Root>/Call Service' */

    /* Start for Enabled SubSystem: '<S291>/Hugw(s)' */
    (void) memset(&(pid_control_V1_XDis.ug_p_CSTATE), 1,
                  2*sizeof(boolean_T));

    /* End of Start for SubSystem: '<S291>/Hugw(s)' */

    /* Start for Enabled SubSystem: '<S291>/Hvgw(s)' */
    (void) memset(&(pid_control_V1_XDis.vg_p1_CSTATE), 1,
                  4*sizeof(boolean_T));

    /* End of Start for SubSystem: '<S291>/Hvgw(s)' */

    /* Start for Enabled SubSystem: '<S291>/Hwgw(s)' */
    (void) memset(&(pid_control_V1_XDis.wg_p1_CSTATE), 1,
                  4*sizeof(boolean_T));

    /* End of Start for SubSystem: '<S291>/Hwgw(s)' */

    /* Start for If: '<S296>/if Height < Max low altitude  elseif Height > Min isotropic altitude ' */
    pid_control_V1_DW.ifHeightMaxlowaltitudeelseifHei = -1;

    /* Start for MATLABSystem: '<S288>/SourceBlock' */
    pid_control_V1_DW.obj_h.QOSAvoidROSNamespaceConventions = false;
    pid_control_V1_DW.obj_h.matlabCodegenIsDeleted = false;
    pid_control_V1_DW.objisempty_a = true;
    pid_control_V1_DW.obj_h.isSetupComplete = false;
    pid_control_V1_DW.obj_h.isInitialized = 1;
    pid_co_Subscriber_setupImpl_onh(&pid_control_V1_DW.obj_h);
    pid_control_V1_DW.obj_h.isSetupComplete = true;

    /* Start for Enabled SubSystem: '<S290>/Hpgw' */
    (void) memset(&(pid_control_V1_XDis.pgw_p_CSTATE), 1,
                  2*sizeof(boolean_T));

    /* End of Start for SubSystem: '<S290>/Hpgw' */

    /* Start for Enabled SubSystem: '<S290>/Hqgw' */
    (void) memset(&(pid_control_V1_XDis.qgw_p_CSTATE), 1,
                  2*sizeof(boolean_T));

    /* End of Start for SubSystem: '<S290>/Hqgw' */

    /* Start for Enabled SubSystem: '<S290>/Hrgw' */
    (void) memset(&(pid_control_V1_XDis.rgw_p_CSTATE), 1,
                  2*sizeof(boolean_T));

    /* End of Start for SubSystem: '<S290>/Hrgw' */

    /* Start for If: '<S295>/if Height < Max low altitude  elseif Height > Min isotropic altitude ' */
    pid_control_V1_DW.ifHeightMaxlowaltitudeelseifH_a = -1;

    /* Start for MATLABSystem: '<S285>/SourceBlock' */
    pid_control_V1_DW.obj_h4.QOSAvoidROSNamespaceConventions = false;
    pid_control_V1_DW.obj_h4.matlabCodegenIsDeleted = false;
    pid_control_V1_DW.objisempty_c = true;
    pid_control_V1_DW.obj_h4.isSetupComplete = false;
    pid_control_V1_DW.obj_h4.isInitialized = 1;
    pid_contro_Subscriber_setupImpl(&pid_control_V1_DW.obj_h4);
    pid_control_V1_DW.obj_h4.isSetupComplete = true;

    /* Start for MATLABSystem: '<S286>/SourceBlock' */
    pid_control_V1_DW.obj_hy.QOSAvoidROSNamespaceConventions = false;
    pid_control_V1_DW.obj_hy.matlabCodegenIsDeleted = false;
    pid_control_V1_DW.objisempty_l = true;
    pid_control_V1_DW.obj_hy.isSetupComplete = false;
    pid_control_V1_DW.obj_hy.isInitialized = 1;
    pid_cont_Subscriber_setupImpl_o(&pid_control_V1_DW.obj_hy);
    pid_control_V1_DW.obj_hy.isSetupComplete = true;

    /* Start for MATLABSystem: '<S287>/SourceBlock' */
    pid_control_V1_DW.obj_p.QOSAvoidROSNamespaceConventions = false;
    pid_control_V1_DW.obj_p.matlabCodegenIsDeleted = false;
    pid_control_V1_DW.objisempty_e = true;
    pid_control_V1_DW.obj_p.isSetupComplete = false;
    pid_control_V1_DW.obj_p.isInitialized = 1;
    pid_con_Subscriber_setupImpl_on(&pid_control_V1_DW.obj_p);
    pid_control_V1_DW.obj_p.isSetupComplete = true;
  }

  pid_control_V1_PrevZCX.Integrator_Reset_ZCE = UNINITIALIZED_ZCSIG;

  /* InitializeConditions for Memory: '<S12>/Memory2' */
  memcpy(&pid_control_V1_DW.Memory2_PreviousInput[0],
         &pid_control_V1_ConstP.pooled15[0], 12U * sizeof(real_T));

  /* InitializeConditions for Integrator: '<S12>/Integrator' */
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

  /* End of InitializeConditions for Integrator: '<S12>/Integrator' */

  /* InitializeConditions for UnitDelay: '<Root>/Unit Delay3' */
  pid_control_V1_DW.UnitDelay3_DSTATE = 0.55;

  /* InitializeConditions for Integrator: '<S104>/Integrator' */
  pid_control_V1_X.Integrator_CSTATE_n = 0.0;

  /* InitializeConditions for Integrator: '<S99>/Filter' */
  pid_control_V1_X.Filter_CSTATE = 0.0;

  /* InitializeConditions for Integrator: '<S52>/Integrator' */
  pid_control_V1_X.Integrator_CSTATE_m = 0.0;

  /* InitializeConditions for Integrator: '<S47>/Filter' */
  pid_control_V1_X.Filter_CSTATE_g = 0.0;

  /* InitializeConditions for RateLimiter: '<Root>/Rate Limiter' */
  pid_control_V1_DW.LastMajorTime = (rtInf);

  /* InitializeConditions for Integrator: '<S156>/Integrator' */
  pid_control_V1_X.Integrator_CSTATE_p = 0.0;

  /* InitializeConditions for Integrator: '<S151>/Filter' */
  pid_control_V1_X.Filter_CSTATE_m = 0.0;

  /* InitializeConditions for Integrator: '<S208>/Integrator' */
  pid_control_V1_X.Integrator_CSTATE_d = 0.0;

  /* InitializeConditions for Integrator: '<S203>/Filter' */
  pid_control_V1_X.Filter_CSTATE_f = 0.0;

  /* InitializeConditions for Integrator: '<S262>/Integrator' */
  pid_control_V1_X.Integrator_CSTATE_f = 0.0;

  /* InitializeConditions for Integrator: '<S257>/Filter' */
  pid_control_V1_X.Filter_CSTATE_l = 0.0;

  /* InitializeConditions for RandomNumber: '<S300>/White Noise' */
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

  /* InitializeConditions for Integrator: '<S12>/Integrator1' */
  pid_control_V1_X.Integrator1_CSTATE = 0.0;

  /* InitializeConditions for RandomNumber: '<S280>/White Noise' */
  pid_control_V1_DW.RandSeed_a = 1529675776U;
  pid_control_V1_DW.NextOutput_k = rt_nrand_Upu32_Yd_f_pw_snf
    (&pid_control_V1_DW.RandSeed_a);

  /* InitializeConditions for TransferFcn: '<S12>/Transfer Fcn' */
  pid_control_V1_X.TransferFcn_CSTATE[0] = 0.0;
  pid_control_V1_X.TransferFcn_CSTATE[1] = 0.0;

  /* InitializeConditions for TransferFcn: '<S12>/Transfer Fcn1' */
  pid_control_V1_X.TransferFcn1_CSTATE = 0.0;

  /* SystemInitialize for Enabled SubSystem: '<S13>/Enabled Subsystem' */
  pid_con_EnabledSubsystem_i_Init(&pid_control_V1_B.EnabledSubsystem_b);

  /* End of SystemInitialize for SubSystem: '<S13>/Enabled Subsystem' */

  /* SystemInitialize for Enabled SubSystem: '<S15>/Enabled Subsystem' */
  pid_con_EnabledSubsystem_i_Init(&pid_control_V1_B.EnabledSubsystem_a);

  /* End of SystemInitialize for SubSystem: '<S15>/Enabled Subsystem' */

  /* SystemInitialize for Enabled SubSystem: '<S291>/Hugw(s)' */
  /* InitializeConditions for Integrator: '<S304>/ug_p' */
  pid_control_V1_X.ug_p_CSTATE[0] = 0.0;

  /* End of SystemInitialize for SubSystem: '<S291>/Hugw(s)' */

  /* SystemInitialize for Enabled SubSystem: '<S291>/Hvgw(s)' */
  /* InitializeConditions for Integrator: '<S305>/vg_p1' */
  pid_control_V1_X.vg_p1_CSTATE[0] = 0.0;

  /* InitializeConditions for Integrator: '<S305>/vgw_p2' */
  pid_control_V1_X.vgw_p2_CSTATE[0] = 0.0;

  /* End of SystemInitialize for SubSystem: '<S291>/Hvgw(s)' */

  /* SystemInitialize for Enabled SubSystem: '<S291>/Hwgw(s)' */
  /* InitializeConditions for Integrator: '<S306>/wg_p1' */
  pid_control_V1_X.wg_p1_CSTATE[0] = 0.0;

  /* InitializeConditions for Integrator: '<S306>/wg_p2' */
  pid_control_V1_X.wg_p2_CSTATE[0] = 0.0;

  /* End of SystemInitialize for SubSystem: '<S291>/Hwgw(s)' */

  /* SystemInitialize for Enabled SubSystem: '<S291>/Hugw(s)' */
  /* InitializeConditions for Integrator: '<S304>/ug_p' */
  pid_control_V1_X.ug_p_CSTATE[1] = 0.0;

  /* End of SystemInitialize for SubSystem: '<S291>/Hugw(s)' */

  /* SystemInitialize for Enabled SubSystem: '<S291>/Hvgw(s)' */
  /* InitializeConditions for Integrator: '<S305>/vg_p1' */
  pid_control_V1_X.vg_p1_CSTATE[1] = 0.0;

  /* InitializeConditions for Integrator: '<S305>/vgw_p2' */
  pid_control_V1_X.vgw_p2_CSTATE[1] = 0.0;

  /* End of SystemInitialize for SubSystem: '<S291>/Hvgw(s)' */

  /* SystemInitialize for Enabled SubSystem: '<S291>/Hwgw(s)' */
  /* InitializeConditions for Integrator: '<S306>/wg_p1' */
  pid_control_V1_X.wg_p1_CSTATE[1] = 0.0;

  /* InitializeConditions for Integrator: '<S306>/wg_p2' */
  pid_control_V1_X.wg_p2_CSTATE[1] = 0.0;

  /* End of SystemInitialize for SubSystem: '<S291>/Hwgw(s)' */

  /* SystemInitialize for Enabled SubSystem: '<S288>/Enabled Subsystem' */
  pid_contr_EnabledSubsystem_Init(&pid_control_V1_B.EnabledSubsystem_pt);

  /* End of SystemInitialize for SubSystem: '<S288>/Enabled Subsystem' */

  /* SystemInitialize for Enabled SubSystem: '<S290>/Hpgw' */
  /* InitializeConditions for Integrator: '<S301>/pgw_p' */
  pid_control_V1_X.pgw_p_CSTATE[0] = 0.0;

  /* End of SystemInitialize for SubSystem: '<S290>/Hpgw' */

  /* SystemInitialize for Enabled SubSystem: '<S290>/Hqgw' */
  /* InitializeConditions for Integrator: '<S302>/qgw_p' */
  pid_control_V1_X.qgw_p_CSTATE[0] = 0.0;

  /* End of SystemInitialize for SubSystem: '<S290>/Hqgw' */

  /* SystemInitialize for Enabled SubSystem: '<S290>/Hrgw' */
  /* InitializeConditions for Integrator: '<S303>/rgw_p' */
  pid_control_V1_X.rgw_p_CSTATE[0] = 0.0;

  /* End of SystemInitialize for SubSystem: '<S290>/Hrgw' */

  /* SystemInitialize for Enabled SubSystem: '<S290>/Hpgw' */
  /* InitializeConditions for Integrator: '<S301>/pgw_p' */
  pid_control_V1_X.pgw_p_CSTATE[1] = 0.0;

  /* End of SystemInitialize for SubSystem: '<S290>/Hpgw' */

  /* SystemInitialize for Enabled SubSystem: '<S290>/Hqgw' */
  /* InitializeConditions for Integrator: '<S302>/qgw_p' */
  pid_control_V1_X.qgw_p_CSTATE[1] = 0.0;

  /* End of SystemInitialize for SubSystem: '<S290>/Hqgw' */

  /* SystemInitialize for Enabled SubSystem: '<S290>/Hrgw' */
  /* InitializeConditions for Integrator: '<S303>/rgw_p' */
  pid_control_V1_X.rgw_p_CSTATE[1] = 0.0;

  /* End of SystemInitialize for SubSystem: '<S290>/Hrgw' */

  /* SystemInitialize for Enabled SubSystem: '<S285>/Enabled Subsystem' */
  pid_contr_EnabledSubsystem_Init(&pid_control_V1_B.EnabledSubsystem);

  /* End of SystemInitialize for SubSystem: '<S285>/Enabled Subsystem' */

  /* SystemInitialize for Enabled SubSystem: '<S286>/Enabled Subsystem' */
  pid_con_EnabledSubsystem_i_Init(&pid_control_V1_B.EnabledSubsystem_k);

  /* End of SystemInitialize for SubSystem: '<S286>/Enabled Subsystem' */

  /* SystemInitialize for Enabled SubSystem: '<S287>/Enabled Subsystem' */
  pid_con_EnabledSubsystem_i_Init(&pid_control_V1_B.EnabledSubsystem_p);

  /* End of SystemInitialize for SubSystem: '<S287>/Enabled Subsystem' */

  /* SystemInitialize for Enabled SubSystem: '<S14>/Enabled Subsystem' */
  pid_con_EnabledSubsystem_i_Init(&pid_control_V1_B.EnabledSubsystem_ps);

  /* End of SystemInitialize for SubSystem: '<S14>/Enabled Subsystem' */

  /* InitializeConditions for MATLABSystem: '<Root>/IMU1' */
  pid_con_IMUSensorBase_resetImpl(&pid_control_V1_DW.obj);

  /* set "at time zero" to false */
  if (rtmIsFirstInitCond((&pid_control_V1_M))) {
    rtmSetFirstInitCond((&pid_control_V1_M), 0);
  }
}

/* Model terminate function */
void pid_control_V1::terminate()
{
  /* Terminate for MATLABSystem: '<S13>/SourceBlock' */
  if (!pid_control_V1_DW.obj_m.matlabCodegenIsDeleted) {
    pid_control_V1_DW.obj_m.matlabCodegenIsDeleted = true;
    if ((pid_control_V1_DW.obj_m.isInitialized == 1) &&
        pid_control_V1_DW.obj_m.isSetupComplete) {
      Sub_pid_control_V1_435.resetSubscriberPtr();//();
    }
  }

  /* End of Terminate for MATLABSystem: '<S13>/SourceBlock' */

  /* Terminate for MATLABSystem: '<S15>/SourceBlock' */
  if (!pid_control_V1_DW.obj_k.matlabCodegenIsDeleted) {
    pid_control_V1_DW.obj_k.matlabCodegenIsDeleted = true;
    if ((pid_control_V1_DW.obj_k.isInitialized == 1) &&
        pid_control_V1_DW.obj_k.isSetupComplete) {
      Sub_pid_control_V1_377.resetSubscriberPtr();//();
    }
  }

  /* End of Terminate for MATLABSystem: '<S15>/SourceBlock' */

  /* Terminate for MATLABSystem: '<S14>/SourceBlock' */
  if (!pid_control_V1_DW.obj_i.matlabCodegenIsDeleted) {
    pid_control_V1_DW.obj_i.matlabCodegenIsDeleted = true;
    if ((pid_control_V1_DW.obj_i.isInitialized == 1) &&
        pid_control_V1_DW.obj_i.isSetupComplete) {
      Sub_pid_control_V1_538.resetSubscriberPtr();//();
    }
  }

  /* End of Terminate for MATLABSystem: '<S14>/SourceBlock' */

  /* Terminate for Atomic SubSystem: '<Root>/Call Service' */
  /* Terminate for MATLABSystem: '<S2>/ServiceCaller' */
  if (!pid_control_V1_DW.obj_j.matlabCodegenIsDeleted) {
    pid_control_V1_DW.obj_j.matlabCodegenIsDeleted = true;
    if ((pid_control_V1_DW.obj_j.isInitialized == 1) &&
        pid_control_V1_DW.obj_j.isSetupComplete) {
      ServCall_pid_control_V1_326.resetSvcClientPtr();//();
    }
  }

  /* End of Terminate for MATLABSystem: '<S2>/ServiceCaller' */
  /* End of Terminate for SubSystem: '<Root>/Call Service' */

  /* Terminate for MATLABSystem: '<S288>/SourceBlock' */
  if (!pid_control_V1_DW.obj_h.matlabCodegenIsDeleted) {
    pid_control_V1_DW.obj_h.matlabCodegenIsDeleted = true;
    if ((pid_control_V1_DW.obj_h.isInitialized == 1) &&
        pid_control_V1_DW.obj_h.isSetupComplete) {
      Sub_pid_control_V1_417.resetSubscriberPtr();//();
    }
  }

  /* End of Terminate for MATLABSystem: '<S288>/SourceBlock' */

  /* Terminate for MATLABSystem: '<S285>/SourceBlock' */
  if (!pid_control_V1_DW.obj_h4.matlabCodegenIsDeleted) {
    pid_control_V1_DW.obj_h4.matlabCodegenIsDeleted = true;
    if ((pid_control_V1_DW.obj_h4.isInitialized == 1) &&
        pid_control_V1_DW.obj_h4.isSetupComplete) {
      Sub_pid_control_V1_423.resetSubscriberPtr();//();
    }
  }

  /* End of Terminate for MATLABSystem: '<S285>/SourceBlock' */

  /* Terminate for MATLABSystem: '<S286>/SourceBlock' */
  if (!pid_control_V1_DW.obj_hy.matlabCodegenIsDeleted) {
    pid_control_V1_DW.obj_hy.matlabCodegenIsDeleted = true;
    if ((pid_control_V1_DW.obj_hy.isInitialized == 1) &&
        pid_control_V1_DW.obj_hy.isSetupComplete) {
      Sub_pid_control_V1_443.resetSubscriberPtr();//();
    }
  }

  /* End of Terminate for MATLABSystem: '<S286>/SourceBlock' */

  /* Terminate for MATLABSystem: '<S287>/SourceBlock' */
  if (!pid_control_V1_DW.obj_p.matlabCodegenIsDeleted) {
    pid_control_V1_DW.obj_p.matlabCodegenIsDeleted = true;
    if ((pid_control_V1_DW.obj_p.isInitialized == 1) &&
        pid_control_V1_DW.obj_p.isSetupComplete) {
      Sub_pid_control_V1_445.resetSubscriberPtr();//();
    }
  }

  /* End of Terminate for MATLABSystem: '<S287>/SourceBlock' */
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
