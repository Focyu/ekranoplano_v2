/*
 * pid_control_V1_types.h
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

#ifndef pid_control_V1_types_h_
#define pid_control_V1_types_h_
#include "rtwtypes.h"
#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_ROSVariableLengthArrayInfo_
#define DEFINED_TYPEDEF_FOR_SL_Bus_ROSVariableLengthArrayInfo_

struct SL_Bus_ROSVariableLengthArrayInfo
{
  uint32_T CurrentLength;
  uint32_T ReceivedLength;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_geometry_msgs_Point_
#define DEFINED_TYPEDEF_FOR_SL_Bus_geometry_msgs_Point_

struct SL_Bus_geometry_msgs_Point
{
  real_T x;
  real_T y;
  real_T z;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_geometry_msgs_Quaternion_
#define DEFINED_TYPEDEF_FOR_SL_Bus_geometry_msgs_Quaternion_

struct SL_Bus_geometry_msgs_Quaternion
{
  real_T x;
  real_T y;
  real_T z;
  real_T w;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_geometry_msgs_Pose_
#define DEFINED_TYPEDEF_FOR_SL_Bus_geometry_msgs_Pose_

struct SL_Bus_geometry_msgs_Pose
{
  SL_Bus_geometry_msgs_Point position;
  SL_Bus_geometry_msgs_Quaternion orientation;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_geometry_msgs_Vector3_
#define DEFINED_TYPEDEF_FOR_SL_Bus_geometry_msgs_Vector3_

struct SL_Bus_geometry_msgs_Vector3
{
  real_T x;
  real_T y;
  real_T z;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_geometry_msgs_Twist_
#define DEFINED_TYPEDEF_FOR_SL_Bus_geometry_msgs_Twist_

struct SL_Bus_geometry_msgs_Twist
{
  SL_Bus_geometry_msgs_Vector3 linear;
  SL_Bus_geometry_msgs_Vector3 angular;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_gazebo_msgs_EntityState_
#define DEFINED_TYPEDEF_FOR_SL_Bus_gazebo_msgs_EntityState_

struct SL_Bus_gazebo_msgs_EntityState
{
  uint8_T name[128];
  SL_Bus_ROSVariableLengthArrayInfo name_SL_Info;
  SL_Bus_geometry_msgs_Pose pose;
  SL_Bus_geometry_msgs_Twist twist;
  uint8_T reference_frame[128];
  SL_Bus_ROSVariableLengthArrayInfo reference_frame_SL_Info;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_gazebo_msgs_SetEntityStateRequest_
#define DEFINED_TYPEDEF_FOR_SL_Bus_gazebo_msgs_SetEntityStateRequest_

struct SL_Bus_gazebo_msgs_SetEntityStateRequest
{
  SL_Bus_gazebo_msgs_EntityState state;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_gazebo_msgs_SetEntityStateResponse_
#define DEFINED_TYPEDEF_FOR_SL_Bus_gazebo_msgs_SetEntityStateResponse_

struct SL_Bus_gazebo_msgs_SetEntityStateResponse
{
  boolean_T success;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_std_msgs_Bool_
#define DEFINED_TYPEDEF_FOR_SL_Bus_std_msgs_Bool_

struct SL_Bus_std_msgs_Bool
{
  boolean_T data;
};

#endif

#ifndef DEFINED_TYPEDEF_FOR_SL_Bus_std_msgs_Float64_
#define DEFINED_TYPEDEF_FOR_SL_Bus_std_msgs_Float64_

struct SL_Bus_std_msgs_Float64
{
  real_T data;
};

#endif

#ifndef struct_sJ4ih70VmKcvCeguWN0mNVF
#define struct_sJ4ih70VmKcvCeguWN0mNVF

struct sJ4ih70VmKcvCeguWN0mNVF
{
  real_T sec;
  real_T nsec;
};

#endif                                 /* struct_sJ4ih70VmKcvCeguWN0mNVF */

#ifndef struct_ros_slros2_internal_block_Ser_T
#define struct_ros_slros2_internal_block_Ser_T

struct ros_slros2_internal_block_Ser_T
{
  boolean_T matlabCodegenIsDeleted;
  int32_T isInitialized;
  boolean_T isSetupComplete;
  boolean_T QOSAvoidROSNamespaceConventions;
};

#endif                              /* struct_ros_slros2_internal_block_Ser_T */

#ifndef struct_robotics_slcore_internal_bloc_T
#define struct_robotics_slcore_internal_bloc_T

struct robotics_slcore_internal_bloc_T
{
  int32_T isInitialized;
};

#endif                              /* struct_robotics_slcore_internal_bloc_T */

#ifndef struct_ros_slros2_internal_block_Sub_T
#define struct_ros_slros2_internal_block_Sub_T

struct ros_slros2_internal_block_Sub_T
{
  boolean_T matlabCodegenIsDeleted;
  int32_T isInitialized;
  boolean_T isSetupComplete;
  boolean_T QOSAvoidROSNamespaceConventions;
};

#endif                              /* struct_ros_slros2_internal_block_Sub_T */

#ifndef struct_sI9OZ8YWn5qr2iby6yfJzBB_pid_c_T
#define struct_sI9OZ8YWn5qr2iby6yfJzBB_pid_c_T

struct sI9OZ8YWn5qr2iby6yfJzBB_pid_c_T
{
  real_T Numerator;
  real_T Denominator[2];
};

#endif                              /* struct_sI9OZ8YWn5qr2iby6yfJzBB_pid_c_T */

#ifndef struct_rtString_pid_control_V1_T
#define struct_rtString_pid_control_V1_T

struct rtString_pid_control_V1_T
{
  char_T Value[12];
};

#endif                                 /* struct_rtString_pid_control_V1_T */

#ifndef struct_d_fusion_internal_Acceleromet_T
#define struct_d_fusion_internal_Acceleromet_T

struct d_fusion_internal_Acceleromet_T
{
  boolean_T tunablePropertyChanged[12];
  int32_T isInitialized;
  boolean_T TunablePropsChanged;
  real_T MeasurementRange;
  real_T Resolution;
  real_T ConstantBias[3];
  real_T AxesMisalignment[9];
  real_T NoiseDensity[3];
  real_T BiasInstability[3];
  real_T RandomWalk[3];
  sI9OZ8YWn5qr2iby6yfJzBB_pid_c_T BiasInstabilityCoefficients;
  rtString_pid_control_V1_T NoiseType;
  real_T TemperatureBias[3];
  real_T TemperatureScaleFactor[3];
  real_T Temperature;
  real_T pBandwidth;
  real_T pCorrelationTime;
  real_T pBiasInstFilterNum;
  real_T pBiasInstFilterDen[2];
  real_T pBiasInstFilterStates[3];
  real_T pStdDevBiasInst[3];
  real_T pStdDevWhiteNoise[3];
  real_T pRandWalkFilterStates[3];
  real_T pStdDevRandWalk[3];
  real_T pGain[9];
};

#endif                              /* struct_d_fusion_internal_Acceleromet_T */

#ifndef struct_e_fusion_internal_GyroscopeSi_T
#define struct_e_fusion_internal_GyroscopeSi_T

struct e_fusion_internal_GyroscopeSi_T
{
  boolean_T tunablePropertyChanged[13];
  int32_T isInitialized;
  boolean_T TunablePropsChanged;
  real_T MeasurementRange;
  real_T Resolution;
  real_T ConstantBias[3];
  real_T AxesMisalignment[9];
  real_T NoiseDensity[3];
  real_T BiasInstability[3];
  real_T RandomWalk[3];
  sI9OZ8YWn5qr2iby6yfJzBB_pid_c_T BiasInstabilityCoefficients;
  rtString_pid_control_V1_T NoiseType;
  real_T TemperatureBias[3];
  real_T TemperatureScaleFactor[3];
  real_T Temperature;
  real_T pBandwidth;
  real_T pCorrelationTime;
  real_T pBiasInstFilterNum;
  real_T pBiasInstFilterDen[2];
  real_T pBiasInstFilterStates[3];
  real_T pStdDevBiasInst[3];
  real_T pStdDevWhiteNoise[3];
  real_T pRandWalkFilterStates[3];
  real_T pStdDevRandWalk[3];
  real_T pGain[9];
  real_T AccelerationBias[3];
  real_T pAcceleration[3];
};

#endif                              /* struct_e_fusion_internal_GyroscopeSi_T */

#ifndef struct_e_fusion_internal_Magnetomete_T
#define struct_e_fusion_internal_Magnetomete_T

struct e_fusion_internal_Magnetomete_T
{
  boolean_T tunablePropertyChanged[12];
  int32_T isInitialized;
  boolean_T TunablePropsChanged;
  real_T MeasurementRange;
  real_T Resolution;
  real_T ConstantBias[3];
  real_T AxesMisalignment[9];
  real_T NoiseDensity[3];
  real_T BiasInstability[3];
  real_T RandomWalk[3];
  sI9OZ8YWn5qr2iby6yfJzBB_pid_c_T BiasInstabilityCoefficients;
  rtString_pid_control_V1_T NoiseType;
  real_T TemperatureBias[3];
  real_T TemperatureScaleFactor[3];
  real_T Temperature;
  real_T pBandwidth;
  real_T pCorrelationTime;
  real_T pBiasInstFilterNum;
  real_T pBiasInstFilterDen[2];
  real_T pBiasInstFilterStates[3];
  real_T pStdDevBiasInst[3];
  real_T pStdDevWhiteNoise[3];
  real_T pRandWalkFilterStates[3];
  real_T pStdDevRandWalk[3];
  real_T pGain[9];
};

#endif                              /* struct_e_fusion_internal_Magnetomete_T */

#ifndef struct_fusion_simulink_imuSensor_pid_T
#define struct_fusion_simulink_imuSensor_pid_T

struct fusion_simulink_imuSensor_pid_T
{
  boolean_T tunablePropertyChanged[38];
  int32_T isInitialized;
  boolean_T TunablePropsChanged;
  real_T Temperature;
  uint32_T pStreamState[625];
  d_fusion_internal_Acceleromet_T *pAccel;
  e_fusion_internal_GyroscopeSi_T *pGyro;
  e_fusion_internal_Magnetomete_T *pMag;
  real_T MagneticFieldNED[3];
  real_T MagneticField[3];
  real_T AccelParamsMeasurementRange;
  real_T AccelParamsResolution;
  real_T AccelParamsConstantBias[3];
  real_T AccelParamsAxesMisalignment[9];
  real_T AccelParamsNoiseDensity[3];
  real_T AccelParamsBiasInstability[3];
  real_T AccelParamsBiasInstabilityNumerator;
  real_T AccelParamsBiasInstabilityDenominator[2];
  real_T AccelParamsRandomWalk[3];
  real_T AccelParamsTemperatureBias[3];
  real_T AccelParamsTemperatureScaleFactor[3];
  real_T GyroParamsMeasurementRange;
  real_T GyroParamsResolution;
  real_T GyroParamsConstantBias[3];
  real_T GyroParamsAxesMisalignment[9];
  real_T GyroParamsNoiseDensity[3];
  real_T GyroParamsBiasInstability[3];
  real_T GyroParamsBiasInstabilityNumerator;
  real_T GyroParamsBiasInstabilityDenominator[2];
  real_T GyroParamsRandomWalk[3];
  real_T GyroParamsTemperatureBias[3];
  real_T GyroParamsTemperatureScaleFactor[3];
  real_T GyroParamsAccelerationBias[3];
  real_T MagParamsMeasurementRange;
  real_T MagParamsResolution;
  real_T MagParamsConstantBias[3];
  real_T MagParamsAxesMisalignment[9];
  real_T MagParamsNoiseDensity[3];
  real_T MagParamsBiasInstability[3];
  real_T MagParamsBiasInstabilityNumerator;
  real_T MagParamsBiasInstabilityDenominator[2];
  real_T MagParamsRandomWalk[3];
  real_T MagParamsTemperatureBias[3];
  real_T MagParamsTemperatureScaleFactor[3];
  e_fusion_internal_Magnetomete_T coder_buffer_pobj0;
  e_fusion_internal_GyroscopeSi_T coder_buffer_pobj1;
  d_fusion_internal_Acceleromet_T coder_buffer_pobj2;
};

#endif                              /* struct_fusion_simulink_imuSensor_pid_T */

/* Forward declaration for rtModel */
typedef struct tag_RTM_pid_control_V1_T RT_MODEL_pid_control_V1_T;

#endif                                 /* pid_control_V1_types_h_ */
