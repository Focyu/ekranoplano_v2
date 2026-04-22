// Copyright 2022-2024 The MathWorks, Inc.
// Generated 22-Apr-2026 18:56:48
#include "slros2_initialize.h"
const std::string SLROSNodeName("pid_control_V1_OLD");
// pid_control_V1_OLD/SUBSYSTEM_MODEL/Subscribe1_TURBULENCIA1
SimulinkSubscriber<std_msgs::msg::Bool,SL_Bus_std_msgs_Bool> Sub_pid_control_V1_OLD_423;
// pid_control_V1_OLD/SUBSYSTEM_MODEL/Subscribe_HEAVE
SimulinkSubscriber<std_msgs::msg::Float64,SL_Bus_std_msgs_Float64> Sub_pid_control_V1_OLD_443;
// pid_control_V1_OLD/SUBSYSTEM_MODEL/Subscribe_RATE
SimulinkSubscriber<std_msgs::msg::Float64,SL_Bus_std_msgs_Float64> Sub_pid_control_V1_OLD_445;
// pid_control_V1_OLD/SUBSYSTEM_MODEL/Subscribe_TURBULENCIA
SimulinkSubscriber<std_msgs::msg::Bool,SL_Bus_std_msgs_Bool> Sub_pid_control_V1_OLD_417;
// pid_control_V1_OLD/Subscribe-ALTURA1
SimulinkSubscriber<std_msgs::msg::Float64,SL_Bus_std_msgs_Float64> Sub_pid_control_V1_OLD_435;
// pid_control_V1_OLD/Subscribe-YAW
SimulinkSubscriber<std_msgs::msg::Float64,SL_Bus_std_msgs_Float64> Sub_pid_control_V1_OLD_377;
// pid_control_V1_OLD/Call Service
SimulinkServiceCaller<gazebo_msgs::srv::SetEntityState,SL_Bus_gazebo_msgs_SetEntityStateRequest,SL_Bus_gazebo_msgs_SetEntityStateResponse> ServCall_pid_control_V1_OLD_326;
