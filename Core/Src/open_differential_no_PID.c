/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: open_differential_no_PID.c
 *
 * Code generated for Simulink model 'open_differential_no_PID'.
 *
 * Model version                  : 1.28
 * Simulink Coder version         : 24.2 (R2024b) 21-Jun-2024
 * C/C++ source code generated on : Wed Jun 10 02:57:15 2026
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. RAM efficiency
 * Validation result: Not run
 */

#include "open_differential_no_PID.h"
#include <math.h>
#include "rtwtypes.h"

/* Block signals and states (default storage) */
DW_OD_No_PID simulink_OD_No_PID_rtDW;

/* External inputs (root inport signals with default storage) */
ExtU_OD_No_PID simulink_OD_No_PID_inports;

/* External outputs (root outports fed by signals with default storage) */
ExtY_OD_No_PID simulink_OD_No_PID_outports;

/* Real-time model */
static RT_MODEL_OD_No_PID simulink_OD_No_PID_rtM_;
RT_MODEL_OD_No_PID *const simulink_OD_No_PID_rtM = &simulink_OD_No_PID_rtM_;

/* Model step function */
void open_differential_no_PID_step(void)
{
  real_T Vx;
  real_T Vx_FL;
  real_T Vx_FR;
  real_T Vx_RL;
  real_T positive_slip_FL;
  real_T positive_slip_FR;
  real_T rtb_Switch2_idx_2;
  real_T rtb_Switch2_idx_3;
  real_T slip_low;
  real_T yaw_rate_rad_s;
  boolean_T slip_status_FL;
  boolean_T slip_status_FR;
  boolean_T slip_status_RL;
  boolean_T slip_status_RR;

  /* MATLAB Function: '<S4>/MATLAB Function' incorporates:
   *  Constant: '<S4>/Track_Wdith_Front_m'
   *  Constant: '<S4>/Track_Wdith_Rear_m'
   *  Inport: '<Root>/Car_Speed(Vx)'
   *  Inport: '<Root>/Wheel_Speed_FL'
   *  Inport: '<Root>/Wheel_Speed_FR'
   *  Inport: '<Root>/Wheel_Speed_RL'
   *  Inport: '<Root>/Yaw_Rate(deg//s)'
   */
  yaw_rate_rad_s = simulink_OD_No_PID_inports.Yaw_Ratedegs * 0.017453292519943295;
  Vx = fmax(simulink_OD_No_PID_inports.Car_SpeedVx, 0.2);
  Vx_FR = yaw_rate_rad_s * 1.19 / 2.0;
  Vx_FL = fmax(Vx - Vx_FR, 0.2);
  Vx_FR = fmax(Vx_FR + Vx, 0.2);
  yaw_rate_rad_s = yaw_rate_rad_s * 1.23 / 2.0;
  Vx_RL = fmax(Vx - yaw_rate_rad_s, 0.2);
  yaw_rate_rad_s = fmax(yaw_rate_rad_s + Vx, 0.2);
  Vx_FL = (simulink_OD_No_PID_inports.Wheel_Speed_FL - Vx_FL) / Vx_FL;
  Vx_FR = (simulink_OD_No_PID_inports.Wheel_Speed_FR - Vx_FR) / Vx_FR;
  Vx_RL = (simulink_OD_No_PID_inports.Wheel_Speed_RL - Vx_RL) / Vx_RL;

  /* Product: '<S5>/Product' incorporates:
   *  Gain: '<S5>/Gain'
   *  Inport: '<Root>/Maximum_Torque'
   *  Inport: '<Root>/Throttle(%)'
   */
  Vx = 0.01 * simulink_OD_No_PID_inports.Throttle * simulink_OD_No_PID_inports.Maximum_Torque;

  /* MATLAB Function: '<S1>/MATLAB Function' incorporates:
   *  Inport: '<Root>/Slip_Traction_Lim'
   */
  positive_slip_FL = fmax(Vx_FL, 0.0);
  positive_slip_FR = fmax(Vx_FR, 0.0);
  slip_low = simulink_OD_No_PID_inports.Slip_Traction_Lim - 0.03;
  if (simulink_OD_No_PID_inports.Slip_Traction_Lim - 0.03 < 0.0) {
    slip_low = 0.0;
  }

  slip_status_FL = (positive_slip_FL > simulink_OD_No_PID_inports.Slip_Traction_Lim);
  slip_status_FR = (positive_slip_FR > simulink_OD_No_PID_inports.Slip_Traction_Lim);
  simulink_OD_No_PID_rtDW.recovering_traction_o = (slip_status_FL || slip_status_FR ||
    (((!(positive_slip_FL < slip_low)) || (!(positive_slip_FR < slip_low))) &&
     simulink_OD_No_PID_rtDW.recovering_traction_o));
  if (simulink_OD_No_PID_rtDW.recovering_traction_o) {
    simulink_OD_No_PID_rtDW.active_torque_cmd_g -= 0.25;
  } else {
    simulink_OD_No_PID_rtDW.active_torque_cmd_g += 0.25;
  }

  if (simulink_OD_No_PID_rtDW.active_torque_cmd_g > Vx) {
    simulink_OD_No_PID_rtDW.active_torque_cmd_g = Vx;
  }

  if (simulink_OD_No_PID_rtDW.active_torque_cmd_g < 0.0) {
    simulink_OD_No_PID_rtDW.active_torque_cmd_g = 0.0;
  }

  /* MATLAB Function: '<S3>/MATLAB Function' incorporates:
   *  Inport: '<Root>/Slip_Traction_Lim'
   *  MATLAB Function: '<S1>/MATLAB Function'
   */
  positive_slip_FL = fmax(Vx_RL, 0.0);
  slip_low = simulink_OD_No_PID_inports.Slip_Traction_Lim - 0.03;
  if (simulink_OD_No_PID_inports.Slip_Traction_Lim - 0.03 < 0.0) {
    slip_low = 0.0;
  }

  slip_status_RL = (positive_slip_FL > simulink_OD_No_PID_inports.Slip_Traction_Lim);
  slip_status_RR = (positive_slip_FL > simulink_OD_No_PID_inports.Slip_Traction_Lim);
  simulink_OD_No_PID_rtDW.recovering_traction = (slip_status_RL || slip_status_RR ||
    ((!(positive_slip_FL < slip_low)) && simulink_OD_No_PID_rtDW.recovering_traction));
  if (simulink_OD_No_PID_rtDW.recovering_traction) {
    simulink_OD_No_PID_rtDW.active_torque_cmd -= 0.25;
  } else {
    simulink_OD_No_PID_rtDW.active_torque_cmd += 0.25;
  }

  if (simulink_OD_No_PID_rtDW.active_torque_cmd > Vx) {
    simulink_OD_No_PID_rtDW.active_torque_cmd = Vx;
  }

  if (simulink_OD_No_PID_rtDW.active_torque_cmd < 0.0) {
    simulink_OD_No_PID_rtDW.active_torque_cmd = 0.0;
  }

  /* Gain: '<S2>/Gain1' incorporates:
   *  Inport: '<Root>/Maximum_Torque'
   */
  slip_low = 0.25 * simulink_OD_No_PID_inports.Maximum_Torque;

  /* Switch: '<S8>/Switch2' incorporates:
   *  MATLAB Function: '<S1>/MATLAB Function'
   *  RelationalOperator: '<S8>/LowerRelop1'
   */
  if (simulink_OD_No_PID_rtDW.active_torque_cmd_g > slip_low) {
    rtb_Switch2_idx_2 = slip_low;
  } else {
    rtb_Switch2_idx_2 = simulink_OD_No_PID_rtDW.active_torque_cmd_g;
  }

  /* Gain: '<S2>/1//kt' incorporates:
   *  Switch: '<S8>/Switch2'
   */
  positive_slip_FL = 3.8910505836575875 * rtb_Switch2_idx_2;

  /* Switch: '<S8>/Switch2' incorporates:
   *  MATLAB Function: '<S1>/MATLAB Function'
   *  RelationalOperator: '<S8>/LowerRelop1'
   */
  if (simulink_OD_No_PID_rtDW.active_torque_cmd_g > slip_low) {
    rtb_Switch2_idx_2 = slip_low;
  } else {
    rtb_Switch2_idx_2 = simulink_OD_No_PID_rtDW.active_torque_cmd_g;
  }

  /* Gain: '<S2>/1//kt' incorporates:
   *  Switch: '<S8>/Switch2'
   */
  positive_slip_FR = 3.8910505836575875 * rtb_Switch2_idx_2;

  /* Switch: '<S8>/Switch2' incorporates:
   *  MATLAB Function: '<S3>/MATLAB Function'
   *  RelationalOperator: '<S8>/LowerRelop1'
   */
  if (simulink_OD_No_PID_rtDW.active_torque_cmd > slip_low) {
    rtb_Switch2_idx_2 = slip_low;
  } else {
    rtb_Switch2_idx_2 = simulink_OD_No_PID_rtDW.active_torque_cmd;
    slip_low = simulink_OD_No_PID_rtDW.active_torque_cmd;
  }

  /* Gain: '<S2>/1//kt' incorporates:
   *  Switch: '<S8>/Switch2'
   */
  rtb_Switch2_idx_2 *= 3.8910505836575875;
  rtb_Switch2_idx_3 = 3.8910505836575875 * slip_low;

  /* Gain: '<S2>/Gain' incorporates:
   *  Inport: '<Root>/Current_Limit'
   */
  slip_low = 0.25 * simulink_OD_No_PID_inports.Current_Limit;

  /* Switch: '<S7>/Switch2' incorporates:
   *  Constant: '<S2>/Constant'
   *  RelationalOperator: '<S7>/LowerRelop1'
   *  RelationalOperator: '<S7>/UpperRelop'
   *  Switch: '<S7>/Switch'
   */
  if (positive_slip_FL > slip_low) {
    positive_slip_FL = slip_low;
  } else if (positive_slip_FL < 0.0) {
    /* Switch: '<S7>/Switch' incorporates:
     *  Constant: '<S2>/Constant'
     */
    positive_slip_FL = 0.0;
  }

  if (positive_slip_FR > slip_low) {
    positive_slip_FR = slip_low;
  } else if (positive_slip_FR < 0.0) {
    /* Switch: '<S7>/Switch' incorporates:
     *  Constant: '<S2>/Constant'
     */
    positive_slip_FR = 0.0;
  }

  if (rtb_Switch2_idx_2 > slip_low) {
    rtb_Switch2_idx_2 = slip_low;
  } else if (rtb_Switch2_idx_2 < 0.0) {
    /* Switch: '<S7>/Switch' incorporates:
     *  Constant: '<S2>/Constant'
     */
    rtb_Switch2_idx_2 = 0.0;
  }

  if (rtb_Switch2_idx_3 > slip_low) {
    rtb_Switch2_idx_3 = slip_low;
  } else if (rtb_Switch2_idx_3 < 0.0) {
    /* Switch: '<S7>/Switch' incorporates:
     *  Constant: '<S2>/Constant'
     */
    rtb_Switch2_idx_3 = 0.0;
  }

  /* End of Switch: '<S7>/Switch2' */

  /* Outport: '<Root>/Current_FL' */
  simulink_OD_No_PID_outports.Current_FL = positive_slip_FL;

  /* Outport: '<Root>/Current_FR' */
  simulink_OD_No_PID_outports.Current_FR = positive_slip_FR;

  /* Outport: '<Root>/Current_RL' */
  simulink_OD_No_PID_outports.Current_RL = rtb_Switch2_idx_2;

  /* Outport: '<Root>/Current_RR' */
  simulink_OD_No_PID_outports.Current_RR = rtb_Switch2_idx_3;

  /* Outport: '<Root>/slip_status_RL' incorporates:
   *  MATLAB Function: '<S3>/MATLAB Function'
   */
  simulink_OD_No_PID_outports.slip_status_RL = slip_status_RR;

  /* Outport: '<Root>/slip_status_RR' incorporates:
   *  MATLAB Function: '<S3>/MATLAB Function'
   */
  simulink_OD_No_PID_outports.slip_status_RR = slip_status_RL;

  /* Outport: '<Root>/Torque_RL' incorporates:
   *  MATLAB Function: '<S3>/MATLAB Function'
   */
  simulink_OD_No_PID_outports.Torque_RL = simulink_OD_No_PID_rtDW.active_torque_cmd;

  /* Outport: '<Root>/Torque_RR' incorporates:
   *  MATLAB Function: '<S3>/MATLAB Function'
   */
  simulink_OD_No_PID_outports.Torque_RR = simulink_OD_No_PID_rtDW.active_torque_cmd;

  /* Outport: '<Root>/slip_status_FL' incorporates:
   *  MATLAB Function: '<S1>/MATLAB Function'
   */
  simulink_OD_No_PID_outports.slip_status_FL = slip_status_FR;

  /* Outport: '<Root>/slip_status_FR' incorporates:
   *  MATLAB Function: '<S1>/MATLAB Function'
   */
  simulink_OD_No_PID_outports.slip_status_FR = slip_status_FL;

  /* Outport: '<Root>/Torque_FL' incorporates:
   *  MATLAB Function: '<S1>/MATLAB Function'
   */
  simulink_OD_No_PID_outports.Torque_FL = simulink_OD_No_PID_rtDW.active_torque_cmd_g;

  /* Outport: '<Root>/Torque_FR' incorporates:
   *  MATLAB Function: '<S1>/MATLAB Function'
   */
  simulink_OD_No_PID_outports.Torque_FR = simulink_OD_No_PID_rtDW.active_torque_cmd_g;

  /* Outport: '<Root>/Total_Torque_Cmd' */
  simulink_OD_No_PID_outports.Total_Torque_Cmd = Vx;

  /* Outport: '<Root>/Total_Current_Cmd' incorporates:
   *  Gain: '<S5>/Gain1'
   */
  simulink_OD_No_PID_outports.Total_Current_Cmd = 3.8910505836575875 * Vx;

  /* Outport: '<Root>/Slip_FL' */
  simulink_OD_No_PID_outports.Slip_FL = Vx_FL;

  /* Outport: '<Root>/Slip_FR' */
  simulink_OD_No_PID_outports.Slip_FR = Vx_FR;

  /* Outport: '<Root>/Slip_RL' */
  simulink_OD_No_PID_outports.Slip_RL = Vx_RL;

  /* Outport: '<Root>/Slip_RR' incorporates:
   *  Inport: '<Root>/Wheel_Speed_RR'
   *  MATLAB Function: '<S4>/MATLAB Function'
   */
  simulink_OD_No_PID_outports.Slip_RR = (simulink_OD_No_PID_inports.Wheel_Speed_RR - yaw_rate_rad_s) / yaw_rate_rad_s;
}

/* Model initialize function */
void open_differential_no_PID_initialize(void)
{
  /* (no initialization code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
