/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: open_differential_no_PID.c
 *
 * Code generated for Simulink model 'open_differential_no_PID'.
 *
 * Model version                  : 1.18
 * Simulink Coder version         : 24.2 (R2024b) 21-Jun-2024
 * C/C++ source code generated on : Sun Jun  7 01:58:54 2026
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
DW rtDW;

/* External inputs (root inport signals with default storage) */
ExtU simulink_OD_No_PID_inports;

/* External outputs (root outports fed by signals with default storage) */
ExtY simulink_OD_No_PID_outports;

/* Real-time model */
static RT_MODEL rtM_;
RT_MODEL *const rtM = &rtM_;

/* Model step function */
void open_differential_no_PID_step(void)
{
  real_T rtb_Gain_g;
  real_T rtb_Max1_idx_0;
  real_T rtb_Max1_idx_1;
  real_T rtb_Max1_idx_3;
  real_T rtb_Product;
  boolean_T slip_status_FL;
  boolean_T slip_status_FR;
  boolean_T slip_status_RL;
  boolean_T slip_status_RR;

  /* MinMax: '<S4>/Max' incorporates:
   *  Constant: '<S4>/Constant'
   *  Constant: '<S4>/Vmin'
   *  Inport: '<Root>/Car_Speed(Vx)'
   *  MinMax: '<S4>/Max2'
   */
  rtb_Gain_g = fmax(fmax(simulink_OD_No_PID_inports.Car_SpeedVx, 0.0), 0.2);

  /* MinMax: '<S4>/Max1' incorporates:
   *  Constant: '<S4>/Constant1'
   *  Inport: '<Root>/Wheel_Speed_FL'
   *  Inport: '<Root>/Wheel_Speed_FR'
   *  Inport: '<Root>/Wheel_Speed_RL'
   *  Product: '<S4>/Divide'
   *  Sum: '<S4>/Subtract'
   */
  rtb_Max1_idx_0 = fmax((simulink_OD_No_PID_inports.Wheel_Speed_FL - rtb_Gain_g) / rtb_Gain_g, 0.0);
  rtb_Max1_idx_1 = fmax((simulink_OD_No_PID_inports.Wheel_Speed_FR - rtb_Gain_g) / rtb_Gain_g, 0.0);
  rtb_Max1_idx_3 = fmax((simulink_OD_No_PID_inports.Wheel_Speed_RL - rtb_Gain_g) / rtb_Gain_g, 0.0);

  /* Outport: '<Root>/Slip_FL' */
  simulink_OD_No_PID_outports.Slip_FL = rtb_Max1_idx_0;

  /* Outport: '<Root>/Slip_FR' */
  simulink_OD_No_PID_outports.Slip_FR = rtb_Max1_idx_1;

  /* Outport: '<Root>/Slip_RL' */
  simulink_OD_No_PID_outports.Slip_RL = rtb_Max1_idx_3;

  /* Outport: '<Root>/Slip_RR' incorporates:
   *  Constant: '<S4>/Constant1'
   *  Inport: '<Root>/Wheel_Speed_RR'
   *  MinMax: '<S4>/Max1'
   *  Product: '<S4>/Divide'
   *  Sum: '<S4>/Subtract'
   */
  simulink_OD_No_PID_outports.Slip_RR = fmax((simulink_OD_No_PID_inports.Wheel_Speed_RR - rtb_Gain_g) / rtb_Gain_g, 0.0);

  /* Product: '<S5>/Product' incorporates:
   *  Gain: '<S5>/Gain'
   *  Inport: '<Root>/Maximum_Torque'
   *  Inport: '<Root>/Throttle(%)'
   */
  rtb_Product = 0.01 * simulink_OD_No_PID_inports.Throttle * simulink_OD_No_PID_inports.Maximum_Torque;

  /* MATLAB Function: '<S1>/MATLAB Function' incorporates:
   *  Inport: '<Root>/Slip_Traction_Lim'
   */
  rtb_Gain_g = simulink_OD_No_PID_inports.Slip_Traction_Lim - 0.03;
  if (simulink_OD_No_PID_inports.Slip_Traction_Lim - 0.03 < 0.0) {
    rtb_Gain_g = 0.0;
  }

  slip_status_FL = (rtb_Max1_idx_0 > simulink_OD_No_PID_inports.Slip_Traction_Lim);
  slip_status_FR = (rtb_Max1_idx_1 > simulink_OD_No_PID_inports.Slip_Traction_Lim);
  rtDW.recovering_traction_o = (slip_status_FL || slip_status_FR ||
    (((!(rtb_Max1_idx_0 < rtb_Gain_g)) || (!(rtb_Max1_idx_1 < rtb_Gain_g))) &&
     rtDW.recovering_traction_o));
  if (rtDW.recovering_traction_o) {
    rtDW.active_torque_cmd_g -= 0.25;
  } else {
    rtDW.active_torque_cmd_g += 0.25;
  }

  if (rtDW.active_torque_cmd_g > rtb_Product) {
    rtDW.active_torque_cmd_g = rtb_Product;
  }

  if (rtDW.active_torque_cmd_g < 0.0) {
    rtDW.active_torque_cmd_g = 0.0;
  }

  /* MATLAB Function: '<S2>/MATLAB Function' incorporates:
   *  Inport: '<Root>/Slip_Traction_Lim'
   *  MATLAB Function: '<S1>/MATLAB Function'
   */
  rtb_Gain_g = simulink_OD_No_PID_inports.Slip_Traction_Lim - 0.03;
  if (simulink_OD_No_PID_inports.Slip_Traction_Lim - 0.03 < 0.0) {
    rtb_Gain_g = 0.0;
  }

  slip_status_RL = (rtb_Max1_idx_3 > simulink_OD_No_PID_inports.Slip_Traction_Lim);
  slip_status_RR = (rtb_Max1_idx_3 > simulink_OD_No_PID_inports.Slip_Traction_Lim);
  rtDW.recovering_traction = (slip_status_RL || slip_status_RR ||
    ((!(rtb_Max1_idx_3 < rtb_Gain_g)) && rtDW.recovering_traction));
  if (rtDW.recovering_traction) {
    rtDW.active_torque_cmd -= 0.25;
  } else {
    rtDW.active_torque_cmd += 0.25;
  }

  if (rtDW.active_torque_cmd > rtb_Product) {
    rtDW.active_torque_cmd = rtb_Product;
  }

  if (rtDW.active_torque_cmd < 0.0) {
    rtDW.active_torque_cmd = 0.0;
  }

  /* Gain: '<S3>/1//kt' incorporates:
   *  MATLAB Function: '<S1>/MATLAB Function'
   *  MATLAB Function: '<S2>/MATLAB Function'
   */
  rtb_Max1_idx_1 = 3.8910505836575875 * rtDW.active_torque_cmd_g;
  rtb_Max1_idx_0 = rtb_Max1_idx_1;
  rtb_Max1_idx_3 = 3.8910505836575875 * rtDW.active_torque_cmd;

  /* Gain: '<S3>/Gain' incorporates:
   *  Inport: '<Root>/Current_Limit'
   */
  rtb_Gain_g = 0.25 * simulink_OD_No_PID_inports.Current_Limit;

  /* Switch: '<S8>/Switch2' incorporates:
   *  Gain: '<S3>/1//kt'
   *  RelationalOperator: '<S8>/LowerRelop1'
   */
  if (rtb_Max1_idx_1 > rtb_Gain_g) {
    rtb_Max1_idx_0 = rtb_Gain_g;
    rtb_Max1_idx_1 = rtb_Gain_g;
  }

  if (rtb_Max1_idx_3 > rtb_Gain_g) {
    rtb_Max1_idx_3 = rtb_Gain_g;
  }

  /* End of Switch: '<S8>/Switch2' */

  /* Outport: '<Root>/Current_FL' */
  simulink_OD_No_PID_outports.Current_FL = rtb_Max1_idx_0;

  /* Outport: '<Root>/Current_FR' */
  simulink_OD_No_PID_outports.Current_FR = rtb_Max1_idx_1;

  /* Outport: '<Root>/Current_RL' */
  simulink_OD_No_PID_outports.Current_RL = rtb_Max1_idx_3;

  /* Outport: '<Root>/Current_RR' */
  simulink_OD_No_PID_outports.Current_RR = rtb_Max1_idx_3;

  /* Outport: '<Root>/slip_status_RL' incorporates:
   *  MATLAB Function: '<S2>/MATLAB Function'
   */
  simulink_OD_No_PID_outports.slip_status_RL = slip_status_RR;

  /* Outport: '<Root>/slip_status_RR' incorporates:
   *  MATLAB Function: '<S2>/MATLAB Function'
   */
  simulink_OD_No_PID_outports.slip_status_RR = slip_status_RL;

  /* Outport: '<Root>/Torque_RL' incorporates:
   *  MATLAB Function: '<S2>/MATLAB Function'
   */
  simulink_OD_No_PID_outports.Torque_RL = rtDW.active_torque_cmd;

  /* Outport: '<Root>/Torque_RR' incorporates:
   *  MATLAB Function: '<S2>/MATLAB Function'
   */
  simulink_OD_No_PID_outports.Torque_RR = rtDW.active_torque_cmd;

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
  simulink_OD_No_PID_outports.Torque_FL = rtDW.active_torque_cmd_g;

  /* Outport: '<Root>/Torque_FR' incorporates:
   *  MATLAB Function: '<S1>/MATLAB Function'
   */
  simulink_OD_No_PID_outports.Torque_FR = rtDW.active_torque_cmd_g;

  /* Outport: '<Root>/Total_Torque_Cmd' */
  simulink_OD_No_PID_outports.Total_Torque_Cmd = rtb_Product;

  /* Outport: '<Root>/Total_Current_Cmd' incorporates:
   *  Gain: '<S5>/Gain1'
   */
  simulink_OD_No_PID_outports.Total_Current_Cmd = 3.8910505836575875 * rtb_Product;
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
