/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: open_differential.c
 *
 * Code generated for Simulink model 'open_differential'.
 *
 * Model version                  : 1.18
 * Simulink Coder version         : 24.2 (R2024b) 21-Jun-2024
 * C/C++ source code generated on : Tue Jun  9 04:11:39 2026
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. RAM efficiency
 * Validation result: Not run
 */

#include "open_differential.h"
#include <math.h>
#include "rtwtypes.h"
#include "zero_crossing_types.h"
#include "solver_zc.h"
#ifndef slZcHadEvent
#define slZcHadEvent(ev, zcsDir)       (((ev) & (zcsDir)) != 0x00 )
#endif

#ifndef slZcUnAliasEvents
#define slZcUnAliasEvents(evL, evR)    ((((slZcHadEvent((evL), (SL_ZCS_EVENT_N2Z)) && slZcHadEvent((evR), (SL_ZCS_EVENT_Z2P))) || (slZcHadEvent((evL), (SL_ZCS_EVENT_P2Z)) && slZcHadEvent((evR), (SL_ZCS_EVENT_Z2N)))) ? (SL_ZCS_EVENT_NUL) : (evR)))
#endif

/* Block signals and states (default storage) */
DW_OD simulink_OD_rtDW;

/* Previous zero-crossings (trigger) states */
PrevZCX_OD simulink_OD_rtPrevZCX;

/* External inputs (root inport signals with default storage) */
ExtU_OD simulink_OD_inports;

/* External outputs (root outports fed by signals with default storage) */
ExtY_OD simulink_OD_outports;

/* Real-time model */
static RT_MODEL_OD simulink_OD_rtM_;
RT_MODEL_OD *const simulink_OD_rtM = &simulink_OD_rtM_;
static ZCEventType rt_ZCFcn(ZCDirection zcDir, ZCSigState *prevZc, real_T
  currValue);

/* Detect zero crossings events. */
static ZCEventType rt_ZCFcn(ZCDirection zcDir, ZCSigState *prevZc, real_T
  currValue)
{
  slZcEventType zcsDir;
  slZcEventType tempEv;
  ZCEventType zcEvent = NO_ZCEVENT;    /* assume */

  /* zcEvent matrix */
  static const slZcEventType eventMatrix[4][4] = {
    /*          ZER              POS              NEG              UNK */
    { SL_ZCS_EVENT_NUL, SL_ZCS_EVENT_Z2P, SL_ZCS_EVENT_Z2N, SL_ZCS_EVENT_NUL },/* ZER */

    { SL_ZCS_EVENT_P2Z, SL_ZCS_EVENT_NUL, SL_ZCS_EVENT_P2N, SL_ZCS_EVENT_NUL },/* POS */

    { SL_ZCS_EVENT_N2Z, SL_ZCS_EVENT_N2P, SL_ZCS_EVENT_NUL, SL_ZCS_EVENT_NUL },/* NEG */

    { SL_ZCS_EVENT_NUL, SL_ZCS_EVENT_NUL, SL_ZCS_EVENT_NUL, SL_ZCS_EVENT_NUL }/* UNK */
  };

  /* get prevZcEvent and prevZcSign from prevZc */
  const slZcEventType prevEv = (slZcEventType)(((uint8_T)(*prevZc)) >> 2);
  const slZcSignalSignType prevSign = (slZcSignalSignType)(((uint8_T)(*prevZc))
    & (uint8_T)0x03);

  /* get current zcSignal sign from current zcSignal value */
  const slZcSignalSignType currSign = (slZcSignalSignType)((currValue) > 0.0 ?
    SL_ZCS_SIGN_POS :
    ((currValue) < 0.0 ? SL_ZCS_SIGN_NEG : SL_ZCS_SIGN_ZERO));

  /* get current zcEvent based on prev and current zcSignal value */
  slZcEventType currEv = eventMatrix[prevSign][currSign];

  /* get slZcEventType from ZCDirection */
  switch (zcDir) {
   case ANY_ZERO_CROSSING:
    zcsDir = SL_ZCS_EVENT_ALL;
    break;

   case FALLING_ZERO_CROSSING:
    zcsDir = SL_ZCS_EVENT_ALL_DN;
    break;

   case RISING_ZERO_CROSSING:
    zcsDir = SL_ZCS_EVENT_ALL_UP;
    break;

   default:
    zcsDir = SL_ZCS_EVENT_NUL;
    break;
  }

  /* had event, check if zc happened */
  if (slZcHadEvent(currEv, zcsDir)) {
    currEv = (slZcEventType)(slZcUnAliasEvents(prevEv, currEv));
  } else {
    currEv = SL_ZCS_EVENT_NUL;
  }

  /* Update prevZc */
  tempEv = (slZcEventType)(currEv << 2);/* shift left by 2 bits */
  *prevZc = (ZCSigState)((currSign) | (tempEv));
  if ((currEv & SL_ZCS_EVENT_ALL_DN) != 0) {
    zcEvent = FALLING_ZCEVENT;
  } else if ((currEv & SL_ZCS_EVENT_ALL_UP) != 0) {
    zcEvent = RISING_ZCEVENT;
  } else {
    zcEvent = NO_ZCEVENT;
  }

  return zcEvent;
}                                      /* rt_ZCFcn */

/* Model step function */
void open_differential_step(void)
{
  real_T Vx;
  real_T Vx_FL;
  real_T Vx_FR;
  real_T Vx_RL;
  real_T rtb_IProdOut;
  real_T rtb_Max1_p;
  real_T rtb_Product;
  real_T rtb_Subtract;
  real_T rtb_Subtract_b;
  real_T rtb_Subtract_tmp;
  real_T rtb_Sum;
  real_T rtb_Sum_m;
  real_T rtb_Switch2_idx_0;
  real_T rtb_Switch2_idx_1;
  real_T rtb_Switch2_idx_2;
  real_T rtb_Switch2_idx_3;
  real_T rtb_Tsamp;
  real_T rtb_Tsamp_g;
  real_T yaw_rate_rad_s;
  int8_T rtb_Switch1_i;
  int8_T tmp;
  boolean_T rtb_RelationalOperator;
  boolean_T rtb_fixforDTpropagationissue;

  /* Product: '<S5>/Product' incorporates:
   *  Gain: '<S5>/Gain'
   *  Inport: '<Root>/Maximum_Torque'
   *  Inport: '<Root>/Throttle(%)'
   */
  rtb_Product = 0.01 * simulink_OD_inports.Throttle * simulink_OD_inports.Maximum_Torque;

  /* Gain: '<S1>/torque_wheel_split' incorporates:
   *  Gain: '<S1>/axle_bias'
   *  Gain: '<S2>/torque_wheel_split'
   */
  rtb_Subtract_tmp = 0.5 * rtb_Product * 0.5;

  /* MATLAB Function: '<S4>/MATLAB Function' incorporates:
   *  Constant: '<S4>/Track_Wdith_Front_m'
   *  Constant: '<S4>/Track_Wdith_Rear_m'
   *  Inport: '<Root>/Car_Speed(Vx)'
   *  Inport: '<Root>/Wheel_Speed_FL'
   *  Inport: '<Root>/Wheel_Speed_FR'
   *  Inport: '<Root>/Wheel_Speed_RL'
   *  Inport: '<Root>/Yaw_Rate(deg//s)'
   */
  yaw_rate_rad_s = simulink_OD_inports.Yaw_Ratedegs * 0.017453292519943295;
  Vx = fmax(simulink_OD_inports.Car_SpeedVx, 0.2);
  Vx_FR = yaw_rate_rad_s * 1.19 / 2.0;
  Vx_FL = fmax(Vx - Vx_FR, 0.2);
  Vx_FR = fmax(Vx_FR + Vx, 0.2);
  yaw_rate_rad_s = yaw_rate_rad_s * 1.23 / 2.0;
  Vx_RL = fmax(Vx - yaw_rate_rad_s, 0.2);
  yaw_rate_rad_s = fmax(yaw_rate_rad_s + Vx, 0.2);
  Vx_FL = fmax((simulink_OD_inports.Wheel_Speed_FL - Vx_FL) / Vx_FL, 0.0);
  Vx_FR = fmax((simulink_OD_inports.Wheel_Speed_FR - Vx_FR) / Vx_FR, 0.0);
  Vx_RL = fmax((simulink_OD_inports.Wheel_Speed_RL - Vx_RL) / Vx_RL, 0.0);

  /* MinMax: '<S1>/Max1' incorporates:
   *  Constant: '<S1>/Constant2'
   *  Inport: '<Root>/Slip_Traction_Lim'
   *  MinMax: '<S1>/Max'
   *  Sum: '<S1>/Subtract1'
   */
  Vx = fmax(fmax(Vx_FL, Vx_FR) - simulink_OD_inports.Slip_Traction_Lim, 0.0);

  /* DiscreteIntegrator: '<S48>/Integrator' incorporates:
   *  Inport: '<Root>/Integral_Reset'
   */
  if ((simulink_OD_inports.Integral_Reset > 0.0) && (simulink_OD_rtDW.Integrator_PrevResetState <= 0)) {
    simulink_OD_rtDW.Integrator_DSTATE = 0.0;
  }

  /* SampleTimeMath: '<S43>/Tsamp' incorporates:
   *  Constant: '<S1>/Constant'
   *  Constant: '<S1>/Constant1'
   *  Product: '<S39>/DProd Out'
   *  Product: '<S65>/cProd Out'
   *  Sum: '<S6>/Sum3'
   *
   * About '<S43>/Tsamp':
   *  y = u * K where K = 1 / ( w * Ts )
   *   */
  rtb_Tsamp = (0.0 - Vx) * 0.0 * 100.0;

  /* Delay: '<S41>/UD' incorporates:
   *  Inport: '<Root>/Integral_Reset'
   */
  if (rt_ZCFcn(RISING_ZERO_CROSSING,&simulink_OD_rtPrevZCX.UD_Reset_ZCE,
               (simulink_OD_inports.Integral_Reset)) != NO_ZCEVENT) {
    simulink_OD_rtDW.UD_DSTATE = 0.0;
  }

  /* Sum: '<S58>/Sum' incorporates:
   *  Constant: '<S1>/Constant'
   *  Constant: '<S1>/Constant1'
   *  Delay: '<S41>/UD'
   *  DiscreteIntegrator: '<S48>/Integrator'
   *  Inport: '<Root>/P'
   *  Product: '<S53>/PProd Out'
   *  Product: '<S64>/bProd Out'
   *  Sum: '<S41>/Diff'
   *  Sum: '<S6>/Sum1'
   */
  rtb_Sum = ((0.0 - Vx) * simulink_OD_inports.P_h + simulink_OD_rtDW.Integrator_DSTATE) + (rtb_Tsamp -
    simulink_OD_rtDW.UD_DSTATE);

  /* Switch: '<S56>/Switch2' incorporates:
   *  Constant: '<S1>/Constant3'
   *  Gain: '<S1>/Multiply'
   *  Gain: '<S1>/torque_wheel_split'
   *  RelationalOperator: '<S56>/LowerRelop1'
   *  RelationalOperator: '<S56>/UpperRelop'
   *  Switch: '<S56>/Switch'
   */
  if (rtb_Sum > 0.0) {
    rtb_Switch2_idx_2 = 0.0;
  } else if (rtb_Sum < -rtb_Subtract_tmp) {
    /* Switch: '<S56>/Switch' incorporates:
     *  Gain: '<S1>/Multiply'
     *  Gain: '<S1>/torque_wheel_split'
     */
    rtb_Switch2_idx_2 = -rtb_Subtract_tmp;
  } else {
    rtb_Switch2_idx_2 = rtb_Sum;
  }

  /* Sum: '<S1>/Subtract' incorporates:
   *  Gain: '<S1>/torque_wheel_split'
   *  Switch: '<S56>/Switch2'
   */
  rtb_Subtract = rtb_Subtract_tmp + rtb_Switch2_idx_2;

  /* MinMax: '<S2>/Max1' incorporates:
   *  Constant: '<S2>/Constant2'
   *  Inport: '<Root>/Slip_Traction_Lim'
   *  MinMax: '<S2>/Max'
   *  Sum: '<S2>/Subtract1'
   */
  rtb_Max1_p = fmax(fmax(Vx_RL, Vx_RL) - simulink_OD_inports.Slip_Traction_Lim, 0.0);

  /* DiscreteIntegrator: '<S111>/Integrator' incorporates:
   *  Inport: '<Root>/Integral_Reset'
   */
  if ((simulink_OD_inports.Integral_Reset > 0.0) && (simulink_OD_rtDW.Integrator_PrevResetState_a <= 0)) {
    simulink_OD_rtDW.Integrator_DSTATE_a = 0.0;
  }

  /* SampleTimeMath: '<S106>/Tsamp' incorporates:
   *  Constant: '<S2>/Constant'
   *  Constant: '<S2>/Constant1'
   *  Product: '<S102>/DProd Out'
   *  Product: '<S128>/cProd Out'
   *  Sum: '<S69>/Sum3'
   *
   * About '<S106>/Tsamp':
   *  y = u * K where K = 1 / ( w * Ts )
   *   */
  rtb_Tsamp_g = (0.0 - rtb_Max1_p) * 0.0 * 100.0;

  /* Delay: '<S104>/UD' incorporates:
   *  Inport: '<Root>/Integral_Reset'
   */
  if (rt_ZCFcn(RISING_ZERO_CROSSING,&simulink_OD_rtPrevZCX.UD_Reset_ZCE_p,
               (simulink_OD_inports.Integral_Reset)) != NO_ZCEVENT) {
    simulink_OD_rtDW.UD_DSTATE_b = 0.0;
  }

  /* Sum: '<S121>/Sum' incorporates:
   *  Constant: '<S2>/Constant'
   *  Constant: '<S2>/Constant1'
   *  Delay: '<S104>/UD'
   *  DiscreteIntegrator: '<S111>/Integrator'
   *  Inport: '<Root>/P'
   *  Product: '<S116>/PProd Out'
   *  Product: '<S127>/bProd Out'
   *  Sum: '<S104>/Diff'
   *  Sum: '<S69>/Sum1'
   */
  rtb_Sum_m = ((0.0 - rtb_Max1_p) * simulink_OD_inports.P_h + simulink_OD_rtDW.Integrator_DSTATE_a) +
    (rtb_Tsamp_g - simulink_OD_rtDW.UD_DSTATE_b);

  /* Switch: '<S119>/Switch2' incorporates:
   *  Constant: '<S2>/Constant3'
   *  Gain: '<S2>/Multiply'
   *  RelationalOperator: '<S119>/LowerRelop1'
   *  RelationalOperator: '<S119>/UpperRelop'
   *  Switch: '<S119>/Switch'
   */
  if (rtb_Sum_m > 0.0) {
    rtb_Switch2_idx_2 = 0.0;
  } else if (rtb_Sum_m < -rtb_Subtract_tmp) {
    /* Switch: '<S119>/Switch' incorporates:
     *  Gain: '<S1>/Multiply'
     *  Gain: '<S1>/torque_wheel_split'
     */
    rtb_Switch2_idx_2 = -rtb_Subtract_tmp;
  } else {
    rtb_Switch2_idx_2 = rtb_Sum_m;
  }

  /* Sum: '<S2>/Subtract' incorporates:
   *  Switch: '<S119>/Switch2'
   */
  rtb_Subtract_b = rtb_Subtract_tmp + rtb_Switch2_idx_2;

  /* Gain: '<S3>/Gain1' incorporates:
   *  Inport: '<Root>/Maximum_Torque'
   */
  rtb_IProdOut = 0.25 * simulink_OD_inports.Maximum_Torque;

  /* Switch: '<S133>/Switch2' incorporates:
   *  Constant: '<S3>/Constant1'
   *  RelationalOperator: '<S133>/LowerRelop1'
   *  RelationalOperator: '<S133>/UpperRelop'
   *  Switch: '<S133>/Switch'
   */
  if (rtb_Subtract > rtb_IProdOut) {
    rtb_Switch2_idx_2 = rtb_IProdOut;
  } else if (rtb_Subtract < 0.0) {
    /* Switch: '<S133>/Switch' incorporates:
     *  Constant: '<S3>/Constant1'
     */
    rtb_Switch2_idx_2 = 0.0;
  } else {
    rtb_Switch2_idx_2 = rtb_Subtract;
  }

  /* Gain: '<S3>/1//kt' incorporates:
   *  Switch: '<S133>/Switch2'
   */
  rtb_Switch2_idx_0 = 3.8910505836575875 * rtb_Switch2_idx_2;

  /* Switch: '<S133>/Switch2' incorporates:
   *  Constant: '<S3>/Constant1'
   *  RelationalOperator: '<S133>/LowerRelop1'
   *  RelationalOperator: '<S133>/UpperRelop'
   *  Switch: '<S133>/Switch'
   */
  if (rtb_Subtract > rtb_IProdOut) {
    rtb_Switch2_idx_2 = rtb_IProdOut;
  } else if (rtb_Subtract < 0.0) {
    /* Switch: '<S133>/Switch' incorporates:
     *  Constant: '<S3>/Constant1'
     */
    rtb_Switch2_idx_2 = 0.0;
  } else {
    rtb_Switch2_idx_2 = rtb_Subtract;
  }

  /* Gain: '<S3>/1//kt' incorporates:
   *  Switch: '<S133>/Switch2'
   */
  rtb_Switch2_idx_1 = 3.8910505836575875 * rtb_Switch2_idx_2;

  /* Switch: '<S133>/Switch2' incorporates:
   *  Constant: '<S3>/Constant1'
   *  RelationalOperator: '<S133>/LowerRelop1'
   *  RelationalOperator: '<S133>/UpperRelop'
   *  Switch: '<S133>/Switch'
   */
  if (rtb_Subtract_b > rtb_IProdOut) {
    rtb_Switch2_idx_2 = rtb_IProdOut;
  } else if (rtb_Subtract_b < 0.0) {
    /* Switch: '<S133>/Switch' incorporates:
     *  Constant: '<S3>/Constant1'
     */
    rtb_Switch2_idx_2 = 0.0;
    rtb_IProdOut = 0.0;
  } else {
    rtb_Switch2_idx_2 = rtb_Subtract_b;
    rtb_IProdOut = rtb_Subtract_b;
  }

  /* Gain: '<S3>/1//kt' incorporates:
   *  Switch: '<S133>/Switch2'
   */
  rtb_Switch2_idx_2 *= 3.8910505836575875;
  rtb_Switch2_idx_3 = 3.8910505836575875 * rtb_IProdOut;

  /* Gain: '<S3>/Gain' incorporates:
   *  Inport: '<Root>/Current_Limit'
   */
  rtb_IProdOut = 0.25 * simulink_OD_inports.Current_Limit;

  /* Switch: '<S132>/Switch2' incorporates:
   *  Constant: '<S3>/Constant'
   *  RelationalOperator: '<S132>/LowerRelop1'
   *  RelationalOperator: '<S132>/UpperRelop'
   *  Switch: '<S132>/Switch'
   */
  if (rtb_Switch2_idx_0 > rtb_IProdOut) {
    rtb_Switch2_idx_0 = rtb_IProdOut;
  } else if (rtb_Switch2_idx_0 < 0.0) {
    /* Switch: '<S132>/Switch' incorporates:
     *  Constant: '<S3>/Constant'
     */
    rtb_Switch2_idx_0 = 0.0;
  }

  if (rtb_Switch2_idx_1 > rtb_IProdOut) {
    rtb_Switch2_idx_1 = rtb_IProdOut;
  } else if (rtb_Switch2_idx_1 < 0.0) {
    /* Switch: '<S132>/Switch' incorporates:
     *  Constant: '<S3>/Constant'
     */
    rtb_Switch2_idx_1 = 0.0;
  }

  if (rtb_Switch2_idx_2 > rtb_IProdOut) {
    rtb_Switch2_idx_2 = rtb_IProdOut;
  } else if (rtb_Switch2_idx_2 < 0.0) {
    /* Switch: '<S132>/Switch' incorporates:
     *  Constant: '<S3>/Constant'
     */
    rtb_Switch2_idx_2 = 0.0;
  }

  if (rtb_Switch2_idx_3 > rtb_IProdOut) {
    rtb_Switch2_idx_3 = rtb_IProdOut;
  } else if (rtb_Switch2_idx_3 < 0.0) {
    /* Switch: '<S132>/Switch' incorporates:
     *  Constant: '<S3>/Constant'
     */
    rtb_Switch2_idx_3 = 0.0;
  }

  /* End of Switch: '<S132>/Switch2' */

  /* Outport: '<Root>/Current_FL' */
  simulink_OD_outports.Current_FL = rtb_Switch2_idx_0;

  /* Outport: '<Root>/Current_FR' */
  simulink_OD_outports.Current_FR = rtb_Switch2_idx_1;

  /* Outport: '<Root>/Current_RL' */
  simulink_OD_outports.Current_RL = rtb_Switch2_idx_2;

  /* Outport: '<Root>/Current_RR' */
  simulink_OD_outports.Current_RR = rtb_Switch2_idx_3;

  /* Outport: '<Root>/Torque_RL' */
  simulink_OD_outports.Torque_RL = rtb_Subtract_b;

  /* Outport: '<Root>/Torque_RR' */
  simulink_OD_outports.Torque_RR = rtb_Subtract_b;

  /* Switch: '<S101>/Switch' incorporates:
   *  Constant: '<S2>/Constant3'
   *  Gain: '<S1>/Multiply'
   *  Gain: '<S1>/torque_wheel_split'
   *  RelationalOperator: '<S101>/u_GTE_up'
   *  RelationalOperator: '<S101>/u_GT_lo'
   *  Switch: '<S101>/Switch1'
   */
  if (rtb_Sum_m >= 0.0) {
    rtb_Switch2_idx_2 = 0.0;
  } else if (rtb_Sum_m > -rtb_Subtract_tmp) {
    /* Switch: '<S101>/Switch1' */
    rtb_Switch2_idx_2 = rtb_Sum_m;
  } else {
    rtb_Switch2_idx_2 = -rtb_Subtract_tmp;
  }

  /* Sum: '<S101>/Diff' incorporates:
   *  Switch: '<S101>/Switch'
   */
  rtb_IProdOut = rtb_Sum_m - rtb_Switch2_idx_2;

  /* RelationalOperator: '<S98>/Relational Operator' incorporates:
   *  Constant: '<S98>/Clamping_zero'
   */
  rtb_RelationalOperator = (rtb_IProdOut != 0.0);

  /* RelationalOperator: '<S98>/fix for DT propagation issue' incorporates:
   *  Constant: '<S98>/Clamping_zero'
   */
  rtb_fixforDTpropagationissue = (rtb_IProdOut > 0.0);

  /* Product: '<S108>/IProd Out' incorporates:
   *  Constant: '<S2>/Constant1'
   *  Inport: '<Root>/I'
   *  Sum: '<S69>/Sum2'
   */
  rtb_IProdOut = (0.0 - rtb_Max1_p) * simulink_OD_inports.I;

  /* Switch: '<S98>/Switch1' incorporates:
   *  Constant: '<S98>/Constant'
   *  Constant: '<S98>/Constant2'
   */
  if (rtb_fixforDTpropagationissue) {
    tmp = 1;
  } else {
    tmp = -1;
  }

  /* Switch: '<S98>/Switch2' incorporates:
   *  Constant: '<S98>/Clamping_zero'
   *  Constant: '<S98>/Constant3'
   *  Constant: '<S98>/Constant4'
   *  RelationalOperator: '<S98>/fix for DT propagation issue1'
   */
  if (rtb_IProdOut > 0.0) {
    rtb_Switch1_i = 1;
  } else {
    rtb_Switch1_i = -1;
  }

  /* Switch: '<S98>/Switch' incorporates:
   *  Constant: '<S98>/Constant1'
   *  Logic: '<S98>/AND3'
   *  RelationalOperator: '<S98>/Equal1'
   *  Switch: '<S98>/Switch1'
   *  Switch: '<S98>/Switch2'
   */
  if (rtb_RelationalOperator && (tmp == rtb_Switch1_i)) {
    rtb_Subtract_b = 0.0;
  } else {
    rtb_Subtract_b = rtb_IProdOut;
  }

  /* End of Switch: '<S98>/Switch' */

  /* Outport: '<Root>/Torque_FL' */
  simulink_OD_outports.Torque_FL = rtb_Subtract;

  /* Outport: '<Root>/Torque_FR' */
  simulink_OD_outports.Torque_FR = rtb_Subtract;

  /* Switch: '<S38>/Switch' incorporates:
   *  Constant: '<S1>/Constant3'
   *  Gain: '<S1>/Multiply'
   *  Gain: '<S1>/torque_wheel_split'
   *  RelationalOperator: '<S38>/u_GTE_up'
   *  RelationalOperator: '<S38>/u_GT_lo'
   *  Switch: '<S38>/Switch1'
   */
  if (rtb_Sum >= 0.0) {
    rtb_Switch2_idx_2 = 0.0;
  } else if (rtb_Sum > -rtb_Subtract_tmp) {
    /* Switch: '<S38>/Switch1' */
    rtb_Switch2_idx_2 = rtb_Sum;
  } else {
    rtb_Switch2_idx_2 = -rtb_Subtract_tmp;
  }

  /* Sum: '<S38>/Diff' incorporates:
   *  Switch: '<S38>/Switch'
   */
  rtb_IProdOut = rtb_Sum - rtb_Switch2_idx_2;

  /* RelationalOperator: '<S35>/Relational Operator' incorporates:
   *  Constant: '<S35>/Clamping_zero'
   */
  rtb_RelationalOperator = (rtb_IProdOut != 0.0);

  /* Switch: '<S35>/Switch1' incorporates:
   *  Constant: '<S35>/Clamping_zero'
   *  Constant: '<S35>/Constant'
   *  Constant: '<S35>/Constant2'
   *  RelationalOperator: '<S35>/fix for DT propagation issue'
   */
  if (rtb_IProdOut > 0.0) {
    rtb_Switch1_i = 1;
  } else {
    rtb_Switch1_i = -1;
  }

  /* End of Switch: '<S35>/Switch1' */

  /* Product: '<S45>/IProd Out' incorporates:
   *  Constant: '<S1>/Constant1'
   *  Inport: '<Root>/I'
   *  Sum: '<S6>/Sum2'
   */
  rtb_IProdOut = (0.0 - Vx) * simulink_OD_inports.I;

  /* Outport: '<Root>/Slip_FL' */
  simulink_OD_outports.Slip_FL = Vx_FL;

  /* Outport: '<Root>/Slip_FR' */
  simulink_OD_outports.Slip_FR = Vx_FR;

  /* Outport: '<Root>/Slip_RL' */
  simulink_OD_outports.Slip_RL = Vx_RL;

  /* Outport: '<Root>/Slip_RR' incorporates:
   *  Inport: '<Root>/Wheel_Speed_RR'
   *  MATLAB Function: '<S4>/MATLAB Function'
   */
  simulink_OD_outports.Slip_RR = fmax((simulink_OD_inports.Wheel_Speed_RR - yaw_rate_rad_s) / yaw_rate_rad_s, 0.0);

  /* Outport: '<Root>/slip_status_FL' incorporates:
   *  Inport: '<Root>/Slip_Traction_Lim'
   *  RelationalOperator: '<S1>/GreaterThan'
   */
  simulink_OD_outports.slip_status_FL = (Vx_FL > simulink_OD_inports.Slip_Traction_Lim);

  /* Outport: '<Root>/slip_status_FR' incorporates:
   *  Inport: '<Root>/Slip_Traction_Lim'
   *  RelationalOperator: '<S1>/GreaterThan1'
   */
  simulink_OD_outports.slip_status_FR = (Vx_FR > simulink_OD_inports.Slip_Traction_Lim);

  /* Outport: '<Root>/slip_status_RL' incorporates:
   *  Inport: '<Root>/Slip_Traction_Lim'
   *  RelationalOperator: '<S2>/GreaterThan'
   */
  simulink_OD_outports.slip_status_RL = (Vx_RL > simulink_OD_inports.Slip_Traction_Lim);

  /* Outport: '<Root>/slip_status_RR' incorporates:
   *  Inport: '<Root>/Slip_Traction_Lim'
   *  RelationalOperator: '<S2>/GreaterThan1'
   */
  simulink_OD_outports.slip_status_RR = (Vx_RL > simulink_OD_inports.Slip_Traction_Lim);

  /* Outport: '<Root>/Total_Torque_Cmd' */
  simulink_OD_outports.Total_Torque_Cmd = rtb_Product;

  /* Outport: '<Root>/Total_Current_Cmd' incorporates:
   *  Gain: '<S5>/Gain1'
   */
  simulink_OD_outports.Total_Current_Cmd = 3.8910505836575875 * rtb_Product;

  /* Switch: '<S35>/Switch2' incorporates:
   *  Constant: '<S35>/Clamping_zero'
   *  Constant: '<S35>/Constant3'
   *  Constant: '<S35>/Constant4'
   *  RelationalOperator: '<S35>/fix for DT propagation issue1'
   */
  if (rtb_IProdOut > 0.0) {
    tmp = 1;
  } else {
    tmp = -1;
  }

  /* Switch: '<S35>/Switch' incorporates:
   *  Constant: '<S35>/Constant1'
   *  Logic: '<S35>/AND3'
   *  RelationalOperator: '<S35>/Equal1'
   *  Switch: '<S35>/Switch2'
   */
  if (rtb_RelationalOperator && (rtb_Switch1_i == tmp)) {
    rtb_IProdOut = 0.0;
  }

  /* Update for DiscreteIntegrator: '<S48>/Integrator' incorporates:
   *  DiscreteIntegrator: '<S111>/Integrator'
   *  Inport: '<Root>/Integral_Reset'
   *  Switch: '<S35>/Switch'
   */
  simulink_OD_rtDW.Integrator_DSTATE += 0.01 * rtb_IProdOut;
  if (simulink_OD_inports.Integral_Reset > 0.0) {
    simulink_OD_rtDW.Integrator_PrevResetState = 1;
    simulink_OD_rtDW.Integrator_PrevResetState_a = 1;
  } else if (simulink_OD_inports.Integral_Reset < 0.0) {
    simulink_OD_rtDW.Integrator_PrevResetState = -1;
    simulink_OD_rtDW.Integrator_PrevResetState_a = -1;
  } else if (simulink_OD_inports.Integral_Reset == 0.0) {
    simulink_OD_rtDW.Integrator_PrevResetState = 0;
    simulink_OD_rtDW.Integrator_PrevResetState_a = 0;
  } else {
    simulink_OD_rtDW.Integrator_PrevResetState = 2;
    simulink_OD_rtDW.Integrator_PrevResetState_a = 2;
  }

  /* End of Update for DiscreteIntegrator: '<S48>/Integrator' */

  /* Update for Delay: '<S41>/UD' */
  simulink_OD_rtDW.UD_DSTATE = rtb_Tsamp;

  /* Update for DiscreteIntegrator: '<S111>/Integrator' */
  simulink_OD_rtDW.Integrator_DSTATE_a += 0.01 * rtb_Subtract_b;

  /* Update for Delay: '<S104>/UD' */
  simulink_OD_rtDW.UD_DSTATE_b = rtb_Tsamp_g;
}

/* Model initialize function */
void open_differential_initialize(void)
{
  simulink_OD_rtPrevZCX.UD_Reset_ZCE = UNINITIALIZED_ZCSIG;
  simulink_OD_rtPrevZCX.UD_Reset_ZCE_p = UNINITIALIZED_ZCSIG;

  /* InitializeConditions for DiscreteIntegrator: '<S48>/Integrator' */
  simulink_OD_rtDW.Integrator_PrevResetState = 2;

  /* InitializeConditions for DiscreteIntegrator: '<S111>/Integrator' */
  simulink_OD_rtDW.Integrator_PrevResetState_a = 2;
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
