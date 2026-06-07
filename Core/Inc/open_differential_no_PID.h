/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: open_differential_no_PID.h
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

#ifndef open_differential_no_PID_h_
#define open_differential_no_PID_h_
#ifndef open_differential_no_PID_COMMON_INCLUDES_
#define open_differential_no_PID_COMMON_INCLUDES_
#include "rtwtypes.h"
// #include "rtw_continuous.h"
// #include "rtw_solver.h"
#include "math.h"
#endif                           /* open_differential_no_PID_COMMON_INCLUDES_ */

/* Macros for accessing real-time model data structure */
#ifndef rtmGetErrorStatus
#define rtmGetErrorStatus(rtm)         ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
#define rtmSetErrorStatus(rtm, val)    ((rtm)->errorStatus = (val))
#endif

/* Forward declaration for rtModel */
typedef struct tag_RTM RT_MODEL;

/* Block signals and states (default storage) for system '<Root>' */
typedef struct {
  real_T active_torque_cmd;            /* '<S2>/MATLAB Function' */
  real_T active_torque_cmd_g;          /* '<S1>/MATLAB Function' */
  boolean_T recovering_traction;       /* '<S2>/MATLAB Function' */
  boolean_T recovering_traction_o;     /* '<S1>/MATLAB Function' */
} DW;

/* External inputs (root inport signals with default storage) */
typedef struct {
  real_T Wheel_Speed_FL;               /* '<Root>/Wheel_Speed_FL' */
  real_T Wheel_Speed_FR;               /* '<Root>/Wheel_Speed_FR' */
  real_T Wheel_Speed_RL;               /* '<Root>/Wheel_Speed_RL' */
  real_T Wheel_Speed_RR;               /* '<Root>/Wheel_Speed_RR' */
  real_T Car_SpeedVx;                  /* '<Root>/Car_Speed(Vx)' */
  real_T Maximum_Torque;               /* '<Root>/Maximum_Torque' */
  real_T Slip_Traction_Lim;            /* '<Root>/Slip_Traction_Lim' */
  real_T Throttle;                     /* '<Root>/Throttle(%)' */
  real_T Current_Limit;                /* '<Root>/Current_Limit' */
} ExtU;

/* External outputs (root outports fed by signals with default storage) */
typedef struct {
  real_T Slip_FL;                      /* '<Root>/Slip_FL' */
  real_T Slip_FR;                      /* '<Root>/Slip_FR' */
  real_T Slip_RL;                      /* '<Root>/Slip_RL' */
  real_T Slip_RR;                      /* '<Root>/Slip_RR' */
  boolean_T slip_status_FL;            /* '<Root>/slip_status_FL' */
  boolean_T slip_status_FR;            /* '<Root>/slip_status_FR' */
  boolean_T slip_status_RL;            /* '<Root>/slip_status_RL' */
  boolean_T slip_status_RR;            /* '<Root>/slip_status_RR' */
  real_T Torque_FL;                    /* '<Root>/Torque_FL' */
  real_T Torque_FR;                    /* '<Root>/Torque_FR' */
  real_T Torque_RL;                    /* '<Root>/Torque_RL' */
  real_T Torque_RR;                    /* '<Root>/Torque_RR' */
  real_T Total_Torque_Cmd;             /* '<Root>/Total_Torque_Cmd' */
  real_T Current_FL;                   /* '<Root>/Current_FL' */
  real_T Current_FR;                   /* '<Root>/Current_FR' */
  real_T Current_RL;                   /* '<Root>/Current_RL' */
  real_T Current_RR;                   /* '<Root>/Current_RR' */
  real_T Total_Current_Cmd;            /* '<Root>/Total_Current_Cmd' */
} ExtY;

/* Real-time Model Data Structure */
struct tag_RTM {
  const char_T * volatile errorStatus;
};

/* Block signals and states (default storage) */
extern DW rtDW;

/* External inputs (root inport signals with default storage) */
extern ExtU simulink_OD_No_PID_inports;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY simulink_OD_No_PID_outports;

/* Model entry point functions */
extern void open_differential_no_PID_initialize(void);
extern void open_differential_no_PID_step(void);

/* Real-time Model object */
extern RT_MODEL *const rtM;

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S8>/Data Type Duplicate' : Unused code path elimination
 * Block '<S8>/Data Type Propagation' : Unused code path elimination
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
 * '<Root>' : 'open_differential_no_PID'
 * '<S1>'   : 'open_differential_no_PID/Front Axle Torque Calculation'
 * '<S2>'   : 'open_differential_no_PID/Front Axle Torque Calculation1'
 * '<S3>'   : 'open_differential_no_PID/Limits'
 * '<S4>'   : 'open_differential_no_PID/Slip Calculator '
 * '<S5>'   : 'open_differential_no_PID/Torque Limiting'
 * '<S6>'   : 'open_differential_no_PID/Front Axle Torque Calculation/MATLAB Function'
 * '<S7>'   : 'open_differential_no_PID/Front Axle Torque Calculation1/MATLAB Function'
 * '<S8>'   : 'open_differential_no_PID/Limits/Saturation Dynamic1'
 */
#endif                                 /* open_differential_no_PID_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
