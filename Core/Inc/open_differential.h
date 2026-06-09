/*
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * File: open_differential.h
 *
 * Code generated for Simulink model 'open_differential'.
 *
 * Model version                  : 1.18
 * Simulink Coder version         : 24.2 (R2024b) 21-Jun-2024
 * C/C++ source code generated on : Tue Jun  9 03:35:46 2026
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. RAM efficiency
 * Validation result: Not run
 */

#ifndef open_differential_h_
#define open_differential_h_
#ifndef open_differential_COMMON_INCLUDES_
#define open_differential_COMMON_INCLUDES_
#include "rtwtypes.h"
#include "math.h"
#endif                                 /* open_differential_COMMON_INCLUDES_ */

#include "zero_crossing_types.h"

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
  real_T Integrator_DSTATE;            /* '<S48>/Integrator' */
  real_T UD_DSTATE;                    /* '<S41>/UD' */
  real_T Integrator_DSTATE_a;          /* '<S111>/Integrator' */
  real_T UD_DSTATE_b;                  /* '<S104>/UD' */
  int8_T Integrator_PrevResetState;    /* '<S48>/Integrator' */
  int8_T Integrator_PrevResetState_a;  /* '<S111>/Integrator' */
} DW;

/* Zero-crossing (trigger) state */
typedef struct {
  ZCSigState UD_Reset_ZCE;             /* '<S41>/UD' */
  ZCSigState UD_Reset_ZCE_p;           /* '<S104>/UD' */
} PrevZCX;

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
  real_T Yaw_Ratedegs;                 /* '<Root>/Yaw_Rate(deg//s)' */
  real_T Integral_Reset;               /* '<Root>/Integral_Reset' */
  real_T P_h;                          /* '<Root>/P' */
  real_T I;                            /* '<Root>/I' */
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
extern DW simulink_OD_rtDW;

/* Zero-crossing (trigger) state */
extern PrevZCX simulink_OD_rtPrevZCX;

/* External inputs (root inport signals with default storage) */
extern ExtU simulink_OD_inports;

/* External outputs (root outports fed by signals with default storage) */
extern ExtY simulink_OD_outports;

/* Model entry point functions */
extern void open_differential_initialize(void);
extern void open_differential_step(void);

/* Real-time Model object */
extern RT_MODEL *const rtM;

/*-
 * These blocks were eliminated from the model due to optimizations:
 *
 * Block '<S41>/DTDup' : Unused code path elimination
 * Block '<S56>/Data Type Duplicate' : Unused code path elimination
 * Block '<S56>/Data Type Propagation' : Unused code path elimination
 * Block '<S104>/DTDup' : Unused code path elimination
 * Block '<S119>/Data Type Duplicate' : Unused code path elimination
 * Block '<S119>/Data Type Propagation' : Unused code path elimination
 * Block '<S132>/Data Type Duplicate' : Unused code path elimination
 * Block '<S132>/Data Type Propagation' : Unused code path elimination
 * Block '<S133>/Data Type Duplicate' : Unused code path elimination
 * Block '<S133>/Data Type Propagation' : Unused code path elimination
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
 * '<Root>' : 'open_differential'
 * '<S1>'   : 'open_differential/Front Axle Torque Calculation'
 * '<S2>'   : 'open_differential/Front Axle Torque Calculation1'
 * '<S3>'   : 'open_differential/Limits'
 * '<S4>'   : 'open_differential/Slip Calculator '
 * '<S5>'   : 'open_differential/Torque Limiting'
 * '<S6>'   : 'open_differential/Front Axle Torque Calculation/PID Controller (2DOF)'
 * '<S7>'   : 'open_differential/Front Axle Torque Calculation/PID Controller (2DOF)/Anti-windup'
 * '<S8>'   : 'open_differential/Front Axle Torque Calculation/PID Controller (2DOF)/D Gain'
 * '<S9>'   : 'open_differential/Front Axle Torque Calculation/PID Controller (2DOF)/External Derivative'
 * '<S10>'  : 'open_differential/Front Axle Torque Calculation/PID Controller (2DOF)/Filter'
 * '<S11>'  : 'open_differential/Front Axle Torque Calculation/PID Controller (2DOF)/Filter ICs'
 * '<S12>'  : 'open_differential/Front Axle Torque Calculation/PID Controller (2DOF)/I Gain'
 * '<S13>'  : 'open_differential/Front Axle Torque Calculation/PID Controller (2DOF)/Ideal P Gain'
 * '<S14>'  : 'open_differential/Front Axle Torque Calculation/PID Controller (2DOF)/Ideal P Gain Fdbk'
 * '<S15>'  : 'open_differential/Front Axle Torque Calculation/PID Controller (2DOF)/Integrator'
 * '<S16>'  : 'open_differential/Front Axle Torque Calculation/PID Controller (2DOF)/Integrator ICs'
 * '<S17>'  : 'open_differential/Front Axle Torque Calculation/PID Controller (2DOF)/N Copy'
 * '<S18>'  : 'open_differential/Front Axle Torque Calculation/PID Controller (2DOF)/N Gain'
 * '<S19>'  : 'open_differential/Front Axle Torque Calculation/PID Controller (2DOF)/P Copy'
 * '<S20>'  : 'open_differential/Front Axle Torque Calculation/PID Controller (2DOF)/Parallel P Gain'
 * '<S21>'  : 'open_differential/Front Axle Torque Calculation/PID Controller (2DOF)/Reset Signal'
 * '<S22>'  : 'open_differential/Front Axle Torque Calculation/PID Controller (2DOF)/Saturation'
 * '<S23>'  : 'open_differential/Front Axle Torque Calculation/PID Controller (2DOF)/Saturation Fdbk'
 * '<S24>'  : 'open_differential/Front Axle Torque Calculation/PID Controller (2DOF)/Sum'
 * '<S25>'  : 'open_differential/Front Axle Torque Calculation/PID Controller (2DOF)/Sum Fdbk'
 * '<S26>'  : 'open_differential/Front Axle Torque Calculation/PID Controller (2DOF)/Tracking Mode'
 * '<S27>'  : 'open_differential/Front Axle Torque Calculation/PID Controller (2DOF)/Tracking Mode Sum'
 * '<S28>'  : 'open_differential/Front Axle Torque Calculation/PID Controller (2DOF)/Tsamp - Integral'
 * '<S29>'  : 'open_differential/Front Axle Torque Calculation/PID Controller (2DOF)/Tsamp - Ngain'
 * '<S30>'  : 'open_differential/Front Axle Torque Calculation/PID Controller (2DOF)/b Gain'
 * '<S31>'  : 'open_differential/Front Axle Torque Calculation/PID Controller (2DOF)/c Gain'
 * '<S32>'  : 'open_differential/Front Axle Torque Calculation/PID Controller (2DOF)/postSat Signal'
 * '<S33>'  : 'open_differential/Front Axle Torque Calculation/PID Controller (2DOF)/preInt Signal'
 * '<S34>'  : 'open_differential/Front Axle Torque Calculation/PID Controller (2DOF)/preSat Signal'
 * '<S35>'  : 'open_differential/Front Axle Torque Calculation/PID Controller (2DOF)/Anti-windup/Disc. Clamping Parallel'
 * '<S36>'  : 'open_differential/Front Axle Torque Calculation/PID Controller (2DOF)/Anti-windup/Disc. Clamping Parallel/Dead Zone'
 * '<S37>'  : 'open_differential/Front Axle Torque Calculation/PID Controller (2DOF)/Anti-windup/Disc. Clamping Parallel/Dead Zone/External'
 * '<S38>'  : 'open_differential/Front Axle Torque Calculation/PID Controller (2DOF)/Anti-windup/Disc. Clamping Parallel/Dead Zone/External/Dead Zone Dynamic'
 * '<S39>'  : 'open_differential/Front Axle Torque Calculation/PID Controller (2DOF)/D Gain/External Parameters'
 * '<S40>'  : 'open_differential/Front Axle Torque Calculation/PID Controller (2DOF)/External Derivative/Error'
 * '<S41>'  : 'open_differential/Front Axle Torque Calculation/PID Controller (2DOF)/Filter/Differentiator'
 * '<S42>'  : 'open_differential/Front Axle Torque Calculation/PID Controller (2DOF)/Filter/Differentiator/Tsamp'
 * '<S43>'  : 'open_differential/Front Axle Torque Calculation/PID Controller (2DOF)/Filter/Differentiator/Tsamp/Internal Ts'
 * '<S44>'  : 'open_differential/Front Axle Torque Calculation/PID Controller (2DOF)/Filter ICs/Internal IC - Differentiator'
 * '<S45>'  : 'open_differential/Front Axle Torque Calculation/PID Controller (2DOF)/I Gain/External Parameters'
 * '<S46>'  : 'open_differential/Front Axle Torque Calculation/PID Controller (2DOF)/Ideal P Gain/Passthrough'
 * '<S47>'  : 'open_differential/Front Axle Torque Calculation/PID Controller (2DOF)/Ideal P Gain Fdbk/Disabled'
 * '<S48>'  : 'open_differential/Front Axle Torque Calculation/PID Controller (2DOF)/Integrator/Discrete'
 * '<S49>'  : 'open_differential/Front Axle Torque Calculation/PID Controller (2DOF)/Integrator ICs/Internal IC'
 * '<S50>'  : 'open_differential/Front Axle Torque Calculation/PID Controller (2DOF)/N Copy/Disabled wSignal Specification'
 * '<S51>'  : 'open_differential/Front Axle Torque Calculation/PID Controller (2DOF)/N Gain/Passthrough'
 * '<S52>'  : 'open_differential/Front Axle Torque Calculation/PID Controller (2DOF)/P Copy/Disabled'
 * '<S53>'  : 'open_differential/Front Axle Torque Calculation/PID Controller (2DOF)/Parallel P Gain/External Parameters'
 * '<S54>'  : 'open_differential/Front Axle Torque Calculation/PID Controller (2DOF)/Reset Signal/External Reset'
 * '<S55>'  : 'open_differential/Front Axle Torque Calculation/PID Controller (2DOF)/Saturation/External'
 * '<S56>'  : 'open_differential/Front Axle Torque Calculation/PID Controller (2DOF)/Saturation/External/Saturation Dynamic'
 * '<S57>'  : 'open_differential/Front Axle Torque Calculation/PID Controller (2DOF)/Saturation Fdbk/Disabled'
 * '<S58>'  : 'open_differential/Front Axle Torque Calculation/PID Controller (2DOF)/Sum/Sum_PID'
 * '<S59>'  : 'open_differential/Front Axle Torque Calculation/PID Controller (2DOF)/Sum Fdbk/Disabled'
 * '<S60>'  : 'open_differential/Front Axle Torque Calculation/PID Controller (2DOF)/Tracking Mode/Disabled'
 * '<S61>'  : 'open_differential/Front Axle Torque Calculation/PID Controller (2DOF)/Tracking Mode Sum/Passthrough'
 * '<S62>'  : 'open_differential/Front Axle Torque Calculation/PID Controller (2DOF)/Tsamp - Integral/TsSignalSpecification'
 * '<S63>'  : 'open_differential/Front Axle Torque Calculation/PID Controller (2DOF)/Tsamp - Ngain/Passthrough'
 * '<S64>'  : 'open_differential/Front Axle Torque Calculation/PID Controller (2DOF)/b Gain/External Parameters'
 * '<S65>'  : 'open_differential/Front Axle Torque Calculation/PID Controller (2DOF)/c Gain/External Parameters'
 * '<S66>'  : 'open_differential/Front Axle Torque Calculation/PID Controller (2DOF)/postSat Signal/Forward_Path'
 * '<S67>'  : 'open_differential/Front Axle Torque Calculation/PID Controller (2DOF)/preInt Signal/Internal PreInt'
 * '<S68>'  : 'open_differential/Front Axle Torque Calculation/PID Controller (2DOF)/preSat Signal/Forward_Path'
 * '<S69>'  : 'open_differential/Front Axle Torque Calculation1/PID Controller (2DOF)'
 * '<S70>'  : 'open_differential/Front Axle Torque Calculation1/PID Controller (2DOF)/Anti-windup'
 * '<S71>'  : 'open_differential/Front Axle Torque Calculation1/PID Controller (2DOF)/D Gain'
 * '<S72>'  : 'open_differential/Front Axle Torque Calculation1/PID Controller (2DOF)/External Derivative'
 * '<S73>'  : 'open_differential/Front Axle Torque Calculation1/PID Controller (2DOF)/Filter'
 * '<S74>'  : 'open_differential/Front Axle Torque Calculation1/PID Controller (2DOF)/Filter ICs'
 * '<S75>'  : 'open_differential/Front Axle Torque Calculation1/PID Controller (2DOF)/I Gain'
 * '<S76>'  : 'open_differential/Front Axle Torque Calculation1/PID Controller (2DOF)/Ideal P Gain'
 * '<S77>'  : 'open_differential/Front Axle Torque Calculation1/PID Controller (2DOF)/Ideal P Gain Fdbk'
 * '<S78>'  : 'open_differential/Front Axle Torque Calculation1/PID Controller (2DOF)/Integrator'
 * '<S79>'  : 'open_differential/Front Axle Torque Calculation1/PID Controller (2DOF)/Integrator ICs'
 * '<S80>'  : 'open_differential/Front Axle Torque Calculation1/PID Controller (2DOF)/N Copy'
 * '<S81>'  : 'open_differential/Front Axle Torque Calculation1/PID Controller (2DOF)/N Gain'
 * '<S82>'  : 'open_differential/Front Axle Torque Calculation1/PID Controller (2DOF)/P Copy'
 * '<S83>'  : 'open_differential/Front Axle Torque Calculation1/PID Controller (2DOF)/Parallel P Gain'
 * '<S84>'  : 'open_differential/Front Axle Torque Calculation1/PID Controller (2DOF)/Reset Signal'
 * '<S85>'  : 'open_differential/Front Axle Torque Calculation1/PID Controller (2DOF)/Saturation'
 * '<S86>'  : 'open_differential/Front Axle Torque Calculation1/PID Controller (2DOF)/Saturation Fdbk'
 * '<S87>'  : 'open_differential/Front Axle Torque Calculation1/PID Controller (2DOF)/Sum'
 * '<S88>'  : 'open_differential/Front Axle Torque Calculation1/PID Controller (2DOF)/Sum Fdbk'
 * '<S89>'  : 'open_differential/Front Axle Torque Calculation1/PID Controller (2DOF)/Tracking Mode'
 * '<S90>'  : 'open_differential/Front Axle Torque Calculation1/PID Controller (2DOF)/Tracking Mode Sum'
 * '<S91>'  : 'open_differential/Front Axle Torque Calculation1/PID Controller (2DOF)/Tsamp - Integral'
 * '<S92>'  : 'open_differential/Front Axle Torque Calculation1/PID Controller (2DOF)/Tsamp - Ngain'
 * '<S93>'  : 'open_differential/Front Axle Torque Calculation1/PID Controller (2DOF)/b Gain'
 * '<S94>'  : 'open_differential/Front Axle Torque Calculation1/PID Controller (2DOF)/c Gain'
 * '<S95>'  : 'open_differential/Front Axle Torque Calculation1/PID Controller (2DOF)/postSat Signal'
 * '<S96>'  : 'open_differential/Front Axle Torque Calculation1/PID Controller (2DOF)/preInt Signal'
 * '<S97>'  : 'open_differential/Front Axle Torque Calculation1/PID Controller (2DOF)/preSat Signal'
 * '<S98>'  : 'open_differential/Front Axle Torque Calculation1/PID Controller (2DOF)/Anti-windup/Disc. Clamping Parallel'
 * '<S99>'  : 'open_differential/Front Axle Torque Calculation1/PID Controller (2DOF)/Anti-windup/Disc. Clamping Parallel/Dead Zone'
 * '<S100>' : 'open_differential/Front Axle Torque Calculation1/PID Controller (2DOF)/Anti-windup/Disc. Clamping Parallel/Dead Zone/External'
 * '<S101>' : 'open_differential/Front Axle Torque Calculation1/PID Controller (2DOF)/Anti-windup/Disc. Clamping Parallel/Dead Zone/External/Dead Zone Dynamic'
 * '<S102>' : 'open_differential/Front Axle Torque Calculation1/PID Controller (2DOF)/D Gain/External Parameters'
 * '<S103>' : 'open_differential/Front Axle Torque Calculation1/PID Controller (2DOF)/External Derivative/Error'
 * '<S104>' : 'open_differential/Front Axle Torque Calculation1/PID Controller (2DOF)/Filter/Differentiator'
 * '<S105>' : 'open_differential/Front Axle Torque Calculation1/PID Controller (2DOF)/Filter/Differentiator/Tsamp'
 * '<S106>' : 'open_differential/Front Axle Torque Calculation1/PID Controller (2DOF)/Filter/Differentiator/Tsamp/Internal Ts'
 * '<S107>' : 'open_differential/Front Axle Torque Calculation1/PID Controller (2DOF)/Filter ICs/Internal IC - Differentiator'
 * '<S108>' : 'open_differential/Front Axle Torque Calculation1/PID Controller (2DOF)/I Gain/External Parameters'
 * '<S109>' : 'open_differential/Front Axle Torque Calculation1/PID Controller (2DOF)/Ideal P Gain/Passthrough'
 * '<S110>' : 'open_differential/Front Axle Torque Calculation1/PID Controller (2DOF)/Ideal P Gain Fdbk/Disabled'
 * '<S111>' : 'open_differential/Front Axle Torque Calculation1/PID Controller (2DOF)/Integrator/Discrete'
 * '<S112>' : 'open_differential/Front Axle Torque Calculation1/PID Controller (2DOF)/Integrator ICs/Internal IC'
 * '<S113>' : 'open_differential/Front Axle Torque Calculation1/PID Controller (2DOF)/N Copy/Disabled wSignal Specification'
 * '<S114>' : 'open_differential/Front Axle Torque Calculation1/PID Controller (2DOF)/N Gain/Passthrough'
 * '<S115>' : 'open_differential/Front Axle Torque Calculation1/PID Controller (2DOF)/P Copy/Disabled'
 * '<S116>' : 'open_differential/Front Axle Torque Calculation1/PID Controller (2DOF)/Parallel P Gain/External Parameters'
 * '<S117>' : 'open_differential/Front Axle Torque Calculation1/PID Controller (2DOF)/Reset Signal/External Reset'
 * '<S118>' : 'open_differential/Front Axle Torque Calculation1/PID Controller (2DOF)/Saturation/External'
 * '<S119>' : 'open_differential/Front Axle Torque Calculation1/PID Controller (2DOF)/Saturation/External/Saturation Dynamic'
 * '<S120>' : 'open_differential/Front Axle Torque Calculation1/PID Controller (2DOF)/Saturation Fdbk/Disabled'
 * '<S121>' : 'open_differential/Front Axle Torque Calculation1/PID Controller (2DOF)/Sum/Sum_PID'
 * '<S122>' : 'open_differential/Front Axle Torque Calculation1/PID Controller (2DOF)/Sum Fdbk/Disabled'
 * '<S123>' : 'open_differential/Front Axle Torque Calculation1/PID Controller (2DOF)/Tracking Mode/Disabled'
 * '<S124>' : 'open_differential/Front Axle Torque Calculation1/PID Controller (2DOF)/Tracking Mode Sum/Passthrough'
 * '<S125>' : 'open_differential/Front Axle Torque Calculation1/PID Controller (2DOF)/Tsamp - Integral/TsSignalSpecification'
 * '<S126>' : 'open_differential/Front Axle Torque Calculation1/PID Controller (2DOF)/Tsamp - Ngain/Passthrough'
 * '<S127>' : 'open_differential/Front Axle Torque Calculation1/PID Controller (2DOF)/b Gain/External Parameters'
 * '<S128>' : 'open_differential/Front Axle Torque Calculation1/PID Controller (2DOF)/c Gain/External Parameters'
 * '<S129>' : 'open_differential/Front Axle Torque Calculation1/PID Controller (2DOF)/postSat Signal/Forward_Path'
 * '<S130>' : 'open_differential/Front Axle Torque Calculation1/PID Controller (2DOF)/preInt Signal/Internal PreInt'
 * '<S131>' : 'open_differential/Front Axle Torque Calculation1/PID Controller (2DOF)/preSat Signal/Forward_Path'
 * '<S132>' : 'open_differential/Limits/Saturation Dynamic1'
 * '<S133>' : 'open_differential/Limits/Saturation Dynamic2'
 * '<S134>' : 'open_differential/Slip Calculator /MATLAB Function'
 */
#endif                                 /* open_differential_h_ */

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
