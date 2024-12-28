/*
 * simulink5ms_plotAndGains_data.c
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "simulink5ms_plotAndGains".
 *
 * Model version              : 9.0
 * Simulink Coder version : 9.9 (R2023a) 19-Nov-2022
 * C source code generated on : Wed Oct 16 15:51:01 2024
 *
 * Target selection: sldrt.tlc
 * Note: GRT includes extra infrastructure and instrumentation for prototyping
 * Embedded hardware selection: Intel->x86-64 (Linux 64)
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "simulink5ms_plotAndGains.h"

/* Block parameters (default storage) */
P_simulink5ms_plotAndGains_T simulink5ms_plotAndGains_P = {
  /* Mask Parameter: PacketInput1_MaxMissedTicks
   * Referenced by: '<S1>/Packet Input1'
   */
  50.0,

  /* Mask Parameter: PacketOutput_MaxMissedTicks
   * Referenced by: '<S2>/Packet Output'
   */
  10.0,

  /* Mask Parameter: PacketInput1_YieldWhenWaiting
   * Referenced by: '<S1>/Packet Input1'
   */
  0.0,

  /* Mask Parameter: PacketOutput_YieldWhenWaiting
   * Referenced by: '<S2>/Packet Output'
   */
  0.0,

  /* Mask Parameter: PacketInput1_PacketID
   * Referenced by: '<S1>/Packet Input1'
   */
  1,

  /* Mask Parameter: PacketOutput_PacketID
   * Referenced by: '<S2>/Packet Output'
   */
  1,

  /* Expression: 0
   * Referenced by: '<Root>/Rate Transition'
   */
  0.0,

  /* Expression: [0.001]
   * Referenced by: '<Root>/Discrete Transfer Fcn2'
   */
  0.001,

  /* Expression: [1 -1]
   * Referenced by: '<Root>/Discrete Transfer Fcn2'
   */
  { 1.0, -1.0 },

  /* Expression: 0
   * Referenced by: '<Root>/Discrete Transfer Fcn2'
   */
  0.0,

  /* Expression: 4000/2/pi
   * Referenced by: '<Root>/Gain13'
   */
  636.61977236758139,

  /* Expression: 2*pi/4000
   * Referenced by: '<Root>/Gain14'
   */
  0.0015707963267948967,

  /* Expression: [1 -1]
   * Referenced by: '<Root>/Discrete Transfer Fcn3'
   */
  { 1.0, -1.0 },

  /* Expression: [0.001 0]
   * Referenced by: '<Root>/Discrete Transfer Fcn3'
   */
  { 0.001, 0.0 },

  /* Expression: 0
   * Referenced by: '<Root>/Discrete Transfer Fcn3'
   */
  0.0,

  /* Expression: 0.05
   * Referenced by: '<Root>/Gain12'
   */
  0.05,

  /* Expression: [0.001 0.001]
   * Referenced by: '<Root>/Discrete Transfer Fcn'
   */
  { 0.001, 0.001 },

  /* Expression: [2 -2]
   * Referenced by: '<Root>/Discrete Transfer Fcn'
   */
  { 2.0, -2.0 },

  /* Expression: 0
   * Referenced by: '<Root>/Discrete Transfer Fcn'
   */
  0.0,

  /* Expression: 0.15
   * Referenced by: '<Root>/Gain'
   */
  0.15,

  /* Expression: 10
   * Referenced by: '<Root>/Saturation'
   */
  10.0,

  /* Expression: -10
   * Referenced by: '<Root>/Saturation'
   */
  -10.0,

  /* Expression: [0.169635]
   * Referenced by: '<Root>/Discrete Transfer Fcn1'
   */
  0.169635,

  /* Expression: [1 -0.997287]
   * Referenced by: '<Root>/Discrete Transfer Fcn1'
   */
  { 1.0, -0.997287 },

  /* Expression: 0
   * Referenced by: '<Root>/Discrete Transfer Fcn1'
   */
  0.0,

  /* Expression: 1/10000
   * Referenced by: '<Root>/Gain2'
   */
  0.0001,

  /* Expression: 1
   * Referenced by: '<Root>/Reference'
   */
  1.0,

  /* Expression: 1/1000
   * Referenced by: '<Root>/Gain3'
   */
  0.001,

  /* Expression: 1
   * Referenced by: '<Root>/u'
   */
  1.0,

  /* Expression: 1/1000
   * Referenced by: '<Root>/Gain4'
   */
  0.001,

  /* Expression: 1
   * Referenced by: '<Root>/adc_val'
   */
  1.0,

  /* Expression: 1/10000
   * Referenced by: '<Root>/Gain1'
   */
  0.0001,

  /* Expression: 1
   * Referenced by: '<Root>/Velocity'
   */
  1.0,

  /* Expression: 1
   * Referenced by: '<Root>/Value_16bit2'
   */
  1.0,

  /* Expression: 1
   * Referenced by: '<Root>/Gain6'
   */
  1.0,

  /* Expression: 1
   * Referenced by: '<S2>/Gain1'
   */
  1.0,

  /* Expression: 1
   * Referenced by: '<Root>/Value_16bit3'
   */
  1.0,

  /* Expression: 1
   * Referenced by: '<Root>/Gain7'
   */
  1.0,

  /* Expression: 1
   * Referenced by: '<S2>/Gain2'
   */
  1.0,

  /* Expression: 1
   * Referenced by: '<Root>/Value_16bit1'
   */
  1.0,

  /* Expression: 1
   * Referenced by: '<Root>/Gain5'
   */
  1.0,

  /* Expression: 1
   * Referenced by: '<S2>/Gain3'
   */
  1.0,

  /* Expression: 1
   * Referenced by: '<Root>/Value_16bit4'
   */
  1.0,

  /* Expression: 1
   * Referenced by: '<Root>/Gain8'
   */
  1.0,

  /* Expression: 1
   * Referenced by: '<S2>/Gain4'
   */
  1.0,

  /* Expression: 1
   * Referenced by: '<Root>/Value_16bit5 '
   */
  1.0,

  /* Expression: 1
   * Referenced by: '<Root>/Gain9'
   */
  1.0,

  /* Expression: 1
   * Referenced by: '<S2>/Gain5'
   */
  1.0,

  /* Expression: 1
   * Referenced by: '<Root>/Value_16bit6'
   */
  1.0,

  /* Expression: 1
   * Referenced by: '<Root>/Gain10'
   */
  1.0,

  /* Expression: 1
   * Referenced by: '<S2>/Gain6'
   */
  1.0,

  /* Expression: 1
   * Referenced by: '<Root>/Value_16bit7'
   */
  1.0,

  /* Expression: 1
   * Referenced by: '<Root>/Gain11'
   */
  1.0,

  /* Expression: 1
   * Referenced by: '<S2>/Gain7'
   */
  1.0,

  /* Computed Parameter: ConstantMustbeThisValue0x7fff_Value
   * Referenced by: '<S2>/Constant Must be This Value 0x7fff'
   */
  MAX_int16_T
};
