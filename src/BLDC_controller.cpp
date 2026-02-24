/*
 * File: BLDC_controller.c
 *
 * Code generated for Simulink model 'BLDC_controller'.
 *
 * Model version                  : 16.32
 * Simulink Coder version         : 25.1 (R2025a) 21-Nov-2024
 * C/C++ source code generated on : Mon Feb 16 22:38:23 2026
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Emulation hardware selection:
 *    Differs from embedded hardware (MATLAB Host)
 * Code generation objective: Execution efficiency
 * Validation result: Not run
 */

#include "BLDC_controller.h"
#include "rtwtypes.h"

/* Named constants for Chart: '<S5>/F03_02_Control_Mode_Manager' */
#define IN_ACTIVE                      ((uint8_T)1U)
#define IN_NO_ACTIVE_CHILD             ((uint8_T)0U)
#define IN_OPEN                        ((uint8_T)2U)
#define IN_SPEED_MODE                  ((uint8_T)1U)
#define IN_TORQUE_MODE                 ((uint8_T)2U)
#define IN_VOLTAGE_MODE                ((uint8_T)3U)
#define OPEN_MODE                      ((uint8_T)0U)
#define SPD_MODE                       ((uint8_T)2U)
#define TRQ_MODE                       ((uint8_T)3U)
#define VLT_MODE                       ((uint8_T)1U)
#ifndef UCHAR_MAX
#include <limits.h>
#endif

#if ( UCHAR_MAX != (0xFFU) ) || ( SCHAR_MAX != (0x7F) )
#error Code was generated for compiler with different sized uchar/char. \
Consider adjusting Test hardware word size settings on the \
Hardware Implementation pane to match your compiler word sizes as \
defined in limits.h of the compiler. Alternatively, you can \
select the Test hardware is the same as production hardware option and \
select the Enable portable word sizes option on the Code Generation > \
Verification pane for ERT based targets, which will disable the \
preprocessor word size checks.
#endif

#if ( USHRT_MAX != (0xFFFFU) ) || ( SHRT_MAX != (0x7FFF) )
#error Code was generated for compiler with different sized ushort/short. \
Consider adjusting Test hardware word size settings on the \
Hardware Implementation pane to match your compiler word sizes as \
defined in limits.h of the compiler. Alternatively, you can \
select the Test hardware is the same as production hardware option and \
select the Enable portable word sizes option on the Code Generation > \
Verification pane for ERT based targets, which will disable the \
preprocessor word size checks.
#endif

#if ( UINT_MAX != (0xFFFFFFFFU) ) || ( INT_MAX != (0x7FFFFFFF) )
#error Code was generated for compiler with different sized uint/int. \
Consider adjusting Test hardware word size settings on the \
Hardware Implementation pane to match your compiler word sizes as \
defined in limits.h of the compiler. Alternatively, you can \
select the Test hardware is the same as production hardware option and \
select the Enable portable word sizes option on the Code Generation > \
Verification pane for ERT based targets, which will disable the \
preprocessor word size checks.
#endif

#if ( ULONG_MAX != (0xFFFFFFFFU) ) || ( LONG_MAX != (0x7FFFFFFF) )
#error Code was generated for compiler with different sized ulong/long. \
Consider adjusting Test hardware word size settings on the \
Hardware Implementation pane to match your compiler word sizes as \
defined in limits.h of the compiler. Alternatively, you can \
select the Test hardware is the same as production hardware option and \
select the Enable portable word sizes option on the Code Generation > \
Verification pane for ERT based targets, which will disable the \
preprocessor word size checks.
#endif

/* Skipping ulong_long/long_long check: insufficient preprocessor integer range. */
uint8_T plook_u8s16u16n15_even7ca_gs(int16_T u, int16_T bp0, uint16_T *fraction);
int16_T intrp1d_s16s32s32u8u16n15la_s(uint8_T bpIndex, uint16_T frac, const
  int16_T table[], uint32_T maxIndex);
uint8_T plook_u8s16_evencka(int16_T u, int16_T bp0, uint16_T bpSpace, uint32_T
  maxIndex);
uint8_T plook_u8u16_evenckag(uint16_T u, uint16_T bp0, uint16_T bpSpace);
uint8_T plook_u8s16u8n6_evenca_gs(int16_T u, int16_T bp0, uint16_T bpSpace,
  uint8_T *fraction);
int16_T intrp1d_s16s32s32u8u8n6la_s(uint8_T bpIndex, uint8_T frac, const int16_T
  table[], uint32_T maxIndex);
int32_T div_nde_s32_floor(int32_T numerator, int32_T denominator);
extern void Counter_Init(int16_T rtp_z_cntInit, DW_Counter *localDW);
extern int16_T Counter(int16_T rtu_inc, int16_T rtu_max, boolean_T rtu_rst,
  DW_Counter *localDW);
extern void Low_Pass_Filter_Reset(DW_Low_Pass_Filter *localDW);
extern void Low_Pass_Filter(const int16_T rtu_u[2], uint16_T rtu_coef, int16_T
  rty_y[2], DW_Low_Pass_Filter *localDW);
extern void Counter_e_Init(uint16_T rtp_z_cntInit, DW_Counter_d *localDW);
extern uint16_T Counter_e(uint16_T rtu_inc, uint16_T rtu_max, boolean_T rtu_rst,
  DW_Counter_d *localDW);
extern boolean_T either_edge(boolean_T rtu_u, DW_either_edge *localDW);
extern void Debounce_Filter_Init(boolean_T *rty_y, DW_Debounce_Filter *localDW);
extern void Debounce_Filter(boolean_T rtu_u, uint16_T rtu_tAcv, uint16_T
  rtu_tDeacv, boolean_T *rty_y, DW_Debounce_Filter *localDW);
extern void I_backCalc_fixdt_Init(int32_T rtp_yInit, DW_I_backCalc_fixdt
  *localDW);
extern void I_backCalc_fixdt_Reset(int32_T rtp_yInit, DW_I_backCalc_fixdt
  *localDW);
extern void I_backCalc_fixdt(int16_T rtu_err, uint16_T rtu_I, uint16_T rtu_Kb,
  int16_T rtu_satMax, int16_T rtu_satMin, int16_T *rty_out, DW_I_backCalc_fixdt *
  localDW);
extern void PI_clamp_fixdt_Init(DW_PI_clamp_fixdt *localDW);
extern void PI_clamp_fixdt_Reset(DW_PI_clamp_fixdt *localDW);
extern int16_T PI_clamp_fixdt(int16_T rtu_err, uint16_T rtu_P, uint16_T rtu_I,
  int16_T rtu_init, int16_T rtu_satMax, int16_T rtu_satMin, int32_T
  rtu_ext_limProt, DW_PI_clamp_fixdt *localDW);

/* Forward declaration for local functions */
void enter_internal_ACTIVE(const boolean_T *LogicalOperator1, const boolean_T
  *LogicalOperator2, DW *rtDW);
uint8_T plook_u8s16u16n15_even7ca_gs(int16_T u, int16_T bp0, uint16_T *fraction)
{
  uint16_T uAdjust;
  uint8_T bpIndex;

  /* Prelookup - Index and Fraction
     Index Search method: 'even'
     Use previous index: 'off'
     Use last breakpoint for index at or above upper limit: 'on'
     Remove protection against out-of-range input in generated code: 'on'
     Rounding mode: 'simplest'
   */
  uAdjust = (uint16_T)(u - bp0);
  bpIndex = (uint8_T)((uint32_T)uAdjust >> 7U);
  *fraction = (uint16_T)((uint16_T)(uAdjust & 127) << 8);
  return bpIndex;
}

int16_T intrp1d_s16s32s32u8u16n15la_s(uint8_T bpIndex, uint16_T frac, const
  int16_T table[], uint32_T maxIndex)
{
  int16_T y;

  /* Column-major Interpolation 1-D
     Interpolation method: 'Linear point-slope'
     Use last breakpoint for index at or above upper limit: 'on'
     Rounding mode: 'simplest'
     Overflow mode: 'wrapping'
   */
  if (bpIndex == maxIndex) {
    y = table[(uint32_T)bpIndex];
  } else {
    int16_T yL_0d0;
    yL_0d0 = table[(uint32_T)bpIndex];
    y = (int16_T)((int16_T)(((table[bpIndex + 1U] - yL_0d0) * frac) >> 15) +
                  yL_0d0);
  }

  return y;
}

uint8_T plook_u8s16_evencka(int16_T u, int16_T bp0, uint16_T bpSpace, uint32_T
  maxIndex)
{
  uint8_T bpIndex;

  /* Prelookup - Index only
     Index Search method: 'even'
     Extrapolation method: 'Clip'
     Use previous index: 'off'
     Use last breakpoint for index at or above upper limit: 'on'
     Remove protection against out-of-range input in generated code: 'off'
   */
  if (u <= bp0) {
    bpIndex = 0U;
  } else {
    uint16_T fbpIndex;
    fbpIndex = (uint16_T)((uint32_T)(uint16_T)(u - bp0) / bpSpace);
    if (fbpIndex < maxIndex) {
      bpIndex = (uint8_T)fbpIndex;
    } else {
      bpIndex = (uint8_T)maxIndex;
    }
  }

  return bpIndex;
}

uint8_T plook_u8u16_evenckag(uint16_T u, uint16_T bp0, uint16_T bpSpace)
{
  /* Prelookup - Index only
     Index Search method: 'even'
     Use previous index: 'off'
     Use last breakpoint for index at or above upper limit: 'on'
     Remove protection against out-of-range input in generated code: 'on'
   */
  return (uint8_T)((uint32_T)(uint16_T)((uint32_T)u - bp0) / bpSpace);
}

uint8_T plook_u8s16u8n6_evenca_gs(int16_T u, int16_T bp0, uint16_T bpSpace,
  uint8_T *fraction)
{
  uint16_T uAdjust;
  uint8_T bpIndex;

  /* Prelookup - Index and Fraction
     Index Search method: 'even'
     Use previous index: 'off'
     Use last breakpoint for index at or above upper limit: 'on'
     Remove protection against out-of-range input in generated code: 'on'
     Rounding mode: 'simplest'
   */
  uAdjust = (uint16_T)(u - bp0);
  bpIndex = (uint8_T)((uint32_T)uAdjust / bpSpace);
  *fraction = (uint8_T)((((uint32_T)uAdjust - (uint16_T)((uint32_T)(uint8_T)
    ((uint32_T)uAdjust / bpSpace) * bpSpace)) << 6) / bpSpace);
  return bpIndex;
}

int16_T intrp1d_s16s32s32u8u8n6la_s(uint8_T bpIndex, uint8_T frac, const int16_T
  table[], uint32_T maxIndex)
{
  int16_T y;

  /* Column-major Interpolation 1-D
     Interpolation method: 'Linear point-slope'
     Use last breakpoint for index at or above upper limit: 'on'
     Rounding mode: 'simplest'
     Overflow mode: 'wrapping'
   */
  if (bpIndex == maxIndex) {
    y = table[(uint32_T)bpIndex];
  } else {
    int16_T yL_0d0;
    yL_0d0 = table[(uint32_T)bpIndex];
    y = (int16_T)((int16_T)(((table[bpIndex + 1U] - yL_0d0) * frac) >> 6) +
                  yL_0d0);
  }

  return y;
}

int32_T div_nde_s32_floor(int32_T numerator, int32_T denominator)
{
  return (((numerator < 0) != (denominator < 0)) && (numerator % denominator !=
           0) ? -1 : 0) + numerator / denominator;
}

/* System initialize for atomic system: '<S13>/Counter' */
void Counter_Init(int16_T rtp_z_cntInit, DW_Counter *localDW)
{
  /* InitializeConditions for UnitDelay: '<S18>/UnitDelay' */
  localDW->UnitDelay_DSTATE = rtp_z_cntInit;
}

/* Output and update for atomic system: '<S13>/Counter' */
int16_T Counter(int16_T rtu_inc, int16_T rtu_max, boolean_T rtu_rst, DW_Counter *
                localDW)
{
  int16_T rty_cnt_0;

  /* UnitDelay: '<S18>/UnitDelay' */
  rty_cnt_0 = localDW->UnitDelay_DSTATE;

  /* Switch: '<S18>/Switch1' incorporates:
   *  Constant: '<S18>/Constant23'
   */
  if (rtu_rst) {
    rty_cnt_0 = 0;
  }

  /* End of Switch: '<S18>/Switch1' */

  /* Sum: '<S16>/Sum1' */
  rty_cnt_0 += rtu_inc;

  /* MinMax: '<S16>/MinMax' */
  if (rty_cnt_0 <= rtu_max) {
    /* Update for UnitDelay: '<S18>/UnitDelay' */
    localDW->UnitDelay_DSTATE = rty_cnt_0;
  } else {
    /* Update for UnitDelay: '<S18>/UnitDelay' */
    localDW->UnitDelay_DSTATE = rtu_max;
  }

  /* End of MinMax: '<S16>/MinMax' */
  return rty_cnt_0;
}

/* System reset for atomic system: '<S53>/Low_Pass_Filter' */
void Low_Pass_Filter_Reset(DW_Low_Pass_Filter *localDW)
{
  /* InitializeConditions for UnitDelay: '<S59>/UnitDelay1' */
  localDW->UnitDelay1_DSTATE[0] = 0;
  localDW->UnitDelay1_DSTATE[1] = 0;
}

/* Output and update for atomic system: '<S53>/Low_Pass_Filter' */
void Low_Pass_Filter(const int16_T rtu_u[2], uint16_T rtu_coef, int16_T rty_y[2],
                     DW_Low_Pass_Filter *localDW)
{
  int32_T rtb_Sum3_c_0;

  /* Sum: '<S59>/Sum2' incorporates:
   *  UnitDelay: '<S59>/UnitDelay1'
   */
  rtb_Sum3_c_0 = rtu_u[0] - (localDW->UnitDelay1_DSTATE[0] >> 16);
  if (rtb_Sum3_c_0 > 32767) {
    rtb_Sum3_c_0 = 32767;
  } else if (rtb_Sum3_c_0 < -32768) {
    rtb_Sum3_c_0 = -32768;
  }

  /* Sum: '<S59>/Sum3' incorporates:
   *  Product: '<S59>/Divide3'
   *  Sum: '<S59>/Sum2'
   *  UnitDelay: '<S59>/UnitDelay1'
   */
  rtb_Sum3_c_0 = rtu_coef * rtb_Sum3_c_0 + localDW->UnitDelay1_DSTATE[0];

  /* DataTypeConversion: '<S59>/Data Type Conversion' incorporates:
   *  Sum: '<S59>/Sum3'
   */
  rty_y[0] = (int16_T)(rtb_Sum3_c_0 >> 16);

  /* Update for UnitDelay: '<S59>/UnitDelay1' incorporates:
   *  Sum: '<S59>/Sum3'
   */
  localDW->UnitDelay1_DSTATE[0] = rtb_Sum3_c_0;

  /* Sum: '<S59>/Sum2' incorporates:
   *  UnitDelay: '<S59>/UnitDelay1'
   */
  rtb_Sum3_c_0 = rtu_u[1] - (localDW->UnitDelay1_DSTATE[1] >> 16);
  if (rtb_Sum3_c_0 > 32767) {
    rtb_Sum3_c_0 = 32767;
  } else if (rtb_Sum3_c_0 < -32768) {
    rtb_Sum3_c_0 = -32768;
  }

  /* Sum: '<S59>/Sum3' incorporates:
   *  Product: '<S59>/Divide3'
   *  Sum: '<S59>/Sum2'
   *  UnitDelay: '<S59>/UnitDelay1'
   */
  rtb_Sum3_c_0 = rtu_coef * rtb_Sum3_c_0 + localDW->UnitDelay1_DSTATE[1];

  /* DataTypeConversion: '<S59>/Data Type Conversion' incorporates:
   *  Sum: '<S59>/Sum3'
   */
  rty_y[1] = (int16_T)(rtb_Sum3_c_0 >> 16);

  /* Update for UnitDelay: '<S59>/UnitDelay1' incorporates:
   *  Sum: '<S59>/Sum3'
   */
  localDW->UnitDelay1_DSTATE[1] = rtb_Sum3_c_0;
}

/*
 * System initialize for atomic system:
 *    '<S25>/Counter'
 *    '<S24>/Counter'
 */
void Counter_e_Init(uint16_T rtp_z_cntInit, DW_Counter_d *localDW)
{
  /* InitializeConditions for UnitDelay: '<S30>/UnitDelay' */
  localDW->UnitDelay_DSTATE = rtp_z_cntInit;
}

/*
 * Output and update for atomic system:
 *    '<S25>/Counter'
 *    '<S24>/Counter'
 */
uint16_T Counter_e(uint16_T rtu_inc, uint16_T rtu_max, boolean_T rtu_rst,
                   DW_Counter_d *localDW)
{
  uint16_T rty_cnt_0;
  uint16_T tmp;

  /* Switch: '<S30>/Switch1' incorporates:
   *  Constant: '<S30>/Constant23'
   *  UnitDelay: '<S30>/UnitDelay'
   */
  if (rtu_rst) {
    tmp = 0U;
  } else {
    tmp = localDW->UnitDelay_DSTATE;
  }

  /* Sum: '<S29>/Sum1' incorporates:
   *  Switch: '<S30>/Switch1'
   */
  rty_cnt_0 = (uint16_T)(rtu_inc + tmp);

  /* MinMax: '<S29>/MinMax' */
  if (rty_cnt_0 <= rtu_max) {
    /* Update for UnitDelay: '<S30>/UnitDelay' */
    localDW->UnitDelay_DSTATE = rty_cnt_0;
  } else {
    /* Update for UnitDelay: '<S30>/UnitDelay' */
    localDW->UnitDelay_DSTATE = rtu_max;
  }

  /* End of MinMax: '<S29>/MinMax' */
  return rty_cnt_0;
}

/*
 * Output and update for atomic system:
 *    '<S21>/either_edge'
 *    '<S20>/either_edge'
 */
boolean_T either_edge(boolean_T rtu_u, DW_either_edge *localDW)
{
  boolean_T rty_y_0;

  /* RelationalOperator: '<S26>/Relational Operator' incorporates:
   *  UnitDelay: '<S26>/UnitDelay'
   */
  rty_y_0 = (rtu_u != localDW->UnitDelay_DSTATE);

  /* Update for UnitDelay: '<S26>/UnitDelay' */
  localDW->UnitDelay_DSTATE = rtu_u;
  return rty_y_0;
}

/* System initialize for atomic system: '<S20>/Debounce_Filter' */
void Debounce_Filter_Init(boolean_T *rty_y, DW_Debounce_Filter *localDW)
{
  /* SystemInitialize for IfAction SubSystem: '<S21>/Qualification' */
  /* SystemInitialize for Atomic SubSystem: '<S25>/Counter' */
  Counter_e_Init(0, &localDW->Counter_e2);

  /* End of SystemInitialize for SubSystem: '<S25>/Counter' */
  /* End of SystemInitialize for SubSystem: '<S21>/Qualification' */

  /* SystemInitialize for IfAction SubSystem: '<S21>/Dequalification' */
  /* SystemInitialize for Atomic SubSystem: '<S24>/Counter' */
  Counter_e_Init(0, &localDW->Counter_n);

  /* End of SystemInitialize for SubSystem: '<S24>/Counter' */
  /* End of SystemInitialize for SubSystem: '<S21>/Dequalification' */

  /* SystemInitialize for Merge: '<S21>/Merge' */
  *rty_y = false;
}

/* Output and update for atomic system: '<S20>/Debounce_Filter' */
void Debounce_Filter(boolean_T rtu_u, uint16_T rtu_tAcv, uint16_T rtu_tDeacv,
                     boolean_T *rty_y, DW_Debounce_Filter *localDW)
{
  uint16_T rtb_Sum1_i0;
  boolean_T rtb_RelationalOperator_o;

  /* Outputs for Atomic SubSystem: '<S21>/either_edge' */
  rtb_RelationalOperator_o = either_edge(rtu_u, &localDW->either_edge_n);

  /* End of Outputs for SubSystem: '<S21>/either_edge' */

  /* If: '<S21>/If2' incorporates:
   *  Constant: '<S24>/Constant6'
   *  Constant: '<S25>/Constant6'
   *  Logic: '<S21>/Logical Operator1'
   *  Logic: '<S21>/Logical Operator2'
   *  Logic: '<S21>/Logical Operator3'
   *  Logic: '<S21>/Logical Operator4'
   *  UnitDelay: '<S21>/UnitDelay'
   */
  if (rtu_u && (!localDW->UnitDelay_DSTATE)) {
    /* Outputs for IfAction SubSystem: '<S21>/Qualification' incorporates:
     *  ActionPort: '<S25>/Action Port'
     */
    /* Outputs for Atomic SubSystem: '<S25>/Counter' */
    rtb_Sum1_i0 = Counter_e(1, rtu_tAcv, rtb_RelationalOperator_o,
      &localDW->Counter_e2);

    /* End of Outputs for SubSystem: '<S25>/Counter' */

    /* Switch: '<S25>/Switch2' incorporates:
     *  Constant: '<S25>/Constant6'
     *  RelationalOperator: '<S25>/Relational Operator2'
     */
    *rty_y = (rtb_Sum1_i0 > rtu_tAcv);

    /* End of Outputs for SubSystem: '<S21>/Qualification' */
  } else if ((!rtu_u) && localDW->UnitDelay_DSTATE) {
    /* Outputs for IfAction SubSystem: '<S21>/Dequalification' incorporates:
     *  ActionPort: '<S24>/Action Port'
     */
    /* Outputs for Atomic SubSystem: '<S24>/Counter' */
    rtb_Sum1_i0 = Counter_e(1, rtu_tDeacv, rtb_RelationalOperator_o,
      &localDW->Counter_n);

    /* End of Outputs for SubSystem: '<S24>/Counter' */

    /* Switch: '<S24>/Switch2' incorporates:
     *  Constant: '<S24>/Constant6'
     *  RelationalOperator: '<S24>/Relational Operator2'
     */
    *rty_y = (rtb_Sum1_i0 <= rtu_tDeacv);

    /* End of Outputs for SubSystem: '<S21>/Dequalification' */
  } else {
    /* Outputs for IfAction SubSystem: '<S21>/Default' incorporates:
     *  ActionPort: '<S23>/Action Port'
     */
    /* SignalConversion generated from: '<S23>/yPrev' */
    *rty_y = localDW->UnitDelay_DSTATE;

    /* End of Outputs for SubSystem: '<S21>/Default' */
  }

  /* End of If: '<S21>/If2' */

  /* Update for UnitDelay: '<S21>/UnitDelay' */
  localDW->UnitDelay_DSTATE = *rty_y;
}

/*
 * System initialize for atomic system:
 *    '<S193>/I_backCalc_fixdt'
 *    '<S193>/I_backCalc_fixdt1'
 *    '<S191>/I_backCalc_fixdt'
 */
void I_backCalc_fixdt_Init(int32_T rtp_yInit, DW_I_backCalc_fixdt *localDW)
{
  /* InitializeConditions for UnitDelay: '<S202>/UnitDelay' */
  localDW->UnitDelay_DSTATE_a = rtp_yInit;
}

/*
 * System reset for atomic system:
 *    '<S193>/I_backCalc_fixdt'
 *    '<S193>/I_backCalc_fixdt1'
 *    '<S191>/I_backCalc_fixdt'
 */
void I_backCalc_fixdt_Reset(int32_T rtp_yInit, DW_I_backCalc_fixdt *localDW)
{
  /* InitializeConditions for UnitDelay: '<S200>/UnitDelay' */
  localDW->UnitDelay_DSTATE = 0;

  /* InitializeConditions for UnitDelay: '<S202>/UnitDelay' */
  localDW->UnitDelay_DSTATE_a = rtp_yInit;
}

/*
 * Output and update for atomic system:
 *    '<S193>/I_backCalc_fixdt'
 *    '<S193>/I_backCalc_fixdt1'
 *    '<S191>/I_backCalc_fixdt'
 */
void I_backCalc_fixdt(int16_T rtu_err, uint16_T rtu_I, uint16_T rtu_Kb, int16_T
                      rtu_satMax, int16_T rtu_satMin, int16_T *rty_out,
                      DW_I_backCalc_fixdt *localDW)
{
  int64_T tmp;
  int32_T rtb_Sum1_bk;
  int16_T rtb_DataTypeConversion1_l;

  /* Sum: '<S200>/Sum2' incorporates:
   *  Product: '<S200>/Divide2'
   *  UnitDelay: '<S200>/UnitDelay'
   */
  tmp = (rtu_err * rtu_I + ((int64_T)localDW->UnitDelay_DSTATE << 4)) >> 4;
  if (tmp > 2147483647LL) {
    tmp = 2147483647LL;
  } else if (tmp < -2147483648LL) {
    tmp = -2147483648LL;
  }

  /* Sum: '<S202>/Sum1' incorporates:
   *  Sum: '<S200>/Sum2'
   *  UnitDelay: '<S202>/UnitDelay'
   */
  rtb_Sum1_bk = (int32_T)tmp + localDW->UnitDelay_DSTATE_a;

  /* DataTypeConversion: '<S202>/Data Type Conversion1' incorporates:
   *  Sum: '<S202>/Sum1'
   */
  rtb_DataTypeConversion1_l = (int16_T)(rtb_Sum1_bk >> 12);

  /* Switch: '<S203>/Switch2' incorporates:
   *  DataTypeConversion: '<S202>/Data Type Conversion1'
   *  RelationalOperator: '<S203>/LowerRelop1'
   *  RelationalOperator: '<S203>/UpperRelop'
   *  Switch: '<S203>/Switch'
   */
  if (rtb_DataTypeConversion1_l > rtu_satMax) {
    *rty_out = rtu_satMax;
  } else if (rtb_DataTypeConversion1_l < rtu_satMin) {
    /* Switch: '<S203>/Switch' */
    *rty_out = rtu_satMin;
  } else {
    *rty_out = rtb_DataTypeConversion1_l;
  }

  /* End of Switch: '<S203>/Switch2' */

  /* Update for UnitDelay: '<S200>/UnitDelay' incorporates:
   *  DataTypeConversion: '<S202>/Data Type Conversion1'
   *  Product: '<S200>/Divide1'
   *  Sum: '<S200>/Sum3'
   */
  localDW->UnitDelay_DSTATE = (int16_T)(*rty_out - rtb_DataTypeConversion1_l) *
    rtu_Kb;

  /* Update for UnitDelay: '<S202>/UnitDelay' incorporates:
   *  Sum: '<S202>/Sum1'
   */
  localDW->UnitDelay_DSTATE_a = rtb_Sum1_bk;
}

/* System initialize for atomic system: '<S64>/PI_clamp_fixdt' */
void PI_clamp_fixdt_Init(DW_PI_clamp_fixdt *localDW)
{
  /* InitializeConditions for Delay: '<S70>/Resettable Delay' */
  localDW->icLoad = true;
}

/* System reset for atomic system: '<S64>/PI_clamp_fixdt' */
void PI_clamp_fixdt_Reset(DW_PI_clamp_fixdt *localDW)
{
  /* InitializeConditions for UnitDelay: '<S68>/UnitDelay1' */
  localDW->UnitDelay1_DSTATE = false;

  /* InitializeConditions for Delay: '<S70>/Resettable Delay' */
  localDW->icLoad = true;
}

/* Output and update for atomic system: '<S64>/PI_clamp_fixdt' */
int16_T PI_clamp_fixdt(int16_T rtu_err, uint16_T rtu_P, uint16_T rtu_I, int16_T
  rtu_init, int16_T rtu_satMax, int16_T rtu_satMin, int32_T rtu_ext_limProt,
  DW_PI_clamp_fixdt *localDW)
{
  int16_T rty_out_0;
  int64_T tmp;
  int32_T rtb_Sum1_e;
  int32_T tmp_0;
  int32_T tmp_1;
  int16_T tmp_2;
  boolean_T rtb_LowerRelop1_gm;
  boolean_T rtb_UpperRelop_o;

  /* Sum: '<S68>/Sum2' incorporates:
   *  Product: '<S68>/Divide2'
   */
  tmp = (int64_T)(rtu_err * rtu_I) + rtu_ext_limProt;
  if (tmp > 2147483647LL) {
    tmp = 2147483647LL;
  } else if (tmp < -2147483648LL) {
    tmp = -2147483648LL;
  }

  /* Delay: '<S70>/Resettable Delay' */
  if (localDW->icLoad) {
    localDW->ResettableDelay_DSTATE = rtu_init << 16;
  }

  /* Switch: '<S68>/Switch1' incorporates:
   *  Constant: '<S68>/Constant'
   *  Sum: '<S68>/Sum2'
   *  UnitDelay: '<S68>/UnitDelay1'
   */
  if (localDW->UnitDelay1_DSTATE) {
    tmp_0 = 0;
  } else {
    tmp_0 = (int32_T)tmp;
  }

  /* Sum: '<S70>/Sum1' incorporates:
   *  Delay: '<S70>/Resettable Delay'
   *  Switch: '<S68>/Switch1'
   */
  rtb_Sum1_e = tmp_0 + localDW->ResettableDelay_DSTATE;

  /* Product: '<S68>/Divide5' */
  tmp_0 = (rtu_err * rtu_P) >> 11;
  if (tmp_0 > 32767) {
    tmp_0 = 32767;
  } else if (tmp_0 < -32768) {
    tmp_0 = -32768;
  }

  /* Sum: '<S68>/Sum1' incorporates:
   *  DataTypeConversion: '<S70>/Data Type Conversion1'
   *  Product: '<S68>/Divide5'
   *  Sum: '<S70>/Sum1'
   */
  tmp_0 = (((rtb_Sum1_e >> 16) << 1) + tmp_0) >> 1;
  if (tmp_0 > 32767) {
    tmp_0 = 32767;
  } else if (tmp_0 < -32768) {
    tmp_0 = -32768;
  }

  /* RelationalOperator: '<S71>/LowerRelop1' incorporates:
   *  Sum: '<S68>/Sum1'
   */
  rtb_LowerRelop1_gm = (tmp_0 > rtu_satMax);

  /* RelationalOperator: '<S71>/UpperRelop' incorporates:
   *  Sum: '<S68>/Sum1'
   */
  rtb_UpperRelop_o = (tmp_0 < rtu_satMin);

  /* Switch: '<S71>/Switch1' incorporates:
   *  Sum: '<S68>/Sum1'
   *  Switch: '<S71>/Switch3'
   */
  if (rtb_LowerRelop1_gm) {
    rty_out_0 = rtu_satMax;
  } else if (rtb_UpperRelop_o) {
    /* Switch: '<S71>/Switch3' */
    rty_out_0 = rtu_satMin;
  } else {
    rty_out_0 = (int16_T)tmp_0;
  }

  /* End of Switch: '<S71>/Switch1' */

  /* Signum: '<S69>/SignDeltaU2' incorporates:
   *  Sum: '<S68>/Sum2'
   */
  if ((int32_T)tmp < 0) {
    tmp_1 = -1;
  } else {
    tmp_1 = ((int32_T)tmp > 0);
  }

  /* Signum: '<S69>/SignDeltaU3' incorporates:
   *  Sum: '<S68>/Sum1'
   */
  if (tmp_0 < 0) {
    tmp_2 = -1;
  } else {
    tmp_2 = (int16_T)(tmp_0 > 0);
  }

  /* Update for UnitDelay: '<S68>/UnitDelay1' incorporates:
   *  Logic: '<S68>/AND1'
   *  Logic: '<S69>/AND1'
   *  RelationalOperator: '<S69>/Equal1'
   *  Signum: '<S69>/SignDeltaU2'
   *  Signum: '<S69>/SignDeltaU3'
   */
  localDW->UnitDelay1_DSTATE = ((tmp_1 == tmp_2) && (rtb_LowerRelop1_gm ||
    rtb_UpperRelop_o));

  /* Update for Delay: '<S70>/Resettable Delay' incorporates:
   *  Sum: '<S70>/Sum1'
   */
  localDW->icLoad = false;
  localDW->ResettableDelay_DSTATE = rtb_Sum1_e;
  return rty_out_0;
}

/* Function for Chart: '<S5>/F03_02_Control_Mode_Manager' */
void enter_internal_ACTIVE(const boolean_T *LogicalOperator1, const boolean_T
  *LogicalOperator2, DW *rtDW)
{
  if (*LogicalOperator2) {
    rtDW->is_ACTIVE = IN_TORQUE_MODE;
    rtDW->z_ctrlMod = TRQ_MODE;
  } else if (*LogicalOperator1) {
    rtDW->is_ACTIVE = IN_SPEED_MODE;
    rtDW->z_ctrlMod = SPD_MODE;
  } else {
    rtDW->is_ACTIVE = IN_VOLTAGE_MODE;
    rtDW->z_ctrlMod = VLT_MODE;
  }
}

/* Model step function */
void BLDC_controller_step(RT_MODEL *const rtM)
{
  P *rtP = ((P *) rtM->defaultParam);
  DW *rtDW = rtM->dwork;
  ExtU *rtU = (ExtU *) rtM->inputs;
  ExtY *rtY = (ExtY *) rtM->outputs;
  int32_T rtb_Gain3;
  int32_T rtb_Sum1_h;
  int16_T Abs5;
  int16_T Switch2;
  int16_T rtb_Merge;
  int16_T rtb_Merge1;
  int16_T rtb_MinMax2;
  int16_T rtb_Saturation;
  int16_T rtb_Saturation1;
  int16_T rtb_Switch3_h;
  uint16_T rtb_Divide1;
  uint16_T rtb_f;
  int8_T UnitDelay3;
  int8_T rtb_Sum2_ii;
  uint8_T Sum;
  uint8_T rtb_f_j;
  uint8_T rtb_k_o;
  boolean_T LogicalOperator1;
  boolean_T LogicalOperator2;
  boolean_T rtb_LogicalOperator;
  boolean_T rtb_RelationalOperator;
  boolean_T rtb_RelationalOperator4_i;
  boolean_T rtb_n_commDeacv;

  /* Outputs for Atomic SubSystem: '<Root>/BLDC_controller' */
  /* Sum: '<S11>/Sum' incorporates:
   *  Gain: '<S11>/g_Ha'
   *  Gain: '<S11>/g_Hb'
   */
  Sum = (uint8_T)((uint8_T)((uint8_T)(rtU->b_hallA << 2) + (uint8_T)
    (rtU->b_hallB << 1)) + rtU->b_hallC);

  /* Logic: '<S10>/Logical Operator' incorporates:
   *  UnitDelay: '<S10>/UnitDelay1'
   *  UnitDelay: '<S10>/UnitDelay2'
   *  UnitDelay: '<S10>/UnitDelay3'
   */
  rtb_LogicalOperator = (boolean_T)((rtU->b_hallA != 0) ^ (rtU->b_hallB != 0) ^
    (rtU->b_hallC != 0) ^ (rtDW->UnitDelay3_DSTATE_f != 0) ^
    (rtDW->UnitDelay1_DSTATE != 0)) ^ (rtDW->UnitDelay2_DSTATE_k != 0);

  /* If: '<S13>/If2' incorporates:
   *  If: '<S3>/If2'
   */
  if (rtb_LogicalOperator) {
    /* Outputs for IfAction SubSystem: '<S3>/F01_03_Direction_Detection' incorporates:
     *  ActionPort: '<S12>/Action Port'
     */
    /* UnitDelay: '<S12>/UnitDelay3' */
    UnitDelay3 = rtDW->Switch2_a;

    /* Sum: '<S12>/Sum2' incorporates:
     *  Constant: '<S11>/vec_hallToPos'
     *  Selector: '<S11>/Selector'
     *  UnitDelay: '<S12>/UnitDelay2'
     */
    rtb_Sum2_ii = (int8_T)(rtConstP.vec_hallToPos_Value[Sum] -
      rtDW->UnitDelay2_DSTATE_o);

    /* Switch: '<S12>/Switch2' incorporates:
     *  Constant: '<S12>/Constant20'
     *  Constant: '<S12>/Constant8'
     *  Logic: '<S12>/Logical Operator3'
     *  RelationalOperator: '<S12>/Relational Operator1'
     *  RelationalOperator: '<S12>/Relational Operator6'
     */
    if ((rtb_Sum2_ii == 1) || (rtb_Sum2_ii == -5)) {
      /* Switch: '<S12>/Switch2' incorporates:
       *  Constant: '<S12>/Constant24'
       */
      rtDW->Switch2_a = 1;
    } else {
      /* Switch: '<S12>/Switch2' incorporates:
       *  Constant: '<S12>/Constant23'
       */
      rtDW->Switch2_a = -1;
    }

    /* End of Switch: '<S12>/Switch2' */

    /* Update for UnitDelay: '<S12>/UnitDelay2' incorporates:
     *  Constant: '<S11>/vec_hallToPos'
     *  Selector: '<S11>/Selector'
     */
    rtDW->UnitDelay2_DSTATE_o = rtConstP.vec_hallToPos_Value[Sum];

    /* End of Outputs for SubSystem: '<S3>/F01_03_Direction_Detection' */

    /* Outputs for IfAction SubSystem: '<S13>/Raw_Motor_Speed_Estimation' incorporates:
     *  ActionPort: '<S17>/Action Port'
     */
    /* SignalConversion generated from: '<S17>/z_counterRawPrev' incorporates:
     *  UnitDelay: '<S13>/UnitDelay3'
     */
    rtDW->z_counterRawPrev = rtDW->UnitDelay3_DSTATE;

    /* Sum: '<S17>/Sum7' incorporates:
     *  UnitDelay: '<S17>/UnitDelay4'
     */
    rtb_Switch3_h = (int16_T)(rtDW->z_counterRawPrev - rtDW->UnitDelay4_DSTATE);

    /* Abs: '<S17>/Abs2' */
    if (rtb_Switch3_h < 0) {
      rtb_Switch3_h = (int16_T)-rtb_Switch3_h;
    }

    /* End of Abs: '<S17>/Abs2' */

    /* Relay: '<S17>/dz_cntTrnsDet' */
    rtDW->dz_cntTrnsDet_Mode = ((rtb_Switch3_h >= rtP->dz_cntTrnsDetHi) ||
      ((rtb_Switch3_h > rtP->dz_cntTrnsDetLo) && rtDW->dz_cntTrnsDet_Mode));

    /* Relay: '<S17>/dz_cntTrnsDet' */
    rtDW->dz_cntTrnsDet = rtDW->dz_cntTrnsDet_Mode;

    /* RelationalOperator: '<S17>/Relational Operator4' */
    rtb_RelationalOperator4_i = (rtDW->Switch2_a != UnitDelay3);

    /* Switch: '<S17>/Switch3' incorporates:
     *  Constant: '<S17>/Constant4'
     *  Logic: '<S17>/Logical Operator1'
     *  Switch: '<S17>/Switch1'
     *  Switch: '<S17>/Switch2'
     *  UnitDelay: '<S17>/UnitDelay1'
     */
    if (rtb_RelationalOperator4_i && rtDW->UnitDelay1_DSTATE_g) {
      rtb_Switch3_h = 0;
    } else if (rtb_RelationalOperator4_i) {
      /* Switch: '<S17>/Switch3' incorporates:
       *  Switch: '<S17>/Switch2'
       *  UnitDelay: '<S13>/UnitDelay4'
       */
      rtb_Switch3_h = rtDW->UnitDelay4_DSTATE_f;
    } else if (rtDW->dz_cntTrnsDet) {
      /* Switch: '<S17>/Switch3' incorporates:
       *  Constant: '<S17>/cf_speedCoef'
       *  Product: '<S17>/Divide14'
       *  Switch: '<S17>/Switch1'
       *  Switch: '<S17>/Switch2'
       */
      rtb_Switch3_h = (int16_T)((rtP->cf_speedCoef << 4) / rtDW->z_counterRawPrev);
    } else {
      /* Switch: '<S17>/Switch3' incorporates:
       *  Constant: '<S17>/cf_speedCoef'
       *  Product: '<S17>/Divide13'
       *  Sum: '<S17>/Sum13'
       *  Switch: '<S17>/Switch1'
       *  Switch: '<S17>/Switch2'
       *  UnitDelay: '<S17>/UnitDelay2'
       *  UnitDelay: '<S17>/UnitDelay3'
       *  UnitDelay: '<S17>/UnitDelay5'
       */
      rtb_Switch3_h = (int16_T)((rtP->cf_speedCoef << 4) / (int16_T)((int16_T)
        ((int16_T)(rtDW->UnitDelay2_DSTATE + rtDW->UnitDelay3_DSTATE_k) +
         rtDW->UnitDelay5_DSTATE) + rtDW->z_counterRawPrev));
    }

    /* End of Switch: '<S17>/Switch3' */

    /* Product: '<S17>/Divide11' incorporates:
     *  Switch: '<S17>/Switch3'
     */
    rtDW->Divide11 = (int16_T)(rtb_Switch3_h * rtDW->Switch2_a);

    /* Update for UnitDelay: '<S17>/UnitDelay4' */
    rtDW->UnitDelay4_DSTATE = rtDW->z_counterRawPrev;

    /* Update for UnitDelay: '<S17>/UnitDelay2' incorporates:
     *  UnitDelay: '<S17>/UnitDelay3'
     */
    rtDW->UnitDelay2_DSTATE = rtDW->UnitDelay3_DSTATE_k;

    /* Update for UnitDelay: '<S17>/UnitDelay3' incorporates:
     *  UnitDelay: '<S17>/UnitDelay5'
     */
    rtDW->UnitDelay3_DSTATE_k = rtDW->UnitDelay5_DSTATE;

    /* Update for UnitDelay: '<S17>/UnitDelay5' */
    rtDW->UnitDelay5_DSTATE = rtDW->z_counterRawPrev;

    /* Update for UnitDelay: '<S17>/UnitDelay1' */
    rtDW->UnitDelay1_DSTATE_g = rtb_RelationalOperator4_i;

    /* End of Outputs for SubSystem: '<S13>/Raw_Motor_Speed_Estimation' */
  }

  /* End of If: '<S13>/If2' */

  /* Outputs for Atomic SubSystem: '<S13>/Counter' */
  /* Constant: '<S13>/Constant6' incorporates:
   *  Constant: '<S13>/z_maxCntRst2'
   */
  rtb_Switch3_h = Counter(1, rtP->z_maxCntRst, rtb_LogicalOperator,
    &rtDW->Counter_c);

  /* End of Outputs for SubSystem: '<S13>/Counter' */

  /* Switch: '<S13>/Switch2' incorporates:
   *  Constant: '<S13>/z_maxCntRst'
   *  RelationalOperator: '<S13>/Relational Operator2'
   */
  if (rtb_Switch3_h > rtP->z_maxCntRst) {
    /* Switch: '<S13>/Switch2' incorporates:
     *  Constant: '<S13>/Constant4'
     */
    Switch2 = 0;
  } else {
    /* Switch: '<S13>/Switch2' incorporates:
     *  Product: '<S17>/Divide11'
     */
    Switch2 = rtDW->Divide11;
  }

  /* End of Switch: '<S13>/Switch2' */

  /* Abs: '<S13>/Abs5' incorporates:
   *  Switch: '<S13>/Switch2'
   */
  if (Switch2 < 0) {
    /* Abs: '<S13>/Abs5' */
    Abs5 = (int16_T)-Switch2;
  } else {
    /* Abs: '<S13>/Abs5' */
    Abs5 = Switch2;
  }

  /* End of Abs: '<S13>/Abs5' */

  /* Relay: '<S13>/n_commDeacv' incorporates:
   *  Abs: '<S13>/Abs5'
   */
  rtDW->n_commDeacv_Mode = ((Abs5 >= rtP->n_commDeacvHi) || ((Abs5 >
    rtP->n_commAcvLo) && rtDW->n_commDeacv_Mode));

  /* Logic: '<S13>/Logical Operator3' incorporates:
   *  Constant: '<S13>/b_angleMeasEna'
   *  Logic: '<S13>/Logical Operator1'
   *  Logic: '<S13>/Logical Operator2'
   *  Relay: '<S13>/n_commDeacv'
   */
  rtb_LogicalOperator = (rtP->b_angleMeasEna || (rtDW->n_commDeacv_Mode &&
    (!rtDW->dz_cntTrnsDet)));

  /* UnitDelay: '<S2>/UnitDelay2' */
  rtb_RelationalOperator4_i = rtDW->UnitDelay2_DSTATE_g;

  /* UnitDelay: '<S2>/UnitDelay5' */
  rtb_n_commDeacv = rtDW->UnitDelay5_DSTATE_l;

  /* Saturate: '<S1>/Saturation' */
  rtDW->Switch1_m = rtU->i_phaAB << 4;
  if (rtDW->Switch1_m >= 27200) {
    rtb_Saturation = 27200;
  } else if (rtDW->Switch1_m <= -27200) {
    rtb_Saturation = -27200;
  } else {
    rtb_Saturation = (int16_T)(rtU->i_phaAB << 4);
  }

  /* End of Saturate: '<S1>/Saturation' */

  /* Saturate: '<S1>/Saturation1' */
  rtDW->Switch1_m = rtU->i_phaBC << 4;
  if (rtDW->Switch1_m >= 27200) {
    rtb_Saturation1 = 27200;
  } else if (rtDW->Switch1_m <= -27200) {
    rtb_Saturation1 = -27200;
  } else {
    rtb_Saturation1 = (int16_T)(rtU->i_phaBC << 4);
  }

  /* End of Saturate: '<S1>/Saturation1' */

  /* If: '<S3>/If1' incorporates:
   *  Constant: '<S3>/b_angleMeasEna'
   */
  if (!rtP->b_angleMeasEna) {
    /* Outputs for IfAction SubSystem: '<S3>/F01_05_Electrical_Angle_Estimation' incorporates:
     *  ActionPort: '<S14>/Action Port'
     */
    /* Switch: '<S14>/Switch3' incorporates:
     *  Constant: '<S11>/vec_hallToPos'
     *  Constant: '<S14>/Constant16'
     *  Constant: '<S14>/Constant2'
     *  RelationalOperator: '<S14>/Relational Operator7'
     *  Selector: '<S11>/Selector'
     *  Sum: '<S14>/Sum1'
     */
    if (rtDW->Switch2_a == 1) {
      rtb_Sum2_ii = rtConstP.vec_hallToPos_Value[Sum];
    } else {
      rtb_Sum2_ii = (int8_T)(rtConstP.vec_hallToPos_Value[Sum] + 1);
    }

    /* End of Switch: '<S14>/Switch3' */

    /* Switch: '<S14>/Switch2' incorporates:
     *  MinMax: '<S14>/MinMax'
     *  Product: '<S14>/Divide1'
     *  Product: '<S14>/Divide2'
     *  Product: '<S14>/Divide3'
     *  Sum: '<S14>/Sum3'
     */
    if (rtb_LogicalOperator) {
      /* MinMax: '<S14>/MinMax' */
      if (rtb_Switch3_h <= rtDW->z_counterRawPrev) {
        rtb_Merge1 = rtb_Switch3_h;
      } else {
        rtb_Merge1 = rtDW->z_counterRawPrev;
      }

      rtb_Merge = (int16_T)(((int16_T)((int16_T)((rtb_Merge1 << 14) /
        rtDW->z_counterRawPrev) * rtDW->Switch2_a) + (rtb_Sum2_ii << 14)) >> 2);
    } else {
      rtb_Merge = (int16_T)(rtb_Sum2_ii << 12);
    }

    /* End of Switch: '<S14>/Switch2' */

    /* MinMax: '<S14>/MinMax1' incorporates:
     *  Constant: '<S14>/Constant1'
     *  Product: '<S14>/Divide2'
     */
    if (rtb_Merge < 0) {
      rtb_Merge = 0;
    }

    /* SignalConversion: '<S14>/Signal Conversion2' incorporates:
     *  Merge: '<S3>/Merge'
     *  MinMax: '<S14>/MinMax1'
     *  Product: '<S14>/Divide2'
     */
    rtb_Merge = (int16_T)((15 * rtb_Merge) >> 4);

    /* End of Outputs for SubSystem: '<S3>/F01_05_Electrical_Angle_Estimation' */
  } else {
    /* Outputs for IfAction SubSystem: '<S3>/F01_06_Electrical_Angle_Measurement' incorporates:
     *  ActionPort: '<S15>/Action Port'
     */
    /* Sum: '<S15>/Sum1' incorporates:
     *  Constant: '<S15>/Constant2'
     *  Constant: '<S15>/n_polePairs'
     *  Product: '<S15>/Divide'
     */
    rtDW->Sum1 = rtU->a_mechAngle * rtP->n_polePairs - 1920;

    /* SignalConversion generated from: '<S15>/a_elecAngle' incorporates:
     *  Constant: '<S15>/a_elecPeriod'
     *  DataTypeConversion: '<S15>/Data Type Conversion20'
     *  Merge: '<S3>/Merge'
     *  Product: '<S19>/Divide2'
     *  Product: '<S19>/Divide3'
     *  Sum: '<S15>/Sum1'
     *  Sum: '<S19>/Sum3'
     */
    rtb_Merge = (int16_T)(rtDW->Sum1 - ((int16_T)((int16_T)div_nde_s32_floor
      (rtDW->Sum1, 23040) * 360) << 6));

    /* End of Outputs for SubSystem: '<S3>/F01_06_Electrical_Angle_Measurement' */
  }

  /* End of If: '<S3>/If1' */

  /* If: '<S7>/If1' incorporates:
   *  Constant: '<S1>/z_ctrlTypSel'
   */
  rtb_Sum2_ii = rtDW->If1_ActiveSubsystem;
  UnitDelay3 = -1;
  if (rtP->z_ctrlTypSel == 2) {
    UnitDelay3 = 0;
  }

  rtDW->If1_ActiveSubsystem = UnitDelay3;
  if ((rtb_Sum2_ii != UnitDelay3) && (rtb_Sum2_ii == 0)) {
    /* Disable for If: '<S48>/If2' */
    if (rtDW->If2_ActiveSubsystem_a == 0) {
      /* Disable for Outport: '<S53>/iq' incorporates:
       *  DataTypeConversion: '<S59>/Data Type Conversion'
       * */
      rtDW->DataTypeConversion[0] = 0;

      /* Disable for Abs: '<S53>/Abs5' incorporates:
       *  Outport: '<S53>/iqAbs'
       */
      rtDW->Abs5_d = 0;

      /* Disable for SignalConversion generated from: '<S53>/id' incorporates:
       *  Outport: '<S53>/id'
       */
      rtDW->OutportBufferForid_f = 0;
    }

    rtDW->If2_ActiveSubsystem_a = -1;

    /* End of Disable for If: '<S48>/If2' */

    /* Disable for Interpolation_n-D generated from: '<S55>/r_sin_M1' incorporates:
     *  Outport: '<S48>/r_sin'
     */
    rtDW->r_sin_M1_1 = 0;

    /* Disable for Interpolation_n-D generated from: '<S55>/r_cos_M1' incorporates:
     *  Outport: '<S48>/r_cos'
     */
    rtDW->r_cos_M1_1 = 0;

    /* Disable for Outport: '<Root>/iq' incorporates:
     *  Outport: '<S48>/iq'
     */
    rtY->iq = 0;

    /* Disable for Outport: '<Root>/id' incorporates:
     *  Outport: '<S48>/id'
     */
    rtY->id = 0;

    /* Disable for SignalConversion generated from: '<S48>/iqAbs' incorporates:
     *  Outport: '<S48>/iqAbs'
     */
    rtDW->OutportBufferForiqAbs = 0;
  }

  if (UnitDelay3 == 0) {
    /* Outputs for IfAction SubSystem: '<S7>/Clarke_Park_Transform_Forward' incorporates:
     *  ActionPort: '<S48>/Action Port'
     */
    /* If: '<S52>/If1' incorporates:
     *  Constant: '<S52>/z_selPhaCurMeasABC'
     */
    if (rtP->z_selPhaCurMeasABC == 0) {
      /* Outputs for IfAction SubSystem: '<S52>/Clarke_PhasesAB' incorporates:
       *  ActionPort: '<S56>/Action Port'
       */
      /* Sum: '<S56>/Sum1' incorporates:
       *  Gain: '<S56>/Gain2'
       *  Gain: '<S56>/Gain4'
       *  Merge: '<S52>/Merge1'
       *  Saturate: '<S1>/Saturation'
       *  Saturate: '<S1>/Saturation1'
       */
      rtDW->Switch1_m = ((18919 * rtb_Saturation) >> 15) + (int16_T)((18919 *
        rtb_Saturation1) >> 14);
      if (rtDW->Switch1_m > 32767) {
        rtDW->Switch1_m = 32767;
      } else if (rtDW->Switch1_m < -32768) {
        rtDW->Switch1_m = -32768;
      }

      rtb_Merge1 = (int16_T)rtDW->Switch1_m;

      /* End of Sum: '<S56>/Sum1' */
      /* End of Outputs for SubSystem: '<S52>/Clarke_PhasesAB' */
    } else if (rtP->z_selPhaCurMeasABC == 1) {
      /* Outputs for IfAction SubSystem: '<S52>/Clarke_PhasesBC' incorporates:
       *  ActionPort: '<S58>/Action Port'
       */
      /* Sum: '<S58>/Sum3' incorporates:
       *  Saturate: '<S1>/Saturation'
       *  Saturate: '<S1>/Saturation1'
       */
      rtDW->Switch1_m = rtb_Saturation - rtb_Saturation1;
      if (rtDW->Switch1_m > 32767) {
        rtDW->Switch1_m = 32767;
      } else if (rtDW->Switch1_m < -32768) {
        rtDW->Switch1_m = -32768;
      }

      /* Gain: '<S58>/Gain2' incorporates:
       *  Merge: '<S52>/Merge1'
       *  Sum: '<S58>/Sum3'
       */
      rtDW->Switch1_m *= 18919;
      rtb_Merge1 = (int16_T)(((rtDW->Switch1_m < 0 ? 32767 : 0) +
        rtDW->Switch1_m) >> 15);

      /* Sum: '<S58>/Sum1' incorporates:
       *  Merge: '<S52>/Merge2'
       *  Saturate: '<S1>/Saturation'
       *  Saturate: '<S1>/Saturation1'
       */
      rtDW->Switch1_m = -rtb_Saturation - rtb_Saturation1;
      if (rtDW->Switch1_m > 32767) {
        rtDW->Switch1_m = 32767;
      } else if (rtDW->Switch1_m < -32768) {
        rtDW->Switch1_m = -32768;
      }

      rtb_Saturation = (int16_T)rtDW->Switch1_m;

      /* End of Sum: '<S58>/Sum1' */
      /* End of Outputs for SubSystem: '<S52>/Clarke_PhasesBC' */
    } else {
      /* Outputs for IfAction SubSystem: '<S52>/Clarke_PhasesAC' incorporates:
       *  ActionPort: '<S57>/Action Port'
       */
      /* Gain: '<S57>/Gain4' incorporates:
       *  Saturate: '<S1>/Saturation'
       */
      rtDW->Switch1_m = 18919 * rtb_Saturation;

      /* Gain: '<S57>/Gain2' incorporates:
       *  Saturate: '<S1>/Saturation1'
       */
      rtDW->Sum1 = 18919 * rtb_Saturation1;

      /* Sum: '<S57>/Sum1' incorporates:
       *  Gain: '<S57>/Gain2'
       *  Gain: '<S57>/Gain4'
       *  Merge: '<S52>/Merge1'
       */
      rtDW->Switch1_m = -(((rtDW->Switch1_m < 0 ? 32767 : 0) + rtDW->Switch1_m) >>
                          15) - (int16_T)(((rtDW->Sum1 < 0 ? 16383 : 0) +
        rtDW->Sum1) >> 14);
      if (rtDW->Switch1_m > 32767) {
        rtDW->Switch1_m = 32767;
      } else if (rtDW->Switch1_m < -32768) {
        rtDW->Switch1_m = -32768;
      }

      rtb_Merge1 = (int16_T)rtDW->Switch1_m;

      /* End of Sum: '<S57>/Sum1' */
      /* End of Outputs for SubSystem: '<S52>/Clarke_PhasesAC' */
    }

    /* End of If: '<S52>/If1' */

    /* PreLookup generated from: '<S55>/a_elecAngle_XA' incorporates:
     *  Merge: '<S3>/Merge'
     */
    rtb_k_o = plook_u8s16u16n15_even7ca_gs(rtb_Merge, 0, &rtb_f);

    /* Interpolation_n-D generated from: '<S55>/r_sin_M1' incorporates:
     *  PreLookup generated from: '<S55>/a_elecAngle_XA'
     */
    rtDW->r_sin_M1_1 = intrp1d_s16s32s32u8u16n15la_s(rtb_k_o, rtb_f,
      rtConstP.r_sin_M1_1_Table, 180U);

    /* Interpolation_n-D generated from: '<S55>/r_cos_M1' incorporates:
     *  PreLookup generated from: '<S55>/a_elecAngle_XA'
     */
    rtDW->r_cos_M1_1 = intrp1d_s16s32s32u8u16n15la_s(rtb_k_o, rtb_f,
      rtConstP.r_cos_M1_1_Table, 180U);

    /* If: '<S48>/If2' incorporates:
     *  Constant: '<S53>/cf_currFilt'
     */
    rtb_Sum2_ii = rtDW->If2_ActiveSubsystem_a;
    UnitDelay3 = -1;
    if (rtU->b_motEna) {
      UnitDelay3 = 0;
    }

    rtDW->If2_ActiveSubsystem_a = UnitDelay3;
    if ((rtb_Sum2_ii != UnitDelay3) && (rtb_Sum2_ii == 0)) {
      /* Disable for Outport: '<S53>/iq' incorporates:
       *  DataTypeConversion: '<S59>/Data Type Conversion'
       * */
      rtDW->DataTypeConversion[0] = 0;

      /* Disable for Abs: '<S53>/Abs5' incorporates:
       *  Outport: '<S53>/iqAbs'
       */
      rtDW->Abs5_d = 0;

      /* Disable for SignalConversion generated from: '<S53>/id' incorporates:
       *  Outport: '<S53>/id'
       */
      rtDW->OutportBufferForid_f = 0;
    }

    if (UnitDelay3 == 0) {
      if (rtb_Sum2_ii != 0) {
        /* SystemReset for IfAction SubSystem: '<S48>/Current_Filtering' incorporates:
         *  ActionPort: '<S53>/Action Port'
         */
        /* SystemReset for Atomic SubSystem: '<S53>/Low_Pass_Filter' */
        /* SystemReset for If: '<S48>/If2' */
        Low_Pass_Filter_Reset(&rtDW->Low_Pass_Filter_e);

        /* End of SystemReset for SubSystem: '<S53>/Low_Pass_Filter' */
        /* End of SystemReset for SubSystem: '<S48>/Current_Filtering' */
      }

      /* Sum: '<S54>/Sum6' incorporates:
       *  Interpolation_n-D generated from: '<S55>/r_cos_M1'
       *  Interpolation_n-D generated from: '<S55>/r_sin_M1'
       *  Merge: '<S52>/Merge1'
       *  Merge: '<S52>/Merge2'
       *  Product: '<S54>/Divide1'
       *  Product: '<S54>/Divide4'
       */
      rtDW->Switch1_m = (int16_T)((rtb_Merge1 * rtDW->r_cos_M1_1) >> 14) -
        (int16_T)((rtb_Saturation * rtDW->r_sin_M1_1) >> 14);
      if (rtDW->Switch1_m > 32767) {
        rtDW->Switch1_m = 32767;
      } else if (rtDW->Switch1_m < -32768) {
        rtDW->Switch1_m = -32768;
      }

      /* Outputs for IfAction SubSystem: '<S48>/Current_Filtering' incorporates:
       *  ActionPort: '<S53>/Action Port'
       */
      /* SignalConversion generated from: '<S53>/Low_Pass_Filter' incorporates:
       *  Sum: '<S54>/Sum6'
       */
      rtDW->TmpSignalConversionAtLow_Pa[0] = (int16_T)rtDW->Switch1_m;

      /* End of Outputs for SubSystem: '<S48>/Current_Filtering' */

      /* Sum: '<S54>/Sum1' incorporates:
       *  Interpolation_n-D generated from: '<S55>/r_cos_M1'
       *  Interpolation_n-D generated from: '<S55>/r_sin_M1'
       *  Merge: '<S52>/Merge1'
       *  Merge: '<S52>/Merge2'
       *  Product: '<S54>/Divide2'
       *  Product: '<S54>/Divide3'
       */
      rtDW->Switch1_m = (int16_T)((rtb_Saturation * rtDW->r_cos_M1_1) >> 14) +
        (int16_T)((rtb_Merge1 * rtDW->r_sin_M1_1) >> 14);
      if (rtDW->Switch1_m > 32767) {
        rtDW->Switch1_m = 32767;
      } else if (rtDW->Switch1_m < -32768) {
        rtDW->Switch1_m = -32768;
      }

      /* Outputs for IfAction SubSystem: '<S48>/Current_Filtering' incorporates:
       *  ActionPort: '<S53>/Action Port'
       */
      /* SignalConversion generated from: '<S53>/Low_Pass_Filter' incorporates:
       *  Sum: '<S54>/Sum1'
       */
      rtDW->TmpSignalConversionAtLow_Pa[1] = (int16_T)rtDW->Switch1_m;

      /* Outputs for Atomic SubSystem: '<S53>/Low_Pass_Filter' */
      Low_Pass_Filter(rtDW->TmpSignalConversionAtLow_Pa, rtP->cf_currFilt,
                      rtDW->DataTypeConversion, &rtDW->Low_Pass_Filter_e);

      /* End of Outputs for SubSystem: '<S53>/Low_Pass_Filter' */

      /* Abs: '<S53>/Abs5' incorporates:
       *  Constant: '<S53>/cf_currFilt'
       */
      if (rtDW->DataTypeConversion[0] < 0) {
        /* Abs: '<S53>/Abs5' */
        rtDW->Abs5_d = (int16_T)-rtDW->DataTypeConversion[0];
      } else {
        /* Abs: '<S53>/Abs5' */
        rtDW->Abs5_d = rtDW->DataTypeConversion[0];
      }

      /* End of Abs: '<S53>/Abs5' */

      /* SignalConversion generated from: '<S53>/id' */
      rtDW->OutportBufferForid_f = rtDW->DataTypeConversion[1];

      /* End of Outputs for SubSystem: '<S48>/Current_Filtering' */
    }

    /* End of If: '<S48>/If2' */

    /* Outport: '<Root>/id' incorporates:
     *  SignalConversion generated from: '<S48>/id'
     *  SignalConversion generated from: '<S53>/id'
     */
    rtY->id = rtDW->OutportBufferForid_f;

    /* Outport: '<Root>/iq' incorporates:
     *  SignalConversion generated from: '<S48>/iq'
     */
    rtY->iq = rtDW->DataTypeConversion[0];

    /* SignalConversion generated from: '<S48>/iqAbs' incorporates:
     *  Abs: '<S53>/Abs5'
     */
    rtDW->OutportBufferForiqAbs = rtDW->Abs5_d;

    /* End of Outputs for SubSystem: '<S7>/Clarke_Park_Transform_Forward' */
  }

  /* End of If: '<S7>/If1' */

  /* Chart: '<S1>/Task_Scheduler' incorporates:
   *  UnitDelay: '<S2>/UnitDelay2'
   *  UnitDelay: '<S2>/UnitDelay5'
   *  UnitDelay: '<S2>/UnitDelay6'
   */
  if (rtDW->UnitDelay2_DSTATE_g) {
    /* Outputs for Function Call SubSystem: '<S1>/F02_Diagnostics' */
    /* If: '<S4>/If2' incorporates:
     *  Constant: '<S20>/CTRL_COMM2'
     *  Constant: '<S20>/t_errDequal'
     *  Constant: '<S20>/t_errQual'
     *  Constant: '<S4>/b_diagEna'
     *  RelationalOperator: '<S20>/Relational Operator2'
     */
    if (rtP->b_diagEna) {
      /* Outputs for IfAction SubSystem: '<S4>/Diagnostics_Enabled' incorporates:
       *  ActionPort: '<S20>/Action Port'
       */
      /* Switch: '<S20>/Switch3' incorporates:
       *  Abs: '<S13>/Abs5'
       *  Abs: '<S20>/Abs4'
       *  Constant: '<S13>/n_stdStillDet'
       *  Constant: '<S20>/CTRL_COMM4'
       *  Constant: '<S20>/r_errInpTgtThres'
       *  Logic: '<S20>/Logical Operator1'
       *  RelationalOperator: '<S13>/Relational Operator9'
       *  RelationalOperator: '<S20>/Relational Operator7'
       *  S-Function (sfix_bitop): '<S20>/Bitwise Operator1'
       *  UnitDelay: '<S20>/UnitDelay'
       *  UnitDelay: '<S8>/UnitDelay4'
       */
      if ((rtDW->Switch1 & 4U) != 0U) {
        rtb_RelationalOperator = true;
      } else {
        if (rtDW->UnitDelay4_DSTATE_a < 0) {
          /* Abs: '<S20>/Abs4' incorporates:
           *  UnitDelay: '<S8>/UnitDelay4'
           */
          rtb_Merge1 = (int16_T)-rtDW->UnitDelay4_DSTATE_a;
        } else {
          /* Abs: '<S20>/Abs4' incorporates:
           *  UnitDelay: '<S8>/UnitDelay4'
           */
          rtb_Merge1 = rtDW->UnitDelay4_DSTATE_a;
        }

        rtb_RelationalOperator = (rtU->b_motEna && (Abs5 < rtP->n_stdStillDet) &&
          (rtb_Merge1 > rtP->r_errInpTgtThres));
      }

      /* End of Switch: '<S20>/Switch3' */

      /* Sum: '<S20>/Sum' incorporates:
       *  Constant: '<S20>/CTRL_COMM'
       *  Constant: '<S20>/CTRL_COMM1'
       *  DataTypeConversion: '<S20>/Data Type Conversion3'
       *  Gain: '<S20>/g_Hb'
       *  Gain: '<S20>/g_Hb1'
       *  RelationalOperator: '<S20>/Relational Operator1'
       *  RelationalOperator: '<S20>/Relational Operator3'
       */
      rtb_f_j = (uint8_T)(((uint32_T)((Sum == 7) << 1) + (uint32_T)(Sum == 0)) +
                          (uint32_T)(rtb_RelationalOperator << 2));

      /* Outputs for Atomic SubSystem: '<S20>/Debounce_Filter' */
      Debounce_Filter((rtb_f_j != 0), rtP->t_errQual, rtP->t_errDequal,
                      &rtDW->Merge_a, &rtDW->Debounce_Filter_e);

      /* End of Outputs for SubSystem: '<S20>/Debounce_Filter' */

      /* Outputs for Atomic SubSystem: '<S20>/either_edge' */
      rtb_RelationalOperator = either_edge(rtDW->Merge_a, &rtDW->either_edge_j);

      /* End of Outputs for SubSystem: '<S20>/either_edge' */

      /* Switch: '<S20>/Switch1' incorporates:
       *  Constant: '<S20>/CTRL_COMM2'
       *  Constant: '<S20>/t_errDequal'
       *  Constant: '<S20>/t_errQual'
       *  RelationalOperator: '<S20>/Relational Operator2'
       */
      if (rtb_RelationalOperator) {
        /* Switch: '<S20>/Switch1' */
        rtDW->Switch1 = rtb_f_j;
      }

      /* End of Switch: '<S20>/Switch1' */
      /* End of Outputs for SubSystem: '<S4>/Diagnostics_Enabled' */
    }

    /* End of If: '<S4>/If2' */

    /* Outport: '<Root>/z_errCode' incorporates:
     *  SignalConversion generated from: '<S4>/z_errCode '
     */
    rtY->z_errCode = rtDW->Switch1;

    /* Outputs for Function Call SubSystem: '<S1>/F03_Control_Mode_Manager' */
    /* Logic: '<S31>/Logical Operator4' incorporates:
     *  Constant: '<S31>/constant8'
     *  Logic: '<S31>/Logical Operator7'
     *  RelationalOperator: '<S31>/Relational Operator10'
     *  SignalConversion generated from: '<S4>/b_errFlag'
     */
    rtb_RelationalOperator = (rtDW->Merge_a || (!rtU->b_motEna) ||
      (rtU->z_ctrlModReq == 0));

    /* End of Outputs for SubSystem: '<S1>/F02_Diagnostics' */

    /* Logic: '<S31>/Logical Operator1' incorporates:
     *  Constant: '<S1>/b_cruiseCtrlEna'
     *  Constant: '<S31>/constant1'
     *  RelationalOperator: '<S31>/Relational Operator1'
     */
    LogicalOperator1 = ((rtU->z_ctrlModReq == 2) || rtP->b_cruiseCtrlEna);

    /* Logic: '<S31>/Logical Operator2' incorporates:
     *  Constant: '<S1>/b_cruiseCtrlEna'
     *  Constant: '<S31>/constant'
     *  Logic: '<S31>/Logical Operator5'
     *  RelationalOperator: '<S31>/Relational Operator4'
     */
    LogicalOperator2 = ((rtU->z_ctrlModReq == 3) && (!rtP->b_cruiseCtrlEna));

    /* Chart: '<S5>/F03_02_Control_Mode_Manager' incorporates:
     *  Constant: '<S31>/constant5'
     *  Logic: '<S31>/Logical Operator3'
     *  Logic: '<S31>/Logical Operator6'
     *  Logic: '<S31>/Logical Operator9'
     *  RelationalOperator: '<S31>/Relational Operator5'
     */
    if (rtDW->is_active_c2_BLDC_controller == 0) {
      rtDW->is_active_c2_BLDC_controller = 1U;
      rtDW->is_c2_BLDC_controller = IN_OPEN;
      rtDW->z_ctrlMod = OPEN_MODE;
    } else if (rtDW->is_c2_BLDC_controller == IN_ACTIVE) {
      if (rtb_RelationalOperator) {
        rtDW->is_ACTIVE = IN_NO_ACTIVE_CHILD;
        rtDW->is_c2_BLDC_controller = IN_OPEN;
        rtDW->z_ctrlMod = OPEN_MODE;
      } else {
        switch (rtDW->is_ACTIVE) {
         case IN_SPEED_MODE:
          rtDW->z_ctrlMod = SPD_MODE;
          if (!LogicalOperator1) {
            enter_internal_ACTIVE(&LogicalOperator1, &LogicalOperator2, rtDW);
          }
          break;

         case IN_TORQUE_MODE:
          rtDW->z_ctrlMod = TRQ_MODE;
          if (!LogicalOperator2) {
            enter_internal_ACTIVE(&LogicalOperator1, &LogicalOperator2, rtDW);
          }
          break;

         default:
          /* case IN_VOLTAGE_MODE: */
          rtDW->z_ctrlMod = VLT_MODE;
          if (LogicalOperator2 || LogicalOperator1) {
            enter_internal_ACTIVE(&LogicalOperator1, &LogicalOperator2, rtDW);
          }
          break;
        }
      }
    } else {
      /* case IN_OPEN: */
      rtDW->z_ctrlMod = OPEN_MODE;
      if ((!rtb_RelationalOperator) && ((rtU->z_ctrlModReq == 1) ||
           LogicalOperator1 || LogicalOperator2)) {
        rtDW->is_c2_BLDC_controller = IN_ACTIVE;
        enter_internal_ACTIVE(&LogicalOperator1, &LogicalOperator2, rtDW);
      }
    }

    /* End of Chart: '<S5>/F03_02_Control_Mode_Manager' */

    /* Saturate: '<S33>/Saturation' */
    if (rtU->r_inpTgt > 16000) {
      rtb_Saturation1 = 16000;
    } else if (rtU->r_inpTgt < -16000) {
      rtb_Saturation1 = -16000;
    } else {
      rtb_Saturation1 = rtU->r_inpTgt;
    }

    /* End of Saturate: '<S33>/Saturation' */

    /* If: '<S33>/If1' incorporates:
     *  Constant: '<S1>/z_ctrlTypSel'
     *  Constant: '<S39>/Vd_max1'
     *  Constant: '<S40>/Vd_max3'
     *  VariantMerge generated from: '<S38>/Vd_max_margin'
     */
    if (rtP->z_ctrlTypSel == 2) {
      /* Outputs for IfAction SubSystem: '<S33>/FOC_Control_Type' incorporates:
       *  ActionPort: '<S36>/Action Port'
       */
      /* Outputs for Atomic SubSystem: '<S36>/Variant Subsystem1' */
#if mcu_model == 1

      /* Outputs for Atomic SubSystem: '<S38>/GD32F103' */
      rtb_MinMax2 = 24592;

      /* End of Outputs for SubSystem: '<S38>/GD32F103' */
#elif mcu_model == 0

      /* Outputs for Atomic SubSystem: '<S38>/STM32F103' */
      rtb_MinMax2 = 13600;

      /* End of Outputs for SubSystem: '<S38>/STM32F103' */
#endif

      /* End of Outputs for SubSystem: '<S36>/Variant Subsystem1' */

      /* SignalConversion generated from: '<S36>/Selector' incorporates:
       *  Constant: '<S36>/constant1'
       *  Constant: '<S36>/i_max'
       *  Constant: '<S36>/n_max'
       *  Constant: '<S39>/Vd_max1'
       *  Constant: '<S40>/Vd_max3'
       *  VariantMerge generated from: '<S38>/Vd_max_margin'
       */
      rtDW->iv[0] = 0;
      rtDW->iv[1] = rtb_MinMax2;
      rtDW->iv[2] = rtP->n_max;
      rtDW->iv[3] = rtP->i_max;

      /* Product: '<S36>/Divide1' incorporates:
       *  Merge: '<S33>/Merge'
       *  Product: '<S36>/Divide4'
       *  Saturate: '<S33>/Saturation'
       *  Selector: '<S36>/Selector'
       */
      rtb_Saturation1 = (int16_T)(((uint16_T)((rtDW->iv[rtU->z_ctrlModReq] << 5)
        / 125) * rtb_Saturation1) >> 12);

      /* End of Outputs for SubSystem: '<S33>/FOC_Control_Type' */
    }

    /* End of If: '<S33>/If1' */

    /* If: '<S33>/If2' */
    rtb_Sum2_ii = rtDW->If2_ActiveSubsystem_k;
    UnitDelay3 = (int8_T)(rtDW->z_ctrlMod != 0);
    rtDW->If2_ActiveSubsystem_k = UnitDelay3;
    if (UnitDelay3 == 0) {
      if (rtb_Sum2_ii != 0) {
        /* SystemReset for IfAction SubSystem: '<S33>/Open_Mode' incorporates:
         *  ActionPort: '<S37>/Action Port'
         */
        /* SystemReset for Atomic SubSystem: '<S37>/rising_edge_init' */
        /* SystemReset for If: '<S33>/If2' incorporates:
         *  UnitDelay: '<S42>/UnitDelay'
         *  UnitDelay: '<S43>/UnitDelay'
         */
        rtDW->UnitDelay_DSTATE_c = true;

        /* End of SystemReset for SubSystem: '<S37>/rising_edge_init' */

        /* SystemReset for Atomic SubSystem: '<S37>/Rate_Limiter' */
        rtDW->UnitDelay_DSTATE = 0;

        /* End of SystemReset for SubSystem: '<S37>/Rate_Limiter' */
        /* End of SystemReset for SubSystem: '<S33>/Open_Mode' */
      }

      /* Outputs for IfAction SubSystem: '<S33>/Open_Mode' incorporates:
       *  ActionPort: '<S37>/Action Port'
       */
      /* DataTypeConversion: '<S37>/Data Type Conversion' incorporates:
       *  UnitDelay: '<S8>/UnitDelay4'
       */
      rtDW->Sum1 = rtDW->UnitDelay4_DSTATE_a << 12;

      /* Outputs for Atomic SubSystem: '<S37>/rising_edge_init' */
      /* UnitDelay: '<S42>/UnitDelay' */
      rtb_RelationalOperator = rtDW->UnitDelay_DSTATE_c;

      /* Update for UnitDelay: '<S42>/UnitDelay' incorporates:
       *  Constant: '<S42>/Constant'
       */
      rtDW->UnitDelay_DSTATE_c = false;

      /* End of Outputs for SubSystem: '<S37>/rising_edge_init' */

      /* Outputs for Atomic SubSystem: '<S37>/Rate_Limiter' */
      /* Switch: '<S43>/Switch1' incorporates:
       *  DataTypeConversion: '<S37>/Data Type Conversion'
       *  UnitDelay: '<S43>/UnitDelay'
       */
      if (rtb_RelationalOperator) {
        rtDW->Switch1_m = rtDW->Sum1;
      } else {
        rtDW->Switch1_m = rtDW->UnitDelay_DSTATE;
      }

      /* End of Switch: '<S43>/Switch1' */

      /* Sum: '<S41>/Sum1' incorporates:
       *  Switch: '<S43>/Switch1'
       */
      rtb_Sum1_h = ((uint32_T)-rtDW->Switch1_m & 134217728U) != 0U ?
        -rtDW->Switch1_m | -134217728 : (int32_T)((uint32_T)-rtDW->Switch1_m &
        134217727U);

      /* Switch: '<S44>/Switch2' incorporates:
       *  Constant: '<S37>/dV_openRate'
       *  RelationalOperator: '<S44>/LowerRelop1'
       *  Sum: '<S41>/Sum1'
       */
      if (rtb_Sum1_h > rtP->dV_openRate) {
        rtb_Sum1_h = rtP->dV_openRate;
      } else {
        /* Gain: '<S37>/Gain3' */
        rtb_Gain3 = ((uint32_T)-rtP->dV_openRate & 134217728U) != 0U ?
          -rtP->dV_openRate | -134217728 : (int32_T)((uint32_T)-rtP->dV_openRate &
          134217727U);

        /* Switch: '<S44>/Switch' incorporates:
         *  Gain: '<S37>/Gain3'
         *  RelationalOperator: '<S44>/UpperRelop'
         *  Switch: '<S44>/Switch2'
         */
        if (rtb_Sum1_h < rtb_Gain3) {
          rtb_Sum1_h = rtb_Gain3;
        }

        /* End of Switch: '<S44>/Switch' */
      }

      /* End of Switch: '<S44>/Switch2' */

      /* Sum: '<S41>/Sum2' incorporates:
       *  Switch: '<S43>/Switch1'
       *  Switch: '<S44>/Switch2'
       */
      rtDW->Switch1_m += rtb_Sum1_h;
      rtDW->Switch1_m = ((uint32_T)rtDW->Switch1_m & 134217728U) != 0U ?
        rtDW->Switch1_m | -134217728 : (int32_T)((uint32_T)rtDW->Switch1_m &
        134217727U);

      /* Switch: '<S43>/Switch2' */
      if (rtb_RelationalOperator) {
        /* Update for UnitDelay: '<S43>/UnitDelay' incorporates:
         *  DataTypeConversion: '<S37>/Data Type Conversion'
         */
        rtDW->UnitDelay_DSTATE = rtDW->Sum1;
      } else {
        /* Update for UnitDelay: '<S43>/UnitDelay' incorporates:
         *  Sum: '<S41>/Sum2'
         */
        rtDW->UnitDelay_DSTATE = rtDW->Switch1_m;
      }

      /* End of Switch: '<S43>/Switch2' */
      /* End of Outputs for SubSystem: '<S37>/Rate_Limiter' */

      /* Merge: '<S33>/Merge1' incorporates:
       *  DataTypeConversion: '<S37>/Data Type Conversion1'
       *  Sum: '<S41>/Sum2'
       */
      rtDW->Merge1 = (int16_T)(rtDW->Switch1_m >> 12);

      /* End of Outputs for SubSystem: '<S33>/Open_Mode' */
    } else {
      /* Outputs for IfAction SubSystem: '<S33>/Default_Mode' incorporates:
       *  ActionPort: '<S35>/Action Port'
       */
      /* Merge: '<S33>/Merge1' incorporates:
       *  Merge: '<S33>/Merge'
       *  SignalConversion generated from: '<S35>/r_inpTgtScaRaw'
       */
      rtDW->Merge1 = rtb_Saturation1;

      /* End of Outputs for SubSystem: '<S33>/Default_Mode' */
    }

    /* End of If: '<S33>/If2' */

    /* Abs: '<S5>/Abs1' incorporates:
     *  Merge: '<S33>/Merge1'
     */
    if (rtDW->Merge1 < 0) {
      /* Abs: '<S5>/Abs1' */
      rtDW->Abs1 = (int16_T)-rtDW->Merge1;
    } else {
      /* Abs: '<S5>/Abs1' */
      rtDW->Abs1 = rtDW->Merge1;
    }

    /* End of Abs: '<S5>/Abs1' */
    /* End of Outputs for SubSystem: '<S1>/F03_Control_Mode_Manager' */
  } else if (rtDW->UnitDelay5_DSTATE_l) {
    /* Outputs for Function Call SubSystem: '<S1>/F04_Field_Weakening' */
    /* If: '<S6>/If3' incorporates:
     *  Constant: '<S6>/b_fieldWeakEna'
     */
    if (rtP->b_fieldWeakEna) {
      /* Outputs for IfAction SubSystem: '<S6>/Field_Weakening_Enabled' incorporates:
       *  ActionPort: '<S45>/Action Port'
       */
      /* Abs: '<S45>/Abs5' */
      if (rtU->r_inpTgt < 0) {
        rtb_MinMax2 = (int16_T)-rtU->r_inpTgt;
      } else {
        rtb_MinMax2 = rtU->r_inpTgt;
      }

      /* End of Abs: '<S45>/Abs5' */

      /* Switch: '<S47>/Switch2' incorporates:
       *  Abs: '<S45>/Abs5'
       *  Constant: '<S45>/r_fieldWeakHi'
       *  Constant: '<S45>/r_fieldWeakLo'
       *  RelationalOperator: '<S47>/LowerRelop1'
       *  RelationalOperator: '<S47>/UpperRelop'
       *  Sum: '<S45>/Sum4'
       *  Switch: '<S47>/Switch'
       */
      if (rtb_MinMax2 > rtP->r_fieldWeakHi) {
        rtb_MinMax2 = rtP->r_fieldWeakHi;
      } else if (rtb_MinMax2 < rtP->r_fieldWeakLo) {
        /* Switch: '<S47>/Switch' incorporates:
         *  Constant: '<S45>/r_fieldWeakLo'
         *  Sum: '<S45>/Sum4'
         */
        rtb_MinMax2 = rtP->r_fieldWeakLo;
      }

      /* End of Switch: '<S47>/Switch2' */

      /* Product: '<S45>/Divide14' incorporates:
       *  Constant: '<S45>/r_fieldWeakHi'
       *  Constant: '<S45>/r_fieldWeakLo'
       *  Sum: '<S45>/Sum1'
       *  Sum: '<S45>/Sum3'
       *  Sum: '<S45>/Sum4'
       */
      rtb_f = (uint16_T)(((int16_T)(rtb_MinMax2 - rtP->r_fieldWeakLo) << 15) /
                         (int16_T)(rtP->r_fieldWeakHi - rtP->r_fieldWeakLo));

      /* Switch: '<S46>/Switch2' incorporates:
       *  Abs: '<S13>/Abs5'
       *  Constant: '<S45>/n_fieldWeakAuthHi'
       *  Constant: '<S45>/n_fieldWeakAuthLo'
       *  RelationalOperator: '<S46>/LowerRelop1'
       *  RelationalOperator: '<S46>/UpperRelop'
       *  Switch: '<S46>/Switch'
       */
      if (Abs5 > rtP->n_fieldWeakAuthHi) {
        rtb_Merge1 = rtP->n_fieldWeakAuthHi;
      } else if (Abs5 < rtP->n_fieldWeakAuthLo) {
        /* Switch: '<S46>/Switch' incorporates:
         *  Constant: '<S45>/n_fieldWeakAuthLo'
         */
        rtb_Merge1 = rtP->n_fieldWeakAuthLo;
      } else {
        rtb_Merge1 = Abs5;
      }

      /* Product: '<S45>/Divide1' incorporates:
       *  Constant: '<S45>/n_fieldWeakAuthHi'
       *  Constant: '<S45>/n_fieldWeakAuthLo'
       *  Sum: '<S45>/Sum2'
       *  Sum: '<S45>/Sum4'
       *  Switch: '<S46>/Switch2'
       */
      rtb_Divide1 = (uint16_T)(((int16_T)(rtb_Merge1 - rtP->n_fieldWeakAuthLo) <<
        15) / (int16_T)(rtP->n_fieldWeakAuthHi - rtP->n_fieldWeakAuthLo));

      /* Switch: '<S45>/Switch1' incorporates:
       *  MinMax: '<S45>/MinMax1'
       *  Product: '<S45>/Divide1'
       *  Product: '<S45>/Divide14'
       *  RelationalOperator: '<S45>/Relational Operator6'
       */
      if (rtb_f < rtb_Divide1) {
        /* MinMax: '<S45>/MinMax' incorporates:
         *  Switch: '<S45>/Switch1'
         */
        if (rtb_f < rtb_Divide1) {
          rtb_f = rtb_Divide1;
        }

        /* End of MinMax: '<S45>/MinMax' */
      } else if (rtb_Divide1 <= rtb_f) {
        /* MinMax: '<S45>/MinMax1' incorporates:
         *  Switch: '<S45>/Switch1'
         */
        rtb_f = rtb_Divide1;
      }

      /* End of Switch: '<S45>/Switch1' */

      /* Switch: '<S45>/Switch2' incorporates:
       *  Constant: '<S1>/z_ctrlTypSel'
       *  Constant: '<S45>/CTRL_COMM2'
       *  Constant: '<S45>/a_phaAdvMax'
       *  Constant: '<S45>/id_fieldWeakMax'
       *  RelationalOperator: '<S45>/Relational Operator1'
       */
      if (rtP->z_ctrlTypSel == 2) {
        rtb_Merge1 = rtP->id_fieldWeakMax;
      } else {
        rtb_Merge1 = rtP->a_phaAdvMax;
      }

      /* Product: '<S45>/Divide3' incorporates:
       *  Switch: '<S45>/Switch1'
       *  Switch: '<S45>/Switch2'
       */
      rtDW->Divide3 = (int16_T)((rtb_Merge1 * rtb_f) >> 15);

      /* End of Outputs for SubSystem: '<S6>/Field_Weakening_Enabled' */
    }

    /* End of If: '<S6>/If3' */

    /* SignalConversion generated from: '<S6>/r_fieldWeak ' incorporates:
     *  Product: '<S45>/Divide3'
     */
    rtDW->OutportBufferForr_fieldWeak = rtDW->Divide3;

    /* End of Outputs for SubSystem: '<S1>/F04_Field_Weakening' */

    /* Outputs for Function Call SubSystem: '<S7>/Motor_Limitations' */
    /* If: '<S51>/If1' incorporates:
     *  Constant: '<S1>/z_ctrlTypSel'
     */
    rtb_Sum2_ii = rtDW->If1_ActiveSubsystem_b;
    UnitDelay3 = -1;
    if (rtP->z_ctrlTypSel == 2) {
      UnitDelay3 = 0;
    }

    rtDW->If1_ActiveSubsystem_b = UnitDelay3;
    if ((rtb_Sum2_ii != UnitDelay3) && (rtb_Sum2_ii == 0)) {
      /* Disable for SwitchCase: '<S189>/Switch Case' */
      rtDW->SwitchCase_ActiveSubsystem_o = -1;
    }

    if (UnitDelay3 == 0) {
      /* Outputs for IfAction SubSystem: '<S51>/Motor_Limitations_Enabled' incorporates:
       *  ActionPort: '<S189>/Action Port'
       */
      /* Product: '<S189>/Divide4' incorporates:
       *  Constant: '<S189>/i_max'
       *  SignalConversion generated from: '<S6>/r_fieldWeak '
       */
      rtDW->Switch1_m = rtDW->OutportBufferForr_fieldWeak << 16;
      rtDW->Switch1_m = (rtDW->Switch1_m == MIN_int32_T) && (rtP->i_max == -1) ?
        MAX_int32_T : rtDW->Switch1_m / rtP->i_max;
      if (rtDW->Switch1_m < 0) {
        rtDW->Switch1_m = 0;
      } else if (rtDW->Switch1_m > 65535) {
        rtDW->Switch1_m = 65535;
      }

      /* Product: '<S189>/Divide1' incorporates:
       *  Constant: '<S189>/i_max'
       *  Interpolation_n-D: '<S189>/iq_maxSca_M1'
       *  PreLookup: '<S189>/iq_maxSca_XA'
       *  Product: '<S189>/Divide4'
       */
      rtDW->Divide1_d = (int16_T)
        ((rtConstP.iq_maxSca_M1_Table[plook_u8u16_evenckag((uint16_T)
           rtDW->Switch1_m, 0U, 1311U)] * rtP->i_max) >> 16);

      /* Gain: '<S189>/Gain1' incorporates:
       *  Product: '<S189>/Divide1'
       */
      rtDW->Gain1 = (int16_T)-rtDW->Divide1_d;

      /* Outputs for Atomic SubSystem: '<S189>/Variant Subsystem' */
#if mcu_model == 1

      /* Outputs for Atomic SubSystem: '<S192>/GD32F103' */
      /* Abs: '<S198>/Abs5' incorporates:
       *  UnitDelay: '<S7>/UnitDelay4'
       */
      if (rtDW->Switch2_g < 0) {
        rtb_Merge1 = (int16_T)-rtDW->Switch2_g;
      } else {
        rtb_Merge1 = rtDW->Switch2_g;
      }

      /* PreLookup generated from: '<S198>/GD32_Vq_max_XA' incorporates:
       *  Abs: '<S198>/Abs5'
       */
      rtb_k_o = plook_u8s16u8n6_evenca_gs(rtb_Merge1, 0, 320U, &rtb_f_j);

      /* VariantMerge generated from: '<S192>/Vq_max' incorporates:
       *  Interpolation_n-D generated from: '<S198>/GD32_Vq_max_M1'
       *  PreLookup generated from: '<S198>/GD32_Vq_max_XA'
       */
      rtDW->VariantMergeForOutportVq_max = intrp1d_s16s32s32u8u8n6la_s(rtb_k_o,
        rtb_f_j, rtConstP.GD32_Vq_max_M1_1_Table, 76U);

      /* VariantMerge generated from: '<S192>/Vd_min' incorporates:
       *  Gain: '<S198>/Gain5'
       *  VariantMerge generated from: '<S192>/Vq_max'
       */
      rtDW->VariantMergeForOutportVd_min = (int16_T)
        -rtDW->VariantMergeForOutportVq_max;

      /* VariantMerge generated from: '<S192>/Vd_max' incorporates:
       *  Constant: '<S198>/Vd_max1'
       */
      rtDW->VariantMergeForOutportVd_max = 24592;

      /* VariantMerge generated from: '<S192>/Vq_min' incorporates:
       *  Gain: '<S198>/Gain3'
       */
      rtDW->VariantMergeForOutportVq_min = -24592;

      /* End of Outputs for SubSystem: '<S192>/GD32F103' */
#elif mcu_model == 0

      /* Outputs for Atomic SubSystem: '<S192>/STM32F103' */
      /* Abs: '<S199>/Abs5' incorporates:
       *  UnitDelay: '<S7>/UnitDelay4'
       */
      if (rtDW->Switch2_g < 0) {
        rtb_Merge1 = (int16_T)-rtDW->Switch2_g;
      } else {
        rtb_Merge1 = rtDW->Switch2_g;
      }

      /* PreLookup generated from: '<S199>/STM32_Vq_max_XA' incorporates:
       *  Abs: '<S199>/Abs5'
       */
      rtb_k_o = plook_u8s16u8n6_evenca_gs(rtb_Merge1, 0, 320U, &rtb_f_j);

      /* VariantMerge generated from: '<S192>/Vq_max' incorporates:
       *  Interpolation_n-D generated from: '<S199>/STM32_Vq_max_M1'
       *  PreLookup generated from: '<S199>/STM32_Vq_max_XA'
       */
      rtDW->VariantMergeForOutportVq_max = intrp1d_s16s32s32u8u8n6la_s(rtb_k_o,
        rtb_f_j, rtConstP.STM32_Vq_max_M1_1_Table, 42U);

      /* VariantMerge generated from: '<S192>/Vd_min' incorporates:
       *  Gain: '<S199>/Gain5'
       *  VariantMerge generated from: '<S192>/Vq_max'
       */
      rtDW->VariantMergeForOutportVd_min = (int16_T)
        -rtDW->VariantMergeForOutportVq_max;

      /* VariantMerge generated from: '<S192>/Vd_max' incorporates:
       *  Constant: '<S199>/Vd_max2'
       */
      rtDW->VariantMergeForOutportVd_max = 13600;

      /* VariantMerge generated from: '<S192>/Vq_min' incorporates:
       *  Gain: '<S199>/Gain3'
       */
      rtDW->VariantMergeForOutportVq_min = -13600;

      /* End of Outputs for SubSystem: '<S192>/STM32F103' */
#endif

      /* End of Outputs for SubSystem: '<S189>/Variant Subsystem' */

      /* SwitchCase: '<S189>/Switch Case' incorporates:
       *  Abs: '<S13>/Abs5'
       *  Constant: '<S189>/n_max'
       *  Constant: '<S191>/Constant1'
       *  Constant: '<S191>/cf_KbLimProt'
       *  Constant: '<S191>/cf_nKiLimProt'
       *  Constant: '<S193>/Constant'
       *  Constant: '<S193>/Constant1'
       *  Constant: '<S193>/cf_KbLimProt'
       *  Constant: '<S193>/cf_iqKiLimProt'
       *  Constant: '<S193>/cf_nKiLimProt'
       *  Product: '<S189>/Divide1'
       *  SignalConversion generated from: '<S48>/iqAbs'
       *  Sum: '<S191>/Sum1'
       *  Sum: '<S193>/Sum1'
       *  Sum: '<S193>/Sum2'
       */
      rtb_Sum2_ii = rtDW->SwitchCase_ActiveSubsystem_o;
      UnitDelay3 = -1;
      switch (rtDW->z_ctrlMod) {
       case 1:
        UnitDelay3 = 0;
        break;

       case 2:
        UnitDelay3 = 1;
        break;

       case 3:
        UnitDelay3 = 2;
        break;
      }

      rtDW->SwitchCase_ActiveSubsystem_o = UnitDelay3;
      switch (UnitDelay3) {
       case 0:
        if (UnitDelay3 != rtb_Sum2_ii) {
          /* SystemReset for IfAction SubSystem: '<S189>/Voltage_Mode_Protection' incorporates:
           *  ActionPort: '<S193>/Action Port'
           */
          /* SystemReset for Atomic SubSystem: '<S193>/I_backCalc_fixdt' */
          /* SystemReset for SwitchCase: '<S189>/Switch Case' */
          I_backCalc_fixdt_Reset(65536000, &rtDW->I_backCalc_fixdt_c);

          /* End of SystemReset for SubSystem: '<S193>/I_backCalc_fixdt' */

          /* SystemReset for Atomic SubSystem: '<S193>/I_backCalc_fixdt1' */
          I_backCalc_fixdt_Reset(65536000, &rtDW->I_backCalc_fixdt1);

          /* End of SystemReset for SubSystem: '<S193>/I_backCalc_fixdt1' */
          /* End of SystemReset for SubSystem: '<S189>/Voltage_Mode_Protection' */
        }

        /* Outputs for IfAction SubSystem: '<S189>/Voltage_Mode_Protection' incorporates:
         *  ActionPort: '<S193>/Action Port'
         */
        /* Outputs for Atomic SubSystem: '<S193>/I_backCalc_fixdt' */
        I_backCalc_fixdt((int16_T)(rtDW->Divide1_d - rtDW->OutportBufferForiqAbs),
                         rtP->cf_iqKiLimProt, rtP->cf_KbLimProt, rtDW->Abs1, 0,
                         &rtDW->Switch2_b, &rtDW->I_backCalc_fixdt_c);

        /* End of Outputs for SubSystem: '<S193>/I_backCalc_fixdt' */

        /* Outputs for Atomic SubSystem: '<S193>/I_backCalc_fixdt1' */
        I_backCalc_fixdt((int16_T)(rtP->n_max - Abs5), rtP->cf_nKiLimProt,
                         rtP->cf_KbLimProt, rtDW->Abs1, 0, &rtDW->Switch2_p,
                         &rtDW->I_backCalc_fixdt1);

        /* End of Outputs for SubSystem: '<S193>/I_backCalc_fixdt1' */
        /* End of Outputs for SubSystem: '<S189>/Voltage_Mode_Protection' */
        break;

       case 1:
        /* Outputs for IfAction SubSystem: '<S189>/Speed_Mode_Protection' incorporates:
         *  ActionPort: '<S190>/Action Port'
         */
        /* Switch: '<S194>/Switch2' incorporates:
         *  Gain: '<S189>/Gain1'
         *  Product: '<S189>/Divide1'
         *  RelationalOperator: '<S194>/LowerRelop1'
         *  RelationalOperator: '<S194>/UpperRelop'
         *  Switch: '<S194>/Switch'
         */
        if (rtY->iq > rtDW->Divide1_d) {
          rtb_Merge1 = rtDW->Divide1_d;
        } else if (rtY->iq < rtDW->Gain1) {
          /* Switch: '<S194>/Switch' incorporates:
           *  Gain: '<S189>/Gain1'
           */
          rtb_Merge1 = rtDW->Gain1;
        } else {
          rtb_Merge1 = rtY->iq;
        }

        /* Product: '<S190>/Divide1' incorporates:
         *  Constant: '<S190>/cf_iqKiLimProt'
         *  Sum: '<S190>/Sum3'
         *  Switch: '<S194>/Switch2'
         */
        rtDW->Divide1 = (int16_T)(rtb_Merge1 - rtY->iq) * rtP->cf_iqKiLimProt;

        /* End of Outputs for SubSystem: '<S189>/Speed_Mode_Protection' */
        break;

       case 2:
        if (UnitDelay3 != rtb_Sum2_ii) {
          /* SystemReset for IfAction SubSystem: '<S189>/Torque_Mode_Protection' incorporates:
           *  ActionPort: '<S191>/Action Port'
           */
          /* SystemReset for Atomic SubSystem: '<S191>/I_backCalc_fixdt' */
          /* SystemReset for SwitchCase: '<S189>/Switch Case' */
          I_backCalc_fixdt_Reset(58982400, &rtDW->I_backCalc_fixdt_h);

          /* End of SystemReset for SubSystem: '<S191>/I_backCalc_fixdt' */
          /* End of SystemReset for SubSystem: '<S189>/Torque_Mode_Protection' */
        }

        /* Outputs for IfAction SubSystem: '<S189>/Torque_Mode_Protection' incorporates:
         *  ActionPort: '<S191>/Action Port'
         */
        /* Outputs for Atomic SubSystem: '<S191>/I_backCalc_fixdt' */
        I_backCalc_fixdt((int16_T)(rtP->n_max - Abs5), rtP->cf_nKiLimProt,
                         rtP->cf_KbLimProt, rtDW->VariantMergeForOutportVq_max, 0,
                         &rtDW->Switch2_gd, &rtDW->I_backCalc_fixdt_h);

        /* End of Outputs for SubSystem: '<S191>/I_backCalc_fixdt' */
        /* End of Outputs for SubSystem: '<S189>/Torque_Mode_Protection' */
        break;
      }

      /* End of SwitchCase: '<S189>/Switch Case' */

      /* Gain: '<S189>/Gain4' incorporates:
       *  Constant: '<S189>/i_max'
       */
      rtDW->Gain4 = (int16_T)-rtP->i_max;

      /* SignalConversion generated from: '<S189>/id_max' incorporates:
       *  Constant: '<S189>/i_max'
       */
      rtDW->OutportBufferForid_max = rtP->i_max;

      /* End of Outputs for SubSystem: '<S51>/Motor_Limitations_Enabled' */
    }

    /* End of If: '<S51>/If1' */
    /* End of Outputs for SubSystem: '<S7>/Motor_Limitations' */
  } else if (rtDW->UnitDelay6_DSTATE) {
    /* Outputs for Function Call SubSystem: '<S7>/FOC' */
    /* If: '<S50>/If1' incorporates:
     *  Constant: '<S1>/z_ctrlTypSel'
     */
    rtb_Sum2_ii = rtDW->If1_ActiveSubsystem_f;
    UnitDelay3 = -1;
    if (rtP->z_ctrlTypSel == 2) {
      UnitDelay3 = 0;
    }

    rtDW->If1_ActiveSubsystem_f = UnitDelay3;
    if ((rtb_Sum2_ii != UnitDelay3) && (rtb_Sum2_ii == 0)) {
      /* Disable for SwitchCase: '<S62>/Switch Case' */
      rtDW->SwitchCase_ActiveSubsystem = -1;

      /* Disable for If: '<S62>/If1' */
      rtDW->If1_ActiveSubsystem_a = -1;
    }

    if (UnitDelay3 == 0) {
      /* Outputs for IfAction SubSystem: '<S50>/FOC_Enabled' incorporates:
       *  ActionPort: '<S62>/Action Port'
       */
      /* SwitchCase: '<S62>/Switch Case' incorporates:
       *  Constant: '<S64>/cf_nKi'
       *  Constant: '<S64>/cf_nKp'
       *  Sum: '<S64>/Sum3'
       *  UnitDelay: '<S8>/UnitDelay4'
       */
      rtb_Sum2_ii = rtDW->SwitchCase_ActiveSubsystem;
      switch (rtDW->z_ctrlMod) {
       case 1:
        break;

       case 2:
        UnitDelay3 = 1;
        break;

       case 3:
        UnitDelay3 = 2;
        break;

       default:
        UnitDelay3 = 3;
        break;
      }

      rtDW->SwitchCase_ActiveSubsystem = UnitDelay3;
      switch (UnitDelay3) {
       case 0:
        /* Outputs for IfAction SubSystem: '<S62>/Voltage_Mode' incorporates:
         *  ActionPort: '<S67>/Action Port'
         */
        /* MinMax: '<S67>/MinMax' incorporates:
         *  Abs: '<S5>/Abs1'
         *  Switch: '<S203>/Switch2'
         *  Switch: '<S205>/Switch2'
         */
        if (rtDW->Abs1 <= rtDW->Switch2_b) {
          rtb_Saturation = rtDW->Abs1;
        } else {
          rtb_Saturation = rtDW->Switch2_b;
        }

        if (rtb_Saturation > rtDW->Switch2_p) {
          rtb_Saturation = rtDW->Switch2_p;
        }

        /* Signum: '<S67>/SignDeltaU2' incorporates:
         *  Merge: '<S33>/Merge1'
         */
        if (rtDW->Merge1 < 0) {
          rtb_Merge1 = -1;
        } else {
          rtb_Merge1 = (int16_T)(rtDW->Merge1 > 0);
        }

        /* Product: '<S67>/Divide1' incorporates:
         *  MinMax: '<S67>/MinMax'
         *  Signum: '<S67>/SignDeltaU2'
         */
        rtb_Saturation1 = (int16_T)(rtb_Saturation * rtb_Merge1);

        /* Switch: '<S188>/Switch2' incorporates:
         *  Product: '<S67>/Divide1'
         *  RelationalOperator: '<S188>/LowerRelop1'
         *  RelationalOperator: '<S188>/UpperRelop'
         *  Switch: '<S188>/Switch'
         *  VariantMerge generated from: '<S192>/Vq_max'
         *  VariantMerge generated from: '<S192>/Vq_min'
         */
        if (rtb_Saturation1 > rtDW->VariantMergeForOutportVq_max) {
          /* Merge: '<S62>/Merge' */
          rtDW->Merge = rtDW->VariantMergeForOutportVq_max;
        } else if (rtb_Saturation1 < rtDW->VariantMergeForOutportVq_min) {
          /* Switch: '<S188>/Switch' incorporates:
           *  Merge: '<S62>/Merge'
           *  VariantMerge generated from: '<S192>/Vq_min'
           */
          rtDW->Merge = rtDW->VariantMergeForOutportVq_min;
        } else {
          /* Merge: '<S62>/Merge' incorporates:
           *  Switch: '<S188>/Switch'
           */
          rtDW->Merge = rtb_Saturation1;
        }

        /* End of Switch: '<S188>/Switch2' */
        /* End of Outputs for SubSystem: '<S62>/Voltage_Mode' */
        break;

       case 1:
        if (UnitDelay3 != rtb_Sum2_ii) {
          /* SystemReset for IfAction SubSystem: '<S62>/Speed_Mode' incorporates:
           *  ActionPort: '<S64>/Action Port'
           */
          /* SystemReset for Atomic SubSystem: '<S64>/PI_clamp_fixdt' */
          /* SystemReset for SwitchCase: '<S62>/Switch Case' */
          PI_clamp_fixdt_Reset(&rtDW->PI_clamp_fixdt_j);

          /* End of SystemReset for SubSystem: '<S64>/PI_clamp_fixdt' */
          /* End of SystemReset for SubSystem: '<S62>/Speed_Mode' */
        }

        /* Outputs for IfAction SubSystem: '<S62>/Speed_Mode' incorporates:
         *  ActionPort: '<S64>/Action Port'
         */
        /* DataTypeConversion: '<S64>/Data Type Conversion2' incorporates:
         *  Constant: '<S64>/n_cruiseMotTgt'
         */
        rtb_Saturation1 = (int16_T)(rtP->n_cruiseMotTgt << 4);

        /* Switch: '<S64>/Switch4' incorporates:
         *  Constant: '<S1>/b_cruiseCtrlEna'
         *  DataTypeConversion: '<S64>/Data Type Conversion2'
         *  Logic: '<S64>/Logical Operator1'
         *  RelationalOperator: '<S64>/Relational Operator3'
         */
        if (rtP->b_cruiseCtrlEna && (rtb_Saturation1 != 0)) {
          /* Switch: '<S64>/Switch3' incorporates:
           *  Merge: '<S33>/Merge1'
           *  MinMax: '<S64>/MinMax4'
           *  VariantMerge generated from: '<S192>/Vq_max'
           */
          if (rtb_Saturation1 > 0) {
            /* Switch: '<S64>/Switch4' */
            rtDW->TmpSignalConversionAtLow_Pa[0] =
              rtDW->VariantMergeForOutportVq_max;

            /* MinMax: '<S64>/MinMax3' incorporates:
             *  Merge: '<S33>/Merge1'
             *  VariantMerge generated from: '<S192>/Vq_min'
             */
            if (rtDW->Merge1 >= rtDW->VariantMergeForOutportVq_min) {
              /* Switch: '<S64>/Switch4' */
              rtDW->TmpSignalConversionAtLow_Pa[1] = rtDW->Merge1;
            } else {
              /* Switch: '<S64>/Switch4' */
              rtDW->TmpSignalConversionAtLow_Pa[1] =
                rtDW->VariantMergeForOutportVq_min;
            }

            /* End of MinMax: '<S64>/MinMax3' */
          } else {
            if (rtDW->VariantMergeForOutportVq_max <= rtDW->Merge1) {
              /* MinMax: '<S64>/MinMax4' incorporates:
               *  Switch: '<S64>/Switch4'
               *  VariantMerge generated from: '<S192>/Vq_max'
               */
              rtDW->TmpSignalConversionAtLow_Pa[0] =
                rtDW->VariantMergeForOutportVq_max;
            } else {
              /* Switch: '<S64>/Switch4' incorporates:
               *  Merge: '<S33>/Merge1'
               *  MinMax: '<S64>/MinMax4'
               */
              rtDW->TmpSignalConversionAtLow_Pa[0] = rtDW->Merge1;
            }

            /* Switch: '<S64>/Switch4' */
            rtDW->TmpSignalConversionAtLow_Pa[1] =
              rtDW->VariantMergeForOutportVq_min;
          }

          /* End of Switch: '<S64>/Switch3' */
        } else {
          rtDW->TmpSignalConversionAtLow_Pa[0] =
            rtDW->VariantMergeForOutportVq_max;
          rtDW->TmpSignalConversionAtLow_Pa[1] =
            rtDW->VariantMergeForOutportVq_min;
        }

        /* End of Switch: '<S64>/Switch4' */

        /* Switch: '<S64>/Switch2' incorporates:
         *  Constant: '<S1>/b_cruiseCtrlEna'
         *  Merge: '<S33>/Merge1'
         */
        if (!rtP->b_cruiseCtrlEna) {
          rtb_Saturation1 = rtDW->Merge1;
        }

        /* Sum: '<S64>/Sum3' incorporates:
         *  Switch: '<S13>/Switch2'
         *  Switch: '<S64>/Switch2'
         */
        rtDW->Switch1_m = rtb_Saturation1 - Switch2;
        if (rtDW->Switch1_m > 32767) {
          rtDW->Switch1_m = 32767;
        } else if (rtDW->Switch1_m < -32768) {
          rtDW->Switch1_m = -32768;
        }

        /* Outputs for Atomic SubSystem: '<S64>/PI_clamp_fixdt' */
        rtDW->Merge = PI_clamp_fixdt((int16_T)rtDW->Switch1_m, rtP->cf_nKp,
          rtP->cf_nKi, rtDW->UnitDelay4_DSTATE_a,
          rtDW->TmpSignalConversionAtLow_Pa[0],
          rtDW->TmpSignalConversionAtLow_Pa[1], rtDW->Divide1,
          &rtDW->PI_clamp_fixdt_j);

        /* End of Outputs for SubSystem: '<S64>/PI_clamp_fixdt' */
        /* End of Outputs for SubSystem: '<S62>/Speed_Mode' */
        break;

       case 2:
        if (UnitDelay3 != rtb_Sum2_ii) {
          /* InitializeConditions for IfAction SubSystem: '<S62>/Torque_Mode' incorporates:
           *  ActionPort: '<S65>/Action Port'
           */
          /* InitializeConditions for SwitchCase: '<S62>/Switch Case' incorporates:
           *  DiscreteIntegrator: '<S111>/Integrator'
           */
          rtDW->Integrator_IC_LOADING = 1U;

          /* End of InitializeConditions for SubSystem: '<S62>/Torque_Mode' */
        }

        /* Outputs for IfAction SubSystem: '<S62>/Torque_Mode' incorporates:
         *  ActionPort: '<S65>/Action Port'
         */
        /* MinMax: '<S65>/MinMax2' incorporates:
         *  Gain: '<S65>/Gain4'
         *  Switch: '<S197>/Switch2'
         *  VariantMerge generated from: '<S192>/Vq_min'
         */
        if ((int16_T)-rtDW->Switch2_gd >= rtDW->VariantMergeForOutportVq_min) {
          rtb_MinMax2 = (int16_T)-rtDW->Switch2_gd;
        } else {
          rtb_MinMax2 = rtDW->VariantMergeForOutportVq_min;
        }

        /* End of MinMax: '<S65>/MinMax2' */

        /* Switch: '<S73>/Switch2' incorporates:
         *  Gain: '<S189>/Gain1'
         *  Merge: '<S33>/Merge1'
         *  Product: '<S189>/Divide1'
         *  RelationalOperator: '<S73>/LowerRelop1'
         *  RelationalOperator: '<S73>/UpperRelop'
         *  Switch: '<S73>/Switch'
         */
        if (rtDW->Merge1 > rtDW->Divide1_d) {
          rtb_Merge1 = rtDW->Divide1_d;
        } else if (rtDW->Merge1 < rtDW->Gain1) {
          /* Switch: '<S73>/Switch' incorporates:
           *  Gain: '<S189>/Gain1'
           */
          rtb_Merge1 = rtDW->Gain1;
        } else {
          rtb_Merge1 = rtDW->Merge1;
        }

        /* Sum: '<S65>/Sum2' incorporates:
         *  Switch: '<S73>/Switch2'
         */
        rtDW->Switch1_m = rtb_Merge1 - rtY->iq;
        if (rtDW->Switch1_m > 32767) {
          rtDW->Switch1_m = 32767;
        } else if (rtDW->Switch1_m < -32768) {
          rtDW->Switch1_m = -32768;
        }

        /* DiscreteIntegrator: '<S111>/Integrator' incorporates:
         *  UnitDelay: '<S8>/UnitDelay4'
         */
        if (rtDW->Integrator_IC_LOADING != 0) {
          rtDW->Integrator_DSTATE = rtDW->UnitDelay4_DSTATE_a;
        }

        /* Product: '<S116>/PProd Out' incorporates:
         *  Constant: '<S65>/cf_iqKp1'
         *  Sum: '<S65>/Sum2'
         */
        rtDW->Sum1 = (rtDW->Switch1_m * rtP->cf_iqKp) >> 12;
        if (rtDW->Sum1 > 32767) {
          rtDW->Sum1 = 32767;
        } else if (rtDW->Sum1 < -32768) {
          rtDW->Sum1 = -32768;
        }

        /* Sum: '<S121>/Sum' incorporates:
         *  DiscreteIntegrator: '<S111>/Integrator'
         *  Product: '<S116>/PProd Out'
         */
        rtDW->Sum1 += rtDW->Integrator_DSTATE;
        if (rtDW->Sum1 > 32767) {
          rtDW->Sum1 = 32767;
        } else if (rtDW->Sum1 < -32768) {
          rtDW->Sum1 = -32768;
        }

        /* MinMax: '<S65>/MinMax1' incorporates:
         *  Switch: '<S119>/Switch2'
         *  Switch: '<S197>/Switch2'
         *  VariantMerge generated from: '<S192>/Vq_max'
         */
        if (rtDW->VariantMergeForOutportVq_max <= rtDW->Switch2_gd) {
          rtb_Saturation1 = rtDW->VariantMergeForOutportVq_max;
        } else {
          rtb_Saturation1 = rtDW->Switch2_gd;
        }

        /* End of MinMax: '<S65>/MinMax1' */

        /* Switch: '<S103>/Switch' incorporates:
         *  MinMax: '<S65>/MinMax2'
         *  RelationalOperator: '<S103>/u_GTE_up'
         *  RelationalOperator: '<S103>/u_GT_lo'
         *  Sum: '<S121>/Sum'
         *  Switch: '<S103>/Switch1'
         *  Switch: '<S119>/Switch2'
         */
        if (rtDW->Sum1 >= rtb_Saturation1) {
          rtb_Merge1 = rtb_Saturation1;
        } else if (rtDW->Sum1 > rtb_MinMax2) {
          /* Switch: '<S103>/Switch1' */
          rtb_Merge1 = (int16_T)rtDW->Sum1;
        } else {
          rtb_Merge1 = rtb_MinMax2;
        }

        /* Sum: '<S103>/Diff' incorporates:
         *  Product: '<S108>/IProd Out'
         *  Sum: '<S121>/Sum'
         *  Switch: '<S103>/Switch'
         */
        rtb_Merge1 = (int16_T)(rtDW->Sum1 - rtb_Merge1);

        /* RelationalOperator: '<S100>/Relational Operator' incorporates:
         *  Product: '<S108>/IProd Out'
         */
        rtb_RelationalOperator = (rtb_Merge1 != 0);

        /* Switch: '<S100>/Switch1' incorporates:
         *  Constant: '<S100>/Constant'
         *  Constant: '<S100>/Constant2'
         *  Product: '<S108>/IProd Out'
         *  RelationalOperator: '<S100>/fix for DT propagation issue'
         */
        if (rtb_Merge1 > 0) {
          rtb_Sum2_ii = 1;
        } else {
          rtb_Sum2_ii = -1;
        }

        /* End of Switch: '<S100>/Switch1' */

        /* Product: '<S108>/IProd Out' incorporates:
         *  Constant: '<S65>/cf_iqKi1'
         *  Sum: '<S65>/Sum2'
         */
        rtb_Merge1 = (int16_T)((rtDW->Switch1_m * rtP->cf_iqKi) >> 16);

        /* Switch: '<S119>/Switch2' incorporates:
         *  RelationalOperator: '<S119>/LowerRelop1'
         *  Sum: '<S121>/Sum'
         */
        if (rtDW->Sum1 <= rtb_Saturation1) {
          /* Switch: '<S119>/Switch' incorporates:
           *  MinMax: '<S65>/MinMax2'
           *  RelationalOperator: '<S119>/UpperRelop'
           */
          if (rtDW->Sum1 < rtb_MinMax2) {
            rtb_Saturation1 = rtb_MinMax2;
          } else {
            rtb_Saturation1 = (int16_T)rtDW->Sum1;
          }

          /* End of Switch: '<S119>/Switch' */
        }

        /* End of Switch: '<S119>/Switch2' */

        /* Merge: '<S62>/Merge' incorporates:
         *  SignalConversion: '<S65>/Signal Conversion2'
         *  Switch: '<S119>/Switch2'
         */
        rtDW->Merge = rtb_Saturation1;

        /* Update for DiscreteIntegrator: '<S111>/Integrator' */
        rtDW->Integrator_IC_LOADING = 0U;

        /* Switch: '<S100>/Switch2' incorporates:
         *  Constant: '<S100>/Constant3'
         *  Constant: '<S100>/Constant4'
         *  Product: '<S108>/IProd Out'
         *  RelationalOperator: '<S100>/fix for DT propagation issue1'
         */
        if (rtb_Merge1 > 0) {
          UnitDelay3 = 1;
        } else {
          UnitDelay3 = -1;
        }

        /* Switch: '<S100>/Switch' incorporates:
         *  Constant: '<S100>/Constant1'
         *  Logic: '<S100>/AND3'
         *  RelationalOperator: '<S100>/Equal1'
         *  Switch: '<S100>/Switch2'
         */
        if (rtb_RelationalOperator && (rtb_Sum2_ii == UnitDelay3)) {
          rtb_Merge1 = 0;
        }

        /* Update for DiscreteIntegrator: '<S111>/Integrator' incorporates:
         *  Switch: '<S100>/Switch'
         */
        rtDW->Switch1_m = rtDW->Integrator_DSTATE + rtb_Merge1;
        if (rtDW->Switch1_m > 32767) {
          rtDW->Switch1_m = 32767;
        } else if (rtDW->Switch1_m < -32768) {
          rtDW->Switch1_m = -32768;
        }

        rtDW->Integrator_DSTATE = (int16_T)rtDW->Switch1_m;

        /* End of Outputs for SubSystem: '<S62>/Torque_Mode' */
        break;

       default:
        /* Outputs for IfAction SubSystem: '<S62>/Open_Mode' incorporates:
         *  ActionPort: '<S63>/Action Port'
         */
        /* Merge: '<S62>/Merge' incorporates:
         *  Merge: '<S33>/Merge1'
         *  SignalConversion generated from: '<S63>/r_inpTgtSca'
         */
        rtDW->Merge = rtDW->Merge1;

        /* End of Outputs for SubSystem: '<S62>/Open_Mode' */
        break;
      }

      /* End of SwitchCase: '<S62>/Switch Case' */

      /* If: '<S62>/If1' incorporates:
       *  Constant: '<S66>/constant2'
       */
      rtb_Sum2_ii = rtDW->If1_ActiveSubsystem_a;
      UnitDelay3 = -1;
      if (rtb_LogicalOperator) {
        UnitDelay3 = 0;
      }

      rtDW->If1_ActiveSubsystem_a = UnitDelay3;
      if (UnitDelay3 == 0) {
        if (rtb_Sum2_ii != 0) {
          /* InitializeConditions for IfAction SubSystem: '<S62>/Vd_Calculation' incorporates:
           *  ActionPort: '<S66>/Action Port'
           */
          /* InitializeConditions for If: '<S62>/If1' incorporates:
           *  Constant: '<S66>/constant2'
           *  DiscreteIntegrator: '<S169>/Integrator'
           */
          rtDW->Integrator_DSTATE_k = rtDW->constant2;

          /* End of InitializeConditions for SubSystem: '<S62>/Vd_Calculation' */
        }

        /* Outputs for IfAction SubSystem: '<S62>/Vd_Calculation' incorporates:
         *  ActionPort: '<S66>/Action Port'
         */
        /* Gain: '<S66>/toNegative' incorporates:
         *  SignalConversion generated from: '<S6>/r_fieldWeak '
         */
        rtb_Saturation1 = (int16_T)-rtDW->OutportBufferForr_fieldWeak;

        /* Switch: '<S131>/Switch2' incorporates:
         *  Gain: '<S189>/Gain4'
         *  Gain: '<S66>/toNegative'
         *  Product: '<S166>/IProd Out'
         *  RelationalOperator: '<S131>/LowerRelop1'
         *  RelationalOperator: '<S131>/UpperRelop'
         *  SignalConversion generated from: '<S189>/id_max'
         *  SignalConversion generated from: '<S6>/r_fieldWeak '
         *  Switch: '<S131>/Switch'
         */
        if ((int16_T)-rtDW->OutportBufferForr_fieldWeak >
            rtDW->OutportBufferForid_max) {
          rtb_Saturation1 = rtDW->OutportBufferForid_max;
        } else if ((int16_T)-rtDW->OutportBufferForr_fieldWeak < rtDW->Gain4) {
          /* Switch: '<S131>/Switch' incorporates:
           *  Gain: '<S189>/Gain4'
           *  Product: '<S166>/IProd Out'
           */
          rtb_Saturation1 = rtDW->Gain4;
        }

        /* End of Switch: '<S131>/Switch2' */

        /* Sum: '<S66>/Sum3' incorporates:
         *  Product: '<S166>/IProd Out'
         */
        rtDW->Switch1_m = rtb_Saturation1 - rtY->id;
        if (rtDW->Switch1_m > 32767) {
          rtDW->Switch1_m = 32767;
        } else if (rtDW->Switch1_m < -32768) {
          rtDW->Switch1_m = -32768;
        }

        /* Product: '<S166>/IProd Out' incorporates:
         *  Constant: '<S66>/cf_idKi2'
         *  Sum: '<S66>/Sum3'
         */
        rtb_Saturation1 = (int16_T)((rtDW->Switch1_m * rtP->cf_idKi) >> 16);
        rtDW->constant2 = 0;

        /* Product: '<S174>/PProd Out' incorporates:
         *  Constant: '<S66>/cf_idKp2'
         *  Constant: '<S66>/constant2'
         *  Sum: '<S66>/Sum3'
         */
        rtDW->Switch1_m = (rtDW->Switch1_m * rtP->cf_idKp) >> 12;
        if (rtDW->Switch1_m > 32767) {
          rtDW->Switch1_m = 32767;
        } else if (rtDW->Switch1_m < -32768) {
          rtDW->Switch1_m = -32768;
        }

        /* Sum: '<S179>/Sum' incorporates:
         *  DiscreteIntegrator: '<S169>/Integrator'
         *  Product: '<S174>/PProd Out'
         */
        rtDW->Switch1_m += rtDW->Integrator_DSTATE_k;
        if (rtDW->Switch1_m > 32767) {
          rtDW->Switch1_m = 32767;
        } else if (rtDW->Switch1_m < -32768) {
          rtDW->Switch1_m = -32768;
        }

        /* Switch: '<S161>/Switch' incorporates:
         *  RelationalOperator: '<S161>/u_GTE_up'
         *  RelationalOperator: '<S161>/u_GT_lo'
         *  Sum: '<S179>/Sum'
         *  Switch: '<S161>/Switch1'
         *  VariantMerge generated from: '<S192>/Vd_max'
         *  VariantMerge generated from: '<S192>/Vd_min'
         */
        if (rtDW->Switch1_m >= rtDW->VariantMergeForOutportVd_max) {
          rtb_Merge1 = rtDW->VariantMergeForOutportVd_max;
        } else if (rtDW->Switch1_m > rtDW->VariantMergeForOutportVd_min) {
          /* Switch: '<S161>/Switch1' */
          rtb_Merge1 = (int16_T)rtDW->Switch1_m;
        } else {
          rtb_Merge1 = rtDW->VariantMergeForOutportVd_min;
        }

        /* Sum: '<S161>/Diff' incorporates:
         *  Sum: '<S179>/Sum'
         *  Switch: '<S161>/Switch'
         */
        rtb_Saturation = (int16_T)(rtDW->Switch1_m - rtb_Merge1);

        /* Switch: '<S177>/Switch2' incorporates:
         *  RelationalOperator: '<S177>/LowerRelop1'
         *  RelationalOperator: '<S177>/UpperRelop'
         *  Sum: '<S179>/Sum'
         *  Switch: '<S177>/Switch'
         *  VariantMerge generated from: '<S192>/Vd_max'
         *  VariantMerge generated from: '<S192>/Vd_min'
         */
        if (rtDW->Switch1_m > rtDW->VariantMergeForOutportVd_max) {
          /* Switch: '<S177>/Switch2' */
          rtDW->Switch2_g = rtDW->VariantMergeForOutportVd_max;
        } else if (rtDW->Switch1_m < rtDW->VariantMergeForOutportVd_min) {
          /* Switch: '<S177>/Switch' incorporates:
           *  Switch: '<S177>/Switch2'
           *  VariantMerge generated from: '<S192>/Vd_min'
           */
          rtDW->Switch2_g = rtDW->VariantMergeForOutportVd_min;
        } else {
          /* Switch: '<S177>/Switch2' */
          rtDW->Switch2_g = (int16_T)rtDW->Switch1_m;
        }

        /* End of Switch: '<S177>/Switch2' */

        /* Switch: '<S158>/Switch1' incorporates:
         *  Constant: '<S158>/Constant'
         *  Constant: '<S158>/Constant2'
         *  RelationalOperator: '<S158>/fix for DT propagation issue'
         *  Sum: '<S161>/Diff'
         */
        if (rtb_Saturation > 0) {
          UnitDelay3 = 1;
        } else {
          UnitDelay3 = -1;
        }

        /* Switch: '<S158>/Switch2' incorporates:
         *  Constant: '<S158>/Constant3'
         *  Constant: '<S158>/Constant4'
         *  Product: '<S166>/IProd Out'
         *  RelationalOperator: '<S158>/fix for DT propagation issue1'
         */
        if (rtb_Saturation1 > 0) {
          rtb_Sum2_ii = 1;
        } else {
          rtb_Sum2_ii = -1;
        }

        /* Switch: '<S158>/Switch' incorporates:
         *  Constant: '<S158>/Constant1'
         *  Logic: '<S158>/AND3'
         *  RelationalOperator: '<S158>/Equal1'
         *  RelationalOperator: '<S158>/Relational Operator'
         *  Sum: '<S161>/Diff'
         *  Switch: '<S158>/Switch1'
         *  Switch: '<S158>/Switch2'
         */
        if ((rtb_Saturation != 0) && (UnitDelay3 == rtb_Sum2_ii)) {
          rtb_Saturation1 = 0;
        }

        /* Update for DiscreteIntegrator: '<S169>/Integrator' incorporates:
         *  Switch: '<S158>/Switch'
         */
        rtDW->Switch1_m = rtDW->Integrator_DSTATE_k + rtb_Saturation1;
        if (rtDW->Switch1_m > 32767) {
          rtDW->Switch1_m = 32767;
        } else if (rtDW->Switch1_m < -32768) {
          rtDW->Switch1_m = -32768;
        }

        rtDW->Integrator_DSTATE_k = (int16_T)rtDW->Switch1_m;

        /* End of Update for DiscreteIntegrator: '<S169>/Integrator' */
        /* End of Outputs for SubSystem: '<S62>/Vd_Calculation' */
      }

      /* End of If: '<S62>/If1' */
      /* End of Outputs for SubSystem: '<S50>/FOC_Enabled' */
    }

    /* End of If: '<S50>/If1' */
    /* End of Outputs for SubSystem: '<S7>/FOC' */
  }

  /* End of Chart: '<S1>/Task_Scheduler' */

  /* If: '<S7>/If2' incorporates:
   *  Constant: '<S1>/z_ctrlTypSel'
   *  Constant: '<S8>/CTRL_COMM1'
   *  Merge: '<S33>/Merge1'
   *  Merge: '<S62>/Merge'
   *  RelationalOperator: '<S8>/Relational Operator6'
   *  Switch: '<S8>/Switch2'
   */
  rtb_Sum2_ii = rtDW->If2_ActiveSubsystem;
  UnitDelay3 = -1;
  if (rtP->z_ctrlTypSel == 2) {
    rtb_Saturation1 = rtDW->Merge;
    UnitDelay3 = 0;
  } else {
    rtb_Saturation1 = rtDW->Merge1;
  }

  rtDW->If2_ActiveSubsystem = UnitDelay3;
  if ((rtb_Sum2_ii != UnitDelay3) && (rtb_Sum2_ii == 0)) {
    /* Disable for Gain: '<S60>/Gain4' incorporates:
     *  Outport: '<S49>/V_phaABC_FOC'
     */
    rtDW->Gain4_o[0] = 0;
    rtDW->Gain4_o[1] = 0;
    rtDW->Gain4_o[2] = 0;
  }

  if (UnitDelay3 == 0) {
    /* Outputs for IfAction SubSystem: '<S7>/Clarke_Park_Transform_Inverse' incorporates:
     *  ActionPort: '<S49>/Action Port'
     */
    /* Sum: '<S61>/Sum6' incorporates:
     *  Interpolation_n-D generated from: '<S55>/r_cos_M1'
     *  Interpolation_n-D generated from: '<S55>/r_sin_M1'
     *  Merge: '<S62>/Merge'
     *  Product: '<S61>/Divide1'
     *  Product: '<S61>/Divide4'
     *  Switch: '<S177>/Switch2'
     */
    rtDW->Switch1_m = (int16_T)((rtDW->Switch2_g * rtDW->r_cos_M1_1) >> 14) -
      (int16_T)((rtDW->Merge * rtDW->r_sin_M1_1) >> 14);
    if (rtDW->Switch1_m > 32767) {
      rtDW->Switch1_m = 32767;
    } else if (rtDW->Switch1_m < -32768) {
      rtDW->Switch1_m = -32768;
    }

    /* Sum: '<S61>/Sum1' incorporates:
     *  Interpolation_n-D generated from: '<S55>/r_cos_M1'
     *  Interpolation_n-D generated from: '<S55>/r_sin_M1'
     *  Merge: '<S62>/Merge'
     *  Product: '<S61>/Divide2'
     *  Product: '<S61>/Divide3'
     *  Switch: '<S177>/Switch2'
     */
    rtDW->Sum1 = (int16_T)((rtDW->Switch2_g * rtDW->r_sin_M1_1) >> 14) +
      (int16_T)((rtDW->Merge * rtDW->r_cos_M1_1) >> 14);
    if (rtDW->Sum1 > 32767) {
      rtDW->Sum1 = 32767;
    } else if (rtDW->Sum1 < -32768) {
      rtDW->Sum1 = -32768;
    }

    /* Gain: '<S60>/Gain1' incorporates:
     *  Sum: '<S61>/Sum1'
     */
    rtDW->Sum1 *= 14189;

    /* Sum: '<S60>/Sum6' incorporates:
     *  Gain: '<S60>/Gain1'
     *  Gain: '<S60>/Gain3'
     *  Sum: '<S61>/Sum6'
     */
    rtDW->Sum1 = (((rtDW->Sum1 < 0 ? 16383 : 0) + rtDW->Sum1) >> 14) - ((int16_T)
      ((rtDW->Switch1_m < 0) + rtDW->Switch1_m) >> 1);
    if (rtDW->Sum1 > 32767) {
      rtDW->Sum1 = 32767;
    } else if (rtDW->Sum1 < -32768) {
      rtDW->Sum1 = -32768;
    }

    /* Sum: '<S60>/Sum2' incorporates:
     *  Sum: '<S60>/Sum6'
     *  Sum: '<S61>/Sum6'
     */
    rtb_Sum1_h = -rtDW->Switch1_m - rtDW->Sum1;
    if (rtb_Sum1_h > 32767) {
      rtb_Sum1_h = 32767;
    } else if (rtb_Sum1_h < -32768) {
      rtb_Sum1_h = -32768;
    }

    /* MinMax: '<S60>/MinMax1' incorporates:
     *  Sum: '<S61>/Sum6'
     */
    rtb_Saturation = (int16_T)rtDW->Switch1_m;

    /* MinMax: '<S60>/MinMax2' incorporates:
     *  MinMax: '<S60>/MinMax1'
     *  Sum: '<S61>/Sum6'
     */
    rtb_Merge1 = (int16_T)rtDW->Switch1_m;

    /* MinMax: '<S60>/MinMax1' incorporates:
     *  Sum: '<S60>/Sum6'
     *  Sum: '<S61>/Sum6'
     */
    if (rtDW->Switch1_m > rtDW->Sum1) {
      rtb_Saturation = (int16_T)rtDW->Sum1;
    }

    /* MinMax: '<S60>/MinMax2' incorporates:
     *  Sum: '<S60>/Sum6'
     *  Sum: '<S61>/Sum6'
     */
    if (rtDW->Switch1_m < rtDW->Sum1) {
      rtb_Merge1 = (int16_T)rtDW->Sum1;
    }

    /* MinMax: '<S60>/MinMax1' incorporates:
     *  Sum: '<S60>/Sum2'
     */
    if (rtb_Saturation > rtb_Sum1_h) {
      rtb_Saturation = (int16_T)rtb_Sum1_h;
    }

    /* MinMax: '<S60>/MinMax2' incorporates:
     *  Sum: '<S60>/Sum2'
     */
    if (rtb_Merge1 < rtb_Sum1_h) {
      rtb_Merge1 = (int16_T)rtb_Sum1_h;
    }

    /* Sum: '<S60>/Add' incorporates:
     *  MinMax: '<S60>/MinMax1'
     *  MinMax: '<S60>/MinMax2'
     */
    rtb_Gain3 = rtb_Saturation + rtb_Merge1;
    if (rtb_Gain3 > 32767) {
      rtb_Gain3 = 32767;
    } else if (rtb_Gain3 < -32768) {
      rtb_Gain3 = -32768;
    }

    /* Gain: '<S60>/Gain2' incorporates:
     *  Sum: '<S60>/Add'
     */
    rtb_Merge1 = (int16_T)(rtb_Gain3 >> 1);

    /* Sum: '<S60>/Add1' incorporates:
     *  Gain: '<S60>/Gain2'
     *  Sum: '<S61>/Sum6'
     */
    rtDW->Switch1_m -= rtb_Merge1;
    if (rtDW->Switch1_m > 32767) {
      rtDW->Switch1_m = 32767;
    } else if (rtDW->Switch1_m < -32768) {
      rtDW->Switch1_m = -32768;
    }

    /* Gain: '<S60>/Gain4' incorporates:
     *  Sum: '<S60>/Add1'
     */
    rtDW->Gain4_o[0] = (int16_T)((18919 * rtDW->Switch1_m) >> 14);

    /* Sum: '<S60>/Add1' incorporates:
     *  Gain: '<S60>/Gain2'
     *  Sum: '<S60>/Sum6'
     */
    rtDW->Switch1_m = rtDW->Sum1 - rtb_Merge1;
    if (rtDW->Switch1_m > 32767) {
      rtDW->Switch1_m = 32767;
    } else if (rtDW->Switch1_m < -32768) {
      rtDW->Switch1_m = -32768;
    }

    /* Gain: '<S60>/Gain4' incorporates:
     *  Sum: '<S60>/Add1'
     */
    rtDW->Gain4_o[1] = (int16_T)((18919 * rtDW->Switch1_m) >> 14);

    /* Sum: '<S60>/Add1' incorporates:
     *  Gain: '<S60>/Gain2'
     *  Sum: '<S60>/Sum2'
     */
    rtDW->Switch1_m = rtb_Sum1_h - rtb_Merge1;
    if (rtDW->Switch1_m > 32767) {
      rtDW->Switch1_m = 32767;
    } else if (rtDW->Switch1_m < -32768) {
      rtDW->Switch1_m = -32768;
    }

    /* Gain: '<S60>/Gain4' incorporates:
     *  Sum: '<S60>/Add1'
     */
    rtDW->Gain4_o[2] = (int16_T)((18919 * rtDW->Switch1_m) >> 14);

    /* End of Outputs for SubSystem: '<S7>/Clarke_Park_Transform_Inverse' */
  }

  /* End of If: '<S7>/If2' */

  /* If: '<S8>/If' incorporates:
   *  Constant: '<S1>/z_ctrlTypSel'
   *  Constant: '<S8>/CTRL_COMM2'
   *  Constant: '<S8>/CTRL_COMM3'
   *  Logic: '<S8>/Logical Operator1'
   *  Logic: '<S8>/Logical Operator2'
   *  RelationalOperator: '<S8>/Relational Operator1'
   *  RelationalOperator: '<S8>/Relational Operator2'
   */
  if (rtb_LogicalOperator && (rtP->z_ctrlTypSel == 2)) {
    /* Outputs for IfAction SubSystem: '<S8>/FOC_Method' incorporates:
     *  ActionPort: '<S207>/Action Port'
     */
    /* SignalConversion generated from: '<S207>/V_phaABC_FOC_in' incorporates:
     *  Gain: '<S60>/Gain4'
     *  Merge: '<S8>/Merge'
     */
    rtb_Saturation = rtDW->Gain4_o[0];
    rtb_Merge1 = rtDW->Gain4_o[1];
    rtb_MinMax2 = rtDW->Gain4_o[2];

    /* End of Outputs for SubSystem: '<S8>/FOC_Method' */
  } else if (rtb_LogicalOperator && (rtP->z_ctrlTypSel == 1)) {
    /* Outputs for IfAction SubSystem: '<S8>/SIN_Method' incorporates:
     *  ActionPort: '<S208>/Action Port'
     */
    /* Switch: '<S209>/Switch_PhaAdv' incorporates:
     *  Constant: '<S209>/a_elecPeriod2'
     *  Constant: '<S209>/b_fieldWeakEna'
     *  Interpolation_n-D: '<S208>/r_sin3PhaA_M1'
     *  Merge: '<S3>/Merge'
     *  Product: '<S210>/Divide2'
     *  Product: '<S210>/Divide3'
     *  Sum: '<S209>/Sum3'
     *  Sum: '<S210>/Sum3'
     */
    if (rtP->b_fieldWeakEna) {
      /* Sum: '<S209>/Sum3' incorporates:
       *  Merge: '<S3>/Merge'
       *  Product: '<S209>/Product2'
       *  SignalConversion generated from: '<S6>/r_fieldWeak '
       */
      rtb_MinMax2 = (int16_T)((int16_T)((int16_T)
        (rtDW->OutportBufferForr_fieldWeak * rtDW->Switch2_a) << 2) + rtb_Merge);
      rtb_MinMax2 -= (int16_T)((int16_T)((int16_T)div_nde_s32_floor(rtb_MinMax2,
        23040) * 360) << 6);
    } else {
      rtb_MinMax2 = rtb_Merge;
    }

    /* End of Switch: '<S209>/Switch_PhaAdv' */

    /* PreLookup: '<S208>/a_elecAngle_XA' incorporates:
     *  Interpolation_n-D: '<S208>/r_sin3PhaA_M1'
     */
    Sum = plook_u8s16_evencka(rtb_MinMax2, 0, 128U, 180U);

    /* Product: '<S208>/Divide2' incorporates:
     *  Interpolation_n-D: '<S208>/r_sin3PhaA_M1'
     *  Interpolation_n-D: '<S208>/r_sin3PhaB_M1'
     *  Interpolation_n-D: '<S208>/r_sin3PhaC_M1'
     *  Merge: '<S8>/Merge'
     *  Switch: '<S8>/Switch2'
     */
    rtb_Saturation = (int16_T)((rtb_Saturation1 *
      rtConstP.r_sin3PhaA_M1_Table[Sum]) >> 14);
    rtb_Merge1 = (int16_T)((rtb_Saturation1 * rtConstP.r_sin3PhaB_M1_Table[Sum])
      >> 14);
    rtb_MinMax2 = (int16_T)((rtb_Saturation1 * rtConstP.r_sin3PhaC_M1_Table[Sum])
      >> 14);

    /* End of Outputs for SubSystem: '<S8>/SIN_Method' */
  } else {
    /* Selector: '<S11>/Selector' incorporates:
     *  Constant: '<S11>/vec_hallToPos'
     */
    rtb_Sum2_ii = rtConstP.vec_hallToPos_Value[Sum];

    /* Outputs for IfAction SubSystem: '<S8>/COM_Method' incorporates:
     *  ActionPort: '<S206>/Action Port'
     */
    /* LookupNDDirect: '<S206>/z_commutMap_M1'
     *
     * About '<S206>/z_commutMap_M1':
     *  2-dimensional Direct Look-Up returning a Column,
     *  which is contiguous for column-major array
     *     Remove protection against out-of-range input in generated code: 'off'
     *   */
    if (rtb_Sum2_ii > 5) {
      rtb_Sum2_ii = 5;
    } else if (rtb_Sum2_ii < 0) {
      rtb_Sum2_ii = 0;
    }

    rtDW->Sum1 = rtb_Sum2_ii * 3;

    /* Product: '<S206>/Divide2' incorporates:
     *  LookupNDDirect: '<S206>/z_commutMap_M1'
     *  Merge: '<S8>/Merge'
     *  Switch: '<S8>/Switch2'
     *
     * About '<S206>/z_commutMap_M1':
     *  2-dimensional Direct Look-Up returning a Column,
     *  which is contiguous for column-major array
     *     Remove protection against out-of-range input in generated code: 'off'
     *   */
    rtb_Saturation = (int16_T)(rtb_Saturation1 *
      rtConstP.z_commutMap_M1_table[rtDW->Sum1]);
    rtb_Merge1 = (int16_T)(rtConstP.z_commutMap_M1_table[rtDW->Sum1 + 1] *
      rtb_Saturation1);
    rtb_MinMax2 = (int16_T)(rtConstP.z_commutMap_M1_table[rtDW->Sum1 + 2] *
      rtb_Saturation1);

    /* End of Outputs for SubSystem: '<S8>/COM_Method' */
  }

  /* End of If: '<S8>/If' */

  /* Update for UnitDelay: '<S10>/UnitDelay3' */
  rtDW->UnitDelay3_DSTATE_f = rtU->b_hallA;

  /* Update for UnitDelay: '<S10>/UnitDelay1' */
  rtDW->UnitDelay1_DSTATE = rtU->b_hallB;

  /* Update for UnitDelay: '<S10>/UnitDelay2' */
  rtDW->UnitDelay2_DSTATE_k = rtU->b_hallC;

  /* Update for UnitDelay: '<S13>/UnitDelay3' */
  rtDW->UnitDelay3_DSTATE = rtb_Switch3_h;

  /* Update for UnitDelay: '<S13>/UnitDelay4' incorporates:
   *  Abs: '<S13>/Abs5'
   */
  rtDW->UnitDelay4_DSTATE_f = Abs5;

  /* Update for UnitDelay: '<S2>/UnitDelay2' incorporates:
   *  UnitDelay: '<S2>/UnitDelay6'
   */
  rtDW->UnitDelay2_DSTATE_g = rtDW->UnitDelay6_DSTATE;

  /* Update for UnitDelay: '<S2>/UnitDelay5' */
  rtDW->UnitDelay5_DSTATE_l = rtb_RelationalOperator4_i;

  /* Update for UnitDelay: '<S2>/UnitDelay6' */
  rtDW->UnitDelay6_DSTATE = rtb_n_commDeacv;

  /* Update for UnitDelay: '<S8>/UnitDelay4' incorporates:
   *  Switch: '<S8>/Switch2'
   */
  rtDW->UnitDelay4_DSTATE_a = rtb_Saturation1;

  /* Outport: '<Root>/DC_phaA' incorporates:
   *  DataTypeConversion: '<S8>/Data Type Conversion6'
   *  Merge: '<S8>/Merge'
   */
  rtY->DC_phaA = (int16_T)(rtb_Saturation >> 4);

  /* Outport: '<Root>/DC_phaB' incorporates:
   *  DataTypeConversion: '<S8>/Data Type Conversion6'
   *  Merge: '<S8>/Merge'
   */
  rtY->DC_phaB = (int16_T)(rtb_Merge1 >> 4);

  /* Outport: '<Root>/DC_phaC' incorporates:
   *  DataTypeConversion: '<S8>/Data Type Conversion6'
   *  Merge: '<S8>/Merge'
   */
  rtY->DC_phaC = (int16_T)(rtb_MinMax2 >> 4);

  /* Outport: '<Root>/n_mot' incorporates:
   *  DataTypeConversion: '<S1>/Data Type Conversion1'
   *  Switch: '<S13>/Switch2'
   */
  rtY->n_mot = (int16_T)(Switch2 >> 4);

  /* Outport: '<Root>/a_elecAngle' incorporates:
   *  DataTypeConversion: '<S1>/Data Type Conversion3'
   *  Merge: '<S3>/Merge'
   */
  rtY->a_elecAngle = (int16_T)(rtb_Merge >> 6);

  /* End of Outputs for SubSystem: '<Root>/BLDC_controller' */
}

/* Model initialize function */
void BLDC_controller_initialize(RT_MODEL *const rtM)
{
  P *rtP = ((P *) rtM->defaultParam);
  DW *rtDW = rtM->dwork;

  /* SystemInitialize for Atomic SubSystem: '<Root>/BLDC_controller' */
  /* Start for If: '<S7>/If1' */
  rtDW->If1_ActiveSubsystem = -1;

  /* Start for If: '<S7>/If2' */
  rtDW->If2_ActiveSubsystem = -1;

  /* InitializeConditions for UnitDelay: '<S13>/UnitDelay3' */
  rtDW->UnitDelay3_DSTATE = rtP->z_maxCntRst;

  /* InitializeConditions for UnitDelay: '<S2>/UnitDelay2' */
  rtDW->UnitDelay2_DSTATE_g = true;

  /* SystemInitialize for IfAction SubSystem: '<S13>/Raw_Motor_Speed_Estimation' */
  /* SystemInitialize for SignalConversion generated from: '<S17>/z_counterRawPrev' incorporates:
   *  Outport: '<S17>/z_counter'
   */
  rtDW->z_counterRawPrev = rtP->z_maxCntRst;

  /* End of SystemInitialize for SubSystem: '<S13>/Raw_Motor_Speed_Estimation' */

  /* SystemInitialize for Atomic SubSystem: '<S13>/Counter' */
  Counter_Init(rtP->z_maxCntRst, &rtDW->Counter_c);

  /* End of SystemInitialize for SubSystem: '<S13>/Counter' */

  /* SystemInitialize for IfAction SubSystem: '<S7>/Clarke_Park_Transform_Forward' */
  /* Start for If: '<S48>/If2' */
  rtDW->If2_ActiveSubsystem_a = -1;

  /* End of SystemInitialize for SubSystem: '<S7>/Clarke_Park_Transform_Forward' */

  /* SystemInitialize for Chart: '<S1>/Task_Scheduler' incorporates:
   *  SubSystem: '<S1>/F02_Diagnostics'
   */
  /* SystemInitialize for IfAction SubSystem: '<S4>/Diagnostics_Enabled' */
  /* SystemInitialize for Atomic SubSystem: '<S20>/Debounce_Filter' */
  Debounce_Filter_Init(&rtDW->Merge_a, &rtDW->Debounce_Filter_e);

  /* End of SystemInitialize for SubSystem: '<S20>/Debounce_Filter' */
  /* End of SystemInitialize for SubSystem: '<S4>/Diagnostics_Enabled' */

  /* SystemInitialize for Chart: '<S1>/Task_Scheduler' incorporates:
   *  SubSystem: '<S1>/F03_Control_Mode_Manager'
   */
  /* Start for If: '<S33>/If2' */
  rtDW->If2_ActiveSubsystem_k = -1;

  /* SystemInitialize for IfAction SubSystem: '<S33>/Open_Mode' */
  /* SystemInitialize for Atomic SubSystem: '<S37>/rising_edge_init' */
  /* InitializeConditions for UnitDelay: '<S42>/UnitDelay' */
  rtDW->UnitDelay_DSTATE_c = true;

  /* End of SystemInitialize for SubSystem: '<S37>/rising_edge_init' */
  /* End of SystemInitialize for SubSystem: '<S33>/Open_Mode' */

  /* SystemInitialize for Chart: '<S1>/Task_Scheduler' incorporates:
   *  SubSystem: '<S7>/Motor_Limitations'
   */
  /* Start for If: '<S51>/If1' */
  rtDW->If1_ActiveSubsystem_b = -1;

  /* SystemInitialize for IfAction SubSystem: '<S51>/Motor_Limitations_Enabled' */
  /* Start for SwitchCase: '<S189>/Switch Case' */
  rtDW->SwitchCase_ActiveSubsystem_o = -1;

  /* SystemInitialize for IfAction SubSystem: '<S189>/Voltage_Mode_Protection' */
  /* SystemInitialize for Atomic SubSystem: '<S193>/I_backCalc_fixdt' */
  I_backCalc_fixdt_Init(65536000, &rtDW->I_backCalc_fixdt_c);

  /* End of SystemInitialize for SubSystem: '<S193>/I_backCalc_fixdt' */

  /* SystemInitialize for Atomic SubSystem: '<S193>/I_backCalc_fixdt1' */
  I_backCalc_fixdt_Init(65536000, &rtDW->I_backCalc_fixdt1);

  /* End of SystemInitialize for SubSystem: '<S193>/I_backCalc_fixdt1' */
  /* End of SystemInitialize for SubSystem: '<S189>/Voltage_Mode_Protection' */

  /* SystemInitialize for IfAction SubSystem: '<S189>/Torque_Mode_Protection' */
  /* SystemInitialize for Atomic SubSystem: '<S191>/I_backCalc_fixdt' */
  I_backCalc_fixdt_Init(58982400, &rtDW->I_backCalc_fixdt_h);

  /* End of SystemInitialize for SubSystem: '<S191>/I_backCalc_fixdt' */
  /* End of SystemInitialize for SubSystem: '<S189>/Torque_Mode_Protection' */

  /* SystemInitialize for VariantMerge generated from: '<S192>/Vd_max' incorporates:
   *  Outport: '<S189>/Vd_max'
   */
  rtDW->VariantMergeForOutportVd_max = 14400;

  /* SystemInitialize for VariantMerge generated from: '<S192>/Vd_min' incorporates:
   *  Outport: '<S189>/Vd_min'
   */
  rtDW->VariantMergeForOutportVd_min = -14400;

  /* SystemInitialize for VariantMerge generated from: '<S192>/Vq_max' incorporates:
   *  Outport: '<S189>/Vq_max'
   */
  rtDW->VariantMergeForOutportVq_max = 14400;

  /* SystemInitialize for VariantMerge generated from: '<S192>/Vq_min' incorporates:
   *  Outport: '<S189>/Vq_min'
   */
  rtDW->VariantMergeForOutportVq_min = -14400;

  /* SystemInitialize for SignalConversion generated from: '<S189>/id_max' incorporates:
   *  Outport: '<S189>/id_max'
   */
  rtDW->OutportBufferForid_max = 12000;

  /* SystemInitialize for Gain: '<S189>/Gain4' incorporates:
   *  Outport: '<S189>/id_min'
   */
  rtDW->Gain4 = -12000;

  /* SystemInitialize for Product: '<S189>/Divide1' incorporates:
   *  Outport: '<S189>/iq_max'
   */
  rtDW->Divide1_d = 12000;

  /* SystemInitialize for Gain: '<S189>/Gain1' incorporates:
   *  Outport: '<S189>/iq_min'
   */
  rtDW->Gain1 = -12000;

  /* End of SystemInitialize for SubSystem: '<S51>/Motor_Limitations_Enabled' */

  /* SystemInitialize for Chart: '<S1>/Task_Scheduler' incorporates:
   *  SubSystem: '<S7>/FOC'
   */
  /* Start for If: '<S50>/If1' */
  rtDW->If1_ActiveSubsystem_f = -1;

  /* SystemInitialize for IfAction SubSystem: '<S50>/FOC_Enabled' */
  /* Start for SwitchCase: '<S62>/Switch Case' */
  rtDW->SwitchCase_ActiveSubsystem = -1;

  /* Start for If: '<S62>/If1' */
  rtDW->If1_ActiveSubsystem_a = -1;

  /* SystemInitialize for IfAction SubSystem: '<S62>/Speed_Mode' */
  /* SystemInitialize for Atomic SubSystem: '<S64>/PI_clamp_fixdt' */
  PI_clamp_fixdt_Init(&rtDW->PI_clamp_fixdt_j);

  /* End of SystemInitialize for SubSystem: '<S64>/PI_clamp_fixdt' */
  /* End of SystemInitialize for SubSystem: '<S62>/Speed_Mode' */

  /* SystemInitialize for IfAction SubSystem: '<S62>/Torque_Mode' */
  /* InitializeConditions for DiscreteIntegrator: '<S111>/Integrator' */
  rtDW->Integrator_IC_LOADING = 1U;

  /* End of SystemInitialize for SubSystem: '<S62>/Torque_Mode' */
  /* End of SystemInitialize for SubSystem: '<S50>/FOC_Enabled' */
  /* End of SystemInitialize for SubSystem: '<Root>/BLDC_controller' */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
