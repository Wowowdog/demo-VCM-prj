/*
 * File: ois195y.c
 *
 * Code generated for Simulink model 'ois195y'.
 *
 * Model version                  : 1.6
 * Simulink Coder version         : 9.3 (R2020a) 18-Nov-2019
 * C/C++ source code generated on : Thu Jul  7 18:51:12 2022
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: Intel->x86-64 (Windows64)
 * Code generation objectives:
 *    1. Execution efficiency
 *    2. RAM efficiency
 * Validation result: Not run
 */

#include "ois195y.h"

/* Block signals and states (default storage) */
DW rtDW;

/* External inputs (root inport signals with default storage) */
ExtU rtU;

/* External outputs (root outports fed by signals with default storage) */
ExtY rtY;
extern real_T rt_roundd_snf(real_T u);
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

/* Model step function */
void ois195y_step(void)
{
  real_T rtb_Integrator_p;
  real_T rtb_FilterCoefficient;
  real_T rtb_Integrator_g;
  real_T rtb_FilterCoefficient_l;
  real_T rtb_err;
  real_T rtb_FilterCoefficient_m;
  real_T u0;

  /* Sum: '<Root>/Sum2' incorporates:
   *  Fcn: '<Root>/hallcode2clcode'
   *  Inport: '<Root>/In1'
   *  Inport: '<Root>/In2'
   *  Quantizer: '<Root>/Quantizer'
   */
  rtb_Integrator_p = rt_roundd_snf(rtU.in / 0.5) * 0.5 - (rtU.hallcode - 16896.0)
    * 1024.0 / 29184.0;

  /* Gain: '<S38>/Filter Coefficient' incorporates:
   *  DiscreteIntegrator: '<S30>/Filter'
   *  Gain: '<S29>/Derivative Gain'
   *  Sum: '<S30>/SumD'
   */
  rtb_FilterCoefficient = (10 * rtb_Integrator_p - rtDW.Filter_DSTATE) *
    0.0162581964040484;

  /* Sum: '<Root>/Sum1' incorporates:
   *  Constant: '<Root>/Constant'
   *  DiscreteIntegrator: '<S35>/Integrator'
   *  Sum: '<S44>/Sum'
   */
  u0 = ((1 *rtb_Integrator_p + rtDW.Integrator_DSTATE) + rtb_FilterCoefficient) +
    512.0;

  /* Saturate: '<Root>/Saturation' */
  if (u0 > 1023.0) {
    /* Outport: '<Root>/Out1' */
    rtY.Out1 = 1023.0;
  } else if (u0 < 0.0) {
    /* Outport: '<Root>/Out1' */
    rtY.Out1 = 0.0;
  } else {
    /* Outport: '<Root>/Out1' */
    rtY.Out1 = u0;
  }

  /* End of Saturate: '<Root>/Saturation' */

  /* Sum: '<Root>/Sum4' incorporates:
   *  Fcn: '<Root>/hallcode2clcode1'
   *  Inport: '<Root>/In3'
   *  Inport: '<Root>/In4'
   *  Quantizer: '<Root>/Quantizer1'
   */
  rtb_Integrator_g = rt_roundd_snf(rtU.in_o / 0.5) * 0.5 - (rtU.hallcode_d -
    16896.0) * 1024.0 / 29184.0;

  /* Gain: '<S86>/
	Filter Coefficient' incorporates:
   *  DiscreteIntegrator: '<S78>/Filter'
   *  Gain: '<S77>/Derivative Gain'
   *  Sum: '<S78>/SumD'
   */
  rtb_FilterCoefficient_l = (3 * rtb_Integrator_g - rtDW.Filter_DSTATE_o) *
    0.0162581964040484;

  /* Sum: '<Root>/Sum3' incorporates:
   *  Constant: '<Root>/Constant1'
   *  DiscreteIntegrator: '<S83>/Integrator'
   *  Sum: '<S92>/Sum'
   */
  u0 = ((0.5*rtb_Integrator_g + rtDW.Integrator_DSTATE_e) + rtb_FilterCoefficient_l)
    + 512.0;

  /* Saturate: '<Root>/Saturation1' */
  if (u0 > 1023.0) {
    /* Outport: '<Root>/Out2' */
    rtY.Out2 = 1023.0;
  } else if (u0 < 0.0) {
    /* Outport: '<Root>/Out2' */
    rtY.Out2 = 0.0;
  } else {
    /* Outport: '<Root>/Out2' */
    rtY.Out2 = u0;
  }

  /* End of Saturate: '<Root>/Saturation1' */

  /* Sum: '<Root>/Sum6' incorporates:
   *  Fcn: '<Root>/hallcode2clcode2'
   *  Inport: '<Root>/In5'
   *  Inport: '<Root>/In6'
   *  Quantizer: '<Root>/Quantizer2'
   */
  rtb_err = rt_roundd_snf(rtU.in_b / 0.5) * 0.5 - (rtU.hallcode_g - 16896.0) *
    1024.0 / 29184.0;

  /* Gain: '<S134>/Filter Coefficient' incorporates:
   *  DiscreteIntegrator: '<S126>/Filter'
   *  Gain: '<S125>/Derivative Gain'
   *  Sum: '<S126>/SumD'
   */
  rtb_FilterCoefficient_m = (0.001 * rtb_err - rtDW.Filter_DSTATE_g) *
    95.162581964040484;

  /* Sum: '<Root>/Sum5' incorporates:
   *  Constant: '<Root>/Constant2'
   *  DiscreteIntegrator: '<S131>/Integrator'
   *  Sum: '<S140>/Sum'
   */
  u0 = ((0.1*rtb_err + rtDW.Integrator_DSTATE_l) + rtb_FilterCoefficient_m) + 512.0;

  /* Saturate: '<Root>/Saturation2' */
  if (u0 > 1023.0) {
    /* Outport: '<Root>/Out3' */
    rtY.Out3 = 1023.0;
  } else if (u0 < 0.0) {
    /* Outport: '<Root>/Out3' */
    rtY.Out3 = 0.0;
  } else {
    /* Outport: '<Root>/Out3' */
    rtY.Out3 = u0;
  }

  /* End of Saturate: '<Root>/Saturation2' */

  /* Update for DiscreteIntegrator: '<S30>/Filter' */
  rtDW.Filter_DSTATE += 0 * rtb_FilterCoefficient;

  /* Update for DiscreteIntegrator: '<S35>/Integrator' incorporates:
   *  Gain: '<S32>/Integral Gain'
   */
  rtDW.Integrator_DSTATE += 20 * rtb_Integrator_p * 0.001;

  /* Update for DiscreteIntegrator: '<S83>/Integrator' incorporates:
   *  Gain: '<S80>/Integral Gain'
   */
  rtDW.Integrator_DSTATE_e += 20 * rtb_Integrator_g * 0.001;//x1 ki 10

  /* Update for DiscreteIntegrator: '<S78>/Filter' */
  rtDW.Filter_DSTATE_o += 0.001 * rtb_FilterCoefficient_l;

  /* Update for DiscreteIntegrator: '<S131>/Integrator' incorporates:
   *  Gain: '<S128>/Integral Gain'
   */
  rtDW.Integrator_DSTATE_l += 0 * rtb_err * 0.001;//x2 ki 10

  /* Update for DiscreteIntegrator: '<S126>/Filter' */
  rtDW.Filter_DSTATE_g += 0.001 * rtb_FilterCoefficient_m;
}

/* Model initialize function */
void ois195y_initialize(void)
{
  /* (no initialization code required) */
}

/*
 * File trailer for generated code.
 *
 * [EOF]
 */
