/*
 * Copyright (C) 2025 Ramon Revilla Bouso <>
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

/**
 * @file firmwares/rotorcraft/guidance/guidance_indi_hybrid_tiltrotor.c
 *
 */

#include "firmwares/rotorcraft/guidance/guidance_indi_hybrid.h"
#include "firmwares/rotorcraft/guidance/guidance_indi_hybrid_tiltrotor.h"
#include "filters/low_pass_filter.h"
#include "state.h"
#include "generated/modules.h"

float bodyx_filter_cutoff = 0.2;
float bodyz_filter_cutoff = 0.2;

Butterworth2LowPass accel_bodyx_filt;
Butterworth2LowPass accel_bodyz_filt;

/**
 *
 * Call upon entering indi guidance
 */
void guidance_indi_tiltrotor_init(void) {
  float tau_bodyx = 1.0/(2.0*M_PI*bodyx_filter_cutoff);
  float tau_bodyz = 1.0/(2.0*M_PI*bodyz_filter_cutoff);
  float sample_time = 1.0 / PERIODIC_FREQUENCY;
  init_butterworth_2_low_pass(&accel_bodyx_filt, tau_bodyx, sample_time, 0.0);
  init_butterworth_2_low_pass(&accel_bodyz_filt, tau_bodyz, sample_time, -9.81);
}

/**
 * Low pass the accelerometer measurements to remove noise from vibrations.
 * The roll and pitch also need to be filtered to synchronize them with the
 * acceleration
 * Called as a periodic function with PERIODIC_FREQ
 */
void guidance_indi_tiltrotor_propagate_filters(void) {
   // Propagate filters
  float accelx = ACCEL_FLOAT_OF_BFP(stateGetAccelBody_i()->x);
  float accelz = ACCEL_FLOAT_OF_BFP(stateGetAccelBody_i()->z);
  update_butterworth_2_low_pass(&accel_bodyx_filt, accelx);
  update_butterworth_2_low_pass(&accel_bodyz_filt, accelz);
}

/**
 * Perform WLS
 *
 * @param Gmat Dynamics matrix
 * @param a_diff acceleration errors in earth frame
 * @param body_v 3D vector to write the control objective v
 */
void guidance_indi_calcg_wing(float Gmat[GUIDANCE_INDI_HYBRID_V][GUIDANCE_INDI_HYBRID_U], struct FloatVect3 a_diff, float body_v[GUIDANCE_INDI_HYBRID_V]) {
  // Get attitude
  struct FloatEulers eulers_zxy;
  float_eulers_of_quat_zxy(&eulers_zxy, stateGetNedToBodyQuat_f());

  /*Pre-calculate sines and cosines*/
  float sphi = sinf(eulers_zxy.phi);
  float cphi = cosf(eulers_zxy.phi);
  float stheta = sinf(eulers_zxy.theta);
  float ctheta = cosf(eulers_zxy.theta);
  float spsi = sinf(eulers_zxy.psi);
  float cpsi = cosf(eulers_zxy.psi);

#ifndef GUIDANCE_INDI_PITCH_EFF_SCALING
#define GUIDANCE_INDI_PITCH_EFF_SCALING 1.0
#endif

  gi_pitch_eff_scaling = GUIDANCE_INDI_PITCH_EFF_SCALING;

  /*Force resultants*/
  //float fx = accel_bodyx_filt.o[0];
  float fz = accel_bodyz_filt.o[0];

  // get the derivative of the lift wrt to theta
  float dfz = guidance_indi_get_liftd(stateGetAirspeed_f(), eulers_zxy.theta);
#if 0  
  Gmat[GIHT_X][GIHT_CMD_ROLL] =  0; //-spsi * cphi * stheta * fx + spsi * cphi * ctheta * fz;
  Gmat[GIHT_Y][GIHT_CMD_ROLL] =  cpsi * cphi * stheta * fx - cpsi * cphi * ctheta * fz;
  Gmat[GIHT_Z][GIHT_CMD_ROLL] =  0; //sphi * stheta * fx - sphi * ctheta * fz;

  Gmat[GIHT_X][GIHT_CMD_PITCH] = gi_pitch_eff_scaling * ((-cpsi * stheta - spsi * sphi * ctheta) * fx + (cpsi * ctheta - spsi * sphi * stheta) * fz) + (cpsi * stheta + spsi * sphi * ctheta) * dfz;
  Gmat[GIHT_Y][GIHT_CMD_PITCH] = 0; //(-spsi * stheta + cpsi * sphi * ctheta) * fx + (spsi * ctheta + cpsi * sphi * stheta) * fz + (spsi * stheta - cpsi * sphi * ctheta) * dfz;
  Gmat[GIHT_Z][GIHT_CMD_PITCH] = -gi_pitch_eff_scaling * cphi * ctheta * fx - cphi * stheta * fz + cphi * ctheta * dfz;

  Gmat[GIHT_X][GIHT_CMD_FZ] = 0; //cpsi * stheta + spsi * sphi * ctheta;
  Gmat[GIHT_Y][GIHT_CMD_FZ] = 0; //spsi * stheta - cpsi * sphi * ctheta;
  Gmat[GIHT_Z][GIHT_CMD_FZ] = cphi * ctheta;

  Gmat[GIHT_X][GIHT_CMD_FX] =  cpsi * ctheta - spsi * sphi * stheta;
  Gmat[GIHT_Y][GIHT_CMD_FX] =  0; //spsi * ctheta + cpsi * sphi * stheta;
  Gmat[GIHT_Z][GIHT_CMD_FX] =  0; //-cphi * stheta;
  // Make this term zero to prevent switching 'exploits'
  // Gmat[GIHT_Z][GIHT_CMD_FX] = 0;

  // Convert acceleration error to body axis system
  body_v[GIHT_X] =  a_diff.x;
  body_v[GIHT_Y] =  a_diff.y;
  body_v[GIHT_Z] =  a_diff.z;
  
#endif 

  Gmat[0][0] = -sphi*stheta*fz;
  Gmat[1][0] = -cphi*fz;
  Gmat[2][0] = -sphi*ctheta*fz;

  Gmat[0][1] =  cphi*ctheta*fz*gi_pitch_eff_scaling;
  Gmat[1][1] =  sphi*stheta*fz*gi_pitch_eff_scaling - sphi*dfz;
  Gmat[2][1] = -cphi*stheta*fz*gi_pitch_eff_scaling + cphi*dfz;

  Gmat[0][2] =  cphi*stheta;
  Gmat[1][2] = -sphi;
  Gmat[2][2] =  cphi*ctheta;

  Gmat[0][3] =  ctheta;
  Gmat[1][3] =  0;
  Gmat[2][3] = -stheta;
  // Make this term zero to prevent switching 'exploits'
  // Gmat[2][3] = 0;

  // Convert acceleration error to body axis system
  body_v[0] =  cpsi * a_diff.x + spsi * a_diff.y;
  body_v[1] = -spsi * a_diff.x + cpsi * a_diff.y;
  body_v[2] =  a_diff.z;

}