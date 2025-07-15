/*
 * Copyright (C) 2025 ENAC
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

#ifndef GUIDANCE_INDI_HYBRID_TILTROTOR
#define GUIDANCE_INDI_HYBRID_TILTROTOR

#define GIHT_X 0
#define GIHT_Y 1
#define GIHT_Z 2
#define GIHT_CMD_ROLL 0
#define GIHT_CMD_PITCH 1
#define GIHT_CMD_FZ 2
#define GIHT_CMD_FX 3

extern void guidance_indi_tiltrotor_init(void);
extern void guidance_indi_tiltrotor_propagate_filters(void);

#ifndef GUIDANCE_INDI_MIN_PITCH
#define GUIDANCE_INDI_MIN_PITCH -20
#define GUIDANCE_INDI_MAX_PITCH 20
#endif

extern float gi_roll_eff_scaling;
extern float gi_pitch_eff_scaling;
extern float gi_lift_eff_scaling;
extern float gi_thrust_eff_scaling;

extern float wls_guid_roll;
extern float wls_guid_pitch;
extern float wls_guid_fz;
extern float wls_guid_fx;

#endif // GUIDANCE_INDI_HYBRID_TILTROTOR