/*
 * Copyright (C) 2025  <>
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
 *
 */

/** @file modules/loggers/logger_file_heewing.c
 *  @brief File logger for Linux based autopilots (specially designed for the Heewing T1 Ranger VTOL)
 */

#include "logger_file_heewing.h"

#include <stdio.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>
#include "std.h"

#include "mcu_periph/sys_time.h"
#include "state.h"
#include "generated/airframe.h"

#include "generated/airframe.h"
#include "state.h"
#include "firmwares/rotorcraft/stabilization/stabilization_indi.h"
#include "modules/actuators/actuators.h"

#ifdef COMMAND_THRUST
#include "firmwares/rotorcraft/stabilization.h"
#else
#include "firmwares/fixedwing/stabilization/stabilization_attitude.h"
#include "firmwares/fixedwing/stabilization/stabilization_adaptive.h"
#endif

#include "generated/modules.h"

/** Set the default File logger path to the USB drive */
#ifndef LOGGER_FILE_PATH
#define LOGGER_FILE_PATH /data/video/usb
#endif

/** The file pointer */
static FILE *logger_file = NULL;


/** Logging functions */

/** Write CSV header
 * Write column names at the top of the CSV file. Make sure that the columns
 * match those in logger_file_write_row! Don't forget the \n at the end of the
 * line.
 * @param file Log file pointer
 */
static void logger_file_write_header(FILE *file) {
  fprintf(file, "time,");
  fprintf(file, "pos_x,pos_y,pos_z,");
  fprintf(file, "vel_x,vel_y,vel_z,");
  fprintf(file, "att_phi,att_theta,att_psi,");
  fprintf(file, "rate_p,rate_q,rate_r,");
  fprintf(file, "state_motor_right,state_motor_left,state_motor_back,state_mean_tilt,state_tilt_diff,state_ailerons,state_elevator,");
  fprintf(file, "cmd_motor_right,cmd_motor_left,cmd_motor_back,cmd_mean_tilt,cmd_tilt_diff,cmd_ailerons,cmd_elevator,");
  fprintf(file, "G1_11,G1_12,G1_15,G1_16,G1_21,G1_22,G1_23,G1_24,G1_27,G1_31,G1_32,G1_35,G1_41,G1_42,G1_43,G1_44,G1_51,G1_52,G1_54\n");
}

/** Write CSV row
 * Write values at this timestamp to log file. Make sure that the printf's match
 * the column headers of logger_file_write_header! Don't forget the \n at the
 * end of the line.
 * @param file Log file pointer
 */
static void logger_file_write_row(FILE *file) {
  struct NedCoor_f *pos = stateGetPositionNed_f();
  struct NedCoor_f *vel = stateGetSpeedNed_f();
  struct FloatEulers *att = stateGetNedToBodyEulers_f();
  struct FloatRates *rates = stateGetBodyRates_f();

  fprintf(file, "%f,", get_sys_time_float());
  fprintf(file, "%f,%f,%f,", pos->x, pos->y, pos->z);
  fprintf(file, "%f,%f,%f,", vel->x, vel->y, vel->z);
  fprintf(file, "%f,%f,%f,", att->phi, att->theta, att->psi);
  fprintf(file, "%f,%f,%f,", rates->p, rates->q, rates->r);
  fprintf(file, "%f,%f,%f,%f,%f,%f,%f,",
    actuator_state_filt_vect[0], actuator_state_filt_vect[1],
    actuator_state_filt_vect[2], actuator_state_filt_vect[3],
    actuator_state_filt_vect[4], actuator_state_filt_vect[5],
    actuator_state_filt_vect[6]);
  fprintf(file, "%d,%d,%d,%d,%d,%d,%d,",
    actuators_pprz[0], actuators_pprz[1],
    actuators_pprz[2], actuators_pprz[3],
    actuators_pprz[4], actuators_pprz[5],
    actuators_pprz[6]);
  fprintf(file, "%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",
    g1g2[0][0],g1g2[0][1],g1g2[0][4],g1g2[0][5],g1g2[1][0],g1g2[1][1],g1g2[1][2],g1g2[1][3],g1g2[1][6],g1g2[2][0],
    g1g2[2][1],g1g2[2][4],g1g2[3][0],g1g2[3][1],g1g2[3][2],g1g2[3][3],g1g2[4][0],g1g2[4][1],g1g2[4][3]);
}


/** Start the file logger and open a new file */
void logger_file_heewing_start(void)
{
  // Ensure that the module is running when started with this function
  //logger_file_heewing_logger_file_heewing_periodic_status = MODULES_RUN;
  
  // Create output folder if necessary
  if (access(STRINGIFY(LOGGER_FILE_PATH), F_OK)) {
    char save_dir_cmd[256];
    sprintf(save_dir_cmd, "mkdir -p %s", STRINGIFY(LOGGER_FILE_PATH));
    if (system(save_dir_cmd) != 0) {
      printf("[logger_file] Could not create log file directory %s.\n", STRINGIFY(LOGGER_FILE_PATH));
      return;
    }
  }

  // Get current date/time for filename
  char date_time[80];
  time_t now = time(0);
  struct tm  tstruct;
  tstruct = *localtime(&now);
  strftime(date_time, sizeof(date_time), "%Y%m%d-%H%M%S", &tstruct);

  uint32_t counter = 0;
  char filename[512];

  // Check for available files
  sprintf(filename, "%s/%s.csv", STRINGIFY(LOGGER_FILE_PATH), date_time);
  while ((logger_file = fopen(filename, "r"))) {
    fclose(logger_file);

    

    sprintf(filename, "%s/%s_%05d.csv", STRINGIFY(LOGGER_FILE_PATH), date_time, counter);
    counter++;
  }

  logger_file = fopen(filename, "w");
  if(!logger_file) {
    printf("[logger_file] ERROR opening log file %s!\n", filename);
    return;
  }

  printf("[logger_file] Start logging to %s...\n", filename);

  logger_file_write_header(logger_file);
}

/** Stop the logger an nicely close the file */
void logger_file_heewing_stop(void)
{
  if (logger_file != NULL) {
    fclose(logger_file);
    logger_file = NULL;
  }
}

/** Log the values to a csv file    */
void logger_file_heewing_periodic(void)
{
  if (logger_file == NULL) {
    return;
  }
  logger_file_write_row(logger_file);
}
