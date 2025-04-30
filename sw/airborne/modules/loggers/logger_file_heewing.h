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

/** @file modules/loggers/logger_file_heewing.h
 *  @brief File logger for Linux based autopilots (specially designed for the Heewing T1 Ranger VTOL)
 */

#ifndef LOGGER_FILE_HEEWING_H_
#define LOGGER_FILE_HEEWING_H_

extern void logger_file_heewing_start(void);
extern void logger_file_heewing_stop(void);
extern void logger_file_heewing_periodic(void);

#endif /* LOGGER_FILE_HEEWING_H_ */
