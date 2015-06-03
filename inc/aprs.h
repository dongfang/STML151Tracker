/* trackuino copyright (C) 2010  EA5HAV Javi
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#ifndef __APRS_H__
#define __APRS_H__

#include <avr/pgmspace.h>

struct APRS_PARAM {
	char name[8];
	char unit[8];
	float coeff[3];
};

extern const uint8_t NUM_APRS_PARAMS;
extern const struct APRS_PARAM APRS_PARAMS[];

#ifdef USE_MOTION_SENSOR
void aprs_send(uint16_t sequence, float altitude, float climb, float temperature, uint16_t turbulence);
#else
void aprs_send(uint16_t sequence, float altitude, float climb, float temperature);
#endif

#endif
