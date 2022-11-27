/*

HLW8012_ESP82
also works with BL0937

 Copyright (C) 2020 by David B brown

based on library by Jaromir Kopp <macwyznawca at me dot com>

Based on the library for Arduino created by: Xose Pérez <xose dot perez at gmail dot com>

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

#ifndef HLW8012_h
#define HLW8012_h

#include <espressif/esp_common.h>

// Internal voltage reference value
#define V_REF_HLW               2.43
#define V_REF_BL0               1.218

// The factor of a 1mOhm resistor
// as per recomended circuit in datasheet
// A 1mOhm resistor allows a ~30A max measurement
#define R_CURRENT           (0.001)

// This is the factor of a voltage divider of 6x 470K upstream and 1k downstream
// Sonoff Pow has 5x 470K
// Smart DGM outlet has 3x 680K
// as per recomended circuit in datasheet
#define R_VOLTAGE_HLW       ((5 * 470) + 1) //2351
#define R_VOLTAGE_BL0       ((3 * 680) + 1) //2041

// Frequency of the HLW8012 internal clock
#define F_OSC_HLW           (3579000)
#define F_OSC_BL0           (2000000)

// Maximum pulse with in microseconds
// If longer than this pulse width is reset to 0
// This value is purely experimental.
// Higher values allow for a better precission but reduce sampling rate
// and response speed to change
// Lower values increase sampling rate but reduce precission
// Values below 0.5s are not recommended since current and voltage output
// will have no time to stabilise
#define PULSE_TIMEOUT       (2000000)


// CF1 mode
typedef enum {
    MODE_CURRENT = 0,
    MODE_VOLTAGE
} hlw8012_mode_t;


void HLW8012_init(unsigned char cf_pin, unsigned char cf1_pin, unsigned char sel_pin, unsigned char currentWhen, uint8_t model);
// currentWhen  - 1 for HLW8012 (old Sonoff Pow), 0 for BL0937
// model - 0 for HLW8012, 1 or other value for BL0937

void HLW8012_setMode(hlw8012_mode_t mode);
hlw8012_mode_t HLW8012_getMode();
hlw8012_mode_t HLW8012_toggleMode();

float HLW8012_getCurrent(); // 
uint16_t HLW8012_getVoltage(); 
uint16_t HLW8012_getActivePower(); // moc czynna
uint16_t HLW8012_getApparentPower(); // moc pozorna
float HLW8012_getPowerFactor();
uint32_t HLW8012_getEnergy(); //in Ws
void HLW8012_resetEnergy();

void HLW8012_setResistors(float current, float voltage_upstream, float voltage_downstream);

void HLW8012_expectedCurrent(float current);
void HLW8012_expectedVoltage(uint16_t current);
void HLW8012_expectedActivePower(uint16_t power);

float HLW8012_getCurrentMultiplier();
float HLW8012_getVoltageMultiplier() ;
float HLW8012_getPowerMultiplier();

void HLW8012_setCurrentMultiplier(float current_multiplier);
void HLW8012_setVoltageMultiplier(float voltage_multiplier);
void HLW8012_setPowerMultiplier(float power_multiplier);
void HLW8012_resetMultipliers();
void HLW8012_set_calibrated_mutipliers (float *calibrated_current_multiplier, float *calibrated_voltage_multiplier, float *calibrated_power_multiplier, int load_voltage, int load_power) ;
#endif
