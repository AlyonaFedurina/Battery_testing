/*
 * Charge.h
 *
 *  Created on: May 26, 2023
 *      Author: Alyona
 */

#ifndef INC_CHARGE_H_
#define INC_CHARGE_H_

#include "stm32f3xx_hal.h"

#define ADC_BATVOLTAGE_4_V        166
#define DAC_DISABLE               0
#define DAC_150_MA_Charge         240
#define DAC_1000_MA_Charge        1500



void Check_Charge (const DAC_HandleTypeDef* hdaс, const float temperature, const uint16_t ADC_BatVoltage, const uint16_t current_value);
void Set_Charge_Current(const DAC_HandleTypeDef* hdaс, const float temperature, const uint16_t ADC_BatVoltage, uint16_t current_value);



#endif /* INC_CHARGE_H_ */
