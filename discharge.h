/*!
 * discharge.h
 *
 *  Created on: May 13, 2023
 *      Author: Alyona
 */

#ifndef INC_DISCHARGE_H_
#define INC_DISCHARGE_H_

#include "stm32f3xx_hal.h"


#define ADC_BATVOLTAGE_2500_MV   105
#define DAC_DISABLE              0
#define DAC_50_MA_Discharge      83
#define DAC_100_MA_Discharge     165
#define DAC_400_MA_Discharge     655
#define DAC_1000_MA_Discharge    1500



void Check_Discharge (const DAC_HandleTypeDef* hdaс, const float temperature, const uint16_t ADC_BatVoltage, const uint16_t current_value);
void Set_Discharge_Current(const DAC_HandleTypeDef* hdaс, const float temperature, const uint16_t ADC_BatVoltage, uint16_t current_value);




#endif /* INC_DISCHARGE_H_ */
