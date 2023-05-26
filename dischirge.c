/*
 * dischirge.c
 *
 *  Created on: May 13, 2023
 *      Author: Alyona
 */
#include <discharge.h>

void Check_Discharge (const DAC_HandleTypeDef* hdac, const float temperature, const uint16_t ADC_BatVoltage, const uint16_t current_value){
	if(ADC_BatVoltage < 0 || temperature > 50.f ){
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_SET);
		HAL_DAC_Stop(hdac, DAC_CHANNEL_1);
		HAL_DAC_Start(hdac, DAC_CHANNEL_1);
		HAL_DAC_SetValue(hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, current_value);
	}

}

void Set_Discharge_Current(const DAC_HandleTypeDef* hdac, const float temperature, const uint16_t ADC_BatVoltage, const uint16_t current_value) {
	if(ADC_BatVoltage >= ADC_BATVOLTAGE_2500_MV && temperature <= 50.f ){
        HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_RESET);
	    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_9, GPIO_PIN_SET); // индикатор разрядки
	    HAL_DAC_Stop(hdac, DAC_CHANNEL_1);
	    HAL_DAC_Start(hdac, DAC_CHANNEL_1);
	    HAL_DAC_SetValue(hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, current_value);
	}
}











