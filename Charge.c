/*
 * Charge.c
 *
 *  Created on: May 26, 2023
 *      Author: Alyona
 */
#include <Charge.h>

void Check_Charge (const DAC_HandleTypeDef* hdac, const float temperature, const uint16_t ADC_BatVoltage, const uint16_t current_value){
	if(ADC_BatVoltage > ADC_BATVOLTAGE_4_V || temperature > 50.f ){
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_SET);
		HAL_DAC_Stop(hdac, DAC_CHANNEL_1);
		HAL_DAC_Start(hdac, DAC_CHANNEL_1);
		HAL_DAC_SetValue(hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, DAC_DISABLE);
	}
}

void Set_Charge_Current(const DAC_HandleTypeDef* hdac, const float temperature, const uint16_t ADC_BatVoltage, const uint16_t current_value) {
	if(ADC_BatVoltage <= ADC_BATVOLTAGE_4_V && temperature <= 50.f ){
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_10, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, GPIO_PIN_SET); // индикатор зарядки
		HAL_DAC_Stop(hdac, DAC_CHANNEL_1);
		HAL_DAC_Start(hdac, DAC_CHANNEL_1);
		HAL_DAC_SetValue(hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, DAC_150_MA_Charge);
	}
}
