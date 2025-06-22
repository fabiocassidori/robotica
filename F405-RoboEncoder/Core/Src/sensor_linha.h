/*
 * sensor_linha.h
 *
 *  Created on: Jun 5, 2025
 *      Author: jlour
 */

#ifndef INC_SENSOR_LINHA_H_
#define INC_SENSOR_LINHA_H_

#include "stm32f4xx_hal.h"
#include <stdbool.h>

void sensor_linha_inicializar(ADC_HandleTypeDef *hadc);
void sensor_linha_calibrar(void);
int sensor_linha_ler_posicao(void);
bool sensor_linha_esta_na_linha(void);
int sensor_linha_obter_ultimo_erro(void);
volatile uint16_t* sensor_linha_obter_buffer_dma(void);

#endif /* INC_SENSOR_LINHA_H_ */
