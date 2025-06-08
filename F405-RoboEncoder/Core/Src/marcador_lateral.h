/*
 * marcador_lateral.h
 *
 *  Created on: Jun 5, 2025
 *      Author: jlour
 */

#ifndef INC_MARCADOR_LATERAL_H_
#define INC_MARCADOR_LATERAL_H_

#include "stm32f4xx_hal.h"
#include <stdbool.h>

typedef enum {
    MARCADOR_NENHUM,
    MARCADOR_ESQUERDA,
    MARCADOR_DIREITA,
    MARCADOR_AMBOS
} TipoMarcador;

void marcador_lateral_inicializar(volatile uint16_t* adc_dma_buffer);
TipoMarcador marcador_lateral_verificar(void);

#endif /* INC_MARCADOR_LATERAL_H_ */
