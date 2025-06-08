/*
 * marcador_lateral.c
 *
 *  Created on: Jun 5, 2025
 *      Author: jlour
 */


#include "marcador_lateral.h"
#include "config_robo.h"

static volatile uint16_t* g_adc_buffer = NULL;
static TipoMarcador g_estado_marcador_anterior = MARCADOR_NENHUM;

void marcador_lateral_inicializar(volatile uint16_t* adc_dma_buffer) {
    g_adc_buffer = adc_dma_buffer;
}

TipoMarcador marcador_lateral_verificar(void) {
    if (!g_adc_buffer) return MARCADOR_NENHUM;

    bool direita_detectado = (g_adc_buffer[0] < LIMIAR_SENSOR_LATERAL);
    bool esquerda_detectado = (g_adc_buffer[7] < LIMIAR_SENSOR_LATERAL);

    TipoMarcador marcador_atual;
    if (esquerda_detectado && direita_detectado) {
        marcador_atual = MARCADOR_AMBOS;
    } else if (esquerda_detectado) {
        marcador_atual = MARCADOR_ESQUERDA;
    } else if (direita_detectado) {
        marcador_atual = MARCADOR_DIREITA;
    } else {
        marcador_atual = MARCADOR_NENHUM;
    }

    TipoMarcador evento_transicao = MARCADOR_NENHUM;
    if (marcador_atual != MARCADOR_NENHUM && g_estado_marcador_anterior == MARCADOR_NENHUM) {
        evento_transicao = marcador_atual;
    }

    g_estado_marcador_anterior = marcador_atual;
    return evento_transicao;
}
