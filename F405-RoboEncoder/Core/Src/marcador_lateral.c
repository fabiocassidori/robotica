/*
 * marcador_lateral.c
 *
 * Created on: Jun 5, 2025
 * Author: jlour
 * VERSÃO FINAL COM LÓGICA DE CONFIRMAÇÃO (DEBOUNCE)
 */

#include "marcador_lateral.h"
#include "config_robo.h"

// Variáveis estáticas para a lógica de confirmação
static int g_contador_confirmacao = 0;
static bool g_marcador_ja_processado = false;

static volatile uint16_t* g_adc_buffer = NULL;

void marcador_lateral_inicializar(volatile uint16_t* adc_dma_buffer) {
    g_adc_buffer = adc_dma_buffer;
    g_contador_confirmacao = 0;
    g_marcador_ja_processado = false;
}

TipoMarcador marcador_lateral_verificar(void) {
    if (!g_adc_buffer) return MARCADOR_NENHUM;

    bool direita_detectado = (g_adc_buffer[0] < LIMIAR_SENSOR_LATERAL);
    bool esquerda_detectado = (g_adc_buffer[7] < LIMIAR_SENSOR_LATERAL);

    // Se um dos sensores detetar a linha
    if (direita_detectado || esquerda_detectado) {
        // Incrementa o contador de confirmação até o limiar
        if (g_contador_confirmacao < CICLOS_CONFIRMACAO_MARCADOR) {
            g_contador_confirmacao++;
        }
    } else {
        // Se ambos os sensores estiverem fora da linha, reinicia o processo
        g_contador_confirmacao = 0;
        g_marcador_ja_processado = false;
    }

    TipoMarcador marcador_para_retorno = MARCADOR_NENHUM;

    // A deteção é VÁLIDA se o contador atingir o limiar
    // E o marcador ainda não tiver sido processado nesta passagem
    if (g_contador_confirmacao >= CICLOS_CONFIRMACAO_MARCADOR && !g_marcador_ja_processado) {
        g_marcador_ja_processado = true; // Impede leituras repetidas da MESMA marca

        // A lógica para determinar o TIPO de marcador permanece a mesma
        if (esquerda_detectado && direita_detectado) {
            marcador_para_retorno = MARCADOR_AMBOS;
        } else if (esquerda_detectado) {
            marcador_para_retorno = MARCADOR_ESQUERDA;
        } else if (direita_detectado) {
            marcador_para_retorno = MARCADOR_DIREITA;
        }
    }

    return marcador_para_retorno;
}
