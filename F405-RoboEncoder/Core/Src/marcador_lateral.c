/*
 * marcador_lateral.c
 *
 * Created on: Jun 5, 2025
 * Author: jlour
 *
 */

#include "marcador_lateral.h"
#include "config_robo.h"

// Variáveis estáticas para a máquina de estados de deteção
static int g_geo = 0;
static int g_geo1 = 0;
static int g_geo2 = 0;
static int g_geo3 = 0;
static int g_geo4 = 0; // Histórico mais antigo
static int g_geo5 = 0; // Não utilizado na lógica principal, mantido por fidelidade

static volatile uint16_t* g_adc_buffer = NULL;

void marcador_lateral_inicializar(volatile uint16_t* adc_dma_buffer) {
    g_adc_buffer = adc_dma_buffer;
    // Reseta o estado da máquina
    g_geo = g_geo1 = g_geo2 = g_geo3 = g_geo4 = g_geo5 = 0;
}

TipoMarcador marcador_lateral_verificar(void) {
    if (!g_adc_buffer) return MARCADOR_NENHUM;

    // 1. Determina o estado instantâneo dos marcadores (0: Nenhum, 1: Esq, 2: Dir, 3: Ambos)
    bool direita_detectado = (g_adc_buffer[0] < LIMIAR_SENSOR_LATERAL);
    bool esquerda_detectado = (g_adc_buffer[7] < LIMIAR_SENSOR_LATERAL);

    int geo_instantaneo = 0;
    if (esquerda_detectado && !direita_detectado) {
        geo_instantaneo = 1;
    } else if (!esquerda_detectado && direita_detectado) {
        geo_instantaneo = 2;
    } else if (esquerda_detectado && direita_detectado) {
        geo_instantaneo = 3;
    }

    TipoMarcador marcador_para_retorno = MARCADOR_NENHUM;

    // 2. A lógica de deteção só é acionada na MUDANÇA de estado
    if (g_geo1 != geo_instantaneo) {
        // Deteta o padrão de "borda de descida": o sensor estava sobre a marca e agora não está.
        // Padrão para Esquerda: esteve em '1' e agora voltou para '0'
        if (geo_instantaneo == 0 && g_geo1 == 1 && g_geo2 == 0) {
            marcador_para_retorno = MARCADOR_ESQUERDA;
        }
        // Padrão para Direita: esteve em '2' e agora voltou para '0'
        else if (geo_instantaneo == 0 && g_geo1 == 2 && g_geo2 == 0) {
            marcador_para_retorno = MARCADOR_DIREITA;
        }
        // Padrão para Intersecção: esteve em '3' recentemente e agora voltou para '0'
        else if (geo_instantaneo == 0 && (g_geo1 == 3 || g_geo2 == 3 || g_geo3 == 3)) {
            marcador_para_retorno = MARCADOR_AMBOS;
        }
    }

    // 3. Atualiza o histórico da máquina de estados para o próximo ciclo
    g_geo5 = g_geo4;
    g_geo4 = g_geo3;
    g_geo3 = g_geo2;
    g_geo2 = g_geo1;
    g_geo1 = geo_instantaneo;

    return marcador_para_retorno;
}
