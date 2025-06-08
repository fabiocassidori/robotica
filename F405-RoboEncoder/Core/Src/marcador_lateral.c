/*
 * marcador_lateral.c
 *
 * VERSÃO COM LÓGICA DE CALIBRAÇÃO AUTOMÁTICA
 */

#include "marcador_lateral.h"
#include "config_robo.h"

// Enum e variáveis da máquina de estados (inalterado)
typedef enum { ESTADO_OCIOSO, ESTADO_EM_DETECCAO, ESTADO_DE_ESPERA } EstadoMarcador;
static EstadoMarcador g_estado_marcador = ESTADO_OCIOSO;
static bool g_viu_direito_neste_evento = false;
static bool g_viu_esquerdo_neste_evento = false;
static volatile uint16_t* g_adc_buffer = NULL;

// --- NOVAS Variáveis para a Calibração ---
static uint16_t g_calib_min_dir = 4095, g_calib_max_dir = 0;
static uint16_t g_calib_min_esq = 4095, g_calib_max_esq = 0;

// Limiares dinâmicos que serão calculados pela calibração
static uint16_t g_limiar_calibrado_dir = LIMIAR_SENSOR_LATERAL;
static uint16_t g_limiar_calibrado_esq = LIMIAR_SENSOR_LATERAL;


void marcador_lateral_inicializar(volatile uint16_t* adc_dma_buffer) {
    g_adc_buffer = adc_dma_buffer;
    g_estado_marcador = ESTADO_OCIOSO;
    g_viu_direito_neste_evento = false;
    g_viu_esquerdo_neste_evento = false;

    // Prepara as variáveis para uma nova calibração
    g_calib_min_dir = 4095;
    g_calib_max_dir = 0;
    g_calib_min_esq = 4095;
    g_calib_max_esq = 0;

    g_limiar_calibrado_dir = LIMIAR_SENSOR_LATERAL;
    g_limiar_calibrado_esq = LIMIAR_SENSOR_LATERAL;
}

// --- NOVAS FUNÇÕES DE CALIBRAÇÃO ---
void marcador_lateral_calibrar_ciclo(void) {
    if (!g_adc_buffer) return;

    uint16_t val_dir = g_adc_buffer[0];
    uint16_t val_esq = g_adc_buffer[7];

    if (val_dir < g_calib_min_dir) g_calib_min_dir = val_dir;
    if (val_dir > g_calib_max_dir) g_calib_max_dir = val_dir;

    if (val_esq < g_calib_min_esq) g_calib_min_esq = val_esq;
    if (val_esq > g_calib_max_esq) g_calib_max_esq = val_esq;
}

void marcador_lateral_finalizar_calibracao(void) {
    g_limiar_calibrado_dir = g_calib_min_dir + ((g_calib_max_dir - g_calib_min_dir) / 2);
    g_limiar_calibrado_esq = g_calib_min_esq + ((g_calib_max_esq - g_calib_min_esq) / 2);

    if (g_limiar_calibrado_dir < 50 || g_limiar_calibrado_dir > 1000) {
        g_limiar_calibrado_dir = LIMIAR_SENSOR_LATERAL;
    }
    if (g_limiar_calibrado_esq < 50 || g_limiar_calibrado_esq > 1000) {
        g_limiar_calibrado_esq = LIMIAR_SENSOR_LATERAL;
    }
}
// --- FIM DAS NOVAS FUNÇÕES ---


TipoMarcador marcador_lateral_verificar(void) {
    if (!g_adc_buffer) {
        return MARCADOR_NENHUM;
    }

    bool dir_agora = (g_adc_buffer[0] < g_limiar_calibrado_dir);
    bool esq_agora = (g_adc_buffer[7] < g_limiar_calibrado_esq);

    TipoMarcador marcador_para_retorno = MARCADOR_NENHUM;

    // A lógica da máquina de estados permanece idêntica
    switch (g_estado_marcador) {
        case ESTADO_OCIOSO:
            if (dir_agora || esq_agora) {
                g_estado_marcador = ESTADO_EM_DETECCAO;
                g_viu_direito_neste_evento = dir_agora;
                g_viu_esquerdo_neste_evento = esq_agora;
            }
            break;
        case ESTADO_EM_DETECCAO:
            if (dir_agora) g_viu_direito_neste_evento = true;
            if (esq_agora) g_viu_esquerdo_neste_evento = true;
            if (!dir_agora && !esq_agora) {
                if (g_viu_esquerdo_neste_evento && g_viu_direito_neste_evento) {
                    marcador_para_retorno = MARCADOR_AMBOS;
                } else if (g_viu_esquerdo_neste_evento) {
                    marcador_para_retorno = MARCADOR_ESQUERDA;
                } else if (g_viu_direito_neste_evento) {
                    marcador_para_retorno = MARCADOR_DIREITA;
                }
                g_viu_direito_neste_evento = false;
                g_viu_esquerdo_neste_evento = false;
                g_estado_marcador = ESTADO_DE_ESPERA;
            }
            break;
        case ESTADO_DE_ESPERA:
            if (!dir_agora && !esq_agora) {
                g_estado_marcador = ESTADO_OCIOSO;
            }
            break;
    }

    return marcador_para_retorno;
}
