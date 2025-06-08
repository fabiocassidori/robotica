/*
 * marcador_lateral.c
 *
 * Author: jlour
 * VERSÃO FINAL COM MÁQUINA DE ESTADOS ROBUSTA E COOLDOWN
 * Garante detecção única por evento e correção para cruzamentos.
 */

#include "marcador_lateral.h"
#include "config_robo.h"

// Enum para os estados da máquina de detecção de marcadores
typedef enum {
    ESTADO_OCIOSO,       // Aguardando qualquer marcador
    ESTADO_EM_DETECCAO,  // Atualmente sobre um marcador, acumulando dados
    ESTADO_DE_ESPERA     // Estado de "cooldown" para evitar detecções múltiplas (bounce)
} EstadoMarcador;

// Variáveis estáticas para a máquina de estados
static EstadoMarcador g_estado_marcador = ESTADO_OCIOSO;
static bool g_viu_direito_neste_evento = false;
static bool g_viu_esquerdo_neste_evento = false;

static volatile uint16_t* g_adc_buffer = NULL;

void marcador_lateral_inicializar(volatile uint16_t* adc_dma_buffer) {
    g_adc_buffer = adc_dma_buffer;
    g_estado_marcador = ESTADO_OCIOSO;
    g_viu_direito_neste_evento = false;
    g_viu_esquerdo_neste_evento = false;
}

TipoMarcador marcador_lateral_verificar(void) {
    if (!g_adc_buffer) {
        return MARCADOR_NENHUM;
    }

    bool dir_agora = (g_adc_buffer[0] < LIMIAR_SENSOR_LATERAL);
    bool esq_agora = (g_adc_buffer[7] < LIMIAR_SENSOR_LATERAL);

    TipoMarcador marcador_para_retorno = MARCADOR_NENHUM;

    switch (g_estado_marcador) {
        case ESTADO_OCIOSO:
            // Inicia um novo evento se qualquer sensor encontrar a linha
            if (dir_agora || esq_agora) {
                g_estado_marcador = ESTADO_EM_DETECCAO;

                // Registra o(s) sensor(es) visto(s) no início do evento
                g_viu_direito_neste_evento = dir_agora;
                g_viu_esquerdo_neste_evento = esq_agora;
            }
            break;

        case ESTADO_EM_DETECCAO:
            // Enquanto o evento durar, acumula as detecções de ambos os sensores
            if (dir_agora) {
                g_viu_direito_neste_evento = true;
            }
            if (esq_agora) {
                g_viu_esquerdo_neste_evento = true;
            }

            // O evento termina APENAS quando o robô está completamente fora da linha
            if (!dir_agora && !esq_agora) {

                // Processa o resultado do evento que acabou de terminar
                if (g_viu_esquerdo_neste_evento && g_viu_direito_neste_evento) {
                    marcador_para_retorno = MARCADOR_AMBOS;
                } else if (g_viu_esquerdo_neste_evento) {
                    marcador_para_retorno = MARCADOR_ESQUERDA;
                } else if (g_viu_direito_neste_evento) {
                    marcador_para_retorno = MARCADOR_DIREITA;
                }

                // Reseta as flags para o próximo evento
                g_viu_direito_neste_evento = false;
                g_viu_esquerdo_neste_evento = false;

                // Transiciona para o estado de ESPERA para garantir que esta marca não seja lida novamente
                g_estado_marcador = ESTADO_DE_ESPERA;
            }
            break;

        case ESTADO_DE_ESPERA:
            // O sistema só volta ao estado OCIOSO após um ciclo confirmando que está fora da linha.
            // Isso previne que um ruído no sensor acione um novo evento imediatamente.
            if (!dir_agora && !esq_agora) {
                g_estado_marcador = ESTADO_OCIOSO;
            }
            break;
    }

    return marcador_para_retorno;
}
