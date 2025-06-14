/*
 * marcador_lateral.c
 *
 * VERSÃO COM LÓGICA DE CALIBRAÇÃO AUTOMÁTICA
 */

#include "marcador_lateral.h"
#include "config_robo.h"

// Enum e variáveis da máquina de estados (inalterado)
typedef enum { ESTADO_OCIOSO, ESTADO_EM_DETECCAO } EstadoMarcador;
static EstadoMarcador g_estado_marcador = ESTADO_OCIOSO;
static bool g_viu_direito_neste_evento = false;
static bool g_viu_esquerdo_neste_evento = false;
static volatile uint16_t* g_adc_buffer = NULL;

static uint16_t g_contador_deteccao_esq = 0;
static uint16_t g_contador_deteccao_dir = 0;
static uint16_t g_contador_liberacao = 0;

// --- NOVAS Variáveis para a Calibração ---
static uint16_t g_calib_min_dir = ADC_VALOR_MAXIMO, g_calib_max_dir = 0;
static uint16_t g_calib_min_esq = ADC_VALOR_MAXIMO, g_calib_max_esq = 0;

// Limiares dinâmicos que serão calculados pela calibração
static uint16_t g_limiar_calibrado_dir = LIMIAR_SENSOR_LATERAL;
static uint16_t g_limiar_calibrado_esq = LIMIAR_SENSOR_LATERAL;

void marcador_lateral_resetar_calibracao(void) {
    // Prepara as variáveis para uma nova calibração
    g_calib_min_dir = (uint16_t)ADC_VALOR_MAXIMO;
    g_calib_max_dir = 0;
    g_calib_min_esq = (uint16_t)ADC_VALOR_MAXIMO;
    g_calib_max_esq = 0;

    // Reseta para o limiar padrão antes de calcular o novo
    g_limiar_calibrado_dir = LIMIAR_SENSOR_LATERAL;
    g_limiar_calibrado_esq = LIMIAR_SENSOR_LATERAL;
}

void marcador_lateral_inicializar(volatile uint16_t* adc_dma_buffer) {
	g_adc_buffer = adc_dma_buffer;
	g_estado_marcador = ESTADO_OCIOSO;
	g_viu_direito_neste_evento = false;
	g_viu_esquerdo_neste_evento = false;

	// Agora a inicialização apenas chama a função de reset
	marcador_lateral_resetar_calibracao();
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

    switch (g_estado_marcador) {
        case ESTADO_OCIOSO:
            // Incrementa o contador do sensor correspondente se ele estiver ativo, senão zera.
            if (esq_agora) g_contador_deteccao_esq++; else g_contador_deteccao_esq = 0;
            if (dir_agora) g_contador_deteccao_dir++; else g_contador_deteccao_dir = 0;

            // Se QUALQUER contador atingir o limiar de detecção, a marca é considerada real.
            if (g_contador_deteccao_esq >= MARCADOR_CICLOS_DETECCAO || g_contador_deteccao_dir >= MARCADOR_CICLOS_DETECCAO) {
                // Registra qual(is) sensor(es) disparou(aram) o evento.
                g_viu_esquerdo_neste_evento = (g_contador_deteccao_esq >= MARCADOR_CICLOS_DETECCAO);
                g_viu_direito_neste_evento = (g_contador_deteccao_dir >= MARCADOR_CICLOS_DETECCAO);

                // Zera os contadores e transita para o estado de detecção ativa.
                g_contador_deteccao_esq = 0;
                g_contador_deteccao_dir = 0;
                g_contador_liberacao = 0;
                g_estado_marcador = ESTADO_EM_DETECCAO;
            }
            break;

        case ESTADO_EM_DETECCAO:
            // Enquanto a detecção está ativa, acumula as flags caso o outro sensor seja ativado.
            if (esq_agora) g_viu_esquerdo_neste_evento = true;
            if (dir_agora) g_viu_direito_neste_evento = true;

            // Para finalizar a detecção, os sensores precisam ficar INATIVOS por um tempo.
            if (!esq_agora && !dir_agora) {
                g_contador_liberacao++; // Incrementa o contador de liberação.
            } else {
                g_contador_liberacao = 0; // Se a linha reaparecer, zera o contador.
            }

            // Se a linha sumiu pelo tempo necessário, o evento do marcador é finalmente retornado.
            if (g_contador_liberacao >= MARCADOR_CICLOS_LIBERACAO) {
                // Decide qual tipo de marcador foi visto com base nas flags acumuladas.
                if (g_viu_esquerdo_neste_evento && g_viu_direito_neste_evento) {
                    marcador_para_retorno = MARCADOR_AMBOS;
                } else if (g_viu_esquerdo_neste_evento) {
                    marcador_para_retorno = MARCADOR_ESQUERDA;
                } else if (g_viu_direito_neste_evento) {
                    marcador_para_retorno = MARCADOR_DIREITA;
                }

                // Reseta todas as flags e contadores e volta ao estado inicial.
                g_viu_esquerdo_neste_evento = false;
                g_viu_direito_neste_evento = false;
                g_contador_liberacao = 0;
                g_estado_marcador = ESTADO_OCIOSO;
            }
            break;
    }

    return marcador_para_retorno;
}
