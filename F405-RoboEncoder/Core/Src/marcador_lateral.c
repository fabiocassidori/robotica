/*
 * marcador_lateral.c
 *
 * VERSÃO FINAL COM LÓGICA DE ACUMULADOR DE EVENTO PARA DETECÇÃO ROBUSTA DE INTERSECÇÕES
 */

#include "marcador_lateral.h"
#include "config_robo.h"

// --- NOVA MÁQUINA DE ESTADOS E ACUMULADOR ---
typedef enum {
    ESTADO_MARCADOR_OCIOSO,  // Robô está fora de qualquer marcação
    ESTADO_MARCADOR_EM_EVENTO // Robô detectou o início de uma marcação
} EstadoMarcador;

static EstadoMarcador g_estado_marcador = ESTADO_MARCADOR_OCIOSO;

// Esta variável acumula o tipo de marcador visto durante um único evento
// 1 = viu só esquerda, 2 = viu só direita, 3 = viu ambos em algum momento
static int g_tipo_marcador_acumulado = 0;

// Ponteiro para o buffer ADC e variáveis de calibração permanecem
static volatile uint16_t* g_adc_buffer = NULL;
static uint16_t g_calib_min_dir = ADC_VALOR_MAXIMO, g_calib_max_dir = 0;
static uint16_t g_calib_min_esq = ADC_VALOR_MAXIMO, g_calib_max_esq = 0;
static uint16_t g_limiar_calibrado_dir = LIMIAR_SENSOR_LATERAL;
static uint16_t g_limiar_calibrado_esq = LIMIAR_SENSOR_LATERAL;

// Funções de inicialização e calibração não precisam de mudanças e podem ser mantidas
void marcador_lateral_resetar_calibracao(void) {
    g_calib_min_dir = (uint16_t)ADC_VALOR_MAXIMO; g_calib_max_dir = 0;
    g_calib_min_esq = (uint16_t)ADC_VALOR_MAXIMO; g_calib_max_esq = 0;
    g_limiar_calibrado_dir = LIMIAR_SENSOR_LATERAL;
    g_limiar_calibrado_esq = LIMIAR_SENSOR_LATERAL;
}

void marcador_lateral_inicializar(volatile uint16_t* adc_dma_buffer) {
	g_adc_buffer = adc_dma_buffer;
	marcador_lateral_resetar_calibracao();
    g_estado_marcador = ESTADO_MARCADOR_OCIOSO;
    g_tipo_marcador_acumulado = 0;
}

void marcador_lateral_calibrar_ciclo(void) { /* ...código inalterado... */ }
void marcador_lateral_finalizar_calibracao(void) { /* ...código inalterado... */ }


// --- FUNÇÃO DE VERIFICAÇÃO COMPLETAMENTE REESCRITA COM A NOVA LÓGICA ---
TipoMarcador marcador_lateral_verificar(void) {
    if (!g_adc_buffer) {
        return MARCADOR_NENHUM;
    }

    bool dir_agora = (g_adc_buffer[0] < g_limiar_calibrado_dir);
    bool esq_agora = (g_adc_buffer[7] < g_limiar_calibrado_esq);

    int estado_leitura_atual = 0;
    if (esq_agora) estado_leitura_atual |= 1; // Bit 0 para a esquerda
    if (dir_agora) estado_leitura_atual |= 2; // Bit 1 para a direita
    // Resultado: 0=Nenhum, 1=Esquerda, 2=Direita, 3=Ambos

    TipoMarcador marcador_para_retorno = MARCADOR_NENHUM;

    switch (g_estado_marcador) {
        case ESTADO_MARCADOR_OCIOSO:
            // Se qualquer marcador for detectado, inicia um novo evento
            if (estado_leitura_atual > 0) {
                // Inicia o acumulador com o primeiro estado visto
                g_tipo_marcador_acumulado = estado_leitura_atual;
                // Muda para o estado de evento
                g_estado_marcador = ESTADO_MARCADOR_EM_EVENTO;
            }
            break;

        case ESTADO_MARCADOR_EM_EVENTO:
            // Se ainda estamos vendo qualquer parte de uma marca...
            if (estado_leitura_atual > 0) {
                // ...acumulamos o tipo de marcador.
                // A operação OU (OR) garante que se virmos "ambos" (3) uma vez,
                // ou se virmos "esquerda" (1) e "direita" (2) em momentos diferentes,
                // o resultado final será 3.
                g_tipo_marcador_acumulado |= estado_leitura_atual;
            }
            // Se não vemos mais nenhuma marca, o evento terminou.
            else {
                // Decide qual foi o marcador com base no valor acumulado
                if (g_tipo_marcador_acumulado == 3) {
                    marcador_para_retorno = MARCADOR_AMBOS;
                } else if (g_tipo_marcador_acumulado == 2) {
                    marcador_para_retorno = MARCADOR_DIREITA;
                } else if (g_tipo_marcador_acumulado == 1) {
                    marcador_para_retorno = MARCADOR_ESQUERDA;
                }

                // Reseta tudo para o próximo ciclo
                g_tipo_marcador_acumulado = 0;
                g_estado_marcador = ESTADO_MARCADOR_OCIOSO;
            }
            break;
    }

    return marcador_para_retorno;
}
