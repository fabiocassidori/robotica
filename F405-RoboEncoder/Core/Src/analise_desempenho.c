/*
 * analise_desempenho.c
 *
 * Created on: Jul 8, 2025
 * Author: jlour
 * Refatorado para estabilizar medição de aceleração.
 */

#include "analise_desempenho.h"
#include "controle_velocidade.h"
#include "config_robo.h"
#include <math.h>
#include "main.h"

// --- NOVAS CONSTANTES PARA FILTRAGEM E CONTROLE DE TEMPO ---

// Fator de suavização para o filtro passa-baixa da velocidade.
// Valores menores suavizam mais, mas aumentam o atraso. 0.1 a 0.2 é um bom começo.
#define FILTRO_VELOCIDADE_ALPHA 0.15f

// Intervalo em milissegundos para calcular a aceleração. Aumentar este valor
// torna a medição mais estável e menos suscetível a ruído.
#define INTERVALO_CALCULO_ACELERACAO_MS 10

// Número de cálculos de aceleração a serem ignorados no início para estabilização.
#define CALCS_DE_GRACA 3 // Ignora os primeiros 3 cálculos (30ms)

// --- Variáveis estáticas ---
static float g_velocidade_filtrada_anterior_mmps = 0.0f;
static float g_velocidade_filtrada_atual_mmps = 0.0f;
static float g_max_aceleracao_mmps2 = 0.0f;
static float g_max_desaceleracao_mmps2 = 0.0f;
static uint32_t g_tick_anterior_calculo = 0;
static int g_contador_ciclos_descartados = 0;
static int g_contador_ms_para_calculo = 0;

void analise_desempenho_inicializar(void) {
    g_velocidade_filtrada_anterior_mmps = 0.0f;
    g_velocidade_filtrada_atual_mmps = 0.0f;
    g_max_aceleracao_mmps2 = 0.0f;
    g_max_desaceleracao_mmps2 = 0.0f;
    g_tick_anterior_calculo = HAL_GetTick();
    g_contador_ciclos_descartados = 0;
    g_contador_ms_para_calculo = 0;
}

// Esta função deve ser chamada a cada 1ms
void analise_desempenho_atualizar(void) {
    // 1. Obtém a velocidade instantânea (pode ser ruidosa)
    float velocidade_instantanea_mmps = velocidade_obter_media_mmps();

    // 2. Aplica o filtro passa-baixa para suavizar a velocidade.
    // Isso é feito a cada 1ms para manter o filtro atualizado.
    g_velocidade_filtrada_atual_mmps = (FILTRO_VELOCIDADE_ALPHA * velocidade_instantanea_mmps) +
                                       (1.0f - FILTRO_VELOCIDADE_ALPHA) * g_velocidade_filtrada_atual_mmps;

    // 3. Incrementa o contador para controlar o intervalo de cálculo
    g_contador_ms_para_calculo++;
    if (g_contador_ms_para_calculo < INTERVALO_CALCULO_ACELERACAO_MS) {
        // Ainda não é hora de calcular, apenas retorna.
        return;
    }

    // --- Hora de calcular a aceleração (a cada INTERVALO_CALCULO_ACELERACAO_MS) ---
    g_contador_ms_para_calculo = 0; // Reseta o contador

    uint32_t tick_atual = HAL_GetTick();
    uint32_t delta_ticks = tick_atual - g_tick_anterior_calculo;

    // Prevenção contra divisão por zero, embora improvável com o intervalo de 10ms.
    if (delta_ticks == 0) {
        return;
    }
    float delta_t_s = (float)delta_ticks / 1000.0f;

    // 4. Calcula a aceleração usando a VELOCIDADE FILTRADA
    float aceleracao_atual = (g_velocidade_filtrada_atual_mmps - g_velocidade_filtrada_anterior_mmps) / delta_t_s;

    // --- LÓGICA DO PERÍODO DE GRAÇA (ignora os primeiros cálculos) ---
    if (g_contador_ciclos_descartados < CALCS_DE_GRACA) {
        g_contador_ciclos_descartados++;
    } else {
        // 5. Atualiza os valores máximos APENAS APÓS o período de graça
        if (aceleracao_atual > g_max_aceleracao_mmps2) {
            g_max_aceleracao_mmps2 = aceleracao_atual;
        }

        // Usa fabsf para obter o valor absoluto para comparação
        if (aceleracao_atual < 0 && fabsf(aceleracao_atual) > g_max_desaceleracao_mmps2) {
            g_max_desaceleracao_mmps2 = fabsf(aceleracao_atual);
        }
    }

    // 6. SEMPRE atualiza o estado para a próxima iteração.
    g_velocidade_filtrada_anterior_mmps = g_velocidade_filtrada_atual_mmps;
    g_tick_anterior_calculo = tick_atual;
}

float analise_desempenho_obter_max_desaceleracao(void) {
    return g_max_desaceleracao_mmps2;
}

float analise_desempenho_obter_max_aceleracao(void){
    return g_max_aceleracao_mmps2;
}
