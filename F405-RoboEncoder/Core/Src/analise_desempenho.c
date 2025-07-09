/*
 * analise_desempenho.c
 *
 * Created on: Jul 8, 2025
 * Author: jlour
 */

#include "analise_desempenho.h"
#include "controle_velocidade.h"
#include "config_robo.h"
#include <math.h>
#include <stdlib.h>

// O intervalo de cálculo DEVE ser o mesmo da janela de velocidade para consistência
#define INTERVALO_CALCULO_ACELERACAO_MS 10

static float g_velocidade_anterior_mmps = 0.0f;
static float g_max_aceleracao_mmps2 = 0.0f;
static float g_max_desaceleracao_mmps2 = 0.0f;
static float g_aceleracao_estimada_mmps2 = 0.0f;
static int g_contador_ciclos = 0;

void analise_desempenho_inicializar(void) {
    g_velocidade_anterior_mmps = 0.0f;
    g_max_aceleracao_mmps2 = 0.0f;
    g_max_desaceleracao_mmps2 = 0.0f;
    g_aceleracao_estimada_mmps2 = 0.0f;
    g_contador_ciclos = 0;
}

void analise_desempenho_atualizar(void) {
    // Esta função roda a cada 1ms.
    g_contador_ciclos++;

    // Aguardamos o intervalo definido para realizar o cálculo.
    if (g_contador_ciclos < INTERVALO_CALCULO_ACELERACAO_MS) {
        return;
    }
    // Já se passaram 10ms, hora de calcular.
    g_contador_ciclos = 0;

    // 1. Obtém a velocidade atual (que já é uma média móvel de 10ms)
    float velocidade_atual_mmps = velocidade_obter_media_mmps();

    // 2. Calcula a variação da velocidade ao longo dos últimos 10ms
    float delta_v = velocidade_atual_mmps - g_velocidade_anterior_mmps;

    // 3. Calcula a aceleração sobre esse intervalo de tempo
    const float intervalo_s = (float)INTERVALO_CALCULO_ACELERACAO_MS / 1000.0f;
    g_aceleracao_estimada_mmps2 = delta_v / intervalo_s;

    // 4. Atualiza os valores máximos
    if (g_aceleracao_estimada_mmps2 > g_max_aceleracao_mmps2) {
        g_max_aceleracao_mmps2 = g_aceleracao_estimada_mmps2;
    }
    if (g_aceleracao_estimada_mmps2 < 0 && fabsf(g_aceleracao_estimada_mmps2) > g_max_desaceleracao_mmps2) {
        g_max_desaceleracao_mmps2 = fabsf(g_aceleracao_estimada_mmps2);
    }

    // 5. Salva a velocidade atual para ser a "anterior" no próximo cálculo
    g_velocidade_anterior_mmps = velocidade_atual_mmps;
}

float analise_desempenho_obter_max_desaceleracao(void) {
    return g_max_desaceleracao_mmps2;
}
float analise_desempenho_obter_max_aceleracao(void){
	return g_max_aceleracao_mmps2;
}

float analise_desempenho_obter_aceleracao_instantanea(void){
	// Retorna a última estimativa de aceleração calculada
	return g_aceleracao_estimada_mmps2;
}
