/*
 * controle_velocidade.c
 *
 *  Created on: Jun 5, 2025
 *      Author: jlour
 */


#include "controle_velocidade.h"
#include "leitor_encoder.h"
#include "config_robo.h"

static int32_t g_pos_anterior_esq = 0;
static int32_t g_pos_anterior_dir = 0;

static float g_velocidade_esq_mmps = 0.0f;
static float g_velocidade_dir_mmps = 0.0f;
static float g_velocidade_media_mmps = 0.0f;

void velocidade_inicializar(void) {
    g_pos_anterior_esq = encoder_obter_posicao_esquerda();
    g_pos_anterior_dir = encoder_obter_posicao_direita();
}

void velocidade_atualizar(void) {
    int32_t pos_atual_dir = encoder_obter_posicao_direita();
    int32_t pos_atual_esq = encoder_obter_posicao_esquerda();

    int32_t delta_pulsos_dir = pos_atual_dir - g_pos_anterior_dir;
    int32_t delta_pulsos_esq = pos_atual_esq - g_pos_anterior_esq;

    const float dist_por_pulso_mm = (DIAMETRO_RODA_MM * CONSTANTE_PI) / (float)ENCODER_PPR;

    g_velocidade_dir_mmps = ((float)delta_pulsos_dir * dist_por_pulso_mm) / INTERVALO_LOOP_CONTROLE_S;
    g_velocidade_esq_mmps = ((float)delta_pulsos_esq * dist_por_pulso_mm) / INTERVALO_LOOP_CONTROLE_S;
    g_velocidade_media_mmps = (g_velocidade_dir_mmps + g_velocidade_esq_mmps) / 2.0f;

    g_pos_anterior_dir = pos_atual_dir;
    g_pos_anterior_esq = pos_atual_esq;
}

float velocidade_obter_media_mmps(void) {
    return g_velocidade_media_mmps;
}

int velocidade_rampa_potencia(int potencia_atual, int potencia_alvo) {
    int nova_potencia = potencia_atual;
    if (nova_potencia < potencia_alvo) {
        nova_potencia++;
    } else if (nova_potencia > potencia_alvo) {
        nova_potencia--;
    }
    if (nova_potencia > POTENCIA_MAX_MOTOR) nova_potencia = POTENCIA_MAX_MOTOR;
    if (nova_potencia < 0) nova_potencia = 0;
    return nova_potencia;
}
