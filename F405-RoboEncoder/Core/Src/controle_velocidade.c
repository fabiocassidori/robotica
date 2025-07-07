/*
 * controle_velocidade.c
 *
 * Created on: Jun 5, 2025
 * Author: Fábio Couto
 */

#include "controle_velocidade.h"
#include "leitor_encoder.h"
#include "config_robo.h"


static float g_velocidade_media_mmps = 0.0f;

// +++ VARIÁVEIS PARA O PID DE VELOCIDADE +++
static float g_erro_anterior_vel = 0.0f;
static float g_erro_integral_vel = 0.0f;

void velocidade_inicializar(void) {

	g_velocidade_media_mmps = 0.0f;
	g_erro_anterior_vel = 0.0f;
	g_erro_integral_vel = 0.0f;

}

void velocidade_atualizar(void) {
	// 1. Obtém a variação de pulsos diretamente do módulo do encoder.
	int32_t delta_pulsos_media = encoder_obter_delta_pulsos_media();

	// 2. Calcula a velocidade
	const float dist_por_pulso_mm = (DIAMETRO_RODA_MM * CONSTANTE_PI) / (float)ENCODER_PPR;
	g_velocidade_media_mmps = ((float)delta_pulsos_media * dist_por_pulso_mm) / INTERVALO_LOOP_CONTROLE_S;

	// Reseta os encoders para a próxima medição de 4ms
	encoder_resetar_posicoes();
}



// +++ FUNÇÃO: CONTROLADOR PID DE VELOCIDADE +++
/**
 * @brief Calcula a potência base do motor para atingir uma velocidade alvo.
 * @param velocidade_alvo_mmps A velocidade desejada em mm/s.
 * @return A potência (0-255) a ser usada como base para os motores.
 */
int velocidade_controlar_potencia(float velocidade_alvo_mmps) {

	float velocidade_atual_mmps = g_velocidade_media_mmps;

	float erro = velocidade_alvo_mmps - velocidade_atual_mmps;
	float termo_p = erro * PID_VEL_KP;

	g_erro_integral_vel += erro * INTERVALO_LOOP_CONTROLE_S; // Integração correta com o tempo
	if (g_erro_integral_vel > PID_VEL_LIMITE_INTEGRAL) g_erro_integral_vel = PID_VEL_LIMITE_INTEGRAL;
	if (g_erro_integral_vel < -PID_VEL_LIMITE_INTEGRAL) g_erro_integral_vel = -PID_VEL_LIMITE_INTEGRAL;
	float termo_i = g_erro_integral_vel * PID_VEL_KI;

	float derivativo = (erro - g_erro_anterior_vel) / INTERVALO_LOOP_CONTROLE_S;
	g_erro_anterior_vel = erro;
	float termo_d = derivativo * PID_VEL_KD;

	int32_t saida_pid = (int32_t)(termo_p + termo_i + termo_d); // Divisor agora é 1

	if (saida_pid > POTENCIA_MAX_MOTOR) saida_pid = POTENCIA_MAX_MOTOR;
	if (saida_pid < 0) saida_pid = 0;

	return (int)saida_pid;

}

float velocidade_obter_media_mmps(void) {
    return g_velocidade_media_mmps;
}
