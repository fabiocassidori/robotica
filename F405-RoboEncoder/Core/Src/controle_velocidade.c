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

// Variáveis para rastrear a posição anterior para o cálculo de velocidade.
static int32_t g_pos_anterior_vel_esq = 0;
static int32_t g_pos_anterior_vel_dir = 0;

void velocidade_inicializar(void) {

	g_velocidade_media_mmps = 0.0f;
	g_erro_anterior_vel = 0.0f;
	g_erro_integral_vel = 0.0f;
	// Inicializa as posições anteriores com a posição atual dos encoders.
	g_pos_anterior_vel_esq = encoder_obter_posicao_esquerda();
	g_pos_anterior_vel_dir = encoder_obter_posicao_direita();

}

void velocidade_atualizar(void) {
	// Obtém a posição total atual dos encoders.
	int32_t pos_atual_esq = encoder_obter_posicao_esquerda();
	int32_t pos_atual_dir = encoder_obter_posicao_direita();

	// Calcula o delta (variação) desde a última chamada desta função (1ms atrás).
	int32_t delta_esq = pos_atual_esq - g_pos_anterior_vel_esq;
	int32_t delta_dir = pos_atual_dir - g_pos_anterior_vel_dir;

	int32_t delta_pulsos_media = (delta_dir + delta_esq) / 2;

	// Calcula a velocidade com base no delta e no intervalo de tempo do loop rápido.
	const float dist_por_pulso_mm = (DIAMETRO_RODA_MM * CONSTANTE_PI) / (float)ENCODER_PPR;
	g_velocidade_media_mmps = ((float)delta_pulsos_media * dist_por_pulso_mm) / INTERVALO_LOOP_CONTROLE_S;

	// Salva a posição atual para ser a "anterior" no próximo ciclo.
	g_pos_anterior_vel_esq = pos_atual_esq;
	g_pos_anterior_vel_dir = pos_atual_dir;
}



// +++ FUNÇÃO: CONTROLADOR PID DE VELOCIDADE +++
/**
 * @brief Calcula a potência base do motor para atingir uma velocidade alvo.
 * @param velocidade_alvo_mmps A velocidade desejada em mm/s.
 * @return A potência (0-255) a ser usada como base para os motores.
 */
int velocidade_controlar_potencia(float velocidade_alvo_mmps) {

	float erro = velocidade_alvo_mmps - g_velocidade_media_mmps;

	// Termo integral agora é acumulado sem multiplicação pelo tempo, como na versão estável.
	g_erro_integral_vel += erro;
	if (g_erro_integral_vel > PID_VEL_LIMITE_INTEGRAL) g_erro_integral_vel = PID_VEL_LIMITE_INTEGRAL;
	if (g_erro_integral_vel < -PID_VEL_LIMITE_INTEGRAL) g_erro_integral_vel = -PID_VEL_LIMITE_INTEGRAL;

	float termo_p = erro * PID_VEL_KP;
	float termo_i = g_erro_integral_vel * PID_VEL_KI;

	float derivativo = erro - g_erro_anterior_vel;
	g_erro_anterior_vel = erro;
	float termo_d = derivativo * PID_VEL_KD;

	int32_t saida_pid = (int32_t)((termo_p + termo_i + termo_d) / PID_VEL_DIVISOR);

	if (saida_pid > POTENCIA_MAX_MOTOR) saida_pid = POTENCIA_MAX_MOTOR;
	if (saida_pid < 0) saida_pid = 0;

	return (int)saida_pid;
}

float velocidade_obter_media_mmps(void) {
    return g_velocidade_media_mmps;
}
