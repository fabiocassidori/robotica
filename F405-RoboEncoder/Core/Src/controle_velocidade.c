/*
 * controle_velocidade.c
 *
 * Created on: Jun 5, 2025
 * Author: Fábio Couto
 */

#include "controle_velocidade.h"
#include "leitor_encoder.h"
#include "config_robo.h"
#include "main.h" // Incluído para usar HAL_GetTick()

static float g_velocidade_media_mmps = 0.0f;
// Variáveis para o PID de velocidade
static float g_erro_anterior_vel = 0.0f;
static float g_erro_integral_vel = 0.0f;

// Variáveis para rastrear a posição e o tempo anterior
static int32_t g_pos_anterior_vel_esq = 0;
static int32_t g_pos_anterior_vel_dir = 0;
static uint32_t g_tick_anterior_velocidade = 0; // Para medir o tempo real

void velocidade_inicializar(void) {
	g_velocidade_media_mmps = 0.0f;
	g_erro_anterior_vel = 0.0f;
	g_erro_integral_vel = 0.0f;

	// Inicializa as posições e o tempo com os valores atuais
	g_pos_anterior_vel_esq = encoder_obter_posicao_esquerda();
	g_pos_anterior_vel_dir = encoder_obter_posicao_direita();
	g_tick_anterior_velocidade = HAL_GetTick();
}

void velocidade_atualizar(void) {
    // --- Medição de Tempo Real ---
    uint32_t tick_atual = HAL_GetTick();
    uint32_t delta_ticks = tick_atual - g_tick_anterior_velocidade;

    // Se o tempo não passou (loop muito rápido), não faz nada para evitar divisão por zero.
    if (delta_ticks == 0) {
        return;
    }
    // Converte o delta de tempo de milissegundos para segundos
    float delta_t_s = (float)delta_ticks / 1000.0f;

	// Obtém a posição total atual dos encoders
	int32_t pos_atual_esq = encoder_obter_posicao_esquerda();
	int32_t pos_atual_dir = encoder_obter_posicao_direita();

	// Calcula a variação de pulsos desde a última medição
	int32_t delta_esq = pos_atual_esq - g_pos_anterior_vel_esq;
	int32_t delta_dir = pos_atual_dir - g_pos_anterior_vel_dir;
	int32_t delta_pulsos_media = (delta_dir + delta_esq) / 2;

	// Calcula a velocidade usando o delta_t REAL
	g_velocidade_media_mmps = ((float)delta_pulsos_media * PULSOS_PARA_MM) / delta_t_s;

	// Salva a posição e o tempo atuais para serem os "anteriores" no próximo ciclo
	g_pos_anterior_vel_esq = pos_atual_esq;
	g_pos_anterior_vel_dir = pos_atual_dir;
    g_tick_anterior_velocidade = tick_atual;
}

int velocidade_controlar_potencia(float velocidade_alvo_mmps) {
	float erro = velocidade_alvo_mmps - g_velocidade_media_mmps;

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
