/*
 * config_robo.h
 *
 *  Created on: Jun 5, 2025
 *      Author: jlour
 */

#ifndef INC_CONFIG_ROBO_H_
#define INC_CONFIG_ROBO_H_

#include <stdint.h>
#include <stdbool.h>

// =============================================================================
// CONFIGURAÇÕES GERAIS E FÍSICAS
// =============================================================================
#define NUM_SENSORES_LINHA 6
#define CICLOS_CALIBRACAO 100       // Quantidade de leituras para calibrar o sensor de linha
#define INTERVALO_LOOP_CONTROLE_MS 5.0f // Intervalo do loop de controle (TIM6), em milissegundos
#define INTERVALO_LOOP_CONTROLE_S 0.005f  // Intervalo em segundos

// --- Física do Robô ---
#define DIAMETRO_RODA_MM 22.2f
#define CONSTANTE_PI 3.1415926535f
#define ENCODER_PPR 120              // Pulsos por revolução do encoder
#define CONSTANTE_DISTANCIA 0.335f     // Constante de conversão de pulsos para distância de mapa

// =============================================================================
// CONFIGURAÇÕES DO CONTROLE E MOTORES
// =============================================================================
// --- Potência e Velocidade ---
#define POTENCIA_INICIAL 80
#define POTENCIA_MAX_MOTOR 255
#define PERIODO_PWM_MOTOR 5599         // Período do registrador do TIM8 para PWM
#define POTENCIA_RETA 245
#define POTENCIA_CURVA 80

// --- PID do Seguidor de Linha ---
#define PID_SETPOINT 0
#define PID_KP 300
#define PID_KD 900
#define PID_KI 0
#define PID_DIVISOR 100
#define PID_LIMITE_SAIDA 250

// =============================================================================
// CONFIGURAÇÕES DOS SENSORES
// =============================================================================
#define ADC_VALOR_MAXIMO 1023.0f

// --- Sensores Laterais (Marcadores) ---
#define LIMIAR_SENSOR_LATERAL 600    // Valor ADC para considerar uma marcação detectada

// =============================================================================
// CONFIGURAÇÕES DO MAPEAMENTO
// =============================================================================
#define TAMANHO_MAPA 50
#define LIMIAR_CURVA_RETA 300 // Raio calculado a partir do qual se considera uma reta

// =============================================================================
// CONFIGURAÇÕES DA BATERIA
// =============================================================================
#define BATERIA_ADC_TENSAO_REF 3.3f
#define BATERIA_R1_DIV 10000.0f
#define BATERIA_R2_DIV 33000.0f
#define BATERIA_RELACAO_DIVISOR (BATERIA_R1_DIV / (BATERIA_R1_DIV + BATERIA_R2_DIV))
#define BATERIA_MULTIPLICADOR_TENSAO (1.0f / BATERIA_RELACAO_DIVISOR)
#define LIMIAR_BATERIA_FRACA_VOLTS 6.80f // Para 2S. Mudar para ~10.5V para 3S.
#define BATERIA_FILTRO_ALPHA 0.1f
#define BATERIA_INTERVALO_MONITOR_MS 200

#endif
