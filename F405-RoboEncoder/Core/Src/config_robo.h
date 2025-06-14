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
#define INTERVALO_LOOP_CONTROLE_S 0.001f  // Intervalo em segundos

// --- Física do Robô ---
#define DIAMETRO_RODA_MM 22.2f
#define CONSTANTE_PI 3.1415926535f
#define ENCODER_PPR 120              // Pulsos por revolução do encoder
// Converte pulsos do encoder para milímetros. (DIAMETRO_RODA_MM * PI) / ENCODER_PPR
#define PULSOS_PARA_MM 0.5812f
// =============================================================================
// CONFIGURAÇÕES DO CONTROLE E MOTORES
// =============================================================================
// --- Potência e Velocidade ---
#define POTENCIA_INICIAL 80
#define POTENCIA_MAX_MOTOR 255
#define PERIODO_PWM_MOTOR 3359         // Período do registrador do TIM8 para PWM 50khz
#define POTENCIA_RETA 245
#define POTENCIA_CURVA 80

// --- PID do Seguidor de Linha ---
#define PID_SETPOINT 0
#define PID_KP 310
#define PID_KD 2900
#define PID_KI 0

// --- NOVAS DEFINIÇÕES PARA PID EM RETAS ---
#define PID_KP_RETA 140      // Ganho Proporcional para retas de alta velocidade
#define PID_KD_RETA 4500     // Ganho Derivativo para retas de alta velocidade

#define PID_DIVISOR 100
#define PID_LIMITE_SAIDA 250

// =============================================================================
// CONFIGURAÇÕES DOS SENSORES
// =============================================================================
#define ADC_VALOR_MAXIMO 1023.0f

// --- Sensores Laterais (Marcadores) ---
#define LIMIAR_SENSOR_LATERAL 600    // Valor ADC para considerar uma marcação detectada
// Número de ciclos (de 10ms cada) que o sensor precisa estar ativo para confirmar uma marcação.
#define MARCADOR_CICLOS_DETECCAO 2
// Número de ciclos (de 10ms cada) que o sensor precisa estar inativo para confirmar o fim da marcação.
#define MARCADOR_CICLOS_LIBERACAO 1
// =============================================================================
// CONFIGURAÇÕES DO MAPEAMENTO
// =============================================================================
#define TAMANHO_MAPA 50
// ADICIONE A NOVA LINHA ABAIXO:
// Distância (em unidades de mapa) a partir da qual uma reta é considerada "longa"
// o suficiente para aplicar potência máxima. (Valor ~50cm)
#define LIMIAR_DISTANCIA_RETA_LONGA 500
#define LIMIAR_CURVA_RETA 300 // Raio calculado a partir do qual se considera uma reta
// Passo de incremento/decremento da potência na rampa. Valor maior = aceleração/frenagem mais rápidas.
#define PASSO_RAMPA_POTENCIA 5
#define PORCENTAGEM_PULSOS_EM_VRETA 50
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
