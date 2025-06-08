/*
 * controle_motor.c
 *
 *  Created on: Jun 5, 2025
 *      Author: jlour
 */

// Core/Src/controle_motor.c

#include "controle_motor.h"
#include "config_robo.h"
#include "main.h" // <<< CORREÇÃO ADICIONADA AQUI
#include <stdlib.h>

static TIM_HandleTypeDef *g_htim_pwm = NULL;

static uint16_t interpolar(int valor, int entrada_min, int entrada_max, int saida_min, int saida_max);
static void motor_esquerdo_definir(int potencia);
static void motor_direito_definir(int potencia);

void motor_inicializar(TIM_HandleTypeDef *htim_pwm) {
    g_htim_pwm = htim_pwm;
    motor_definir_standby(true);
    HAL_TIM_PWM_Start(g_htim_pwm, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(g_htim_pwm, TIM_CHANNEL_2);
}

void motor_definir_standby(bool ligar) {
    HAL_GPIO_WritePin(STBY_GPIO_Port, STBY_Pin, ligar ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void motor_definir_potencia(int potencia_esquerda, int potencia_direita) {
    motor_esquerdo_definir(potencia_esquerda);
    motor_direito_definir(potencia_direita);
}

static uint16_t interpolar(int valor, int entrada_min, int entrada_max, int saida_min, int saida_max) {
    if (entrada_min == entrada_max) return saida_min;
    return (uint16_t)(((int32_t)(valor - entrada_min) * (saida_max - saida_min)) / (entrada_max - entrada_min) + saida_min);
}

static void motor_esquerdo_definir(int potencia) {
    if (potencia > 0) {
        HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(AIN2_GPIO_Port, AIN2_Pin, GPIO_PIN_SET);
    }
    int potencia_abs = abs(potencia);
    if (potencia_abs > POTENCIA_MAX_MOTOR) potencia_abs = POTENCIA_MAX_MOTOR;
    uint16_t valor_pwm = interpolar(potencia_abs, 0, POTENCIA_MAX_MOTOR, 0, PERIODO_PWM_MOTOR);
    __HAL_TIM_SET_COMPARE(g_htim_pwm, TIM_CHANNEL_1, valor_pwm);
}

static void motor_direito_definir(int potencia) {
    if (potencia > 0) {
        HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(BIN1_GPIO_Port, BIN1_Pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(BIN2_GPIO_Port, BIN2_Pin, GPIO_PIN_SET);
    }
    int potencia_abs = abs(potencia);
    if (potencia_abs > POTENCIA_MAX_MOTOR) potencia_abs = POTENCIA_MAX_MOTOR;
    uint16_t valor_pwm = interpolar(potencia_abs, 0, POTENCIA_MAX_MOTOR, 0, PERIODO_PWM_MOTOR);
    __HAL_TIM_SET_COMPARE(g_htim_pwm, TIM_CHANNEL_2, valor_pwm);
}
