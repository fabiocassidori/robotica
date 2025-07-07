/*
 * mpu9250.h
 *
 * Created on: Jul 7, 2025
 * Author: jlour
 */

#ifndef SRC_MPU9250_H_
#define SRC_MPU9250_H_

#include "stm32f4xx_hal.h"
#include <stdbool.h>

// Endereços dos Registradores do MPU9250
#define MPU9250_REG_ACCEL_XOUT_H    0x3B
#define MPU9250_REG_GYRO_YOUT_H     0x45
#define MPU9250_REG_PWR_MGMT_1      0x6B
#define MPU9250_REG_WHO_AM_I        0x75
#define MPU9250_REG_GYRO_CONFIG     0x1B
#define MPU9250_REG_ACCEL_CONFIG    0x1C

#define MPU9250_WHO_AM_I_VAL        0x71 // Valor esperado para o MPU9250

// Enum para o status da inicialização
typedef enum {
    MPU9250_OK = 0,
    MPU9250_ERRO_SPI,
    MPU9250_ERRO_ID_INVALIDO
} MPU9250_StatusTypeDef;

/**
 * @brief Inicializa o sensor MPU9250.
 * @param hspi Ponteiro para o handle do SPI (ex: &hspi3).
 * @return O status da operação (MPU9250_OK se bem-sucedido).
 */
MPU9250_StatusTypeDef mpu9250_inicializar(SPI_HandleTypeDef *hspi);

/**
 * @brief Lê os dados brutos do acelerômetro e giroscópio.
 */
void mpu9250_ler_sensores(void);

/**
 * @brief Retorna a velocidade angular do eixo Y (pitch) em graus por segundo.
 * @return Velocidade angular (dps).
 */
float mpu9250_obter_velocidade_angular_y(void);

/**
 * @brief Calcula e retorna o ângulo de inclinação (pitch) usando o acelerômetro.
 * @return Ângulo em graus.
 */
float mpu9250_obter_angulo_acelerometro(void);

#endif /* SRC_MPU9250_H_ */
