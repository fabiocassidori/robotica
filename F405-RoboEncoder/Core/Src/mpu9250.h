/*
 * mpu9250.h
 *
 *  Created on: Jul 6, 2025
 *      Author: jlour
 */

/*
 * mpu9250.h
 *
 * Created on: Jul 6, 2025
 * Author: jlour
 */

#ifndef SRC_MPU9250_H_
#define SRC_MPU9250_H_

// Inclui o cabeçalho principal da biblioteca HAL do STM32F4.
// Isso garante que todos os tipos de periféricos, como SPI_HandleTypeDef,
// e constantes (GPIO_PIN_RESET, etc.) sejam definidos e reconhecidos.
#include "stm32f4xx_hal.h"
#include <stdbool.h>

// Endereços dos Registradores do MPU9250 (lista parcial para a funcionalidade)
#define MPU9250_REG_ACCEL_XOUT_H    0x3B
#define MPU9250_REG_ACCEL_YOUT_H    0x3D
#define MPU9250_REG_ACCEL_ZOUT_H    0x3F
#define MPU9250_REG_GYRO_XOUT_H     0x43
#define MPU9250_REG_GYRO_YOUT_H     0x45
#define MPU9250_REG_GYRO_ZOUT_H     0x47

#define MPU9250_REG_PWR_MGMT_1      0x6B
#define MPU9250_REG_WHO_AM_I        0x75
#define MPU9250_REG_GYRO_CONFIG     0x1B
#define MPU9250_REG_ACCEL_CONFIG    0x1C
#define MPU9250_REG_ACCEL_CONFIG_2  0x1D

#define MPU9250_WHO_AM_I_VAL        0x71 // Ou 0x73 dependendo da revisão

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
 * Esta função deve ser chamada uma vez por ciclo de controle antes
 * de obter os valores processados.
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
