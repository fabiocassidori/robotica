/*
 * mpu9250.c
 *
 * Created on: Jul 6, 2025
 * Author: jlour
 */

#include "mpu9250.h"
#include "main.h"
#include <math.h>
#include <stdio.h>

// ... (definições de constantes permanecem as mesmas) ...
#define GRAVIDADE 9.80665f
#define RAD_PARA_GRAUS 57.295779513f

static SPI_HandleTypeDef *g_hspi;
static float g_sensibilidade_giro;
static float g_sensibilidade_acel;
static int16_t g_acel_raw_x, g_acel_raw_y, g_acel_raw_z;
static int16_t g_giro_raw_x, g_giro_raw_y, g_giro_raw_z;
static float g_offset_giro_y = 0.0f;

// --- Funções de comunicação SPI (permanecem as mesmas) ---
static void mpu9250_cs_selecionar(void) {
    HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_RESET);
}

static void mpu9250_cs_desslecionar(void) {
    HAL_GPIO_WritePin(NSS_GPIO_Port, NSS_Pin, GPIO_PIN_SET);
}

static void mpu9250_escrever_registro(uint8_t reg, uint8_t data) {
    uint8_t tx_buffer[2] = {reg, data};
    mpu9250_cs_selecionar();
    HAL_SPI_Transmit(g_hspi, tx_buffer, 2, HAL_MAX_DELAY);
    mpu9250_cs_desslecionar();
}

static void mpu9250_ler_registros(uint8_t reg, uint8_t *buffer, uint8_t len) {
    uint8_t tx_data = reg | 0x80;
    mpu9250_cs_selecionar();
    HAL_SPI_Transmit(g_hspi, &tx_data, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(g_hspi, buffer, len, HAL_MAX_DELAY);
    mpu9250_cs_desslecionar();
}

// --- Função Pública de Inicialização (COM DIAGNÓSTICO) ---

MPU9250_StatusTypeDef mpu9250_inicializar(SPI_HandleTypeDef *hspi) {
    g_hspi = hspi;
    uint8_t who_am_i = 0;

    // --- CORREÇÃO E DIAGNÓSTICO ---
    // 1. Tenta resetar o dispositivo primeiro
    mpu9250_escrever_registro(MPU9250_REG_PWR_MGMT_1, 0x80); // Escreve 1 no bit de reset
    HAL_Delay(100); // Aguarda o reset

    // 2. Acorda o sensor selecionando uma fonte de clock
    mpu9250_escrever_registro(MPU9250_REG_PWR_MGMT_1, 0x01); // Usa o giroscópio como referência de clock
    HAL_Delay(100);

    // 3. Verifica a identidade do sensor e imprime o valor lido
    printf("Tentando ler o registrador WHO_AM_I...\r\n");
    mpu9250_ler_registros(MPU9250_REG_WHO_AM_I, &who_am_i, 1);

    // Imprime o valor lido em hexadecimal para fácil comparação com o datasheet
    printf("Valor lido do WHO_AM_I: 0x%02X\r\n", who_am_i);

    // Compara com os valores conhecidos
    if (who_am_i != MPU9250_WHO_AM_I_VAL && who_am_i != 0x73 && who_am_i != 0x68) {
        printf("ERRO: ID do dispositivo (0x%02X) e invalido!\r\n", who_am_i);
        printf("Valores esperados: 0x71, 0x73 ou 0x68.\r\n");
        printf("Verifique a fiacao, solda e alimentacao do sensor.\r\n");
        return MPU9250_ERRO_ID_INVALIDO;
    }

    printf("ID do dispositivo correto (0x%02X). Sensor detectado!\r\n", who_am_i);
    // --- FIM DA CORREÇÃO ---

    // 4. Configura o Giroscópio e o Acelerômetro (código original)
    uint8_t config_giro = (1 << 3);
    mpu9250_escrever_registro(MPU9250_REG_GYRO_CONFIG, config_giro);
    g_sensibilidade_giro = 65.5f;

    uint8_t config_acel = (1 << 3);
    mpu9250_escrever_registro(MPU9250_REG_ACCEL_CONFIG, config_acel);
    g_sensibilidade_acel = 8192.0f;

    // 5. Calibração do Giroscópio (código original)
    printf("Iniciando calibracao do giroscopio. Mantenha o robo parado...\r\n");
    long temp_offset = 0;
    int num_amostras_calibracao = 500;
    for (int i = 0; i < num_amostras_calibracao; i++) {
        mpu9250_ler_sensores();
        temp_offset += g_giro_raw_y;
        HAL_Delay(3); // Aumentar um pouco o delay pode dar mais estabilidade à leitura
    }
    g_offset_giro_y = (float)temp_offset / num_amostras_calibracao;
    printf("Calibracao finalizada. Offset Y: %f\r\n", g_offset_giro_y);

    return MPU9250_OK;
}

void mpu9250_ler_sensores(void) {
    uint8_t buffer[14];
    mpu9250_ler_registros(MPU9250_REG_ACCEL_XOUT_H, buffer, 14);
    g_acel_raw_x = (int16_t)(buffer[0] << 8 | buffer[1]);
    g_acel_raw_y = (int16_t)(buffer[2] << 8 | buffer[3]);
    g_acel_raw_z = (int16_t)(buffer[4] << 8 | buffer[5]);
    g_giro_raw_x = (int16_t)(buffer[8] << 8 | buffer[9]);
    g_giro_raw_y = (int16_t)(buffer[10] << 8 | buffer[11]);
    g_giro_raw_z = (int16_t)(buffer[12] << 8 | buffer[13]);
}

float mpu9250_obter_velocidade_angular_y(void) {
    return ((float)g_giro_raw_y - g_offset_giro_y) / g_sensibilidade_giro;
}

float mpu9250_obter_angulo_acelerometro(void) {
    float angulo_rad = atan2f((float)g_acel_raw_x, (float)g_acel_raw_z);
    return angulo_rad * RAD_PARA_GRAUS;
}
