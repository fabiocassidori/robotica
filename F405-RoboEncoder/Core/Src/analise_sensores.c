/*
 * analise_sensores.c
 *
 * Created on: Jul 7, 2025
 * Author: jlour
 */

#include "analise_sensores.h"
#include "config_robo.h"
#include "mpu9250.h"
#include <stdio.h>
#include <math.h>

#define NUM_AMOSTRAS_ANALISE 1000

static float g_amostras_omega[NUM_AMOSTRAS_ANALISE];
static float g_amostras_theta[NUM_AMOSTRAS_ANALISE];

static float calcular_media(float* dados, int tamanho) {
    float soma = 0.0f;
    for (int i = 0; i < tamanho; i++) {
        soma += dados[i];
    }
    return soma / tamanho;
}

static float calcular_variancia(float* dados, float media, int tamanho) {
    float soma_diff_quadrada = 0.0f;
    for (int i = 0; i < tamanho; i++) {
        float diff = dados[i] - media;
        soma_diff_quadrada += diff * diff;
    }
    return soma_diff_quadrada / (tamanho - 1);// Variância amostral
}

void analisar_variancia_sensores(void) {
    printf("--- Iniciando Analise de Variancia dos Sensores ---\r\n");
    printf("Mantenha o robo parado na posicao vertical.\r\n");
    printf("Coletando %d amostras...\r\n", NUM_AMOSTRAS_ANALISE);
    HAL_Delay(2000);

    for (int i = 0; i < NUM_AMOSTRAS_ANALISE; i++) {
        mpu9250_ler_sensores();
        g_amostras_omega[i] = mpu9250_obter_velocidade_angular_y();
        g_amostras_theta[i] = mpu9250_obter_angulo_acelerometro();
        HAL_Delay((uint32_t)(INTERVALO_LOOP_CONTROLE_S * 1000.0f));
    }

    printf("Coleta finalizada. Calculando estatisticas...\r\n");

    float media_omega = calcular_media(g_amostras_omega, NUM_AMOSTRAS_ANALISE);
    float variancia_omega = calcular_variancia(g_amostras_omega, media_omega, NUM_AMOSTRAS_ANALISE);
    float media_theta = calcular_media(g_amostras_theta, NUM_AMOSTRAS_ANALISE);
    float variancia_theta = calcular_variancia(g_amostras_theta, media_theta, NUM_AMOSTRAS_ANALISE);

    printf("\n--- Resultados da Analise ---\r\n");
    printf("Sensor Giroscopio (omega_data):\r\n");
    printf("  - Media: %f\r\n", media_omega);
    printf("  - Variancia (U): %.8f\r\n", variancia_omega);
    printf("Sensor Acelerometro (theta_data):\r\n");
    printf("  - Media: %f\r\n", media_theta);
    printf("  - Variancia (W): %.8f\r\n", variancia_theta);
    printf("ANALISE CONCLUIDA.\r\n");
    printf("----------------------------------\r\n");
}
