/*
 * analise_sensores.c
 *
 *  Created on: Jul 6, 2025
 *      Author: jlour
 */


// Core/Src/analise_sensores.c

#include "analise_sensores.h"
#include "config_robo.h" // Para usar as configurações como número de amostras
#include <stdio.h>
#include <math.h>
#include <stdint.h>

// Inclua aqui os cabeçalhos dos seus módulos de sensores
// Exemplo:
#include "mpu9250.h" // Supondo que a classe IMU esteja aqui

// Número de amostras a serem coletadas para a análise.
// Pode ser definido em config_robo.h para fácil ajuste.
#ifndef NUM_AMOSTRAS_ANALISE
#define NUM_AMOSTRAS_ANALISE 1000
#endif

// Buffers para armazenar as leituras dos sensores
static float g_amostras_omega[NUM_AMOSTRAS_ANALISE];
static float g_amostras_theta[NUM_AMOSTRAS_ANALISE];

/**
 * @brief Calcula a média de um array de floats.
 * @param dados Array com os dados.
 * @param tamanho Número de elementos no array.
 * @return A média dos valores.
 */
static float calcular_media(float* dados, int tamanho) {
    float soma = 0.0f;
    for (int i = 0; i < tamanho; i++) {
        soma += dados[i];
    }
    return soma / tamanho;
}

/**
 * @brief Calcula a variância de um array de floats.
 * @param dados Array com os dados.
 * @param media A média dos dados (deve ser calculada previamente).
 * @param tamanho Número de elementos no array.
 * @return A variância dos valores.
 */
static float calcular_variancia(float* dados, float media, int tamanho) {
    float soma_diff_quadrada = 0.0f;
    for (int i = 0; i < tamanho; i++) {
        float diff = dados[i] - media;
        soma_diff_quadrada += diff * diff;
    }
    return soma_diff_quadrada / (tamanho - 1); // Variância amostral
}


void analisar_variancia_sensores(void) {
    printf("--- Iniciando Analise de Variancia dos Sensores ---\r\n");
    printf("Mantenha o robo parado na posicao vertical.\r\n");
    printf("Coletando %d amostras...\r\n", NUM_AMOSTRAS_ANALISE);

    // Aguarda um momento para o usuário posicionar o robô
    HAL_Delay(2000);

    // --- Etapa de Coleta de Dados ---
    for (int i = 0; i < NUM_AMOSTRAS_ANALISE; i++) {
    	 // 1. Lê os valores brutos mais recentes do sensor via SPI
		mpu9250_ler_sensores();

		// 2. Obtém os valores já processados (ângulo e velocidade angular) e armazena
		g_amostras_omega[i] = mpu9250_obter_velocidade_angular_y();
		g_amostras_theta[i] = mpu9250_obter_angulo_acelerometro();

        // Aguarda o intervalo do loop de controle para manter a mesma taxa de amostragem
        HAL_Delay((uint32_t)(INTERVALO_LOOP_CONTROLE_S * 1000.0f));
    }

    printf("Coleta finalizada. Calculando estatisticas...\r\n");

    // --- Etapa de Cálculo ---

    // Estatísticas do Giroscópio (Omega)
    float media_omega = calcular_media(g_amostras_omega, NUM_AMOSTRAS_ANALISE);
    float variancia_omega = calcular_variancia(g_amostras_omega, media_omega, NUM_AMOSTRAS_ANALISE);

    // Estatísticas do Acelerômetro (Theta)
    float media_theta = calcular_media(g_amostras_theta, NUM_AMOSTRAS_ANALISE);
    float variancia_theta = calcular_variancia(g_amostras_theta, media_theta, NUM_AMOSTRAS_ANALISE);


    // --- Etapa de Impressão dos Resultados ---
    // Usando `printf` que deve ser redirecionado para UART ou SWO.
    printf("\n--- Resultados da Analise ---\r\n");
    printf("Sensor Giroscopio (omega_data):\r\n");
    printf("  - Media: %f\r\n", media_omega);
    printf("  - Variancia (U): %.8f\r\n", variancia_omega); // Imprime com mais casas decimais
    printf("\r\n");
    printf("Sensor Acelerometro (theta_data):\r\n");
    printf("  - Media: %f\r\n", media_theta);
    printf("  - Variancia (W): %.8f\r\n", variancia_theta); // Imprime com mais casas decimais
    printf("\r\n");
    printf("ANALISE CONCLUIDA.\r\n");
    printf("Use os valores de variancia para os parametros U e W do Filtro de Kalman.\r\n");
    printf("----------------------------------\r\n");

}
