/*
 * sensor_linha.c
 *
 *  Created on: Jun 5, 2025
 *      Author: jlour
 */

#include "sensor_linha.h"
#include "config_robo.h"
#include "interface_usuario.h"
#include <stdio.h>

static ADC_HandleTypeDef *g_hadc = NULL;
static volatile uint16_t g_adc_buffer[8];

static uint16_t g_valores_minimos[NUM_SENSORES_LINHA + 1];
static uint16_t g_valores_maximos[NUM_SENSORES_LINHA + 1];

static bool g_esta_na_linha = false;
static int g_ultimo_erro = 0;

static uint16_t interpolar(uint16_t valor, uint16_t entrada_min, uint16_t entrada_max, uint16_t saida_min, uint16_t saida_max);
static void ler_e_normalizar(uint16_t* valores_normalizados);

void sensor_linha_inicializar(ADC_HandleTypeDef *hadc) {
    g_hadc = hadc;
    for (int i = 0; i <= NUM_SENSORES_LINHA; i++) {
        g_valores_minimos[i] = (uint16_t)ADC_VALOR_MAXIMO;
        g_valores_maximos[i] = 0;
    }
    HAL_ADC_Start_DMA(g_hadc, (uint32_t*)g_adc_buffer, 8);
}

void sensor_linha_calibrar(void) {
	iu_bip_bloqueante(100);

    for (int i = 0; i <= NUM_SENSORES_LINHA; i++) {
        g_valores_minimos[i] = (uint16_t)ADC_VALOR_MAXIMO;
        g_valores_maximos[i] = 0;
    }

    printf("Iniciando calibracao. Mova o robo sobre a linha.\r\n");
    for (uint16_t i = 0; i < CICLOS_CALIBRACAO; i++) {
        for (uint8_t j = 1; j <= NUM_SENSORES_LINHA; j++) {
            uint16_t valor_atual = g_adc_buffer[j];
            if (valor_atual < g_valores_minimos[j]) g_valores_minimos[j] = valor_atual;
            if (valor_atual > g_valores_maximos[j]) g_valores_maximos[j] = valor_atual;
        }
        HAL_Delay(30);
    }
    printf("Calibracao finalizada.\r\n");
    iu_bip_bloqueante(100);
}

int sensor_linha_ler_posicao(void) {
    uint16_t normalizados[NUM_SENSORES_LINHA + 1];
    ler_e_normalizar(normalizados);

    const int pesos[] = {0, -25, -15, -5, 5, 15, 25};
    int32_t numerador = 0;
    int32_t denominador = 0;

    for (uint8_t i = 1; i <= NUM_SENSORES_LINHA; i++) {
        numerador += (int32_t)normalizados[i] * pesos[i];
        denominador += normalizados[i];
    }

    g_esta_na_linha = (denominador > 100);

    int posicao;
    if (g_esta_na_linha) {
        posicao = (denominador == 0) ? 0 : (numerador * 10) / denominador;
        g_ultimo_erro = posicao;
    } else {
        posicao = (g_ultimo_erro < 0) ? -255 : 255;
    }
    return posicao;
}

bool sensor_linha_esta_na_linha(void) {
    return g_esta_na_linha;
}

int sensor_linha_obter_ultimo_erro(void) {
    return g_ultimo_erro;
}

static void ler_e_normalizar(uint16_t* valores_normalizados) {
    for (uint8_t i = 1; i <= NUM_SENSORES_LINHA; i++) {
        uint16_t valor_cru = g_adc_buffer[i];
        if (valor_cru < g_valores_minimos[i]) valor_cru = g_valores_minimos[i];
        if (valor_cru > g_valores_maximos[i]) valor_cru = g_valores_maximos[i];
        valores_normalizados[i] = interpolar(valor_cru, g_valores_minimos[i], g_valores_maximos[i], 0, 100);
    }
}

static uint16_t interpolar(uint16_t valor, uint16_t entrada_min, uint16_t entrada_max, uint16_t saida_min, uint16_t saida_max) {
    if (entrada_min == entrada_max) return saida_min;
    return (uint16_t)(((int32_t)(valor - entrada_min) * (saida_max - saida_min)) / (entrada_max - entrada_min) + saida_min);
}

volatile uint16_t* sensor_linha_obter_buffer_dma(void) {
    return g_adc_buffer;
}
