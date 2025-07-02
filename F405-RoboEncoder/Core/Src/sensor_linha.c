/*
 * sensor_linha.c
 *
 *  Created on: Jun 5, 2025
 *      Author: jlour
 */

#include "sensor_linha.h"
#include "config_robo.h"
#include "interface_usuario.h"
#include "marcador_lateral.h"
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
	// Reseta os valores de calibração
	for (int i = 0; i <= NUM_SENSORES_LINHA; i++) {
	g_valores_minimos[i] = (uint16_t)ADC_VALOR_MAXIMO;
	g_valores_maximos[i] = 0;
	}

	printf("Iniciando calibracao. Mova o robo sobre a linha...\r\n");

	// Bip inicial para marcar o começo
	iu_bip_bloqueante(100);
	HAL_Delay(100);

	// O loop de calibração dura (CICLOS_CALIBRACAO * 50ms) segundos
	for (uint16_t i = 0; i < CICLOS_CALIBRACAO; i++) {
	// A cada ciclo, atualiza os valores mínimos e máximos para cada sensor
	// começa em 1 pois g_adc_buffer 1 a NUM_SENSORES_LINHA são os sensores QTR.
	for (uint8_t j = 1; j <= NUM_SENSORES_LINHA; j++) {
		uint16_t valor_atual = g_adc_buffer[j];
		if (valor_atual < g_valores_minimos[j]) {
			g_valores_minimos[j] = valor_atual;
		}
		if (valor_atual > g_valores_maximos[j]) {
			g_valores_maximos[j] = valor_atual;
		}
	}
		marcador_lateral_calibrar_ciclo();

		HAL_Delay(50); // Pausa de 50ms a cada ciclo para que seja possivel mover o robo sobre a linha
	}

	printf("Calibracao finalizada.\r\n");

	// Log para exibir os resultados da calibração para cada sensor.
	printf("--- Resultados da Calibracao do Sensor de Linha ---\r\n");
		for (uint8_t j = 1; j <= NUM_SENSORES_LINHA; j++) {
			// Imprime o número do sensor, o valor mínimo (branco) e máximo (preto) encontrados.
			printf("Sensor %d | Min: %u | Max: %u\r\n", j, g_valores_minimos[j], g_valores_maximos[j]);
		}
	printf("---------------------------------------------------\r\n");


	// Bip longo para indicar o fim
	iu_bip_bloqueante(300);
}

int sensor_linha_ler_posicao(void) {
    uint16_t normalizados[NUM_SENSORES_LINHA + 1];
    ler_e_normalizar(normalizados); // Agora normaliza de 0 a 1000

    g_esta_na_linha = false;

    unsigned long avg = 0; // Acumulador para o numerador
    unsigned long sum = 0; // Acumulador para o denominador
    int last_value = 0;    // Posição calculada

    for (uint8_t i = 0; i < NUM_SENSORES_LINHA; i++) {
        long value = normalizados[i + 1]; // Nossos sensores começam no índice 1

        if (value > 50) { // Um limiar mínimo para considerar que o sensor vê a linha
            g_esta_na_linha = true;
        }

        // Se o sensor está vendo a linha, ele contribui para a média ponderada
        if (value > 50) {
            // O peso é a própria posição do sensor (i * 1000)
            avg += (unsigned long)value * (i * 1000);
            sum += value;
        }
    }

    if (!g_esta_na_linha) {
        // Se nenhum sensor vê a linha, decide para que lado o robô "provavelmente" saiu
        // e retorna o valor mínimo (0) ou máximo (5000).
        // O valor 2500 é o ponto central ( (NUM_SENSORES_LINHA-1)*1000 / 2 )
        if (g_ultimo_erro < 2500) {
            return 0; // Perdeu pela esquerda
        } else {
            return 5000; // Perdeu pela direita
        }
    }

    last_value = avg / sum;
    g_ultimo_erro = last_value; // Atualiza a última posição válida conhecida

    return last_value; // Retorna o valor bruto (entre 0 e 5000)
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

		// Mantém o "clamp" para garantir que os valores estejam dentro do intervalo calibrado
		if (valor_cru < g_valores_minimos[i]) valor_cru = g_valores_minimos[i];
		if (valor_cru > g_valores_maximos[i]) valor_cru = g_valores_maximos[i];


		// Invertemos a saída: o valor mínimo (branco) agora é 1000 e o máximo (preto) é 0.
		valores_normalizados[i] = interpolar(valor_cru, g_valores_minimos[i], g_valores_maximos[i], 1000, 0);
	}
}

static uint16_t interpolar(uint16_t valor, uint16_t entrada_min, uint16_t entrada_max, uint16_t saida_min, uint16_t saida_max) {
    if (entrada_min == entrada_max) return saida_min;
    return (uint16_t)(((int32_t)(valor - entrada_min) * (saida_max - saida_min)) / (entrada_max - entrada_min) + saida_min);
}

volatile uint16_t* sensor_linha_obter_buffer_dma(void) {
    return g_adc_buffer;
}
