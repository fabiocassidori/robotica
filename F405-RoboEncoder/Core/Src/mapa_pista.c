/*
 * mapa_pista.c
 *
 *  Created on: Jun 5, 2025
 *      Author: jlour
 */


// Core/Src/mapa_pista.c

#include "mapa_pista.h"
#include "config_robo.h"
#include "leitor_encoder.h"
#include <stdio.h>
#include <stdlib.h>

// ... (variáveis estáticas do mapa permanecem as mesmas) ...
static int g_mapa_dist_esq[TAMANHO_MAPA];
static int g_mapa_dist_dir[TAMANHO_MAPA];
static int g_mapa_dist_media[TAMANHO_MAPA];
static int32_t g_mapa_dist_media_pulsos[TAMANHO_MAPA];
static int g_mapa_raio[TAMANHO_MAPA];
static int g_mapa_potencia[TAMANHO_MAPA];

static int g_segmentos_gravados = 0;
static int g_ponteiro_segmento = 0;


// ... (funções mapa_inicializar, mapa_gravar_segmento, mapa_mapeamento_terminou, mapa_corrida_terminou, mapa_resetar_para_corrida permanecem as mesmas) ...
void mapa_inicializar(void) {
    for (int i = 0; i < TAMANHO_MAPA; i++) {
        g_mapa_dist_esq[i] = 0; g_mapa_dist_dir[i] = 0; g_mapa_dist_media[i] = 0;
        g_mapa_raio[i] = 0; g_mapa_potencia[i] = 0;
        g_mapa_dist_media_pulsos[i] = 0;
    }
    g_segmentos_gravados = 0; g_ponteiro_segmento = 0;
}

void mapa_gravar_segmento(void) {
    if (g_segmentos_gravados >= TAMANHO_MAPA) { printf("ERRO: Mapa cheio!\r\n"); return; }

	// Primeiro, captura as posições em pulsos
	int32_t pos_esq_pulsos = encoder_obter_posicao_esquerda();
	int32_t pos_dir_pulsos = encoder_obter_posicao_direita();

	// Salva a distância média em pulsos
	g_mapa_dist_media_pulsos[g_segmentos_gravados] = (pos_esq_pulsos + pos_dir_pulsos) / 2;


	// Converte para mm e continua com a lógica original
	g_mapa_dist_esq[g_segmentos_gravados] = (int)(PULSOS_PARA_MM * pos_esq_pulsos);
	g_mapa_dist_dir[g_segmentos_gravados] = (int)(PULSOS_PARA_MM * pos_dir_pulsos);
	g_mapa_dist_media[g_segmentos_gravados] = (g_mapa_dist_esq[g_segmentos_gravados] + g_mapa_dist_dir[g_segmentos_gravados]) / 2;

	int32_t diff = g_mapa_dist_esq[g_segmentos_gravados] - g_mapa_dist_dir[g_segmentos_gravados];
	if (diff == 0) diff = 1;
	int32_t sum = g_mapa_dist_esq[g_segmentos_gravados] + g_mapa_dist_dir[g_segmentos_gravados];
	g_mapa_raio[g_segmentos_gravados] = abs((int)(6 * sum / diff));
	g_mapa_potencia[g_segmentos_gravados] = (g_mapa_raio[g_segmentos_gravados] > LIMIAR_CURVA_RETA) ? POTENCIA_RETA : POTENCIA_CURVA;

	if (g_segmentos_gravados == 0) { g_mapa_potencia[g_segmentos_gravados] = POTENCIA_CURVA; }

	encoder_resetar_posicoes();
	g_segmentos_gravados++;
}

bool mapa_corrida_terminou(uint8_t contador_fim) {
    // A corrida termina na SEGUNDA marcação da direita.
    // A primeira (início) faz o contador ser 1.
    // A segunda (fim) faz o contador ser 2, terminando a prova.
    return (contador_fim >= 2);
}
void mapa_resetar_para_corrida(void) { g_ponteiro_segmento = 0; encoder_resetar_posicoes(); }


// CORREÇÃO: Retorna a potência do segmento ATUAL.
int mapa_obter_potencia_alvo(void) {
    if (g_ponteiro_segmento >= g_segmentos_gravados) return 0; // Se já terminou, para
    return g_mapa_potencia[g_ponteiro_segmento];
}

// CORREÇÃO: Compara a distância percorrida com a distância do segmento ATUAL.
bool mapa_segmento_atual_concluido(void) {
    if (g_ponteiro_segmento >= g_segmentos_gravados) return true;
    int32_t dist_media_pulsos = (encoder_obter_posicao_esquerda() + encoder_obter_posicao_direita()) / 2;

    // Converte a distância do mapa para pulsos para fazer a comparação
    // A constante de distância é ~0.335, o inverso é ~2.985
	// O fator de conversão é o inverso de PULSOS_PARA_MM.
    // Converte a distância do mapa (que está em mm) de volta para pulsos para a comparação.
	float pulsos_por_mm = 1.0f / PULSOS_PARA_MM;
	int32_t pulsos_alvo = (int32_t)(g_mapa_dist_media[g_ponteiro_segmento] * pulsos_por_mm);

	return (dist_media_pulsos > pulsos_alvo);
}

bool mapa_segmento_atual_e_reta(void) {
    if (g_ponteiro_segmento >= g_segmentos_gravados) return false;
    return (g_mapa_potencia[g_ponteiro_segmento] == POTENCIA_RETA);
}

void mapa_avancar_segmento(void) {
	if (g_ponteiro_segmento < g_segmentos_gravados - 1) {
	       g_ponteiro_segmento++;
	   }
}

/**
 * @brief Retorna a distância média (em unidades de mapa) do segmento atual.
 * @return A distância do segmento.
 */
int mapa_obter_distancia_segmento_atual(void) {
    if (g_ponteiro_segmento >= g_segmentos_gravados) {
        return 0;
    }
    return g_mapa_dist_media[g_ponteiro_segmento];
}

/**
 * @brief Verifica se o segmento atual é uma reta E se seu comprimento é
 * superior ao limiar definido para retas longas.
 * @return true se for uma reta longa, false caso contrário.
 */
bool mapa_segmento_atual_e_reta_longa(void) {
    // Garante que não estamos tentando ler fora dos dados do mapa
    if (g_ponteiro_segmento >= g_segmentos_gravados) {
        return false;
    }

    // 1. Verifica se o segmento é classificado como uma reta
    bool e_reta = (g_mapa_potencia[g_ponteiro_segmento] == POTENCIA_RETA);
    if (!e_reta) {
        return false; // Se não for reta, não pode ser uma reta longa.
    }

    // 2. Se for uma reta, verifica se o comprimento gravado excede o limiar
    return (g_mapa_dist_media[g_ponteiro_segmento] > LIMIAR_DISTANCIA_RETA_LONGA);
}

int32_t mapa_obter_distancia_pulsos_segmento_atual(void) {
    if (g_ponteiro_segmento >= g_segmentos_gravados) {
        return 0;
    }
    return g_mapa_dist_media_pulsos[g_ponteiro_segmento];
}

void mapa_logar_dados_swo(void) {
    printf("Iniciando impressao dos dados via SWO...\r\n");
    printf("Potencias: ;");
    for (int i = 0; i < g_segmentos_gravados; i++) printf("%d;", g_mapa_potencia[i]);
    printf("\nMapesq: ;");
    for (int i = 0; i < g_segmentos_gravados; i++) printf("%d;", g_mapa_dist_esq[i]);
    printf("\nMapdir: ;");
    for (int i = 0; i < g_segmentos_gravados; i++) printf("%d;", g_mapa_dist_dir[i]);
    printf("\nMapa (media): ;");
    for (int i = 0; i < g_segmentos_gravados; i++) printf("%d;", g_mapa_dist_media[i]);
    printf("\nRmapa (raio): ;");
    for (int i = 0; i < g_segmentos_gravados; i++) printf("%d;", g_mapa_raio[i]);
    printf("\nImpressao concluida.\r\n");
}
