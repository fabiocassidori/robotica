// Core/Src/mapa_pista.c

#include "mapa_pista.h"
#include "config_robo.h"
#include "leitor_encoder.h"
#include "mpu9250.h"
#include "monitor_bateria.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

static int g_mapa_dist_media[TAMANHO_MAPA];
static int32_t g_mapa_dist_media_pulsos[TAMANHO_MAPA];
static int g_mapa_raio[TAMANHO_MAPA];
static TipoSegmento g_mapa_tipo_segmento[TAMANHO_MAPA];
static int g_segmentos_gravados = 0;
static int g_ponteiro_segmento = 0;


// --- VARIÁVEIS PARA O MAPEAMENTO CONTÍNUO ---
static float g_log_distancia_mm[TAMANHO_LOG_PISTA];
static float g_log_angulo_graus[TAMANHO_LOG_PISTA];
static int g_log_indice = 0;
static bool g_log_ativo = false;

static float g_tabela_velocidade[TAMANHO_LOG_PISTA];
// Armazena a velocidade média calculada para cada segmento, para fins de log e verificação.
static float g_velocidade_por_segmento[TAMANHO_MAPA];

// --- Variáveis para calcular o delta de forma independente para o log ---
static int32_t g_pos_anterior_log_esq = 0;
static int32_t g_pos_anterior_log_dir = 0;


void mapa_inicializar(void) {
    for (int i = 0; i < TAMANHO_MAPA; i++) {
    	g_mapa_dist_media[i] = 0;
		g_mapa_raio[i] = 0;
		g_mapa_dist_media_pulsos[i] = 0;
		g_mapa_tipo_segmento[i] = SEGMENTO_CURVA;
		g_velocidade_por_segmento[i] = 0.0f;
    }
    g_segmentos_gravados = 0;
    g_ponteiro_segmento = 0;
    mapa_iniciar_log_pista(); // Inicia o log, mas a flag `g_log_ativo` controla a gravação
    g_log_ativo = false;
}

void mapa_gravar_segmento(void) {
	if (g_segmentos_gravados >= TAMANHO_MAPA) {
	        printf("ERRO: Mapa de segmentos cheio!\r\n");
	        return;
	    }

	int32_t pos_esq = encoder_obter_posicao_esquerda();
	int32_t pos_dir = encoder_obter_posicao_direita();
	g_mapa_dist_media_pulsos[g_segmentos_gravados] = (pos_esq + pos_dir) / 2;
	int dist_esq_mm = (int)((float)pos_esq * PULSOS_PARA_MM);
	int dist_dir_mm = (int)((float)pos_dir * PULSOS_PARA_MM);
	g_mapa_dist_media[g_segmentos_gravados] = (dist_esq_mm + dist_dir_mm) / 2;
	int32_t diff = dist_esq_mm - dist_dir_mm;
	if (diff == 0) diff = 1;
	g_mapa_raio[g_segmentos_gravados] = abs((int)(6 * (dist_esq_mm + dist_dir_mm) / diff));
	g_mapa_tipo_segmento[g_segmentos_gravados] = (g_mapa_raio[g_segmentos_gravados] > LIMIAR_CURVA_RETA) ? SEGMENTO_RETA : SEGMENTO_CURVA;
	if (g_segmentos_gravados == 0) { g_mapa_tipo_segmento[0] = SEGMENTO_CURVA; }
	encoder_resetar_posicoes();
	g_segmentos_gravados++;
}

bool mapa_corrida_terminou(uint8_t contador_fim) {
    return (contador_fim >= 2);
}

void mapa_resetar_para_corrida(void) {
    g_ponteiro_segmento = 0;
    encoder_resetar_posicoes();
}

void mapa_avancar_segmento(void) {
	if (g_ponteiro_segmento < g_segmentos_gravados - 1) {
	    g_ponteiro_segmento++;
	}
}

bool mapa_segmento_atual_e_reta_longa(void) {
    if (g_ponteiro_segmento >= g_segmentos_gravados) {
        return false;
    }

    // +++ LÓGICA CORRIGIDA: Verifica o tipo do segmento e a distância gravada +++
    bool e_reta = (g_mapa_tipo_segmento[g_ponteiro_segmento] == SEGMENTO_RETA);
    if (!e_reta) {
        return false;
    }

    return (g_mapa_dist_media[g_ponteiro_segmento] > LIMIAR_DISTANCIA_RETA_LONGA);
}

int32_t mapa_obter_distancia_pulsos_segmento_atual(void) {
    if (g_ponteiro_segmento >= g_segmentos_gravados) {
        return 0;
    }
    return g_mapa_dist_media_pulsos[g_ponteiro_segmento];
}


void mapa_iniciar_log_pista(void) {
    printf("Iniciando log da pista...\r\n");
    for (int i = 0; i < TAMANHO_LOG_PISTA; i++) {
        g_log_distancia_mm[i] = 0.0f;
        g_log_angulo_graus[i] = 0.0f;
    }
    g_log_indice = 0;
    // Inicializa as posições e o ângulo para o cálculo do primeiro delta
	g_pos_anterior_log_esq = encoder_obter_posicao_esquerda();
	g_pos_anterior_log_dir = encoder_obter_posicao_direita();
    g_log_ativo = true;
}

void mapa_parar_log_pista(void) {
    printf("Log da pista finalizado. %d pontos gravados.\r\n", g_log_indice);
    g_log_ativo = false;
}

void mapa_gravar_passo_log(void) {
	if (!g_log_ativo || g_log_indice >= TAMANHO_LOG_PISTA) {
	        if (g_log_ativo) {
	            printf("ERRO: Log da pista cheio!\r\n");
	            g_log_ativo = false; // Para de gravar para evitar overflow
	        }
	        return;
	    }

		int32_t pos_atual_esq = encoder_obter_posicao_esquerda();
	    int32_t pos_atual_dir = encoder_obter_posicao_direita();

	    // Calcula o delta desde a última chamada desta função (10ms atrás)
	    int32_t delta_esq = pos_atual_esq - g_pos_anterior_log_esq;
	    int32_t delta_dir = pos_atual_dir - g_pos_anterior_log_dir;

	    g_pos_anterior_log_esq = pos_atual_esq;
	    g_pos_anterior_log_dir = pos_atual_dir;

	    g_log_distancia_mm[g_log_indice] = (float)((delta_esq + delta_dir) / 2) * PULSOS_PARA_MM;

	    float vel_angular_dps = mpu9250_obter_velocidade_angular_y();
	    g_log_angulo_graus[g_log_indice] = vel_angular_dps * (INTERVALO_LOOP_CONTROLE_S * 10.0f); // Intervalo de 10ms

	    g_log_indice++;
}

void mapa_logar_pista_swo(void) {
	// A implementação da sua função de log unificada está correta e agora imprimirá os dados consistentes.
	    float tensao_bateria = bateria_obter_tensao();
		printf("\n--- LOG DE DADOS ---\r\n");
		printf("Tensao da Bateria (estimada): %.2fV\r\n\n", tensao_bateria);

		printf("--- Log de Segmentos (Analise) ---\r\n");
		printf("Segmento;Tipo;Dist(mm);Raio;Velocidade Alvo(mm/s)\r\n");
		for (int i = 0; i < g_segmentos_gravados; i++) {
			printf("%d;%s;%d;%d;%.0f\r\n", i + 1,
				   (g_mapa_tipo_segmento[i] == SEGMENTO_RETA ? "RETA" : "CURVA"),
				   g_mapa_dist_media[i],
				   g_mapa_raio[i],
				   g_velocidade_por_segmento[i]);
		}
		printf("----------------------------------------\r\n\n");

		printf("--- Log Detalhado da Pista (Total: %d pontos) ---\r\n", g_log_indice);
		printf("Velocidade Tabela(mm/s);");
		for (int i = 0; i < g_log_indice; i++) { printf("%.0f;", g_tabela_velocidade[i]); }
		printf("\n--------------------------------\r\n");
}

static float raio_segmento_para_velocidade(float raio_segmento) {
    raio_segmento = fabsf(raio_segmento);
    if (raio_segmento > (float)LIMIAR_CURVA_RETA) {
        return VELOCIDADE_RETA_MMPS;
    }
    float fator = raio_segmento / (float)LIMIAR_CURVA_RETA;
    return VELOCIDADE_CORRIDA_MMPS + fator * (VELOCIDADE_RETA_MMPS - VELOCIDADE_CORRIDA_MMPS);
}

/**
 * @brief Converte o RAIO INSTANTÂNEO em uma velocidade alvo para a TABELA DETALHADA.
 * Usa o limiar da nova lógica de mapeamento contínuo (RAIO_PARA_VEL_MAX_MM).
 */
static float raio_instantaneo_para_velocidade(float raio_mm_instantaneo) {
    raio_mm_instantaneo = fabsf(raio_mm_instantaneo);
    if (raio_mm_instantaneo > RAIO_PARA_VEL_MAX_MM) {
        return VELOCIDADE_RETA_MMPS;
    }
    float fator = raio_mm_instantaneo / RAIO_PARA_VEL_MAX_MM;
    return VELOCIDADE_CORRIDA_MMPS + fator * (VELOCIDADE_RETA_MMPS - VELOCIDADE_CORRIDA_MMPS);
}

void mapa_gerar_tabela_velocidade(void) {
	printf("Gerando tabela de velocidade detalhada...\r\n");
	// 1. Gera a tabela de velocidades detalhada (para a corrida futura)
	for (int i = 0; i < g_log_indice; i++) {
		float dist = g_log_distancia_mm[i];
		float angulo_rad = g_log_angulo_graus[i] * (CONSTANTE_PI / 180.0f);
		if (fabsf(angulo_rad) < 1e-6f) {
			g_tabela_velocidade[i] = VELOCIDADE_RETA_MMPS;
		} else {
			float raio_mm_instantaneo = fabsf(dist / angulo_rad);
			g_tabela_velocidade[i] = raio_instantaneo_para_velocidade(raio_mm_instantaneo);
		}
	}

	// 2. CORREÇÃO: Calcula a velocidade para o log de segmentos de forma consistente.
	printf("Calculando velocidade por segmento para o log...\r\n");
	for (int i = 0; i < g_segmentos_gravados; i++) {
		g_velocidade_por_segmento[i] = raio_segmento_para_velocidade((float)g_mapa_raio[i]);
	}

	printf("Tabela de velocidade e log de segmentos gerados com sucesso!\r\n");
}

/**
 * @brief Retorna a velocidade alvo para um ponto específico do log.
 * @param indice_log O índice do ponto na tabela.
 * @return A velocidade alvo em mm/s.
 */
float mapa_obter_velocidade_alvo(int indice_log) {
    if (indice_log >= 0 && indice_log < g_log_indice) {
        return g_tabela_velocidade[indice_log];
    }
    return VELOCIDADE_CORRIDA_MMPS; // Retorna uma velocidade segura caso o índice seja inválido
}

/**
 * @brief Retorna o número de pontos gravados no log.
 * @return O tamanho do log.
 */
int mapa_obter_tamanho_log(void) {
    return g_log_indice;
}
