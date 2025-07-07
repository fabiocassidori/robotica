// Core/Src/mapa_pista.c

#include "mapa_pista.h"
#include "config_robo.h"
#include "leitor_encoder.h"
#include <stdio.h>
#include <stdlib.h>
#include "monitor_bateria.h"
#include "mpu9250.h"

static int g_mapa_dist_esq[TAMANHO_MAPA];
static int g_mapa_dist_dir[TAMANHO_MAPA];
static int g_mapa_dist_media[TAMANHO_MAPA];
static int32_t g_mapa_dist_media_pulsos[TAMANHO_MAPA];
static int g_mapa_raio[TAMANHO_MAPA];
// --- Variável de potência trocada por tipo de segmento ---
static TipoSegmento g_mapa_tipo_segmento[TAMANHO_MAPA];

static int g_segmentos_gravados = 0;
static int g_ponteiro_segmento = 0;

// --- VARIÁVEIS PARA O MAPEAMENTO CONTÍNUO ---
static float g_log_distancia_mm[TAMANHO_LOG_PISTA];
static float g_log_angulo_graus[TAMANHO_LOG_PISTA];
static int g_log_indice = 0;
static bool g_log_ativo = false;

//--- VARIÁVEIS PARA LOG DE DEBUG --

// Buffer para armazenar os pontos de dados de debug
static LogDebugDataPoint g_log_debug_buffer[TAMANHO_LOG_PISTA];

// Flag para controlar se o log de debug está ativo
static bool g_log_debug_ativo = false;




void mapa_inicializar(void) {
    for (int i = 0; i < TAMANHO_MAPA; i++) {
        g_mapa_dist_esq[i] = 0; g_mapa_dist_dir[i] = 0; g_mapa_dist_media[i] = 0;
        g_mapa_raio[i] = 0; g_mapa_dist_media_pulsos[i] = 0;
        g_mapa_tipo_segmento[i] = SEGMENTO_CURVA; // Padrão
    }
    g_segmentos_gravados = 0; g_ponteiro_segmento = 0;
}

void mapa_gravar_segmento(void) {
    if (g_segmentos_gravados >= TAMANHO_MAPA) { printf("ERRO: Mapa cheio!\r\n"); return; }

    int32_t pos_esq_pulsos = encoder_obter_posicao_esquerda();
    int32_t pos_dir_pulsos = encoder_obter_posicao_direita();
    g_mapa_dist_media_pulsos[g_segmentos_gravados] = (pos_esq_pulsos + pos_dir_pulsos) / 2;

    g_mapa_dist_esq[g_segmentos_gravados] = (int)(PULSOS_PARA_MM * pos_esq_pulsos);
    g_mapa_dist_dir[g_segmentos_gravados] = (int)(PULSOS_PARA_MM * pos_dir_pulsos);
    g_mapa_dist_media[g_segmentos_gravados] = (g_mapa_dist_esq[g_segmentos_gravados] + g_mapa_dist_dir[g_segmentos_gravados]) / 2;

    int32_t diff = g_mapa_dist_esq[g_segmentos_gravados] - g_mapa_dist_dir[g_segmentos_gravados];
    if (diff == 0) diff = 1;
    int32_t sum = g_mapa_dist_esq[g_segmentos_gravados] + g_mapa_dist_dir[g_segmentos_gravados];
    g_mapa_raio[g_segmentos_gravados] = abs((int)(6 * sum / diff));

    // ++: Armazena o TIPO do segmento, não a potência +++
    if (g_mapa_raio[g_segmentos_gravados] > LIMIAR_CURVA_RETA) {
        g_mapa_tipo_segmento[g_segmentos_gravados] = SEGMENTO_RETA;
    } else {
        g_mapa_tipo_segmento[g_segmentos_gravados] = SEGMENTO_CURVA;
    }

    // Garante que o primeiro segmento nunca seja uma reta de alta velocidade
    if (g_segmentos_gravados == 0) {
        g_mapa_tipo_segmento[g_segmentos_gravados] = SEGMENTO_CURVA;
    }

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

void mapa_logar_dados_swo(void) {

	// --- INÍCIO DA SEÇÃO DE TENSÃO DA BATERIA ---
	// Obtém a tensão atual da bateria a partir do módulo de monitoramento.
	float tensao_bateria = bateria_obter_tensao();
	// Imprime a tensão formatada com duas casas decimais.
	printf("Tensao da Bateria (estimada): %.2fV\r\n", tensao_bateria);
	printf("----------------------------------------\r\n"); // Separador para clareza
	// --- FIM DA SEÇÃO DE TENSÃO DA BATERIA ---

    printf("Iniciando impressao dos dados via SWO...\r\n");
    printf("Tipo Segmento (0=Curva, 1=Reta): ;");
    for (int i = 0; i < g_segmentos_gravados; i++) printf("%d;", g_mapa_tipo_segmento[i]);
    printf("\nMapesq: ;");
    for (int i = 0; i < g_segmentos_gravados; i++) printf("%d;", g_mapa_dist_esq[i]);
    printf("\nMapdir: ;");
    for (int i = 0; i < g_segmentos_gravados; i++) printf("%d;", g_mapa_dist_dir[i]);
    printf("\nMapa (media): ;");
    for (int i = 0; i < g_segmentos_gravados; i++) printf("%d;", g_mapa_dist_media[i]);
    printf("\nRmapa (raio): ;");
    for (int i = 0; i < g_segmentos_gravados; i++) printf("%d;", g_mapa_raio[i]);
    printf("\nDistancia Retas (cm): ;");
        for (int i = 0; i < g_segmentos_gravados; i++) {
            // Verifica se o segmento atual é uma reta
            if (g_mapa_tipo_segmento[i] == SEGMENTO_RETA) {
                // Converte a distância de mm para cm e imprime
                int dist_cm = g_mapa_dist_media[i] / 10;
                printf("%d;", dist_cm);
            } else {
                // Se for uma curva, imprime 0 para manter o formato
                printf("0;");
            }
        }
    printf("\nImpressao concluida.\r\n");

}

// --- FUNÇÕES MAPEAMENTO CONTÍNUO ---

/**
 * @brief Prepara o sistema para iniciar a gravação do log da pista.
 */
void mapa_iniciar_log_pista(void) {
    printf("Iniciando log da pista...\r\n");
    for (int i = 0; i < TAMANHO_LOG_PISTA; i++) {
        g_log_distancia_mm[i] = 0.0f;
        g_log_angulo_graus[i] = 0.0f;
    }
    g_log_indice = 0;
    g_log_ativo = true;
}

/**
 * @brief Para a gravação do log da pista.
 */
void mapa_parar_log_pista(void) {
    printf("Log da pista finalizado. %d pontos gravados.\r\n", g_log_indice);
    g_log_ativo = false;
}

/**
 * @brief Grava um único passo (distância e ângulo) no log.
 * Deve ser chamada a cada ciclo do loop de controle rápido (1ms).
 */
void mapa_gravar_passo_log(void) {
    if (!g_log_ativo || g_log_indice >= TAMANHO_LOG_PISTA) {
        if (g_log_ativo) {
            printf("ERRO: Log da pista cheio!\r\n");
            g_log_ativo = false; // Para de gravar para evitar overflow
        }
        return;
    }

    // 1. Calcula a distância percorrida neste ciclo
    int32_t delta_pulsos = encoder_obter_delta_pulsos_media();
    float delta_dist_mm = (float)delta_pulsos * PULSOS_PARA_MM;

    // 2. Calcula a variação de ângulo neste ciclo
    // Variação Ângulo = Velocidade Angular * Tempo
    float vel_angular_dps = mpu9250_obter_velocidade_angular_y(); // em graus/segundo
    float delta_angulo_graus = vel_angular_dps * INTERVALO_LOOP_CONTROLE_S; // (graus/s) * s = graus

    // 3. Armazena os valores no log
    g_log_distancia_mm[g_log_indice] = delta_dist_mm;
    g_log_angulo_graus[g_log_indice] = delta_angulo_graus;

    // 4. Avança para o próximo índice
    g_log_indice++;
}

/**
 * @brief Imprime o log detalhado da pista via SWO.
 */
void mapa_logar_pista_swo(void) {
    printf("--- Log Detalhado da Pista ---\r\n");
    printf("Distancia (mm);");
    for (int i = 0; i < g_log_indice; i++) {
        printf("%.4f;", g_log_distancia_mm[i]);
    }
    printf("\n");

    printf("Angulo (graus);");
    for (int i = 0; i < g_log_indice; i++) {
        printf("%.4f;", g_log_angulo_graus[i]);
    }
    printf("\n--------------------------------\r\n");
}

//--- Funcoes para log da pista

int mapa_obter_segmento_atual(void) {
	// g_ponteiro_segmento já contém o segmento atual durante a corrida,
	// mas durante o mapeamento, o segmento atual é g_segmentos_gravados.
	return g_segmentos_gravados;
}

void mapa_iniciar_log_debug(void) {
	printf("Log de debug do PID ativado para o mapeamento.\r\n");
	g_log_indice = 0;
	g_log_debug_ativo = true;
}

void mapa_parar_log_debug(void) {
	if (g_log_debug_ativo) {
	        printf("Log de debug finalizado. %d pontos gravados.\r\n", g_log_indice);
	        g_log_debug_ativo = false;
	}
}

void mapa_gravar_ponto_debug(const LogDebugDataPoint* dados) {
    if (!g_log_debug_ativo || g_log_indice >= TAMANHO_LOG_PISTA) {
        if (g_log_debug_ativo) {
            printf("ERRO: Buffer de log de debug cheio!\r\n");
            g_log_debug_ativo = false; // Para de gravar para evitar overflow
        }
        return;
    }

    // Copia os dados para o buffer de log
    g_log_debug_buffer[g_log_indice] = *dados;
    g_log_indice++;
}

void mapa_logar_dados_debug_swo(void) {
	if (g_log_indice == 0) {
	        printf("Nenhum dado de debug foi logado.\r\n");
	        return;
	}

	    printf("--- Log de Debug do PID (CSV) ---\r\n");
	    printf("Tempo(ms);Segmento;ErroPos;CorrecaoPID;VelReal(mm/s);PotBase\r\n");

	    for (int i = 0; i < g_log_indice; i++) {
	        // CORREÇÃO: Acessando os campos corretos da estrutura no buffer
	        printf("%d;%d;%d;%d;%d;%d\r\n",
	               i,
	               g_log_debug_buffer[i].segmento_pista,
	               g_log_debug_buffer[i].erro_posicao,
	               g_log_debug_buffer[i].correcao_pid,
	               (int)g_log_debug_buffer[i].velocidade_real,
	               g_log_debug_buffer[i].potencia_base);
	    }
	    printf("--- Fim do Log de Debug ---\r\n");
}
