// Core/Src/mapa_pista.c

#include "mapa_pista.h"
#include "config_robo.h"
#include "leitor_encoder.h"
#include "mpu9250.h"
#include "monitor_bateria.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

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

static float g_tabela_velocidade[TAMANHO_LOG_PISTA];


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


}

void mapa_iniciar_log_pista(void) {
    printf("Iniciando log da pista...\r\n");
    for (int i = 0; i < TAMANHO_LOG_PISTA; i++) {
        g_log_distancia_mm[i] = 0.0f;
        g_log_angulo_graus[i] = 0.0f;
    }
    g_log_indice = 0;
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

    // 1. Calcula a distância percorrida neste ciclo
    int32_t delta_pulsos = encoder_obter_delta_pulsos_media();
    float delta_dist_mm = (float)delta_pulsos * PULSOS_PARA_MM;

    // 2. Calcula a variação de ângulo neste ciclo
    // Variação Ângulo = Velocidade Angular * Tempo
    float vel_angular_dps = mpu9250_obter_velocidade_angular_y(); // em graus/segundo
    float delta_angulo_graus = vel_angular_dps * INTERVALO_LOOP_MPU_S; // (graus/s) * s = graus

    // 3. Armazena os valores no log
    g_log_distancia_mm[g_log_indice] = delta_dist_mm;
    g_log_angulo_graus[g_log_indice] = delta_angulo_graus;

    // 4. Avança para o próximo índice
    g_log_indice++;
}

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

static float raio_para_velocidade(float raio_mm) {
    // Garante que o raio não seja negativo
    raio_mm = fabsf(raio_mm);

    // Se o raio for muito grande, considera como reta e usa a velocidade máxima.
    if (raio_mm > LIMIAR_CURVA_RETA) {
        return VELOCIDADE_RETA_MMPS;
    }

    // Interpolação linear simples:
    // Mapeia o raio (de 0 a RAIO_CONSIDERADO_RETA) para a velocidade (de VEL_MIN a VEL_MAX)
    float fator = raio_mm / LIMIAR_CURVA_RETA;
    float velocidade = VELOCIDADE_CORRIDA_MMPS + fator * (VELOCIDADE_RETA_MMPS - VELOCIDADE_CORRIDA_MMPS);

    return velocidade;
}

void mapa_gerar_tabela_velocidade(void) {
    printf("Gerando tabela de velocidade a partir de %d pontos...\r\n", g_log_indice);

    for (int i = 0; i < g_log_indice; i++) {
        float dist = g_log_distancia_mm[i];
        float angulo_rad = g_log_angulo_graus[i] * (CONSTANTE_PI / 180.0f); // Converte para radianos

        // Evita divisão por zero se o robô andou em linha perfeitamente reta
        if (fabsf(angulo_rad) < 1e-6) {
            // Se o ângulo é quase zero, o raio é infinito (reta)
            g_tabela_velocidade[i] = VELOCIDADE_RETA_MMPS;
        } else {
            // Calcula o raio da curva para este passo
            float raio_mm = fabsf(dist / angulo_rad);
            // Converte o raio em uma velocidade alvo
            g_tabela_velocidade[i] = raio_para_velocidade(raio_mm);
        }
    }

    printf("Tabela de velocidade gerada com sucesso!\r\n");
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
