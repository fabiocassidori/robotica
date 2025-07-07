/*
 * mapa_pista.h
 *
 *  Created on: Jun 5, 2025
 *      Author: Fábio Couto
 */

#ifndef INC_MAPA_PISTA_H_
#define INC_MAPA_PISTA_H_

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    SEGMENTO_CURVA,
    SEGMENTO_RETA
} TipoSegmento;

typedef struct {
    int16_t   segmento_pista;    // O segmento atual do mapa (0, 1, 2...)
    int16_t   erro_posicao;      // O erro da linha, já mapeado (-255 a 255)
    int16_t   correcao_pid;      // A saída do PID de direção
    int16_t   potencia_base;     // A potência calculada pelo PID de velocidade
    float     velocidade_real;   // A velocidade real medida em mm/s
} LogDebugDataPoint;


void mapa_inicializar(void);
void mapa_gravar_segmento(void);
void mapa_resetar_para_corrida(void);
void mapa_avancar_segmento(void);
bool mapa_corrida_terminou(uint8_t contador_fim);
bool mapa_segmento_atual_e_reta_longa(void);
int32_t mapa_obter_distancia_pulsos_segmento_atual(void);
void mapa_logar_dados_swo(void);

// --- FUNÇÕES PARA MAPEAMENTO CONTÍNUO ---
void mapa_iniciar_log_pista(void);
void mapa_parar_log_pista(void);
void mapa_gravar_passo_log(void);
void mapa_logar_pista_swo(void); // Nova função para imprimir o log detalhado


//--- FUNÇÕES PARA LOG DA PISTA ----

int mapa_obter_segmento_atual(void);
void mapa_iniciar_log_debug(void);
void mapa_parar_log_debug(void);
/**
 * @brief Grava um único ponto de dados de debug no buffer de log.
 * @param dados Ponteiro para a estrutura com os dados do ciclo atual.
 */
void mapa_gravar_ponto_debug(const LogDebugDataPoint* dados);
void mapa_logar_dados_debug_swo(void);

#endif /* INC_MAPA_PISTA_H_ */
