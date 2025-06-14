/*
 * mapa_pista.h
 *
 *  Created on: Jun 5, 2025
 *      Author: jlour
 */

#ifndef INC_MAPA_PISTA_H_
#define INC_MAPA_PISTA_H_

#include <stdbool.h>
#include <stdint.h>

void mapa_inicializar(void);
void mapa_gravar_segmento(void);
bool mapa_corrida_terminou(uint8_t contador_fim);
void mapa_resetar_para_corrida(void);
int mapa_obter_potencia_alvo(void);
bool mapa_segmento_atual_concluido(void);
bool mapa_segmento_atual_e_reta(void);
bool mapa_segmento_atual_e_reta_longa(void);
void mapa_avancar_segmento(void);
int32_t mapa_obter_distancia_pulsos_segmento_atual(void);
void mapa_logar_dados_swo(void);
int mapa_obter_distancia_segmento_atual(void);

#endif /* INC_MAPA_PISTA_H_ */
