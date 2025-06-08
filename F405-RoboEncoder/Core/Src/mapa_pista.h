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
bool mapa_mapeamento_terminou(void);
bool mapa_corrida_terminou(void);
void mapa_resetar_para_corrida(void);
int mapa_obter_potencia_alvo(void);
bool mapa_segmento_atual_concluido(void);
bool mapa_segmento_atual_e_reta(void);
void mapa_avancar_segmento(void);
void mapa_logar_dados_swo(void);

#endif /* INC_MAPA_PISTA_H_ */
