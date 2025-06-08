/*
 * controle_velocidade.h
 *
 *  Created on: Jun 5, 2025
 *      Author: jlour
 */

#ifndef INC_CONTROLE_VELOCIDADE_H_
#define INC_CONTROLE_VELOCIDADE_H_

#include <stdint.h>

void velocidade_inicializar(void);
void velocidade_atualizar(void);
float velocidade_obter_media_mmps(void);
int velocidade_rampa_potencia(int potencia_atual, int potencia_alvo);

#endif /* INC_CONTROLE_VELOCIDADE_H_ */
