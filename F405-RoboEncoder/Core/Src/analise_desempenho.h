/*
 * analise_desempenho.h
 *
 *  Created on: Jul 8, 2025
 *      Author: jlour
 */

#ifndef SRC_ANALISE_DESEMPENHO_H_
#define SRC_ANALISE_DESEMPENHO_H_

/**
 * @brief Reseta os valores de máxima aceleração e desaceleração.
 * Deve ser chamado no início de cada corrida.
 */
void analise_desempenho_inicializar(void);

/**
 * @brief Calcula a aceleração instantânea e atualiza os valores máximos.
 * Deve ser chamado no loop de controle de alta frequência (1ms).
 */
void analise_desempenho_atualizar(void);

/**
 * @brief Obtém a máxima aceleração registrada.
 * @return Aceleração máxima em mm/s².
 */
float analise_desempenho_obter_max_aceleracao(void);

/**
 * @brief Obtém a máxima desaceleração (frenagem) registrada.
 * O valor retornado será positivo.
 * @return Desaceleração máxima em mm/s².
 */
float analise_desempenho_obter_max_desaceleracao(void);

/**
 * @brief Obtém a aceleração instantânea calculada no último ciclo.
 * @return Aceleração instantânea em mm/s². Pode ser positiva (aceleração) ou negativa (desaceleração).
 */
float analise_desempenho_obter_aceleracao_instantanea(void);

#endif /* SRC_ANALISE_DESEMPENHO_H_ */
