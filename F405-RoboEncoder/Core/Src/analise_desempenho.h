/*
 * analise_desempenho.h
 *
 * Created on: Jul 8, 2025
 * Author: jlour
 */

#ifndef SRC_ANALISE_DESEMPENHO_H_
#define SRC_ANALISE_DESEMPENHO_H_

/**
 * @brief Reseta os valores de máxima aceleração e desaceleração.
 * Deve ser chamado no início de cada corrida.
 */
void analise_desempenho_inicializar(void);

/**
 * @brief Atualiza o estado do módulo de análise de desempenho.
 * Esta função deve ser chamada no loop de alta frequência (1ms) para
 * aplicar o filtro de velocidade continuamente. O cálculo da aceleração
 * será feito internamente em uma frequência menor.
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


#endif /* SRC_ANALISE_DESEMPENHO_H_ */
