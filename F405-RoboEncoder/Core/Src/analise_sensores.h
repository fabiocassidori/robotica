/*
 * analise_sensores.h
 *
 * Created on: Jul 7, 2025
 * Author: jlour
 */

#ifndef SRC_ANALISE_SENSORES_H_
#define SRC_ANALISE_SENSORES_H_

#include "main.h"

/**
 * @brief Executa a rotina de análise dos sensores para calcular a variância,
 * essencial para configurar os parâmetros U e W do Filtro de Kalman.
 */
void analisar_variancia_sensores(void);

#endif /* SRC_ANALISE_SENSORES_H_ */
