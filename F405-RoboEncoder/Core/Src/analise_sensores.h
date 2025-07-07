/*
 * analise_sensores.h
 *
 *  Created on: Jul 6, 2025
 *      Author: jlour
 */

#ifndef SRC_ANALISE_SENSORES_H_
#define SRC_ANALISE_SENSORES_H_

#include "main.h" // Para acesso aos tipos HAL e outras definições globais

/**
 * @brief Executa a rotina de análise dos sensores para calcular a variância.
 *
 * Esta função coloca o robô em um modo de diagnóstico onde ele deve
 * permanecer perfeitamente parado na vertical. Ela coleta um número
 * definido de amostras do giroscópio e do acelerômetro, calcula a média
 * e a variância para cada sensor e imprime os resultados.
 *
 * Os resultados impressos são essenciais para configurar as matrizes de
 * covariância de ruído (U e W) do Filtro de Kalman.
 *
 * @note O robô DEVE estar parado e na vertical durante a execução.
 * @note Os resultados serão impressos via SWO ou UART, dependendo da
 * configuração do seu `printf`.
 */
void analisar_variancia_sensores(void);

#endif /* SRC_ANALISE_SENSORES_H_ */
