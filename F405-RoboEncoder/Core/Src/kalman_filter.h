/*
 * kalman_filter.h
 *
 *  Created on: Jul 6, 2025
 *      Author: jlour
 */

#ifndef SRC_KALMAN_FILTER_H_
#define SRC_KALMAN_FILTER_H_


#include "main.h"

/**
 * @brief Inicializa o filtro de Kalman com os parâmetros do robô.
 * * @param dt O intervalo de tempo do loop de controle (em segundos).
 * Ex: 0.001 para 1ms.
 */
void kalman_inicializar(float dt);

/**
 * @brief Executa um passo do filtro de Kalman para estimar o ângulo do robô.
 * * @param omega_medido A velocidade angular medida pelo giroscópio (em graus/s).
 * @param theta_medido O ângulo medido pelo acelerômetro (em graus).
 * @return O ângulo estimado e filtrado (em graus).
 */
float kalman_atualizar(float omega_medido, float theta_medido);

#endif /* SRC_KALMAN_FILTER_H_ */
