/*
 * kalman_filter.c
 *
 * Created on: Jul 7, 2025
 * Author: jlour
 */

#include "kalman_filter.h"
#include <math.h>

// Matrizes e Vetores do Filtro
static float A_kalman[2][2];
static float B_kalman[2];

// Covariâncias de Ruído (Valores obtidos na análise)
static const float U = 0.01953728f; // Incerteza do giroscópio
static const float W = 0.08160052f; // Incerteza do acelerômetro

// Variáveis de Estado do Filtro
static float P[2][2] = {{1.0f, 0.0f}, {0.0f, 1.0f}};
static float theta_hat[2] = {0.0f, 0.0f};

void kalman_inicializar(float dt) {
    A_kalman[0][0] = 1.0f; A_kalman[0][1] = -dt;
    A_kalman[1][0] = 0.0f; A_kalman[1][1] = 1.0f;
    B_kalman[0] = dt;      B_kalman[1] = 0.0f;
}

float kalman_atualizar(float omega_medido, float theta_medido) {
    // --- PASSO DE PREDIÇÃO ---
    float theta_tilde[2];
    theta_tilde[0] = A_kalman[0][0] * theta_hat[0] + A_kalman[0][1] * theta_hat[1] + B_kalman[0] * omega_medido;
    theta_tilde[1] = A_kalman[1][0] * theta_hat[0] + A_kalman[1][1] * theta_hat[1] + B_kalman[1] * omega_medido;

    float P_dash[2][2];
    P_dash[0][0] = A_kalman[0][0]*(P[0][0]*A_kalman[0][0] + P[1][0]*A_kalman[0][1]) + A_kalman[0][1]*(P[0][1]*A_kalman[0][0] + P[1][1]*A_kalman[0][1]) + U*B_kalman[0]*B_kalman[0];
    P_dash[0][1] = A_kalman[0][0]*(P[0][0]*A_kalman[1][0] + P[1][0]*A_kalman[1][1]) + A_kalman[0][1]*(P[0][1]*A_kalman[1][0] + P[1][1]*A_kalman[1][1]);
    P_dash[1][0] = A_kalman[1][0]*(P[0][0]*A_kalman[0][0] + P[1][0]*A_kalman[0][1]) + A_kalman[1][1]*(P[0][1]*A_kalman[0][0] + P[1][1]*A_kalman[0][1]);
    P_dash[1][1] = A_kalman[1][0]*(P[0][0]*A_kalman[1][0] + P[1][0]*A_kalman[1][1]) + A_kalman[1][1]*(P[0][1]*A_kalman[1][0] + P[1][1]*A_kalman[1][1]);

    // --- PASSO DE ATUALIZAÇÃO ---
    float temp_inv = 1.0f / (W + P_dash[0][0]);
    float G[2];
    G[0] = P_dash[0][0] * temp_inv;
    G[1] = P_dash[1][0] * temp_inv;

    float erro = theta_medido - theta_tilde[0];
    theta_hat[0] = theta_tilde[0] + G[0] * erro;
    theta_hat[1] = theta_tilde[1] + G[1] * erro;

    P[0][0] = (1.0f - G[0]) * P_dash[0][0];
    P[0][1] = (1.0f - G[0]) * P_dash[0][1];
    P[1][0] = -G[1] * P_dash[0][0] + P_dash[1][0];
    P[1][1] = -G[1] * P_dash[0][1] + P_dash[1][1];

    return theta_hat[0];
}
