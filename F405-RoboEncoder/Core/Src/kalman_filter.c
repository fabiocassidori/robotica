/*
 * kalman_filter.c
 *
 * Created on: Jul 6, 2025
 * Author: jlour
 */

#include "kalman_filter.h"
#include <math.h>

// --- Matrizes e Vetores do Filtro ---
// Matrizes de estado (baseado no modelo do MATLAB)
static float A_kalman[2][2]; // Matriz de transição de estado
static float B_kalman[2];    // Matriz de entrada de controle

// Matriz de observação
//static float C_kalman[2] = {1.0f, 0.0f};

// --- Covariâncias de Ruído (Valores Obtidos na Análise) ---
// Covariância do ruído do processo (incerteza do giroscópio)
// Variancia (U): 0.01902881
static const float U = 0.01902881f;

// Covariância do ruído da medição (incerteza do acelerômetro)
// Variancia (W): 0.10482820
static const float W = 0.10482820f;


// --- Variáveis de Estado do Filtro ---
// Matriz de covariância do erro de estimativa
static float P[2][2] = {
    {1.0f, 0.0f},
    {0.0f, 1.0f}
};

// Vetor de estado estimado [ângulo; offset do giroscópio]
static float theta_hat[2] = {0.0f, 0.0f};


void kalman_inicializar(float dt) {
    // Inicializa as matrizes A e B com base no dt
    // A = [1, -dt]
    //     [0,  1 ]
    A_kalman[0][0] = 1.0f;
    A_kalman[0][1] = -dt;
    A_kalman[1][0] = 0.0f;
    A_kalman[1][1] = 1.0f;

    // B = [dt]
    //     [ 0]
    B_kalman[0] = dt;
    B_kalman[1] = 0.0f;
}

float kalman_atualizar(float omega_medido, float theta_medido) {
    // --- PASSO DE PREDIÇÃO ---

    // 1. Predição do estado: theta_tilde = A * theta_hat + B * omega
    float theta_tilde[2];
    theta_tilde[0] = A_kalman[0][0] * theta_hat[0] + A_kalman[0][1] * theta_hat[1] + B_kalman[0] * omega_medido;
    theta_tilde[1] = A_kalman[1][0] * theta_hat[0] + A_kalman[1][1] * theta_hat[1] + B_kalman[1] * omega_medido;

    // 2. Predição da covariância do erro: P_dash = A * P * A' + B * U * B'
    // Como U é um escalar, a segunda parte é B * U * B' = [dt^2*U, 0; 0, 0]
    float P_dash[2][2];
    float P_A_T[2][2]; // Temp para P * A'
    P_A_T[0][0] = P[0][0] * A_kalman[0][0] + P[0][1] * A_kalman[1][0];
    P_A_T[0][1] = P[0][0] * A_kalman[0][1] + P[0][1] * A_kalman[1][1];
    P_A_T[1][0] = P[1][0] * A_kalman[0][0] + P[1][1] * A_kalman[1][0];
    P_A_T[1][1] = P[1][0] * A_kalman[0][1] + P[1][1] * A_kalman[1][1];

    P_dash[0][0] = A_kalman[0][0] * P_A_T[0][0] + A_kalman[0][1] * P_A_T[1][0] + (B_kalman[0] * U * B_kalman[0]);
    P_dash[0][1] = A_kalman[0][0] * P_A_T[0][1] + A_kalman[0][1] * P_A_T[1][1];
    P_dash[1][0] = A_kalman[1][0] * P_A_T[0][0] + A_kalman[1][1] * P_A_T[1][0];
    P_dash[1][1] = A_kalman[1][0] * P_A_T[0][1] + A_kalman[1][1] * P_A_T[1][1];


    // --- PASSO DE ATUALIZAÇÃO ---

    // 3. Cálculo do ganho de Kalman: G = P_dash * C' * inv(W + C * P_dash * C')
    float temp_inv = 1.0f / (W + P_dash[0][0]);
    float G[2]; // Ganho de Kalman
    G[0] = P_dash[0][0] * temp_inv;
    G[1] = P_dash[1][0] * temp_inv;

    // 4. Atualização da estimativa do estado: theta_hat = theta_tilde + G * (theta_medido - C * theta_tilde)
    float erro = theta_medido - theta_tilde[0];
    theta_hat[0] = theta_tilde[0] + G[0] * erro;
    theta_hat[1] = theta_tilde[1] + G[1] * erro;

    // 5. Atualização da covariância do erro: P = (I - G * C) * P_dash
    P[0][0] = (1.0f - G[0]) * P_dash[0][0];
    P[0][1] = (1.0f - G[0]) * P_dash[0][1];
    P[1][0] = -G[1] * P_dash[0][0] + P_dash[1][0];
    P[1][1] = -G[1] * P_dash[0][1] + P_dash[1][1];

    // Retorna o ângulo estimado, que é o primeiro elemento do vetor de estado
    return theta_hat[0];
}
