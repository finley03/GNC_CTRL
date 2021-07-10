#ifndef MAT_H
#define MAT_H

#include <stdint.h>

int mat_multiply(float* matA, uint16_t Am, uint16_t An, float* matB, uint16_t Bm, uint16_t Bn, float* writeback);

void mat_transpose(float* mat, uint16_t m, uint16_t n, float* writeback);
void mat_transpose_sm(float* mat, uint16_t n);

void mat_add(float* matA, float* matB, uint32_t size, float* writeback);
void mat_subtract(float* matA, float* matB, uint32_t size, float* writeback);

void mat_scalar_product(float* mat, float scalar, uint32_t size, float* writeback);

void mat_copy(float* mat, uint32_t size, float* writeback);

float mat_det_2x2(float* mat);
float mat_det_3x3(float* mat);

void mat_inverse_2x2(float* mat, float* writeback);
void mat_inverse_3x3(float* mat, float* writeback);

void mat_LU_decompose(float* mat, uint16_t n, float* L, float* U);
void mat_LU_solve_n(float* L, float* U, float* vector, uint16_t n, float* writeback);
void mat_LU_inverse_n(float* L, float* U, float* workingVector, uint16_t n, float* writeback);
void mat_inverse_n(float* mat, uint16_t n, float* writeback);

void mat_crossp(float* mat1, float* mat2, float* writeback);

void mat_2_normalize(float* mat, float* writeback);
void mat_3_normalize(float* mat, float* writeback);

float poly_newton(float* coeff, uint16_t degree);

void mat_2_eigenvalues(float* mat, float* values);
void mat_3_eigenvalues(float* mat, float* values);

void mat_2_eigenvector(float* mat, float lambda, float* vector);
void mat_3_eigenvector(float* mat, float lambda, float* vector);


#endif