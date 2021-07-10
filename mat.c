#include "mat.h"


#include <math.h>


#define ABS(a) ((a < 0) ? -a : a)
#define MAX_2(a, b) ((a > b) ? a : b)
#define MAX_3(a, b, c) (MAX_2(MAX_2(a, b), c))
#define UMAX_2(a, b) MAX_2(ABS(a), ABS(b))
#define UMAX_3(a, b, c) MAX_3(ABS(a), ABS(b), ABS(c))


void mat_identity(uint16_t n, float* writeback) {
	for (uint32_t i = 0; i < n * n; ++i) {
		writeback[i] = 0;
	}

	for (uint16_t i = 0; i < n; ++i) {
		writeback[i * n + i] = 1;
	}
}



// matrix multiply
// requires row major input
// gives row major output
// returns 0 for success, other values for failures
int mat_multiply(float* matA, uint16_t Am, uint16_t An, float* matB, uint16_t Bm, uint16_t Bn, float* writeback) {
	// check matrices can be multiplied
	if (An != Bm) return -1;

	// iterate through different positions
	for (uint16_t n = 0; n < Bn; ++n) {
		for (uint16_t m = 0; m < Am; ++m) {
			float sum = 0;

			for (uint16_t i = 0; i < An; ++i) {
				// set multiply variables
				float a = matA[i + (An * m)];
				float b = matB[(i * Bn) + n];

				// skip optimizations
				if (
				// zero
				(a == 0 || b == 0)
				) {
					continue;
				}
				
				// multiply accumulate
				sum += a * b;
			}

			// set value in writeback array
			writeback[n + (m * Bn)] = sum;
		}
	}

	return 0;
}


void mat_transpose(float* mat, uint16_t m, uint16_t n, float* writeback) {
	for (uint16_t i = 0; i < m; ++i) {
		for (uint16_t j = 0; j < n; j++) {
			writeback[i + (j * m)] = mat[j + (i * n)];
		}
	}
}


// transposes a square matrix in place
void mat_transpose_sm(float* mat, uint16_t n) {
	// rows
	for (uint16_t i = 0; i < n - 1; ++i) {
		// columns
		for (uint16_t j = i + 1; j < n; ++j) {
			float val1 = mat[i * n + j];
			float val2 = mat[j * n + i];
			mat[j * n + i] = val1;
			mat[i * n + j] = val2;
		}
	}
}


void mat_add(float* matA, float* matB, uint32_t size, float* writeback) {
	for (uint32_t i = 0; i < size; ++i) {
		writeback[i] = matA[i] + matB[i];
	}
}


void mat_subtract(float* matA, float* matB, uint32_t size, float* writeback) {
	for (uint32_t i = 0; i < size; ++i) {
		writeback[i] = matA[i] - matB[i];
	}
}


void mat_scalar_product(float* mat, float scalar, uint32_t size, float* writeback) {
	for (uint32_t i = 0; i < size; ++i) {
		writeback[i] = mat[i] * scalar;
	}
}


void mat_copy(float* mat, uint32_t size, float* writeback) {
	for (uint32_t i = 0; i < size; ++i) {
		writeback[i] = mat[i];
	}
}


float mat_det_2x2(float* mat) {
	return (mat[0] * mat[3]) - (mat[1] * mat[2]);
}


float mat_det_3x3(float* mat) {
	float plus = (mat[0] * mat[4] * mat[8]) + (mat[1] * mat[5] * mat[6]) + (mat[2] * mat[3] * mat[7]);
	float minus = (mat[2] * mat[4] * mat[6]) + (mat[0] * mat[5] * mat[7]) + (mat[1] * mat[3] * mat[8]);
	return plus - minus;
}





void mat_inverse_2x2(float* mat, float* writeback) {
	float det = mat_det_2x2(mat);
	float invdet = 1 / det;

	writeback[0] = mat[3] * invdet;
	writeback[1] = -mat[1] * invdet;
	writeback[2] = -mat[2] * invdet;
	writeback[3] = mat[0] * invdet;
}


void mat_inverse_3x3(float* mat, float* writeback) {
	float det = mat_det_3x3(mat);
	float invdet = 1 / det;
	
	// create cofactor matrix
	float cofactors[9];
	
	cofactors[0] = mat[4] * mat[8] - mat[5] * mat[7];
	cofactors[1] = mat[5] * mat[6] - mat[3] * mat[8];
	cofactors[2] = mat[3] * mat[7] - mat[4] * mat[6];
	cofactors[3] = mat[2] * mat[7] - mat[1] * mat[8];
	cofactors[4] = mat[0] * mat[8] - mat[2] * mat[6];
	cofactors[5] = mat[1] * mat[6] - mat[0] * mat[7];
	cofactors[6] = mat[1] * mat[5] - mat[2] * mat[4];
	cofactors[7] = mat[2] * mat[3] - mat[0] * mat[5];
	cofactors[8] = mat[0] * mat[4] - mat[1] * mat[3];
	
	float cofactors_t[9];
	
	// transpose cofactors
	mat_transpose(cofactors, 3, 3, cofactors_t);
	
	// multiply by inverse determinant
	mat_scalar_product(cofactors_t, invdet, 9, writeback);
}


// decomposes matrix into L and U triangular matrices
void mat_LU_decompose(float* mat, uint16_t n, float* L, float* U) {
	// set L matrix to identity
	mat_identity(n, L);

	// copy input to U matrix
	// U matrix is working matrix
	mat_copy(mat, n * n, U);

	// copy first row of input to U matrix
	for (uint16_t i = 0; i < n; ++i) {
		U[i] = mat[i];
	}

	// iterate through pivots doing gaussian elimination
	// to create L and U triangular matrices
	// iterates to n - 1 because gaussian elimination is not
	// performed on the last column
	for (uint16_t i = 0; i < n - 1; ++i) {
		// iterate over the rows for gaussian elimination
		for (uint16_t j = i + 1; j < n; ++j) {
			// calculate multiplier for row
			// i increments columns, j increments rows
			float multiplier = U[i + j * n] / U[i * n + i];

			//			printf("%f\n", U[i * n + i]);

			// set multiplier in L matrix
			L[i + j * n] = multiplier;

			// loop through numbers in rows in U matrix
			for (uint16_t k = 0; k < n; ++k) {
				U[k + j * n] -= U[k + i * n] * multiplier;
			}
		}

	}
}


// finds matrix inverse through LU decomposition
// you have to pass a vector for the function to work with
void mat_LU_inverse_n(float* L, float* U, float* workingVector, uint16_t n, float* writeback) {
	// loop through dimensions
	// solve columns individually
	// and place in rows
	for (uint16_t i = 0; i < n; ++i) {
		for (uint16_t j = 0; j < n; ++j) {
			workingVector[j] = 0;
		}
		workingVector[i] = 1;

		mat_LU_solve_n(L, U, workingVector, n, writeback + i * n);
	}

	// transpose final matrix to put rows into columns
	mat_transpose_sm(writeback, n);
}


// solves system of equations Ax = b through LU decomposition
void mat_LU_solve_n(float* L, float* U, float* vector, uint16_t n, float* writeback) {
	// first let y = Ux where x is vector
	// writeback will be working space
	for (uint16_t i = 0; i < n; ++i) {
		float value = vector[i];

		for (uint16_t j = 1; j <= i; ++j) {
			value -= L[i * n + i - j] * writeback[i - j];
		}

		writeback[i] = value;
	}

	// transfer y from writeback to vector
	for (uint16_t i = 0; i < n; ++i) {
		vector[i] = writeback[i];
	}

	// now solve Ux = y with y in vector
	// write x to writeback
	// test for i < n because once 0 is decremented it goes to max value
	for (uint16_t i = n - 1; i < n; --i) {
		float value = vector[i];

		for (uint16_t j = n - 1; j > i; --j) {
			value -= U[i * n + j] * writeback[j];
		}

		value /= U[i * n + i];

		writeback[i] = value;
	}
}


void mat_crossp(float* mat1, float* mat2, float* writeback) {
	writeback[0] = mat1[1] * mat2[2] - mat1[2] * mat2[1];
	writeback[1] = mat1[2] * mat2[0] - mat1[0] * mat2[2];
	writeback[2] = mat1[0] * mat2[1] - mat1[1] * mat2[0];
}


void mat_2_normalize(float* mat, float* writeback) {
	float scalar = 1 / sqrt(mat[0] * mat[0] + mat[1] * mat[1]);

	writeback[0] = mat[0] * scalar;
	writeback[1] = mat[1] * scalar;
}

void mat_3_normalize(float* mat, float* writeback) {
	float scalar = 1 / sqrt(mat[0] * mat[0] + mat[1] * mat[1] + mat[2] * mat[2]);
	
	writeback[0] = mat[0] * scalar;
	writeback[1] = mat[1] * scalar;
	writeback[2] = mat[2] * scalar;
}


// runs newton's method on given polynomial
// must have degree of at least 1
// in format A + Bx + Cx^2 ...
float poly_newton(float* coeff, uint16_t degree) {
	float last = -1;
	float a = 1;
	// iteratively find a root
	while (ABS((last - a)) > 1E-5) {
		// assign last
		last = a;
		// find new value
		//
		// evaluate polynomial
		float fx = 0;
		fx += coeff[0];
		float apow = 1;
		for (uint16_t i = 1; i <= degree; ++i) {
			apow *= a;
			fx += coeff[i] * apow;
		}

		// evaluate derivative
		float fxdx = 0;
		fxdx += coeff[1];
		apow = 1;
		for (uint16_t i = 2; i <= degree; ++i) {
			apow *= a;
			fxdx += coeff[i] * i * apow;
		}

		a = a - (fx / fxdx);
	}

	return a;
	
}


void mat_2_eigenvalues(float* mat, float* values) {
	float a = 1;
	float b = -(mat[0] + mat[3]);
	float c = mat[0] * mat[3] - mat[1] * mat[2];
	float discriminant = sqrt(b * b - 4 * a * c);
	values[0] = (-b + discriminant) / (2 * a);
	values[1] = (-b - discriminant) / (2 * a);
}


void mat_3_eigenvalues(float* mat, float* values) {
	// construct third degree polynomial
	// Ax^3 + Bx^2 + Cx + D
	// det(A - lambda I)
	float A = -1;
	float B = mat[0] + mat[4] + mat[8];
	float C = -(mat[0] * mat[4] + mat[0] * mat[8] + mat[4] * mat[8]) +
	(mat[1] * mat[3] + mat[2] * mat[6] + mat[5] * mat[7]);
	float D = mat_det_3x3(mat);

	// construct vector to get root
	float coeff3[4] = {D, C, B, A};
	values[0] = poly_newton(coeff3, 3);

	// divide polynomial to get quadratic
	// A is unchanged
	B = -values[0] + B;
	C = B * values[0] + C;
	// ignore D
	
	// solve quadratic for final two roots
	float discriminant = sqrt(B * B - 4 * A * C);
	values[1] = (-B + discriminant) / (2 * A);
	values[2] = (-B - discriminant) / (2 * A);
}


void mat_2_eigenvector(float* mat, float lambda, float* vector) {
	vector[0] = 1;
	vector[1] = -(mat[0] - lambda) / mat[1];
	mat_2_normalize(vector, vector);
}


void mat_3_eigenvector(float* mat, float lambda, float* vector) {
	// there are infinitely many solutions, so one value must be initialized
	vector[0] = 1;
	
	// find value of bv2 + cv3 in (a - lambda)v1 + bv2 + cv3 = 0
	float bv2cv3 = lambda - mat[0];
	// find value of (e - lambda)v2 + fv3 in dv1 + (e - lambda)v2 + cv3 = 0
	float elv2fv3 = -mat[3];

	// subtract elv2fv3 from bv2cv3 and calculate v2
	vector[1] = (bv2cv3 - (mat[2] / mat[5]) * elv2fv3) / (mat[1] - (mat[4] - lambda) * (mat[2] / mat[5]));
	vector[2] = -(mat[1] * vector[1] + mat[0] - lambda) / mat[2];

	mat_3_normalize(vector, vector);
}