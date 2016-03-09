/*
 * predictive.c
 *
 *  Author: Jan Boucek
 */

#include "CMatrixLib.h"
#include "system.h"

typedef struct {

	vector_float * x0;
	matrix_float * Tr_reduced;	// size [T x 6]
	matrix_float * Tr_full;		// size [T x 2]

	float error_x;
	float error_y;

	float position_reference_x;
	float position_reference_y;

	float position_x;
	float position_y;

	int   obstacle_n;
	float obstacle_x[MAX_BLOBS];
	float obstacle_y[MAX_BLOBS];
	float obstacle_r[MAX_BLOBS];

	float max_speed;	// not used yet.
	int type;	// 1 for trajectory, 2 for reference, 0 for not initializes

} prHandler;

int pr_calculateMPC( prHandler * handler, float * action_x, float * action_y);

void prepare();

prHandler * initializePrHandler();

void compute_Tr_reduced(prHandler * handler);

void create_Tr_reference(prHandler * handler);

void test_handler();

void constraint_matrixes(const matrix_float * Av, matrix_float * Acx, matrix_float * Acy);

void test_predictive();

// returns 0 if A*u + B > 0, else index of the first wrong vector.
int check(const matrix_float * A, const vector_float * B, const vector_float * u);

// edits u0 vector for feasible input such as A*u0 + B < 0
void correct(const matrix_float * A, const vector_float * B, vector_float * u0);

void my_constraint(float * x0, float * y0, float * radius, float * k, float * q, float * sig);

// turns matrix [N x 2] into matrix [2*N x 1]
void u2Uv(matrix_float * u);

// turns matrix [2*N x 1] into matrix [N x 2]
void Uv2u(matrix_float * Uv);

int sign(float num);

float my_abs(float num);

void my_quadprog(const matrix_float * H, const matrix_float * c, const matrix_float * A, const vector_float * B, vector_float * u, const int obstacle_n);

void mountain_gradient(const matrix_float * A, const vector_float * B, const vector_float * u, vector_float * grad);

void cost_gradient(const matrix_float * H, const vector_float * c, const vector_float * u, vector_float * grad);

void run_simulation();
