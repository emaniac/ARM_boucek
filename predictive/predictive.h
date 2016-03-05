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

	int   obsticle_n;
	float obsticle_x[MAX_BLOBS];
	float obsticle_y[MAX_BLOBS];
	float obsticle_r[MAX_BLOBS];

	float max_speed;	// not used yet.
	int type;	// 1 for trajectory, 2 for reference, 0 for not initializes

} prHandler;

void string_print(const char * a);

void test_predictive();

void test_constraint();

int check (const matrix_float * A, const vector_float * B, const vector_float * u);
