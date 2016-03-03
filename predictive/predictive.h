/*
 * predictive.c
 *
 *  Author: Jan Boucek
 */

#include "commTask.h"

typedef struct {

	vector_float * initial_cond;
	matrix_float * Tr_reduced;	// size [T x 2]
	matrix_float * Tr;

	float position_reference_x;
	float position_reference_y;

	float obsticle_x;
	float obsticle_y;
	float obsticle_radius;

	int type;	// 1 for trajectory, 2 for reference

} prHandler;

void string_print(const char * a);

void test_predictive();

void test_constraint();

int check (const matrix_float * A, const vector_float * B, const vector_float * u);
