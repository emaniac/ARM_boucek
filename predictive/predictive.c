/*
 * predictive.c
 *
 *  Author: Jan Boucek
 */

/* 	To do:
 *
 *  prejmenovat fci na pring a predavat hodnotu
 *  ctvrtek: vygenerovat vsechny matice a dat je do C.
 *
 *  where handler->max_speed is assigned?
 *  where the computed error is assigned?
 *
 */

// vTaskDelay

// vector orientation: 0 - height, 1 - width

#include "predictive.h"
#include "predictiveMatrices.h"
//#include "mpc.h"

//#define T 4	//prediction horizon, will sed all Ts?, 15
#define FLOAT_MAX 1000000

// constant parameters
#define pr_dt 0.01148
#define pr_q 300
#define pr_p 1e-6
#define MOUNTAIN_K 1000		// the value of mountain gradient function

#define GRAD_STEP 50	// grad step
#define grad_iterations 200

#define OBSTACLES 0		// if recieving obstacles
#define MAX_SPEED 1		// in m/dt
#define pr_B_k 0.000050719	// matrix B constant

// constants for testing
float final_x = 10;
float final_y = 7;
int first_time_running = 1;


// constant matrixes allocations, arrays in separate file predictiveMatrices.c

const matrix_float pr_A  = {  6,   6, (float*) pr_A_arr, "pr_A"};		//set
const matrix_float pr_B  = {  2,   6, (float*) pr_B_arr, "pr_B"};		//set
const matrix_float pr_test = {2, 2, (float*) pr_test_arr, "pt_test"};

const vector_float pr_block = {T,  1, (float*) pr_block_arr, "pr_block, which time predictions are computed with."};
const matrix_float pr_Av = {  6, 6*T, (float*) pr_Av_arr, "pr_Av"};	// in set_matrixes(), tested
const matrix_float pr_Bv = {2*T, 6*T, (float*) pr_Bv_arr, "pr_Bv"};	// in set_matrixes(), tested
const matrix_float pr_Hv = {2*T, 2*T, (float*) pr_Hv_arr, "pr_H"};	// done in set_matrixes(), nto tested
const vector_float pr_Pv = {2*T,   0, (float*) pr_Pv_arr, "pr_Pv, diag vector"};		// done in set_matrixes(), nto tested
const matrix_float pr_Qv = {6*T, 6*T, (float*) pr_Qv_arr, "pr_Qv"};
const matrix_float Qv_Bv_t={6*T, 2*T, (float*) Qv_Bv_t_arr, "Qv_Bv_t - (Qv*Bv)'"};

//	dynamic matrix allocations

float pr_Uv_arr[(2*T)];			//input vector
float pr_c_arr [(2*T)];			//column vector

vector_float pr_Uv = {2*T, 0, (float*) pr_Uv_arr, "pr_Uv"};
vector_float pr_c  = {2*T, 0, (float*) pr_c_arr,  "pr_c"};

float pr_Acx_arr[6*T];
float pr_Acy_arr[6*T];
float pr_Bcx_arr[T];
float pr_Bcy_arr[T];
float pr_Acy_x0_att[T];
float pr_Acy_x0_arr[T];
float pr_Av_x0_arr[6*T];

matrix_float pr_Acx = {6, T, (float*) pr_Acx_arr, "pr_Acx"};
matrix_float pr_Acy = {6, T, (float*) pr_Acy_arr, "pr_Acy"};
matrix_float pr_Bcx = {2*T, T, (float*) pr_Bcx_arr, "pr_Bcx"};
matrix_float pr_Bcy = {2*T, T, (float*) pr_Bcy_arr, "pr_Bcy"};
vector_float pr_Acy_x0 = {T, 0, (float*) pr_Acy_x0_arr, "pr_Acy_x0"};
vector_float pr_Av_x0 = {6*T, 0, (float*) pr_Av_x0_arr, "Av_x0"};

// handler properties
float pr_Tr_full_arr[2*LAST_TIME];		// computed trajectory
float pr_Tr_red_arr[6*T];		// desired trajectory
float pr_x0_arr[6];				// lost reference after inicialization?

matrix_float pr_Tr_red={6, T, (float*) pr_Tr_red_arr, "pr_Tr_red - dsired trajectory"};
matrix_float pr_Tr_full={2, LAST_TIME, (float*) pr_Tr_full_arr, "pr_Tr_full - UAV's trajectory"};
vector_float pr_x0 = {  6, 0, (float*) pr_x0_arr, "pr_x0"};

prHandler pr_handler;

int pr_calculateMPC( prHandler * handler, float * action_x, float * action_y) {
	usart4PutString("------pr_calculateMPC running 2.------\n\r");
	// need to allocate tr_reduced

	if(handler->type == 0) {			// uninitialized
		usart4PutString("ERROR in pr_calculateMPC() - uninitialized handler type.\n\r");
		return 0;
	} else if(handler->type == 1){		// trajectory
		compute_Tr_reduced(handler);
	} else{								// set point
		create_Tr_reference(handler);
		compute_Tr_reduced(handler);
	}
	if(!OBSTACLES){
		handler->obsticle_x = 1000;
		handler->obsticle_y = 1000;
		handler->obsticle_r = 1;
	}

	int i, N = handler->obsticle_n;

	/* CONSTRAINTS */

	// for all obstacles and all reduced states
	float Ac_arr[(N*T)*(2*T)];
	float Bc_arr[T*N];
	matrix_float Ac = {2*T, N*T, (float*) Ac_arr, "Ac - constraint matrix for all obstacles"};
	vector_float Bc = {N*T,   0, (float*) Bc_arr, "Bc - constraint matrix for all obstacles"};

	// for each obstacle
	float Ac_tmp_arr[T*(2*T)];
	float Bc_tmp_arr[T];
	matrix_float Ac_tmp = {2*T, T, (float*) Ac_tmp_arr, "Ac_tmp - constraint matrix for one obstacle"};
	vector_float Bc_tmp = {  T, 0, (float*) Bc_tmp_arr, "Bc_tmp - constraint matrix for one obstacle"};

	float k, q, sig;

	for(i = 0; i < N; i++){
		my_constraint(&(handler->obsticle_x[i]), &(handler->obsticle_y[i]), &(handler->obsticle_r[i]), &k, &q, &sig);

		// needs to be coputed, changing every loop
		constraint_matrixes(&pr_Av, &pr_Acx, &pr_Acy);
		constraint_matrixes(&pr_Bv, &pr_Bcx, &pr_Bcy);

		// compute Ac and Bc
		matrix_float_times(&pr_Bcx, -k);
		matrix_float_add(&pr_Bcx, &pr_Bcy);
		if(sig != 1) matrix_float_times(&pr_Bcx, sig);
		matrix_float_copy(&Ac_tmp, &pr_Bcx);

		matrix_float_mul_vec_right(&pr_Acx, &pr_x0, &Bc_tmp);
		vector_float_times(&Bc_tmp, -k);
		matrix_float_mul_vec_right(&pr_Acy, &pr_x0, &pr_Acy_x0);
		vector_float_add(&Bc_tmp, &pr_Acy_x0);
		vector_float_set_all(&pr_Acy_x0, -q);
		vector_float_add(&Bc_tmp, &pr_Acy_x0);
		if(sig != 1) vector_float_times(&Bc_tmp, sig);

		matrix_float_set_submatrix(&Ac, &Ac_tmp, 1, i*T+1);
		vector_float_set_subvector(&Bc, &Bc_tmp, i*T+1);
	}


//	reshape Tr
	vector_float Tr_vec = {T*6, 0, (float*) pr_Tr_red_arr, "Tr_vec - reshaped pr_Tr_red, pr_calculateMPC()"};


//  c = (Qv*Bv)'*(Av*x0-Tr_desired);
	matrix_float_mul_vec_right(&pr_Av, &pr_x0, &pr_Av_x0);
	vector_float_subtract(&pr_Av_x0, &Tr_vec);
	matrix_float_mul_vec_right(&Qv_Bv_t, &pr_Av_x0, &pr_c);

	my_quadprog(&pr_Hv, &pr_c, &Ac, &Bc, &pr_Uv);

	*action_x = vector_float_get(&pr_Uv, 1) - handler->error_x/pr_B_k;
	*action_y = vector_float_get(&pr_Uv, 2) - handler->error_y/pr_B_k;
}

void prepare(){
	//needs to be called first
	vector_float_set_zero(&pr_Uv);
	matrix_float_set_zero(&pr_Tr_red);
	matrix_float_set_zero(&pr_Tr_full);
}

prHandler * initializePrHandler() {
	pr_handler.x0 = &pr_x0;
	pr_handler.Tr_reduced = &pr_Tr_red;	// size [T x 2]
	pr_handler.Tr_full = &pr_Tr_full;

	pr_handler.error_x = 0;
	pr_handler.error_y = 0;

	pr_handler.position_reference_x = 0;
	pr_handler.position_reference_y = 0;

	float obsticle_x[MAX_BLOBS];
	float obsticle_y[MAX_BLOBS];
	float obsticle_r[MAX_BLOBS];

	pr_handler.obsticle_x = (float*)obsticle_x;
	pr_handler.obsticle_y = obsticle_y;
	pr_handler.obsticle_r = obsticle_r;
	pr_handler.obsticle_n = -1;

	if(first_time_running) {
		prepare();
		first_time_running = 0;
	}

	pr_handler.type = 0;	// 1 for trajectory, 2 for reference, 0 for not initializes
	return &pr_handler;
}

void compute_Tr_reduced(prHandler * handler){
	// done, not tested
	int i, full_idx;
	for(i = 1; i <= T; i++){
		full_idx = vector_float_get(&pr_block, i);
		matrix_float_set(handler->Tr_reduced, i, 1, matrix_float_get(handler->Tr_full, full_idx, 1));
		matrix_float_set(handler->Tr_reduced, i, 4, matrix_float_get(handler->Tr_full, full_idx, 2));
	}
}

void create_Tr_reference(prHandler * handler){
	// done, not tested
	float x0 = vector_float_get(&pr_x0, 1);
	float y0 = vector_float_get(&pr_x0, 4);
	float dx = (handler->position_reference_x-x0);
	float dy = (handler->position_reference_y-y0);
	float distance = sqrt(dx*dx+dy*dy);
	float dx_n = MAX_SPEED*(dx/distance);
	float dy_n = MAX_SPEED*(dy/distance);
	float lastTime = vector_float_get(&pr_block, T);
	int length, i;
	float step_dx, step_dy, step;

	if(distance/MAX_SPEED > lastTime){
		length = distance/MAX_SPEED;
	} else {
		length = lastTime;
	}
	for(i = 1; i <= T; i++){
		step = vector_float_get(&pr_block, i);
		step_dx = step*dx_n;
		if(abs(step_dx) < abs(dx)){
			matrix_float_set(&pr_Tr_red, i, 1, step_dx);
		} else{
			matrix_float_set(&pr_Tr_red, i, 1, dx);
		}
		step_dy = step*dy_n;
		if(abs(step_dy) < abs(dy)){
			matrix_float_set(&pr_Tr_red, i, 4, step_dy);
		} else{
			matrix_float_set(&pr_Tr_red, i, 4, dy);
		}
	}
}


void test_handler(){	//h
	// working
	prHandler handler;
	int i;
	prepare();

	float Tr_arr[2*T], x0_arr[6];
	matrix_float Tr = {2, T, (float *) Tr_arr, "Tr - desired trajectory from handler"};
	vector_float x0 = {6, 0, (float *) x0_arr, "x0 -  initial condition from handler"};
	for(i = 1; i <= T; i++){
		matrix_float_set(&Tr, i, 1, final_x*i/T);
		matrix_float_set(&Tr, i, 2, final_y*i/T);
	}
	vector_float_set_zero(&x0);

	handler.obsticle_x[0] = -1;
	handler.obsticle_y[0] = 5;
	handler.obsticle_r[0] = 1.5;
	handler.obsticle_n = 1;
	handler.x0 = &x0;
	handler.Tr_reduced = &Tr;


	float action_x, action_y;

	usart4PutString("s\n\r");
	pr_calculateMPC(&handler, &action_x, &action_y);
	usart4PutString("e\n\r");

//	usart_string_float_print("action_x = ", &action_x);
//	usart_string_float_print("action_y = ", &action_y);
}



void constraint_matrixes(const matrix_float * Av, matrix_float * Acx, matrix_float * Acy){
	// width Acx,Acy = width Av
	// height Acx,Acy = height Av / 6
	// tested, working

	if((Av->width!=Acx->width)||(Av->height!=6*Acx->height)||(Av->width!=Acy->width)||(Av->height!=6*Acy->height)){
		usart4PutString("ERROR in function constraint_matrixes(), wrong dimensions\n\r");
		return;
	}

	float tmp;
	int c, l;
	int C = (*Av).width;
	int L = (*Av).height/6;

	for(l = 1; l <= L; l++){
		for(c = 1; c <= C; c++){
			tmp = matrix_float_get(Av, l*6-5, c);
			matrix_float_set(Acx, l, c, tmp);
			tmp = matrix_float_get(Av, l*6-2, c);
			matrix_float_set(Acy, l, c, tmp);
		}
	}
}


void test_predictive(){		//p
	usart4PutString("Predictive class running 2.\n\r");
	float B_arr[] = {50, 60, 70};
	float u_arr[] = {4, 5, 6};
	float V_arr[] = {1, 2, 3, 4};
	float A_arr[] = {1, 2, 3, 4, 5, 6, 7, 8, 9};
	float Get_test_arr[] = {9, 9, 9};

	matrix_float A = {3, 3, (float*) &A_arr, "A_test_predictive"};
	vector_float vf = {4, 0, (float*) &V_arr, "Vector with orientation 1"};
	vector_float Get_test = {3, 0, (float*) &Get_test_arr, "Get Test, should not be 9,9,9"};

	matrix_float_print(&A);
	matrix_float_add_diag(&A, &Get_test);
	matrix_float_print(&A);
}


// returns 0 if A*u + B > 0, else index of the first wrong vector.
int check(const matrix_float * A, const vector_float * B, const vector_float * u){
	// tested, working

//	usart4PutString("check running.\n\r");
	int16_t N = B->length;
	float A_u_arr[N];	//array for A*u
	float Sum_arr[N];

	//getting the product of A*u into A_u
	vector_float A_u = {N, 0, (float*) A_u_arr, "A_u"};
	matrix_float_mul_vec_right(A, u, &A_u);

	//getting A*u + B into Sum
	vector_float_add(&A_u, B); //the final vector to A_u


	int correct = 1;
	int i;
	for(i = 1; i <= N; i++){
		float tmp = vector_float_get(&A_u, i);
		if(tmp >= 0){
			return i;
		}
	}
	return 0;
}



// edits u0 vector for feasible input such as A*u0 + B < 0
void correct(const matrix_float * A, const vector_float * B, vector_float * u0){
	// tested, working

	int index_test = check(A, B, u0);
	if(index_test == 0) return;

	int16_t N = u0->length;
	float d_arr[N];
	float W_arr[N];
	float k = 2;
	vector_float d = {N, 0, (float*) d_arr, "d"};
	vector_float W = {N, 0, (float*) W_arr, "W"};
	usart4PutChar("-");
	while(1){
		// finds first wrong constraint
//		usart_string_int_print("index_test ", index_test);

		// d = d - (dot(u0,w)+B(i))*w/(norm(w)^2);
		vector_float_set_zero(&d);
		matrix_float_get_row(A, &W, index_test);				// A(i, :), sets orientation 1
		W.orientation = 0;										// transpose

		float norm = vector_float_norm(&W);						// norm(w)
		norm = norm*norm;										// norm(w)^2, ok
		float df = vector_float_inner_product(u0, &W);  		// dot(u0,w), ok
		float Bi = vector_float_get(B, index_test);				// B(i), ok
		vector_float_times(&W, df + Bi);						// (dot(u0,w)+B(i))*w, ok
		vector_float_times(&W, 1/norm);							// (dot(u0,w)+B(i))*w/(norm(w)^2), ok
		vector_float_subtract(&d, &W);							// d = d - (dot(u0,w)+B(i))*w/(norm(w)^2), ok

		// u0 = u0 + k*d;
		vector_float_times(&d, k);								// d := k*d
		vector_float_add(u0, &d);								// u0 := u0 + k*d;

		index_test = check(A, B, u0);
		if(index_test == 0){
//			usart4PutString("condition OK, u corrected\n\r");
			return;
		}
	}
}

void my_constraint(float * x0, float * y0, float * radius, float * k, float * q, float * sig){
	// working

	float d, bx, by;

	float fx0 = *(x0);				// crazy
	float fy0 = *(y0);				// crazy
	float fradius = *(radius);		// crazy

	d = sqrt(fx0*fx0+fy0*fy0);
	bx = fx0-fradius*fx0/d;
	by = fy0-fradius*fy0/d;

	*k = -((float)fx0/(float)fy0);

	if(y0 > 0){
		*sig = 1;
	} else {
		*sig = -1;
	}

	*q = (by - (*k)*bx);
}

// turns matrix [N x 2] into matrix [2*N x 1]
void u2Uv(matrix_float * u){
	// tested, working

	u->height = u->height*u->width;
	u->width = 1;
}

// turns matrix [2*N x 1] into matrix [N x 2]
void Uv2u(matrix_float * Uv){
	// tested, working

	Uv->height = Uv->height/2;
	Uv->width = 2;
}

int sign(float num){
	// returns the sign of number
	if(num < 0){
		return -1;
	} else if (num > 0) {
		return 1;
	} else{
		return 0;
	}
}


// ------------------------------------------------- QUADRATIC PROGRAMMING ---------------------------------------------


void my_quadprog(const matrix_float * H, const matrix_float * c, const matrix_float * A, const vector_float * B, vector_float * u){

	int i, N = u->length;
	float grad_arr[N];
	float grad_mountain_arr[N];

	vector_float grad = {N, 0, (float*) grad_arr, "grad"};
	vector_float grad_mountain = {N, 0, (float*) grad_mountain_arr, "grad_mountain"};


	correct(A, B, u);
	for(i = 1; i <= grad_iterations; i++){

		cost_gradient(H, c, u, &grad);
		if(OBSTACLES) {
			mountain_gradient(A, B, u, &grad_mountain);
			vector_float_add(&grad, &grad_mountain);
		}
		vector_float_times(&grad, GRAD_STEP);
		vector_float_subtract(u, &grad);
		correct(A, B, u);
	}
}



void mountain_gradient(const matrix_float * A, const vector_float * B, const vector_float * u, vector_float * grad){
	// done, tested

	if((u->length!=grad->length)||(u->orientation!=0)||(grad->orientation!=0)||(A->height!=B->length)||(A->width!=u->length)){
		usart4PutString("ERROR in mountain_grad(), dimensions do not agree.\n\r");
	}

	grad->name = "mountain gradieng";

	int i;
	int N = B->length;
	float W_arr[N], dot_W_u, B_i, sig, norm_w;
	vector_float W = {N, 0, (float*) W_arr, "W"};


	float tmp_arr[N];
	vector_float tmp = {N, 0, (float*) tmp_arr, "W"};
	vector_float_set_zero(grad);

	for(i = 1; i <= B->length; i++){
		matrix_float_get_row(A, &W, i);							// w = A(i, :), sets orientation 1
		W.orientation = 0;										// transpose;

		dot_W_u = vector_float_inner_product(&W, u);			// dot(w,u)
		B_i = vector_float_get(B, i);

		sig = sign(dot_W_u+B_i);								// may work
		norm_w = vector_float_norm(&W);							// norm(w)

		// w = (w)*(norm(w)*sign(dot(w,u)+B(i))/((dot(w,u)+B(i))^2)
		vector_float_times(&W, (MOUNTAIN_K*norm_w*sig/((dot_W_u+B_i)*(dot_W_u+B_i))));
		vector_float_subtract(grad, &W);						// d = d - W
	}
}

void cost_gradient(const matrix_float * H, const vector_float * c, const vector_float * u, vector_float * grad) {
	// done, not tested
	// df = @(u) H*u+c;                % gradient of cost function

	if((H->width != u->length) || (grad->length != H->height) || (u->orientation != 0) || (c->orientation != 0) || grad->orientation != 0){
		usart4PutString("ERROR in cost_gradient(), dimentions does not agree\n\r");
		return;
	}
	vector_float_set_zero(grad);			// added
	matrix_float_mul_vec_right(H, u, grad);
	vector_float_add(grad, c);
}

void run_simulation(){	// r
	// working as in Matlab

	/*
	usart4PutString("----------------Run_simulation running 2.-----------------------------\n\r");

	float obsticle_x = -1;
	float obsticle_y = 5;
	float obsticle_radius = 1.5;

	int i;
	vector_float_set_zero(&pr_x0);

	// create trajectory, done
	matrix_float_set_zero(&pr_Tr_red);
	for(i = 1; i <= T; i++){
		matrix_float_set(&pr_Tr_red, i, 1, final_x*i/T);
		matrix_float_set(&pr_Tr_red, i, 4, final_y*i/T);
	}


	*//* CONSTRAINTS *//*

	constraint_matrixes(&pr_Av, &pr_Acx, &pr_Acy);
	constraint_matrixes(&pr_Bv, &pr_Bcx, &pr_Bcy);

	float k, q, sig;
	my_constraint(&obsticle_x, &obsticle_y, &obsticle_radius, &k, &q, &sig);

//	Ac = sig*(-k*Bcx + Bcy);
//	Bc = sig*(-k*Acx*x0 + Acy*x0 - q);

	matrix_float_times(&pr_Bcx, -k);
	matrix_float_add(&pr_Bcx, &pr_Bcy);
	if(sig != 1) matrix_float_times(&pr_Bcx, sig);
	matrix_float_copy(&pr_Ac, &pr_Bcx);

	matrix_float_mul_vec_right(&pr_Acx, &pr_x0, &pr_Bc);
	vector_float_times(&pr_Bc, -k);
	matrix_float_mul_vec_right(&pr_Acy, &pr_x0, &pr_Acy_x0);
	vector_float_add(&pr_Bc, &pr_Acy_x0);
	vector_float_set_all(&pr_Acy_x0, -q);
	vector_float_add(&pr_Bc, &pr_Acy_x0);
	if(sig != 1) vector_float_times(&pr_Bc, sig);

//	Tr_desired=reshape(Tr_desired',lastTime*6, 1)
	vector_float Tr_vec = {T*6, 0, (float*) pr_Tr_red_arr, "Tr_vec - reshaped pr_Tr_red, run_simulation"};
	vector_float_print(&pr_x0);

//  c = (Qv*Bv)'*(Av*x0-Tr_desired);
	matrix_float_mul_vec_right(&pr_Av, &pr_x0, &pr_Av_x0);
	vector_float_subtract(&pr_Av_x0, &Tr_vec);
	matrix_float_mul_vec_right(&Qv_Bv_t, &pr_Av_x0, &pr_c);

	vector_float_set_all(&pr_Uv, 0);				// should be pr_Uv from last computing
	my_quadprog(&pr_Hv, &pr_c, &pr_Ac, &pr_Bc, &pr_Uv);

	vector_float_print(&pr_Uv);
	*/
}


