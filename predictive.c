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
 */

// vTaskDelay

// vector orientation: 0 - height, 1 - width

#include "predictive.h"
#include "commTask.h"
//#include "mpc.h"

//#define T 4	//prediction horizon, will sed all Ts?, 15
#define FLOAT_MAX 1000000

// constants
#define pr_dt 0.01148
#define pr_step 10
#define pr_q 300
#define pr_p 1e-6

// constants for testing
float obsticle_x = 0;
float obsticle_y = 5;
float obsticle_radius = 1.5;
float final_x = 10;
float final_y = 7;


// constant matrixes allocations

const float pr_A_arr[6*6] = {1,pr_dt*pr_step,0,0,0,0, 0,1,pr_dt*pr_step,0,0,0, 0,0,(float)(pow(0.9799, pr_step)),0,0,0, 0,0,0,1,pr_dt*pr_step,0, 0,0,0,0,1,pr_dt*pr_step, 0,0,0,0,0,(float)(pow(0.9799, pr_step))};
const float pr_B_arr[6*2] = {0, 0, 0, 0, 5.0719e+0, 0, 0, 0, 0, 0, 0, 5.0719e+0};	// ERROR!!!! needs to be edited to 'e-5'

/*	in predictiveMatrixes.c
const float pr_block_arr[T] = {1, 2, 5, 10};	//has to fit
const float pr_Av_arr[(6*T)*(6)];				//extended A
const float pr_Bv_arr[(6*T)*(2*T)];				//extended B
const float pr_Hv_arr[(2*T)*(2*T)];
const float pr_Qv_arr[6*T*6*T];
const float pr_Pv_arr[2*T*2*T];					// diag
*/

const matrix_float pr_A  = {  6,   6, (float*) pr_A_arr, "pr_A"};		//set
const matrix_float pr_B  = {  2,   6, (float*) pr_B_arr, "pr_B"};		//set


const vector_float pr_block = {T,  1, (float*) pr_block_arr, "pr_block, which time predictions are computed with."};
const matrix_float pr_Av = {  6, 6*T, (float*) pr_Av_arr, "pr_Av"};	// in set_matrixes(), tested
const matrix_float pr_Bv = {2*T, 6*T, (float*) pr_Bv_arr, "pr_Bv"};	// in set_matrixes(), tested
const matrix_float pr_Hv = {2*T, 2*T, (float*) pr_Hv_arr, "pr_H"};	// done in set_matrixes(), nto tested
const vector_float pr_Pv = {2*T,   0, (float*) pr_Pv_arr, "pr_Pv, diag vector"};		// done in set_matrixes(), nto tested
const matrix_float pr_Qv = {6*T, 6*T, (float*) pr_Qv_arr, "pr_Qv"};


//	dynamic matrix allocations

float pr_Uv_arr[(2*T)];			//input vector
float pr_Ac_arr[T*(2*T)];
float pr_Bc_arr[T];
float pr_c_arr [(2*T)];			//column vector
float pr_x0_arr[6];
float pr_Tr_uav_arr[2*T];		// computed trajectory
float pr_Tr_des_arr[2*T];		// desired trajectory

vector_float pr_Uv = {2*T, 0, (float*) pr_Uv_arr, "pr_Uv"};
vector_float pr_c  = {2*T, 0, (float*) pr_c_arr,  "pr_c"};
matrix_float pr_Ac = {2*T, T, (float*) pr_Ac_arr, "pr_Ac"};
vector_float pr_Bc = {  T, 0, (float*) pr_Bc_arr, "pr_Bc"};
vector_float pr_x0 = {  6, 0, (float*) pr_x0_arr, "pr_x0"};
matrix_float pr_Tr_des={6,   T, (float*) pr_Tr_des_arr, "pr_Tr_des - dsired trajectory"};
matrix_float pr_Tr_uav={6,   T, (float*) pr_Tr_uav_arr, "pr_Tr_uav - UAV's trajectory"};

float Qv_Bv_t_arr[6*T*6*T];
matrix_float Qv_Bv_t = {6*T, 6*T, (float*) Qv_Bv_t_arr, "Qv_Bv_t"};


void run_simulation(){

	int i;
	create_Av();
	create_Bv();
	vector_float_set_zero(&pr_x0);

	// create trajectory, done, not tested
	matrix_float_set_zero(&pr_Tr_des);
	for(i = 1; i <= T; i++){
		matrix_float_set(&pr_Tr_des, i, 1, final_x*i/T);
		matrix_float_set(&pr_Tr_des, i, 4, final_y*i/T);
	}
	matrix_float_print(&pr_Tr_des);

	/* CONSTRAINTS */

	float Acx_arr[6*T];
	float Acy_arr[6*T];
	float Bcx_arr[T];
	float Bcy_arr[T];
	float Acy_x0_att[T];

	matrix_float Acx = {6, T, (float*) Acx_arr, "Acx"};
	matrix_float Acy = {6, T, (float*) Acy_arr, "Acy"};
	matrix_float Bcx = {1, T, (float*) Bcx_arr, "Bcx"};
	matrix_float Bcy = {1, T, (float*) Bcy_arr, "Bcy"};
	vector_float Acy_x0 = {T, 0, (float*) Acy_x0_arr, "Acy_x0"};

	constraint_matrixes(&pr_Av, &Acx, &Acy);
	constraint_matrixes(&pr_Bv, &Bcx, &Bcy);


	float k, q, sig;
	my_constraint(&obsticle_x, &obsticle_y, &obsticle_radius, &k, &q, &sig);


//	Ac = sig*(-k*Bcx + Bcy);
//	Bc = sig*(-k*Acx*x0 + Acy*x0 - q);, done, not tested

	matrix_float_times(&Bcx, -k);
	matrix_float_add(&Bcx, Bcy);
	if(sig != 1) matrix_float_times(&Bcx, sig);

	matrix_float_mul_vec_right(&Acx, &pr_x0, &pr_Bc);
	vector_float_times(&pr_Bc, -k);
	matrix_float_mul_vec_right(&Acy, &x0, &Acy_x0);
	vector_float_add(&Bc, &Acy_x0);
	vector_float_set_all(&Acy_x0, -q);
	vector_float_add(&pr_Bc, &Acy_x0);
	if(sig != 1) vector_float_times(&pr_Bc, sig);

	/* TRAJECTORY */

//	Tr_desired=reshape(Tr_desired',lastTime*6, 1)
	float Tr_cp_arr[6*T];
	matrix_float Tr_cp = {T, 6, (float*) Tr_cp_arr, "Tr_cp"};
	matrix_float_transpose(pr_Tr_des, Tr_cp);
	vector_float Tr_vec = {T*6, 0, (float*) Tr_cp_arr, "Tr_vec - reshaped pr_Tr_des"};

//  c = (Qv*Bv)'*(Av*x0-Tr_desired);
	float Av_x0_arr[6*T];
	vector_float Av_x0 = {6*T, 0, (float*) Av_x0_arr, "Av_x0"};
	matrix_float_mul_vec_right(&Av, &x0, &Av_x0);
	vector_float_subtract(Av_x0, Tr_vec);

// (Qv*Bv)'*...
	float tmp;
	int x, y;
	for(y = 1; y <= 6*T; y++){
		if(y % 3 != 0) continue;
		for(x = 0; x <= 6*T; x++){
			tmp = matrix_float_get(&pr_Bv, y, x)*pr_q;
			matrix_float_set(&Qv_Bv_t, x, y, tmp);
		}
	}




}





void set_matrixes(){	//s



//	pr_A_arr = {1, pr_dt};//, 0, 0, 1, pr_dt, 0, 0, pow((double)0.9799, (double)pr_step)};
//	const float X3_arr[8];
//	matrix_float X3 = {2, 4, (float*) X3_arr, "X"};
//	matrix_float_set_all(&X3, 7);
//
//	const float Y_arr[] = {1, 2, 3, 4};
//	matrix_float Y = {2, 2, (float*) Y_arr, "Y"};
//	matrix_float_print(&Y);
//	matrix_float_set_submatrix(&X3, &Y, 1, 4);
//
//	matrix_float_print(&X3);

	/*
	create_Av();
	create_Bv();
	*/

	// SETTING Qv //

	matrix_float_set_zero(pr_Qv);
	int i;
	for(i = 1; i <= T; i++){
		if(i%3 == 0) matrix_float_set(&pr_Qv, i, i, pr_q);
	}

	matrix_float_print(&Qv);

	// SETTING Pv //
	float Pv_arr[6*6];
	matrix_float Pv = {6, 6, (float*) Pv_arr, "Pv"};
	matrix_float_set_zero(&Pv);

	for(i = 1; i <= 6; i++){
		matrix_float_set(&Pv, i, i, pr_p);
	}

	// SETTING Hv, Hv=Bv'*Qv*Bv+Pv; //
	float Qv_Bv_arr[T*6*6];
	matrix_float Qv_Bv = {6, T*3, (float*) Qv_Bv_arr, "Qv_Bv"};
	float Bv_tr_arr[(6*T)*(2*T)];
	matrix_float Bv_tr = {6*T, 2*T, (float*) Bv_tr_arr, "Bv_tr"};
	matrix_float_transpose(&pr_Bv, &Bv_tr);

	matrix_float_mul(&Qv, &pr_Bv, &Qv_Bv);
	matrix_float_mul(&Bv_tr, &Qv_Bv, &pr_Hv);
	matrix_float_mul(&Bv_tr, &Qv_Bv, &pr_Hv);
	matrix_float_add(&Hv, &Pv);			// Hv=Bv'*Qv*Bv+Pv;, not tested;

	usart4PutString("Ending set_matrixes()\n\r");
}

void create_Bv(){
	// done, working
//	[x, y]=size(A);
//	Av=zeros(siz*x,y);
//	for i=1:siz        %line
//	    Av(((i-1)*x+1):x*i,:)=A^i;
//	end
//	matrix_float pr_Bv = {2*T, 6*T, (float*) pr_Bv_arr, "pr_Bv"};

	int X = pr_Bv.width;
	int Y = pr_Bv.height;
	int pos_y;

	float A_tmp_arr[6*6], tmp;
	float A_cp_arr[6*6];
	float A_b_arr[6*2];
	matrix_float A_tmp = {6, 6, (float*) A_tmp_arr, "A_tmp"};
	matrix_float A_cp = {6, 6, (float*) A_cp_arr, "A_cp"};
	matrix_float A_b = {2, 6, (float*) A_b_arr, "A_b"};

	matrix_float_set_zero(&pr_Av);

	int x, y;
	for(x = 0; x < T; x++){
		matrix_float_set_identity(&A_tmp);
		for(y = x; y < T; y++){
			if(x == y){
				matrix_float_set_submatrix(&pr_Bv, &pr_B, x*2+1, y*6+1);
				continue;
			}
			matrix_float_mul(&A_tmp, &pr_A, &A_cp);
			matrix_float_copy(&A_tmp, &A_cp);
			matrix_float_mul(&A_tmp, &pr_B, &A_b);
			matrix_float_set_submatrix(&pr_Bv, &A_b, x*2+1, y*6+1);
		}
	}
}

void create_Av(){
	// tested, working

	int X = pr_Av.width;
	int Y = pr_Av.height;

	float A_tmp_arr[6*6], tmp;
	matrix_float A_tmp = {6, 6, (float*) A_tmp_arr, "A_tmp"};
	float A_cp_arr[6*6], tmp2;
	matrix_float A_cp = {6, 6, (float*) A_cp_arr, "A_cp"};
	matrix_float_copy(&A_tmp, &pr_A);
	matrix_float_copy(&A_cp, &pr_A);

	int x, y, i;
	for(i = 0; i < T; i++){
		matrix_float_set_submatrix(&pr_Av, &A_tmp, 1, i*6+1);
		matrix_float_mul(&pr_A, &A_cp, &A_tmp);
		matrix_float_copy(&A_cp, &A_tmp);
	}
}


void string_print(const char * a) {
	usart4PutString("a\n\n");
}



void test_predictive(){		//p
	usart4PutString("Predictive class running 2.\n\r");
//	test_check();
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

void test_check(){	//t
	//testing function check
	usart4PutString("test check runing\n\r");
	float A_arr[] = {1,-2,2,-4,3,-6,4,-8,5,-10,6,-12};
//	float A_arr[120];
	float B_arr[] = {-100, -100, -100};
	float u_arr[] = {8, 9, 11, 4};
	float Acx_arr[5*4];
	float Acy_arr[5*4];

	matrix_float A = {2, 6, (float*) A_arr, "A - test_check"};
	vector_float B = {3, 0, (float*) B_arr, "B - test_check"};
	vector_float u = {4, 0, (float*) u_arr, "u - before correction"};
	matrix_float Acx = {5, 4, (float*) Acx_arr, "Acx - test_check"};
	matrix_float Acy = {5, 4, (float*) Acy_arr, "Acy - test_check"};

	matrix_float_print(&A);
	u2Uv(&A);
	matrix_float_print(&A);
	Uv2u(&A);
	matrix_float_print(&A);

	return;
	vector_float_print(&B);
	vector_float_print(&u);
	int wrong_index = check(&A, &B, &u);
	usart_string_int_print("wrong index = ", wrong_index);
	vector_float_print(&u);

	correct(&A, &B, &u);
	u.name = "u - after correction";
	vector_float_print(&u);

//	usart_string_float_print("answer check: ", ans);
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


// probably infinite loop
// edits u0 vector for feasible input such as A*u0 + B < 0
void correct(const matrix_float * A, const vector_float * B, vector_float * u0){
	// tested, working

	usart4PutString("Correcting running.\n\r");
	int index_test = check(A, B, u0);
	if(index_test == 0){
		usart4PutString("condition OK, u corrected\n\r");
		return;
	}

	int16_t N = u0->length;
	float d_arr[N];
	float W_arr[N];
	float k = 2;
	vector_float d = {N, 0, (float*) d_arr, "d"};
	vector_float W = {N, 0, (float*) W_arr, "W"};

	while(1){
		// finds first wrong constraint
		usart_string_int_print("index_test ", index_test);

		// d = d - (dot(u0,w)+B(i))*w/(norm(w)^2);
		vector_float_set_zero(&d);
		matrix_float_get_row(A, &W, index_test);				// A(i, :), sets orientation 1
		W.orientation = 0;										// transpose
		vector_float_print(&W);

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
			usart4PutString("condition OK, u corrected\n\r");
			return;
		}
	}
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


void my_quadprog(const matrix_float * H, const matrix_float * c, const matrix_float * A, const vector_float * B, vector_float * u, const int iterations, const int step){

//	for i = 1:iterations
//	    k = 1;
//	    tmp_grad = grad(u);
//	    u = u - k*step*tmp_grad;
//	    if my_check(A, B, u) < 1
//	          u = correct(A, B, u);
//	          if my_check(A, B, u) < 1
//	            error('mimo rozsah')
//	          end
//	    end
//	   %[i f(u) g(u)];% cost(u) my_check(A, B, u)]
//	end

	int i, N = u->length;
	float grad_arr[N];
	float grad_mountain_arr[N];

	vector_float grad = {N, 0, (float*) grad_arr, "grad"};
	vector_float grad_mountain = {N, 0, (float*) grad_mountain_arr, "grad_mountain"};
	vector_float_set_zero(&grad);
	vector_float_set_zero(&grad_mountain);

	correct(A, B, u);
	for(i = 1; i <= iterations; i++){
		usart_string_int_print("my_quadprog, iterations = ", iterations);
		usart_string_int_print("my_quadprog, i = ", i);
		cost_gradient(H, c, u, &grad);
		mountain_gradient(A, B, u, &grad_mountain);
		vector_float_add(&grad, &grad_mountain);
		vector_float_times(&grad, step);
		vector_float_subtract(u, &grad);
		correct(A, B, u);
	}


}



void mountain_gradient(const matrix_float * A, const vector_float * B, const vector_float * u, vector_float * grad){
	// done, not tested
/* function g = mountain_gradient(A, B, u)
    % returns [1 x 6] gradient of all constraining mountains
    % inputs: A, [time x 6]
    %         B, [time x 1]
    %         u, [time x 1], state
    % for A*u + B = 0;
    k = 1000;    % 500
    g = zeros(size(A,2), 1);     % [1 x 6]
    for i = 1:length(B)
       w = A(i,:)';
       g = g - k*(norm(w).*w)/((dot(w,u)+B(i))^2)*sign(dot(w,u)+B(i));
       g = g - (w)*(k*norm(w)*sign(dot(w,u)+B(i))/((dot(w,u)+B(i))^2)
    end
	end
*/

	if((u->length!=grad->length)||(u->orientation!=0)||(grad->orientation!=0)||(A->height!=B->length)||(A->width!=u->length)){
		usart4PutString("ERROR in mountain_grad(), dimensions do not agree.\n\r");
	}

	grad->name = "mountain gradieng";

	int i, k = 1000;
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
		vector_float_times(&W, (k*norm_w*sig/((dot_W_u+B_i)*(dot_W_u+B_i))));
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

	matrix_float_mul_vec_right(H, u, grad);
	vector_float_add(grad, c);
}

void test_quadprog(){		// q
		//testing function check
		usart4PutString("test quadprog runing\n\r");
		int max = 3;
		float A_arr[] = {1, 2, 3, 4, 5, 6, 7, 8, 9};
		float B_arr[] = {4, 6, 8};
		float u_arr[] = {-4, 3, -8};
		float H_arr[] = {235, -53, -45, -55, 210, -49, -48, -50, 269};
		float c_arr[] = {7, 9, 4};
		float grad_arr[max];
		int step = 1;
		int iterations = 10;


		matrix_float A = {max, max, (float*) A_arr, "A - quadprog"};
		vector_float B = {max, 0, (float*) B_arr, "B - quadprog"};
		vector_float u = {max, 0, (float*) u_arr, "u - quadprog"};
		matrix_float H = {max, max, (float*) H_arr, "H - quadprog"};
		vector_float c = {max, 0, (float*) c_arr, "c - quadprog"};
		vector_float grad = {max, 0, (float*) grad_arr, "grad - quadprog"};

		matrix_float_print(&A);
		matrix_float_print(&B);
		matrix_float_print(&H);

		vector_float_print(&u);
		vector_float_print(&c);
		vector_float_set_zero(&grad);

		my_quadprog(&H, &c, &A, &B, &u, iterations, step);

		u.name = "u - final";
//		vector_float_print(&u);

		mountain_gradient(&A, &B, &u, &grad);

		vector_float_print(&grad);
}

void test_constraint(){		// c




	return;
	float x0 = 3;
	float y0 = 4;
	int radius = 1;
	float k, q, sig;

	usart_string_float_print("my_constraints_test, x0 = ", &x0);
	usart_string_float_print("my_constraints_test, x0 = ", &y0);

	my_constraint(&x0, &y0, &radius, &k, &q, &sig);

	usart_string_float_print("k = ", &k);
	usart_string_float_print("q = ", &q);
	usart_string_float_print("sig=", &sig);

}

void reference(float * x){
	*x = 6;
}

void my_constraint(float * x0, float * y0, int * radius, float * k, float * q, float * sig){
//	function [k, q, q_g, bx, by, sig] = my_constraint(x0, y0, radius)
//	    % return:
//	    % half plain y < kx + q
//	    % line is orthogonal to the vector [x0, y0]
//	    %
//	    % [bx, by] is point of intersection of the line and circle
//	    % [k] is the slope
//	    % [g] is the bias
//	    % [g_q] is the bias for graph pourposes
//
//	    d = sqrt(x0^2 + y0^2);
//	    bx = x0-radius*x0/d;
//	    by = y0-radius*y0/d;
//	    q_g = -bx^2 - by^2;
//	    k = -x0/y0;
//	    q = by - k*bx;
//	    if y0 > 0
//	        sig = 1;   % obsticle is before UAV
//	    else
//	        sig = -1;  % obsticle is behind UAV
//	    end
//	end

	float d, bx, by;

	float fx0 = *(x0);				// crazy
	float fy0 = *(y0);				// crazy
	float fradius = *(radius);		// crazy

	usart_string_float_print("my_constraints_float3, fx0 = ", &fx0);
	usart_string_float_print("my_constraints_float3, fy0 = ", &fy0);

	usart_string_int_print("my_constraints_int, x0 = ", x0);
	usart_string_int_print("my_constraints_int, y0 = ", y0);

	d = sqrt(fx0*fx0+fy0*fy0);
	bx = fx0-fradius*fx0/d;
	by = fy0-fradius*fy0/d;

	usart_string_float_print("my_constraints_float3, x0 = ", &x0);
	usart_string_float_print("my_constraints_float3, y0 = ", &y0);

	*k = -((float)fx0/(float)fy0);

	usart_string_float_print("k2 = ", k);

	if(y0 > 0){
		*sig = 1;
	} else {
		*sig = -1;
	}

	*q = (by - (*k)*bx);
}



