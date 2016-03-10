/*
 * main.c
 *
 *  Author: Tomas Baca
 */

#include "system.h"
#include "commTask.h"
#include "kalmanTask.h"
#include "mpcTask.h"

/*
#define T 100

// constants allocation
const float dt = 0.01148;


// constant matrixes allocations

const float pr_A[6*6];	//state space A
const float pr_B[6*2];	//state space B
const float pr_Av[(6*T)*(T*T)];	//extended A
const float pr_Bv[(6*T)*(2*T)];	//extended B
const float pr_H[];
const float pr_Qv[];
const float pr_Pv[];
const float pr_Sv[];

// allocation of variables with constant size that change
float pr_Uv[(2*T)*1];	//input vector
float pr_Ac[T*(2*T)];		//
float pr_Bc[T*1];		//
float pr_c[(2*T)*1];		//column vector

*/

int rekurze(int i){
	usart4PutString("rekurze start 1\n\r");
	i = 3;
	rekurze(i);
	return i;
}



void testTask1(void) {
//	rekurze(4);

	vTaskDelay(100);
	usart4PutString("testTask 1() start 1\n\r");
	int i = 0;

	while(1) {
		vTaskDelay(100);
//		usart_string_int_print("testTask1() while ", i++);
	}
}

void testTask2(void) {
//	rekurze(4);

	vTaskDelay(100);
	usart4PutString("testTask 2() start 1\n\r");
	int i = 0;

	while(1) {
		vTaskDelay(100);
//		usart_string_int_print("testTask2() while ", i++);
	}
}


int main(void) {

	// this line is one ******* SUNDAY
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

	// initialize the hardware
	boardInit();

	/* -------------------------------------------------------------------- */
	/*	Start the communication task routine								*/
	/* -------------------------------------------------------------------- */
	xTaskCreate(commTask, (char*) "commTask", 4092, NULL, 2, NULL);

	/* -------------------------------------------------------------------- */
	/*	Start the kalman filter task										*/
	/* -------------------------------------------------------------------- */
	xTaskCreate(kalmanTask, (char*) "kalman", 4092, NULL, 2, NULL);

	/* -------------------------------------------------------------------- */
	/*	Start the mpc task filter task										*/
	/* -------------------------------------------------------------------- */

	xTaskCreate(mpcTask, (char*) "mpcTask", 4092, NULL, 2, NULL);

//	xTaskCreate(testTask1, (char*) "testTask", 4092, NULL, 2, NULL);

//	xTaskCreate(testTask2, (char*) "testTask", 4092, NULL, 2, NULL);



	/* -------------------------------------------------------------------- */
	/*	Start the FreeRTOS scheduler										*/
	/* -------------------------------------------------------------------- */

	usart4PutString("main end ------\n\r");

	vTaskStartScheduler();

	return 0;
}
