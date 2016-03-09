/*
 * mpcTask.c
 *
 *  Author: Tomas Baca
 */

#include "mpcTask.h"
#include "kalmanTask.h"
#include "commTask.h"
#include "mpc/elevator/elevatorMpc.h"
#include "mpc/aileron/aileronMpc.h"
#include "CMatrixLib.h"
#include "predictive.h"	// ok?
#define OBSTACLES 1			// if running Honza's code
#define T_MAX 100			// last value in pr_block

void mpcTask(void *p) {

	usart4PutString("mpcTask() starting 1\r\n");		// doesn't do anything
	vTaskDelay(100);
	/* -------------------------------------------------------------------- */
	/*	Create handlers for all systems										*/
	/* -------------------------------------------------------------------- */
	mpcHandler_t * elevatorMpcHandler = initializeElevatorMPC();
	mpcHandler_t * aileronMpcHandler = initializeAileronMPC();
	prHandler * pr_handler = initializePrHandler();

	/* -------------------------------------------------------------------- */
	/*	Messages between tasks												*/
	/* -------------------------------------------------------------------- */
	mpc2commMessage_t mpc2commMessage;
	kalman2mpcMessage_t kalman2mpcMessage;
	comm2mpcMessage_t comm2mpcMessage;

//	prepare();

//	vTaskDelay(100);
//	usart4PutString("mpcTask() starting 2\r\n");		// doesn't do anything

	while (1) {

		/* -------------------------------------------------------------------- */
		/*	If there is a message from commTask									*/
		/* -------------------------------------------------------------------- */



		/*
		while (xQueueReceive(comm2mpcQueue, &comm2mpcMessage, 0)) {
			usart4PutString("---------something recieved-----------\n\r");


			if (comm2mpcMessage.messageType == SETPOINT) {
				if(OBSTACLES){
					pr_handler->type = 1;
					pr_handler->position_reference_x = comm2mpcMessage.elevatorReference[0];	// predictive
					pr_handler->position_reference_y = comm2mpcMessage.elevatorReference[0];	// predictive
				} else{
					// copy the incoming set point/s into the local vector
					vector_float_set_to(elevatorMpcHandler->position_reference, comm2mpcMessage.elevatorReference[0]);
					vector_float_set_to(aileronMpcHandler->position_reference, comm2mpcMessage.aileronReference[0]);
				}
			} else if (comm2mpcMessage.messageType == TRAJECTORY) {
				if(OBSTACLES){
					int i, j;
					float step;
					int granularity = (T_MAX+4)/5;
					pr_handler->type = 2;

					// for all key-point in the aileron reference
					for (j = 0; j < 4; j++) {

						// calculate the interpolation step
						float step_x = (comm2mpcMessage.aileronReference[j+1] - comm2mpcMessage.aileronReference[j])/granularity;
						float step_y = (comm2mpcMessage.elevatorReference[j+1] - comm2mpcMessage.elevatorReference[j])/granularity;

						matrix_float_set(pr_handler->Tr_full, j*granularity+1, 1, comm2mpcMessage.aileronReference[j]);
						matrix_float_set(pr_handler->Tr_full, j*granularity+1, 2, comm2mpcMessage.elevatorReference[j]);

						for (i = 1; i < granularity; i++) {
							matrix_float_set(pr_handler->Tr_full, j*granularity+1+i, 1, matrix_float_get(pr_handler->Tr_full, j*granularity+i, 1) + step_x);
							matrix_float_set(pr_handler->Tr_full, j*granularity+1+i, 2, matrix_float_get(pr_handler->Tr_full, j*granularity+i, 2) + step_y);
						}
					}

					*//* RECIEVING TRAJECTORY *//*
					for(i = 1; i < 4; i++){

					}



				} else {
					int i, j;
					float step;

					// for all key-point in the elevator reference
					for (j = 0; j < 4; j++) {

						// calculate the interpolation step
						step = (comm2mpcMessage.elevatorReference[j+1] - comm2mpcMessage.elevatorReference[j])/50;

						vector_float_set(elevatorMpcHandler->position_reference, j*50+1, comm2mpcMessage.elevatorReference[j]);

						for (i = 1; i < 50; i++) {

							vector_float_set(elevatorMpcHandler->position_reference, j*50+1+i, vector_float_get(elevatorMpcHandler->position_reference, j*50+i) + step);
						}
					}

					// for all key-point in the aileron reference
					for (j = 0; j < 4; j++) {

						// calculate the interpolation step
						step = (comm2mpcMessage.aileronReference[j+1] - comm2mpcMessage.aileronReference[j])/50;

						vector_float_set(aileronMpcHandler->position_reference, j*50+1, comm2mpcMessage.aileronReference[j]);

						for (i = 1; i < 50; i++) {

							vector_float_set(aileronMpcHandler->position_reference, j*50+1+i, vector_float_get(aileronMpcHandler->position_reference, j*50+i) + step);
						}
					}
				}
			} else if (comm2mpcMessage.messageType == BLOBS) {
				//done, not tested
				usart4PutString("---------mpcTask() recieving blobs-----------");

				pr_handler->obstacle_n = comm2mpcMessage.obstacle_n;
				memcpy(&pr_handler->obstacle_x, &comm2mpcMessage.obstacle_x, comm2mpcMessage.obstacle_n*sizeof(float));
				memcpy(&pr_handler->obstacle_y, &comm2mpcMessage.obstacle_y, comm2mpcMessage.obstacle_n*sizeof(float));
				memcpy(&pr_handler->obstacle_r, &comm2mpcMessage.obstacle_r, comm2mpcMessage.obstacle_n*sizeof(float));

				*//* RECIEVING BLOBS *//*
				int i;
				for(i = 0; i < pr_handler->obstacle_n; i++){
					usart_string_int_print("OBSTACLE ", i);
					usart_string_float_print("obstacle x = ", &(pr_handler->obstacle_x[i]));
					usart_string_float_print("obstacle y = ", &(pr_handler->obstacle_y[i]));
					usart_string_float_print("obstacle r = ", &(pr_handler->obstacle_r[i]));
				}


			}
		}
		*/


		/* -------------------------------------------------------------------- */
		/*	If there is a message from kalmanTask								*/
		/* -------------------------------------------------------------------- */
		if (xQueueReceive(kalman2mpcQueue, &kalman2mpcMessage, 0)) {
			if(0){		// OBSTACLES

				// set initial condition
				pr_handler->position_x = vector_float_get(&kalman2mpcMessage.aileronData , 1);
				pr_handler->position_y = vector_float_get(&kalman2mpcMessage.elevatorData , 1);

				vector_float_set(pr_handler->x0, 1, 0);
				vector_float_set(pr_handler->x0, 2, vector_float_get(&kalman2mpcMessage.aileronData , 2));
				vector_float_set(pr_handler->x0, 3, vector_float_get(&kalman2mpcMessage.aileronData , 3));
				vector_float_set(pr_handler->x0, 4, 0);
				vector_float_set(pr_handler->x0, 5, vector_float_get(&kalman2mpcMessage.elevatorData, 2));
				vector_float_set(pr_handler->x0, 6, vector_float_get(&kalman2mpcMessage.elevatorData, 3));

				pr_handler->error_x = kalman2mpcMessage.aileronData[4];
				pr_handler->error_y = kalman2mpcMessage.aileronData[4];

				// filter the reference
				filterReferenceTrajectory(elevatorMpcHandler);
				filterReferenceTrajectory(aileronMpcHandler);

				// calculate the elevator MPC
				float action_x, action_y;
				pr_calculateMPC(pr_handler, &action_x, &action_y);
				mpc2commMessage.aileronOutput  = action_x;
				mpc2commMessage.elevatorOutput = action_y;

				// copy the current setpoint (main for debug)
				mpc2commMessage.aileronSetpoint = pr_handler->position_reference_x;
				mpc2commMessage.elevatorSetpoint = pr_handler->position_reference_y;
				// send outputs to commTask
				xQueueOverwrite(mpc2commQueue, &mpc2commMessage);

				led_toggle();

			}else{
				// copy the elevatorStates to states
				memcpy(elevatorMpcHandler->initial_cond->data, &kalman2mpcMessage.elevatorData, elevatorMpcHandler->number_of_states*sizeof(float));

				// filter the reference
				filterReferenceTrajectory(elevatorMpcHandler);

				// calculate the elevator MPC
				mpc2commMessage.elevatorOutput = calculateMPC(elevatorMpcHandler);

				// copy the elevatorStates to states
				memcpy(aileronMpcHandler->initial_cond->data, &kalman2mpcMessage.aileronData, aileronMpcHandler->number_of_states*sizeof(float));

				// filter the reference
				filterReferenceTrajectory(aileronMpcHandler);

				// calculate the aileron MPC
				mpc2commMessage.aileronOutput = calculateMPC(aileronMpcHandler);

				// copy the current setpoint (main for debug)
				mpc2commMessage.elevatorSetpoint = vector_float_get(elevatorMpcHandler->position_reference, 1);
				mpc2commMessage.aileronSetpoint = vector_float_get(aileronMpcHandler->position_reference, 1);

				// send outputs to commTask
				xQueueOverwrite(mpc2commQueue, &mpc2commMessage);

				led_toggle();
			}

		}
	}
}
