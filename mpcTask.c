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
#define OBSTACLES 1			// if running Honza's code
#define T_MAX 10			// last value in pr_block

void mpcTask(void *p) {

	/* -------------------------------------------------------------------- */
	/*	Create handlers for all systems										*/
	/* -------------------------------------------------------------------- */
	mpcHandler_t * elevatorMpcHandler = initializeElevatorMPC();
	mpcHandler_t * aileronMpcHandler = initializeAileronMPC();
	prHandler * prHandler;

	/* -------------------------------------------------------------------- */
	/*	Messages between tasks												*/
	/* -------------------------------------------------------------------- */
	mpc2commMessage_t mpc2commMessage;
	kalman2mpcMessage_t kalman2mpcMessage;
	comm2mpcMessage_t comm2mpcMessage;

	vTaskDelay(100);

	while (1) {

		/* -------------------------------------------------------------------- */
		/*	If there is a message from commTask									*/
		/* -------------------------------------------------------------------- */
		while (xQueueReceive(comm2mpcQueue, &comm2mpcMessage, 0)) {

			if (comm2mpcMessage.messageType == SETPOINT) {
				if(OBSTACLES){
					vector_float_set_to(prHandler->position_reference_x, comm2mpcMessage.elevatorReference[0]);	// predictive
					vector_float_set_to(prHandler->position_reference_y, comm2mpcMessage.elevatorReference[0]);	// predictive
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

					// for all key-point in the aileron reference
					for (j = 0; j < 4; j++) {

						// calculate the interpolation step
						step_x = (comm2mpcMessage.aileronReference[j+1] - comm2mpcMessage.aileronReference[j])/granularity;
						step_y = (comm2mpcMessage.elevatorReference[j+1] - comm2mpcMessage.elevatorReference[j])/granularity;

						matrix_float_set(prHandler->Tr, j*granularity+1, 1, comm2mpcMessage.aileronReference[j]);
						matrix_float_set(prHandler->Tr, j*granularity+1, 2, comm2mpcMessage.elevatorReference[j]);

						for (i = 1; i < granularity; i++) {
							matrix_float_set(prHandler->Tr, j*granularity+1+i, 1, matrix_float_get(prHandler->Tr, j*granularity+i, 1) + step_x);
							matrix_float_set(prHandler->Tr, j*granularity+1+i, 2, matrix_float_get(prHandler->Tr, j*granularity+i, 2) + step_y);
						}
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
			}
		}

		/* -------------------------------------------------------------------- */
		/*	If there is a message from kalmanTask								*/
		/* -------------------------------------------------------------------- */
		if (xQueueReceive(kalman2mpcQueue, &kalman2mpcMessage, 0)) {
			if(OBSTACLES){

				// set initial condition
				vector_float_set(prHandler->initial_cond, 1, vector_float_get(&kalman2mpcMessage.aileronData , 1));
				vector_float_set(prHandler->initial_cond, 2, vector_float_get(&kalman2mpcMessage.aileronData , 2));
				vector_float_set(prHandler->initial_cond, 3, vector_float_get(&kalman2mpcMessage.aileronData , 3));
				vector_float_set(prHandler->initial_cond, 4, vector_float_get(&kalman2mpcMessage.elevatorData, 1));
				vector_float_set(prHandler->initial_cond, 5, vector_float_get(&kalman2mpcMessage.elevatorData, 2));
				vector_float_set(prHandler->initial_cond, 6, vector_float_get(&kalman2mpcMessage.elevatorData, 3));

				// filter the reference
				filterReferenceTrajectory(elevatorMpcHandler);
				filterReferenceTrajectory(aileronMpcHandler);

				// calculate the elevator MPC
				float action_x, action_y;
				pr_calculateMPC(prHandler, &action_x, &action_y);
				mpc2commMessage.aileronOutput  = action_x;
				mpc2commMessage.elevatorOutput = action_y;

				// copy the current setpoint (main for debug)
				mpc2commMessage.elevatorSetpoint = vector_float_get(prHandler->position_reference_x, 1);
				mpc2commMessage.aileronSetpoint = vector_float_get(prHandler->position_reference_y, 1);

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
