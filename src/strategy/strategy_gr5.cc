#include "../../include/strategy/strategy_gr5.h"

/*! \brief startegy during the game
 * 
 * \param[in,out] cvs controller main structure
 */

void FSM_Def_init_pos_actuators(CtrlStruct *cvs){
	RouePanneauSTOP(cvs);
	BrasPanneauEnHaut(cvs);
	EntonnoirIN(cvs);
	BrasEntonnoirMiddle(cvs);
	FourcheEnBas(cvs);
	FeuilleIN(cvs);
	TapisStop(cvs);
	CtrlActuators(cvs);

	sleep(4);
}


void FSM_Do_Solar_panel(CtrlStruct *cvs) // Bleu = 0
{	
	RouePanneauSTOP(cvs);
	BrasPanneauEnHaut(cvs);
	CtrlActuators(cvs);

	sleep(0.5);

	BrasPanneauEnBas(cvs);
	CtrlActuators(cvs);
	
	sleep(1);

	if (cvs->robot_id == ROBOT_Y)
	{
		RouePanneauBleu(cvs);
	}
	else 
	{
		RouePanneauJaune(cvs);
	}
	CtrlActuators(cvs);

	sleep(1);

	RouePanneauSTOP(cvs);
	BrasPanneauEnHaut(cvs);
	CtrlActuators(cvs);

	sleep(0.5);
}

void FSM_PrepInTake_Plant(CtrlStruct *cvs)
{	
	TapisRentrant(cvs);
	EntonnoirOUT(cvs);
	BrasEntonnoirSeq(cvs);
	FourcheEnHaut(cvs);
	CtrlActuators(cvs);

}

void FSM_StopPrepInTake_Plant(CtrlStruct *cvs){

	EntonnoirIN(cvs);
	BrasEntonnoirMiddle(cvs);
	CtrlActuators(cvs);

	sleep(0.5);

	EntonnoirSTOP(cvs);
	CtrlActuators(cvs);

}

void Take_Plant(CtrlStruct *cvs){

	BrasEntonnoirMiddle(cvs);
	FourcheEnHaut(cvs);
	EntonnoirOUT(cvs);
	CtrlActuators(cvs);

	sleep(4);

	BrasEntonnoirSeq(cvs);
	TapisRentrant(cvs);
	CtrlActuators(cvs);

	sleep(7);

	BrasEntonnoirFermé(cvs);
	EntonnoirIN(cvs);
	TapisStop(cvs);
	CtrlActuators(cvs);

	sleep(1.8);
	
	EntonnoirSTOP(cvs);
	CtrlActuators(cvs);

	sleep(2);
}


void FSM_takeOff_Plant(CtrlStruct *cvs){
	FourcheEnBas(cvs);
	BrasEntonnoirMiddle(cvs);
	EntonnoirOUT(cvs);
	CtrlActuators(cvs);

	sleep(0.5);

	SortirBlocFourche(cvs);
	FeuilleOUT(cvs);
	CtrlActuators(cvs);
}

void FSM_WheelSecurityAntiBlocking(CtrlStruct *cvs){
	RentrerBlocFourche(cvs);
	CtrlActuators(cvs);

	sleep(0.5);

	FourcheEnHaut(cvs);
	StopBlocFourche(cvs);
	CtrlActuators(cvs);
}

void FSM_PrepDropPlantGarden(CtrlStruct *cvs){

	EntonnoirIN(cvs);
	BrasEntonnoirMiddle(cvs);
	CtrlActuators(cvs);

}

void FSM_DropPlantGarden(CtrlStruct *cvs){

	FeuilleIN(cvs);
	CtrlActuators(cvs);

	sleep(2);

	RentrerBlocFourche(cvs);
	CtrlActuators(cvs);
}

void FSM_MouvPrisePlantes(CtrlStruct *cvs){
	speed_regulation(cvs, 0.0, -0.1, 0.0);
	sleep(0.2);
	speed_regulation(cvs, 0.0, 0.0, 0.0);
}

void FSM_MouvPrisePot(CtrlStruct *cvs){
	speed_regulation(cvs, -0.2, 0, 0);
	sleep(0.8);
	speed_regulation(cvs, 0, 0.2, 0);
	sleep(0.4);
	speed_regulation(cvs, 0, 0, 0);
}

void FSM_MouvPanneau(CtrlStruct *cvs){

	FSM_Do_Solar_panel(cvs);
	speed_regulation(cvs, -0.2, 0, 0);
	speed_regulation(cvs, 0, 0, 0);
	FSM_Do_Solar_panel(cvs);
	speed_regulation(cvs, 0, 0.2, 0);
	speed_regulation(cvs, 0, 0, 0);
	speed_regulation(cvs, -0.2, 0, 0);
	speed_regulation(cvs, 0, 0.2, 0);
	speed_regulation(cvs, 0, 0, 0);
}

void FSM_Take_Pot_v1(CtrlStruct *cvs){

	EntonnoirIN(cvs);
	TapisStop(cvs);
	CtrlActuators(cvs);

	sleep(5);

	EntonnoirOUT(cvs);
	CtrlActuators(cvs);

	sleep(5);

	EntonnoirIN(cvs);
	CtrlActuators(cvs);

	sleep(1);

	EntonnoirSTOP(cvs);
	CtrlActuators(cvs);

	sleep(5);
}


void FSM_Take_Pot_v2(CtrlStruct *cvs){

	EntonnoirIN(cvs);
	TapisStop(cvs);
	CtrlActuators(cvs);

	sleep(5);


	EntonnoirOUT(cvs);
	CtrlActuators(cvs);

	sleep(5);

	TapisRentrant_Pot(cvs);
	CtrlActuators(cvs);

	sleep(15);

	EntonnoirIN(cvs);
	TapisStop(cvs);
	CtrlActuators(cvs);

	sleep(1);

	EntonnoirSTOP(cvs);
	CtrlActuators(cvs);
}

void main_strategy(CtrlStruct *cvs)
{
	// variables declaration
	Strategy *strat;
	double t;
	CtrlIn *inputs;

	inputs = cvs->inputs;
	t = inputs->t;
	
	// variables initialization
	strat = cvs->strat;

	switch (strat->state)
	{
		case STRAT_STATE_1:
			path_planning_update(cvs, 1.8, 0.3); //1.82593, 1.0974 pour le 1er panneau () le plus a droite
			
			speed_regulation(cvs, NULL, NULL, NULL);

			cvs->path->sigma_th = 0.1;
			if (cvs->path->target_reached) {
				cvs->position->flagUpdate = 1;
				printf("rob xR:%f, yR:%f, theta:%f\n", cvs->rob_pos->x, cvs->rob_pos->y, cvs->rob_pos->theta);
				speed_regulation(cvs, -1, -1, -1);	
				cvs->main_state = STOP_END_STATE;
				FSM_Do_Solar_panel(cvs);
				printf("Panneau Fait 1\n");
				strat->state = STRAT_STATE_2;
				//cvs->main_state = STRAT_STATE_2;
				strat->t_strat = t;
			}
			break;

		case STRAT_STATE_2:
			path_planning_update(cvs, 1.0, 2.6);
			speed_regulation(cvs, NULL, NULL, NULL);

			cvs->path->sigma_th = 0.05;
			if (cvs->path->target_reached) {
				printf("State 2\n");
				//strat->state = STRAT_STATE_3;
				cvs->main_state = STOP_END_STATE;

				printf("rob xR:%f, yR:%f, theta:%f\n", cvs->rob_pos->x, cvs->rob_pos->y, cvs->rob_pos->theta);
				strat->t_strat = t;
			}
			break;

		case STRAT_STATE_3:
			path_planning_update(cvs, 1.8, 1.5474);
			speed_regulation(cvs, NULL, NULL, NULL);


			cvs->path->sigma_th = 0.10;
			if (cvs->path->target_reached) {
				printf("State 3\n");
				strat->state = STRAT_STATE_4;
				//cvs->main_state = STOP_END_STATE;
				printf("rob xR:%f, yR:%f, theta:%f\n", cvs->rob_pos->x, cvs->rob_pos->y, cvs->rob_pos->theta);
				strat->t_strat = t;
			}
			break;

		case STRAT_STATE_4:
			speed_regulation(cvs, -1, -1, -1);
			/*
			path_planning_update(cvs, 0.2, 0.2);
			speed_regulation(cvs, NULL, NULL, NULL);


			cvs->path->sigma_th = 0.10;
			if (cvs->path->target_reached) {
				printf("State 4\n");
				cvs->main_state = STOP_END_STATE;
				printf("rob xR:%f, yR:%f, theta:%f\n", cvs->rob_pos->x, cvs->rob_pos->y, cvs->rob_pos->theta);
				strat->t_strat = t;
			}
			*/
			break;	
	}
}

void main_strategy_sonar(CtrlStruct *cvs)
{
	// variables declaration
	Strategy *strat;
	double t;
	CtrlIn *inputs;

	inputs = cvs->inputs;
	t = inputs->t;
	
	// variables initialization
	strat = cvs->strat;

	switch (strat->state)
	{
		case STRAT_STATE_1:
			// sonar(cvs);
			// if ((cvs->inputs->sonars[FRONT] <= 20) | (cvs->inputs->sonars[LEFT1] <= 20) | (cvs->inputs->sonars[RIGHT] <= 20)){
			// 	speed_regulation(cvs, -1, -1, -1);
			// }else {
			// 	path_planning_update(cvs, 0.0, 1.0); //1.82593, 1.0974 pour le 1er panneau () le plus a droite
			// 	speed_regulation(cvs, NULL, NULL, NULL);
			// }

			path_planning_update(cvs, 1.5, 2.0);
			speed_regulation(cvs, NULL, NULL, NULL);

			cvs->path->sigma_th = 0.1;
			if (cvs->path->target_reached) {
				cvs->position->flagUpdate = 1;
				printf("rob xR:%f, yR:%f, theta:%f\n", cvs->rob_pos->x, cvs->rob_pos->y, cvs->rob_pos->theta);
				speed_regulation(cvs, -1, -1, -1);	
				cvs->main_state = STOP_END_STATE;
				FSM_Do_Solar_panel(cvs);
				printf("Panneau Fait 1\n");
				strat->state = STRAT_STATE_2;
				//cvs->main_state = STRAT_STATE_2;
				strat->t_strat = t;
			}
			break;

		case STRAT_STATE_2:
			path_planning_update(cvs, 1.0, 2.6);
			speed_regulation(cvs, NULL, NULL, NULL);

			cvs->path->sigma_th = 0.05;
			if (cvs->path->target_reached) {
				printf("State 2\n");
				//strat->state = STRAT_STATE_3;
				cvs->main_state = STOP_END_STATE;

				printf("rob xR:%f, yR:%f, theta:%f\n", cvs->rob_pos->x, cvs->rob_pos->y, cvs->rob_pos->theta);
				strat->t_strat = t;
			}
			break;

		case STRAT_STATE_3:
			path_planning_update(cvs, 1.8, 1.5474);
			speed_regulation(cvs, NULL, NULL, NULL);


			cvs->path->sigma_th = 0.10;
			if (cvs->path->target_reached) {
				printf("State 3\n");
				strat->state = STRAT_STATE_4;
				//cvs->main_state = STOP_END_STATE;
				printf("rob xR:%f, yR:%f, theta:%f\n", cvs->rob_pos->x, cvs->rob_pos->y, cvs->rob_pos->theta);
				strat->t_strat = t;
			}
			break;

		case STRAT_STATE_4:
			speed_regulation(cvs, -1, -1, -1);
			/*
			path_planning_update(cvs, 0.2, 0.2);
			speed_regulation(cvs, NULL, NULL, NULL);


			cvs->path->sigma_th = 0.10;
			if (cvs->path->target_reached) {
				printf("State 4\n");
				cvs->main_state = STOP_END_STATE;
				printf("rob xR:%f, yR:%f, theta:%f\n", cvs->rob_pos->x, cvs->rob_pos->y, cvs->rob_pos->theta);
				strat->t_strat = t;
			}
			*/
			break;
	}
}

void panel_strategy(CtrlStruct *cvs)
{	
	FSM_Do_Solar_panel(cvs);
	sleep(1);
	speed_regulation(cvs, 0.2, -1, -1);
	sleep(1);
	speed_regulation(cvs, -1, -1, -1);
	FSM_Do_Solar_panel(cvs);
	sleep(1);
	speed_regulation(cvs, 0.2, -1, -1);
	sleep(1);
	speed_regulation(cvs, -1, -1, -1);
	FSM_Do_Solar_panel(cvs);

}

void stratLidar(CtrlStruct *cvs)
{
	// variables declaration
	Strategy *strat;
	double t;
	CtrlIn *inputs;

	inputs = cvs->inputs;
	t = inputs->t;
	
	// variables initialization
	strat = cvs->strat;

	switch (strat->state)
	{
		case STRAT_STATE_1:
			
			
			speed_regulation(cvs, -1, -1, -1);
			strat->state = STRAT_STATE_2;
			strat->t_strat = t;

			break;

		case STRAT_STATE_2:
			speed_regulation(cvs, 0.2, -1, -1);
			if (t-strat->t_strat >= 3.0) {
				strat->state = STRAT_STATE_3;
				//cvs->main_state = STOP_END_STATE;
				strat->t_strat = t;
			}
			break;

		case STRAT_STATE_3:
			speed_regulation(cvs, -1, -0.4, -1);
			if (t-strat->t_strat >= 3.0) {
				//strat->state = STRAT_STATE_4;
				cvs->main_state = STOP_END_STATE;
				strat->t_strat = t;
			}
			break;
		
		case STRAT_STATE_4:
			speed_regulation(cvs, -1, -1, -1);
			if (t-strat->t_strat >= 7.0) {
				cvs->main_state = STOP_END_STATE;
				strat->t_strat = t;
			}
			break;
	}
}

void BlueStrat(CtrlStruct *cvs)
{
	// variables declaration
	Strategy *strat;
	double t;
	CtrlIn *inputs;

	inputs = cvs->inputs;
	t = inputs->t;
	
	// variables initialization
	strat = cvs->strat;



	switch (strat->state)
	{
		case STRAT_STATE_1:
			speed_regulation(cvs, -1 , -1, -1);
			
			strat->state = STRAT_STATE_2;
			strat->t_strat = t;
			break;

		case STRAT_STATE_2:
			speed_regulation(cvs, 0.2, -1, -1);
			if (t - strat->t_strat > 1.0) {
				printf("rob xR:%f, yR:%f, theta:%f\n", cvs->rob_pos->x, cvs->rob_pos->y, cvs->rob_pos->theta);
				speed_regulation(cvs, -1, -1, -1);	
				FSM_Do_Solar_panel(cvs);
				strat->state = STRAT_STATE_3;
				strat->t_strat = t;
				printf("t_strat:%f\n", strat->t_strat);
				printf("t:%f\n", t);
			}
			break;

		case STRAT_STATE_3:
			speed_regulation(cvs, 0.2, -1, -1);
			printf("t_diff:%f\n", t - strat->t_strat);

			if (t - strat->t_strat > 1.0) {
				printf("rob xR:%f, yR:%f, theta:%f\n", cvs->rob_pos->x, cvs->rob_pos->y, cvs->rob_pos->theta);
				speed_regulation(cvs, -1, -1, -1);	
				FSM_Do_Solar_panel(cvs);
				sleep(2);
				strat->state = STRAT_STATE_4;
				strat->t_strat = t;
				printf("t_strat:%f\n", strat->t_strat);
				printf("t:%f\n", t);
				printf("t_diff:%f\n", t - strat->t_strat);
			}

			break;

		case STRAT_STATE_4:
			speed_regulation(cvs, 0.2, -1, -1);
			if (t - strat->t_strat > 2.0) {
				printf("rob xR:%f, yR:%f, theta:%f\n", cvs->rob_pos->x, cvs->rob_pos->y, cvs->rob_pos->theta);
				speed_regulation(cvs, -1, -1, -1);	
				FSM_Do_Solar_panel(cvs);
				sleep(1);
				strat->state = STRAT_STATE_5;
				strat->t_strat = t;
			}
			break;
		
		case STRAT_STATE_5:
			path_planning_update(cvs, 0.3, 0.2);
			speed_regulation(cvs, NULL, NULL, NULL);
			
			cvs->path->sigma_th = 0.1;
			if (cvs->path->target_reached) {
				speed_regulation(cvs, -1, -1, -1);	
				cvs->main_state = STOP_END_STATE;
			}
			break;
			
	}
}

void YellowStrat(CtrlStruct *cvs)
{
	// variables declaration
	Strategy *strat;
	double t;
	CtrlIn *inputs;

	inputs = cvs->inputs;
	t = inputs->t;
	
	// variables initialization
	strat = cvs->strat;

	switch (strat->state)
	{
		case STRAT_STATE_1:

			break;

		case STRAT_STATE_2:

			break;

		case STRAT_STATE_3:

			break;

		case STRAT_STATE_4:
			break;
			
	}
}


void Opposite_base_Strat(CtrlStruct *cvs)
{
	// variables declaration
	Strategy *strat;
	double t;
	CtrlIn *inputs;

	inputs = cvs->inputs;
	t = inputs->t;
	
	// variables initialization
	strat = cvs->strat;

	switch (strat->state)
	{
		case STRAT_STATE_1:
			speed_regulation(cvs, 0.2, -1, -1);
			sleep(7.0);
			strat->state = STRAT_STATE_2;
			speed_regulation(cvs, -1, -1, -1);
			increment_score(11);
			break;

		case STRAT_STATE_2:
			FSM_Do_Solar_panel(cvs);
			cvs->main_state = STOP_END_STATE;
			break;
	}
}

void Get_out_Strat(CtrlStruct *cvs)
{
	speed_regulation(cvs, 0.2, -1, -1);
	sleep(2.5);
	cvs->main_state = STOP_END_STATE;
	increment_score(1);
}


void free_strategy(CtrlStruct *cvs)
{
	free(cvs->strat);
}
