/*!
 * \file ctrl_main_gr5.cc
 * \brief Initialization, loop and finilization of the controller written in C (but compiled as C++)
 */

#include "../../include/main/ctrl_main_gr5.h"

#include <cstdio>
#include <math.h>



/*! \brief initialize controller operations (called once)
 *
 * \param[in] cvs controller main structure
 */

bool Avoidance(CtrlStruct *cvs) {
  if ((cvs->lidar->rad_opp <= 0.5) && (cvs->lidar->ang_opp < 60.0) && (360.0 - cvs->lidar->ang_opp > -60.0)) {
    return true;
  }
  return false;
}

void controller_init(CtrlStruct *cvs)
{
  
  double t;
  CtrlIn *inputs;

  inputs = cvs->inputs;
  t = inputs->t;

  switch (cvs->robot_id)
  {
		case ROBOT_B: cvs->team_id = TEAM_A; break;
		case ROBOT_Y: cvs->team_id = TEAM_B; break;

		default:
			printf("Team detection error: unknown robot ID: %d !\n", cvs->robot_id);
			exit(EXIT_FAILURE);
  }
  
	set_init_position(cvs->robot_id, cvs->rob_pos);
	cvs->rob_pos->last_t = t;

  cvs->sp_reg->last_t = t;
  cvs->strat->state = STRAT_STATE_1;

  //makeHeatmap(cvs);
}

/*! \brief controller loop (called every timestep)
 *
 * \param[in] cvs controller main structure
 */
void controller_loop(CtrlStruct *cvs)
{  
  double t;
  double vitesseX;
  double vitesseY;
  double vitesseZ;
  CtrlIn *inputs;

  inputs = cvs->inputs;

	t = cvs->inputs->t;

  update_robotPosition(cvs);

  switch (cvs->main_state) {

    case RUN_STATE:
        //ResetActuator(cvs);
        //EntonnoirIN(cvs);
        //ParralaxIn(cvs);
        //FeetechIn(cvs);
        //DSSForkMid(cvs);
        //EntonnoirIN(cvs);

        //RunActuators(cvs);

        //main_strategy_bleu(cvs);
        
        if (Avoidance(cvs)) {
          printf("Opponent\n");
          speed_regulation(cvs, -1, -1, -1);
        } else {
          if (cvs->robot_id == ROBOT_B) main_strategy_bleu;
          if (cvs->robot_id == ROBOT_Y) main_strategy_jaune;
        }
        
        break;

    case BACK_HOME_STATE:
        speed_regulation(cvs, -1, -1, -1);
        break;
    case STOP_END_STATE:
        // -1 if you want to stop the motors
        speed_regulation(cvs, -1, -1, -1);
        printf("Finished\n");
        break;
        
    default:
        printf("main loop error : no state : %d\n", cvs->main_state);        
        exit(EXIT_FAILURE);
  }
} 

/*! \brief last controller operations (called once)
 *
 * \param[in] cvs controller main structure
 */
void controller_finish(CtrlStruct *cvs)
{

  //free_spi(cvs);
  free_uart(cvs);

  free_lidar(cvs);

  free_pathPlanning(cvs);

  free_speedRegulation(cvs);

  free_odometry(cvs);

  free_Opponent(cvs);

  free_strategy(cvs);

  free_calibration(cvs);

  free_position(cvs);

  free_robotPosition(cvs);

  free(cvs->inputs);

  free(cvs);
}