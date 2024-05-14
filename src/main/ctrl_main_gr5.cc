/*!
 * \file ctrl_main_gr5.cc
 * \brief Initialization, loop and finilization of the controller written in C (but compiled as C++)
 */

#include "../../include/main/ctrl_main_gr5.h"

#include <cstdio>
#include <math.h>

FILE *fp_control; // File pointer for writing data

/*! \brief initialize controller operations (called once)
 *
 * \param[in] cvs controller main structure
 */

bool Avoidance(CtrlStruct *cvs) {
  if ((euclidean(cvs->rob_pos->x, cvs->rob_pos->y, cvs->opp_pos->x, cvs->opp_pos->y) < 0.75) && (cvs->rob_pos->x > 0.0) && (cvs->rob_pos->y > 0.0)) {
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

	t = inputs->t;



  switch (cvs->main_state) {

    case RUN_STATE:
        /*
        
        if (cvs->inputs->t >= 0.0){
          vitesseX = 0.2;
          vitesseY = 0.0;
          vitesseZ = 0.0;
        }
        if (cvs->inputs->t > 4.0){
          vitesseX = 0.3;
          vitesseY = 0.0;
          vitesseZ = 0.0;
        }
        if (cvs->inputs->t > 7.0){
          vitesseX = -0.2;
          vitesseY = 0.0;
          vitesseZ = 0.0;
        }
        if (cvs->inputs->t > 12.0){
          vitesseX = 0.0;
          vitesseY = 0.0;
          vitesseZ = 0.0;
        }
        
        
        speed_regulation(cvs, vitesseX, vitesseY, vitesseZ);*/
        //speed_regulation(cvs, 0.0, 0.0, 0.0);
      
        
        //speed_regulation(cvs, 0.2, -1, -1);
        //main_strategy(cvs);
        /*
        if (cvs->inputs->t > 4.0){
          cvs->main_state = STOP_END_STATE;
        }
        */
        /*
        fp_control = fopen("../S13_closeloop2roue.txt", "a");
        if (fp_control == NULL) {
          perror("Erreur lors de l'ouverture du fichier");
          // Ou utilisez errno pour obtenir le code d'erreur spécifique
          // printf("Erreur lors de l'ouverture du fichier : %d\n", errno);
        }
        else{
          fprintf(fp_control,"control1 : %f, %f\n", cvs->inputs->wheel_speeds[W2], cvs->outputs->wheel_commands[W2]);
          fclose(fp_control);
        }*/
        
    /*
        if (Avoidance(cvs)) {
          printf("Opponent\n");
          speed_regulation(cvs, -1, -1, -1);
        } else {
          main_strategy(cvs);
        }
    // */
        //speed_regulation(cvs, 0.2, 0, 0);
        //main_strategy(cvs);
        // main_strategy_sonar(cvs);
        
        //stratLidar(cvs);
        
        // speed_regulation(cvs, 0.0, -0.2, -1);
        //Opposite_base_Strat(cvs);


        EntonnoirOUT(cvs);
        CtrlActuators(cvs);

        sleep(3);

        EntonnoirIN(cvs);
        CtrlActuators(cvs);

        sleep(1);

        StopBlocFourche(cvs);
        EntonnoirSTOP(cvs);
        CtrlActuators(cvs);

        sleep(10);

        FeuilleOUT(cvs);
        CtrlActuators(cvs);

        sleep(5);

        FourcheEnHaut(cvs);
        CtrlActuators(cvs);
        sleep(1);

        SortirBlocFourche(cvs);
        EntonnoirIN(cvs);
        CtrlActuators(cvs);

        sleep(10);

        FourcheEnBas(cvs);
        CtrlActuators(cvs);

        sleep(4);

        FeuilleIN(cvs);
        CtrlActuators(cvs);

        sleep(4);



        //FourcheEnBas(cvs);
        /*
        BrasEntonnoirMiddle(cvs);
        */
        /*
        EntonnoirIN(cvs);
        FeuilleIN(cvs);
        CtrlActuators(cvs);
        
        sleep(3);
        */
        /*
        StopBlocFourche(cvs);
        FeuilleOUT(cvs);
        CtrlActuators(cvs);

        sleep(7);
        */

        // FourcheEnBas(cvs);
        // BrasEntonnoirMiddle(cvs);
        // EntonnoirOUT(cvs);
        // StopBlocFourche(cvs);
        // CtrlActuators(cvs);

        // sleep(3);

        // SortirBlocFourche(cvs);
        // FeuilleOUT(cvs);
        // CtrlActuators(cvs);

        // sleep(7);

        // FourcheEnHaut(cvs);
        // CtrlActuators(cvs);

        // sleep(1);

        
        //Get_out_Strat(cvs);

        // Sonars
        /*
        sonar(cvs);
        printf("Sonar 0 : %f,Sonar 1 : %f,Sonar 2 : %f\n",cvs->inputs->sonars[FRONT],
         cvs->inputs->sonars[LEFT1], cvs->inputs->sonars[RIGHT]);
        */
        cvs->main_state = STOP_END_STATE;
        break;

    case BACK_HOME_STATE:
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