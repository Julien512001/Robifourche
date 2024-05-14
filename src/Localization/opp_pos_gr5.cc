#include "../../include/Localization/opp_pos_gr5.h"
#include "../../include/Localization/LiDar_gr5.h"
#include "../../include/Localization/init_pos_gr5.h"
#include <math.h>

/*! \brief compute the opponents position using the tower
 * 
 * \param[in,out] cvs controller main structure
 */
void opponents_tower(CtrlStruct *cvs)
{
	// variables declaration
	int nb_opp; ///< number of opponents

	CtrlIn *inputs; ///< inputs
	RobotPosition *rob_pos; ///< robot own position
	OpponentsPosition *opp_pos; ///< opponents position

	// variables initialization
	inputs  = cvs->inputs;
	rob_pos = cvs->rob_pos;
	opp_pos = cvs->opp_pos;
/*
	double x_opp_R, y_opp_R, x_opp_I, y_opp_I;
	x_opp_R = cvs->lidar->opponents->radius*cos(M_PI/180.0 * cvs->lidar->opponents->angle) + 0.00294;
	y_opp_R = cvs->lidar->opponents->radius*sin(M_PI/180.0 * cvs->lidar->opponents->angle);

	x_opp_I = x_opp_R*cos(M_PI/180.0 * cvs->rob_pos->theta) - y_opp_R*sin(M_PI/180.0 * cvs->rob_pos->theta) + cvs->rob_pos->x;
	y_opp_I = x_opp_R*sin(M_PI/180.0 * cvs->rob_pos->theta) + y_opp_R*cos(M_PI/180.0 * cvs->rob_pos->theta) + cvs->rob_pos->y;

	cvs->opp_pos->nb_opp =  cvs->lidar->nbrOpponents;
	cvs->opp_pos->x = x_opp_I;
	cvs->opp_pos->y = y_opp_I;
*/
	//printf("pos: %f, %f\n", x_opp_I, y_opp_I);
}

void free_Opponent(CtrlStruct *cvs)
{
	free(cvs->opp_pos);
}

