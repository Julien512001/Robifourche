#include "../../include/Localization/odometry_gr5.h"
#include "../../include/Localization/init_pos_gr5.h"
#include "../../include/main/CtrlStruct_gr5.h" /*L'adresse des teensy y est définie ainsi que r le rayon des roues*/
#include "../../include/Protocoles/I2C_gr5.h" /*Contient la fonction select_slave*/
#include <math.h>

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <string.h>

#define SPEED_REG_A 0x00 
#define SPEED_REG_B 0x01


/*! \brief update the robot odometry
 * 
 * \param[in,out] cvs controller main structure
 */

extern int file;

/*Du Coup Rappel, l'odométrie ne fonctionne pas encore hein*/

void update_odometry(CtrlStruct *cvs)
{
	//Lecture des encodeurs envvoyée par la teensy


	// variables declaration
	CtrlIn *inputs;         ///< controller inputs
	RobotPosition *rob_pos; ///< robot position

	// variables initialization
	inputs  = cvs->inputs;
	rob_pos = cvs->rob_pos;

	double t = inputs->t;
	double delta_t = 0.0;

	double vx_odo;
	double vy_odo;
	double omega_odo;

	double vx_I;
	double vy_I;

    double w1, w2, w3, w4;

/*
	receive_i2c(cvs, ADDR_FRONT);
	receive_i2c(cvs, ADDR_REAR);

	w1 = cvs->odometry->speed->w1;
	w2 = cvs->odometry->speed->w2;
	w3 = cvs->odometry->speed->w3;
	w4 = cvs->odometry->speed->w4;
*/
	w1 = cvs->inputs->wheel_speeds[W1];
	w2 = cvs->inputs->wheel_speeds[W2];
	w3 = cvs->inputs->wheel_speeds[W3];
	w4 = cvs->inputs->wheel_speeds[W4];

	
	printf("w1 = %f, w2 = %f, w3 = %f, w4 = %f\n", w1, w2, w3, w4);

	vx_odo = 	(w1 + w2 + w3 + w4) * r/4.0;
	vy_odo = 	(w1 - w2 - w3 + w4) * r/4.0;
	omega_odo = (-w1 + w2 - w3 + w4) * r/0.7422;
	

	delta_t = t - cvs->odometry->last_t;

	vx_I = (- vx_odo*sin(omega_odo*delta_t) + vy_odo*cos(omega_odo*delta_t));
    vy_I = (vx_odo*cos(omega_odo*delta_t) + vy_odo*sin(omega_odo*delta_t));

 	cvs->odometry->x += vx_I*delta_t;
	cvs->odometry->y += vy_I*delta_t;
	cvs->odometry->theta +=  180.0/M_PI * omega_odo*delta_t;
	//printf("%f, %f, %f, %f\n", w1, w2, w3, w4);

	cvs->odometry->last_t = t;
 
/*
	delta_t = t - cvs->odometry->last_t;
	cvs->odometry->theta +=  180.0/M_PI * omega_odo*delta_t;

	vx_I = (vx_odo*cos(cvs->odometry->theta) - vy_odo*sin(cvs->odometry->theta));
    vy_I = (vx_odo*sin(cvs->odometry->theta) + vy_odo*cos(cvs->odometry->theta));

	cvs->odometry->x += vx_I*delta_t;
	cvs->odometry->y += vy_I*delta_t;

	cvs->odometry->last_t = t;

*/
/*
	float VX = (w1+w2+w3+w4)*r/4;
	float VY = (-w1+w2+w3-w4)*r/4;
	float THETA = (-w1+w2-w3+w4)*r/0.7422; 
	float delta = cvs->inputs->t - cvs->odometry->last_t ; //Je pensais que last_t c'est le dernier temps mais pas sur du tout en vrai. Fin l'idée c'est de calculer le temps entre 2 mesures de vitesse

	//Ajustement de la position
	cvs->odometry->x += VX*delta ;
	cvs->odometry->y += VY*delta ;

	//Recalibrage en positif
	if(THETA<0){
		THETA += 360;
	}

	cvs->odometry->theta += THETA*delta;

	//Recalibrage sur le cercle trigonométrique ainsi on travail uniquement entre 0° et 360°
	if (cvs->odometry->theta > 360){
		cvs->odometry->theta = cvs->odometry->theta - 360;
	}
*/
}

/*Le code qui suit permet de lire correctement les vitesses des roues envoyées sous forme d'une liste de 8 bytes
(4 pour chaque roue car il faut 4 bytes pour représenter un float correctement apparement) par la teensy donc ce code s'occupe de
récupérer cette liste de 8 bytes et de les retransformer en float qui représente chacuns la vitesse d'une roue lue par les encodeurs*/

/*
float* readData() {
    uint8_t data_front[8]; // Assuming each wheel speed is represented by 2 bytes (1 float)
    uint8_t data_rear[8];
	float w1 = 0.0;
	float w2 = 0.0;
	float w3 = 0.0;
	float w4 = 0.0;

    // Read data from front address
    select_slave(addr_front);
	//printf("%d", ADDR_FRONT);
    if (read(file, data_front, 8) != 8) {
        printf("Failed to read from front address.\n");
        exit(1);
    }
    float front_data[2];
    memcpy(&front_data[0], data_front, sizeof(float));
    memcpy(&front_data[1], data_front + sizeof(float), sizeof(float));
    printf("Data from front: %f, %f\n", front_data[0], front_data[1]);
	w1 = front_data[0];
	w2 = front_data[1];


    // Read data from rear address
    select_slave(addr_rear);
	//printf("%d", ADDR_REAR);
    if (read(file, data_rear, 8) != 8) {
        printf("Failed to read from rear address.\n");
        exit(1);
    }
    float rear_data[2];
    memcpy(&rear_data[0], data_rear, sizeof(float));
    memcpy(&rear_data[1], data_rear + sizeof(float), sizeof(float));
    printf("Data from rear: %f, %f\n", rear_data[0], rear_data[1]);

	w3 = rear_data[1];
	w4 = rear_data[0];
	float listeVitesse[]  = {w1,w2,w3,w4};
	return listeVitesse;
}
*/
void free_odometry(CtrlStruct *cvs)
{
	free(cvs->odometry);
}
