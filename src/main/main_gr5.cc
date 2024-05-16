#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <pthread.h>

#include "../../include/main/CtrlStruct_gr5.h"
#include "../../include/main/GPIO_initialize_gr5.h"
#include "../../include/main/ctrl_main_gr5.h"
#include "../../include/Localization/LiDar_gr5.h"
#include "../../include/Protocoles/SPI_gr5.h"
#include "../../include/regulation/speed_regulation_gr5.h"
#include "../../include/Protocoles/I2C_gr5.h"
#include "../../include/Sonars/Sonars_gr5.h"
#include "../../include/Localization/calibration_gr5.h"
#include "../../include/Localization/aruco_gr5.h"
#include "../../include/LCD/LCD_gr5.h"
#include "../../include/Protocoles/uart_gr5.h"

FILE *fp0;
FILE *fp1;
FILE *fp2;
FILE *fp3;
FILE *fp4;
FILE *fp5;
FILE *fp6;
FILE *fp7;

FILE *fp_control; // File pointer for writing data for low level

#define BILLION 1000000000L

int main(void) {

    CtrlIn *inputs;
    CtrlOut *outputs;
    CtrlStruct *cvs;

    //pthread_t threadArcuo;
    pthread_t threadLidar;
    pthread_t threadUart;


    // CtrlStruct init
    inputs = (CtrlIn*) malloc(sizeof(CtrlIn));
    outputs = (CtrlOut*) malloc(sizeof(CtrlOut));
    cvs = init_CtrlStruct(inputs, outputs);
    
    speed_regulation(cvs, -1, -1, -1);

    //FILE init
    fp0 = fopen("../data/centroid.txt", "w");
    fp1 = fopen("../data/lidar.txt", "w");
    fp2 = fopen("../data/beacon.txt", "w");
    fp3 = fopen("../data/position.txt", "w");
    fp4 = fopen("../data/opponent.txt", "w");
    fp5 = fopen("../data/positionlidar.txt", "w");
    fp6 = fopen("../data/positionOdometry.txt", "w");
    fp7 = fopen("../data/speed.txt", "w");

    // GPIO init
    init_GPIO();

    // Communication protocole init
    init_spi(cvs);
    init_uart(cvs);

    // Controller init
    cvs->robot_id = ROBOT_B;
    cvs->startPosition = 2;
    controller_init(cvs);

    // Init LCD
    //set_lcd();
    int i = 1;

    // robot ID    
    // Connect Lidar
    cvs->lidar->myLidar->lidar = connectLidar();
    init_lidar(cvs);
    cvs->position->flagUpdate = 0;

    void* arg = (void*) cvs;

    pthread_create(&threadLidar, NULL, &lidar, arg);
    cvs->calib->flag = 1;
    
    calibration(cvs);
    
    while (inputs->StartSwitch == 0)
    {
        startUp(cvs);
    }

    //TIME
    struct timespec start, end;
    clock_gettime(CLOCK_MONOTONIC, &start);
    cvs->lidar->nbrOpponents = 1;
    
    // Communication Thread
    pthread_create(&threadUart, NULL, &uart, arg);
    cvs->main_state = RUN_STATE;
    // control loop
    while (true)
    {
        clock_gettime(CLOCK_MONOTONIC, &end);
        cvs->inputs->t = (end.tv_sec - start.tv_sec) + (double) (end.tv_nsec - start.tv_nsec) / BILLION;
        controller_loop(cvs);
/*
        // time LCD
        if (i % 100 == 0){
            time(cvs->inputs->t);
            // printf("int i : %d\n", i);
        }i += 1;
*/

        // Lidar position update
        /*
        double last_execution = 0.0;
        double timeUpdate = cvs->inputs->t - last_execution;
        if (timeUpdate >= 0.2){
            if (cvs->inputs->t > 2.0){
                cvs->position->flagUpdate = 1; //To modif for lidar
                last_execution = cvs->inputs->t;
            }
        }
        */
        // Stop for the end of the match
        if (cvs->inputs->t >= 90.0) {
            speed_regulation(cvs,-1,-1,-1);
            cvs->main_state = STOP_END_STATE;
            printf("End of the match\n");
        }

        //printf("%f, %f\n", cvs->opp_pos->x, cvs->opp_pos->y);
        //printf("%f, %f, %f\n", cvs->rob_pos->x, cvs->rob_pos->y, cvs->rob_pos->theta);
        //printf("%f, %f, %f\n", cvs->lidar->x, cvs->lidar->y, cvs->lidar->theta);
        //printf("%f, %f, %f\n", cvs->odometry->x, cvs->odometry->y, cvs->odometry->theta);

        //printf("%f, %f\n", cvs->path->pathSpeed->vx, cvs->path->pathSpeed->vy);

        /*
        printf("sonar1 : %f\n", cvs->inputs->sonars[0]);
        printf("sonar2 : %f\n", cvs->inputs->sonars[1]);
        printf("sonar3 : %f\n", cvs->inputs->sonars[3]);
        printf("sonar4 : %f\n", cvs->inputs->sonars[4]);
        */


        
        for (int i = 0; i < cvs->lidar->nbrCentroids; i++) {
            fprintf(fp0, "%f, %f\n", cvs->lidar->myCentroids[i].radius, cvs->lidar->myCentroids[i].angle);
        }
        
        /*
        //printf("%d\n", cvs->lidar->nbrCentroids);
        for (int i = 0; i < inputs->nb_lidar_data; i++) {
            fprintf(fp1, "%f, %f\n", inputs->last_lidar_radius[i], inputs->last_lidar_angle[i]);
        }
        */
        /*
            fprintf(fp2, "%f, %f\n", cvs->lidar->beacons->beacon1.radius, cvs->lidar->beacons->beacon1.angle);
            fprintf(fp2, "%f, %f\n", cvs->lidar->beacons->beacon2.radius, cvs->lidar->beacons->beacon2.angle);
            fprintf(fp2, "%f, %f\n", cvs->lidar->beacons->beacon3.radius, cvs->lidar->beacons->beacon3.angle);
        */
        /*
        printf("%f, %f\n", cvs->lidar->beacons->beacon1.radius, cvs->lidar->beacons->beacon1.angle);
        printf("%f, %f\n", cvs->lidar->beacons->beacon2.radius, cvs->lidar->beacons->beacon2.angle);
        printf("%f, %f\n", cvs->lidar->beacons->beacon3.radius, cvs->lidar->beacons->beacon3.angle);
        */

        //fprintf(fp3, "%f, %f, %f\n", cvs->rob_pos->x, cvs->rob_pos->y, cvs->rob_pos->theta);
        //fprintf(fp4, "%f, %f\n", cvs->opp_pos->x, cvs->opp_pos->y);
        //fprintf(fp5, "%f, %f, %f\n", cvs->lidar->x, cvs->lidar->y, cvs->lidar->theta);
        //fprintf(fp6, "%f, %f, %f\n", cvs->odometry->x, cvs->odometry->y, cvs->odometry->theta);
        
    }

    pthread_join(threadLidar, NULL);
    pthread_join(threadUart, NULL);


    disconnectLidar(cvs->lidar->myLidar->lidar);

    controller_finish(cvs);

    finish_uart(cvs);
    finish_GPIO();

    fclose(fp0);
    fclose(fp1);
    fclose(fp2);
    fclose(fp3);
    fclose(fp4);
    fclose(fp5);
    fclose(fp6);
    fclose(fp7);

    
    return 0;
}