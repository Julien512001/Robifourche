#include "../../include/main/GPIO_initialize_gr5.h"


void init_GPIO()
{
    //init_GPIO();
    if (gpioInitialise() < 0) {
        fprintf(stderr, "Échec de l'initialisation de pigpio.\n");
        exit(1);
    } else {
        printf("GPIO initialisé avec succès !\n");
    }


    gpioSetMode(START_UP_PIN, PI_INPUT);
    
    /*
    gpioSetPullUpDown(SDA_PIN, PI_PUD_UP);
    gpioSetPullUpDown(SCL_PIN, PI_PUD_UP);

    gpioSetMode(SDA_PIN, PI_OUTPUT);
    gpioSetMode(SCL_PIN, PI_OUTPUT);

    */
}


void finish_GPIO()
{
    gpioTerminate();

}