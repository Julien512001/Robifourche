#include "../../include/Actuators/Actuators_gr5.h"
#include <math.h>


void CtrlActuators(CtrlStruct *cvs)
{
    CtrlIn *inputs;
    uint32_t *message = cvs->actuator->message_array;
    inputs = cvs->inputs;
    uint32_t result = 0;
    for (int i = 0; i<14; i++){
        result += message[i]*pow(2,i);
    }
    send_spi(cvs, result, 0);
}

void ResetActuator(CtrlStruct *cvs){
    for (int i = 0; i<14;i++){
        cvs->actuator->message_array[i] = 0;
    }
}

void SortirBlocFourche(CtrlStruct *cvs){
    cvs->actuator->message_array[0] = 1;
    cvs->actuator->message_array[1] = 0;
}

void RentrerBlocFourche(CtrlStruct *cvs){
    cvs->actuator->message_array[0] = 0;
    cvs->actuator->message_array[1] = 1;
}

void StopBlocFourche(CtrlStruct *cvs){
    cvs->actuator->message_array[0] = 0;
    cvs->actuator->message_array[1] = 0;
}

void FourcheEnHaut(CtrlStruct *cvs){
    cvs->actuator->message_array[2] = 1;
}

void FourcheEnBas(CtrlStruct *cvs){
    cvs->actuator->message_array[2] = 0;
}

void FeuilleOUT(CtrlStruct *cvs){
    cvs->actuator->message_array[3] = 1;
    cvs->actuator->message_array[4] = 0;
}

void FeuilleIN(CtrlStruct *cvs){
    cvs->actuator->message_array[3] = 0;
    cvs->actuator->message_array[4] = 1;
}

void FeuilleSTOP(CtrlStruct *cvs){
    cvs->actuator->message_array[3] = 0;
    cvs->actuator->message_array[4] = 0;
}

void EntonnoirOUT(CtrlStruct *cvs){
    cvs->actuator->message_array[5] = 1;
    cvs->actuator->message_array[6] = 0;
}

void EntonnoirIN(CtrlStruct *cvs){
    cvs->actuator->message_array[5] = 0;
    cvs->actuator->message_array[6] = 1;
}

void EntonnoirSTOP(CtrlStruct *cvs){
    cvs->actuator->message_array[5] = 0;
    cvs->actuator->message_array[6] = 0;
}

void BrasEntonnoirFermé(CtrlStruct *cvs){
    cvs->actuator->message_array[7] = 1;
    cvs->actuator->message_array[8] = 0;
}

void BrasEntonnoirOuvert(CtrlStruct *cvs){
    cvs->actuator->message_array[7] = 0;
    cvs->actuator->message_array[8] = 1;
}

void BrasEntonnoirMiddle(CtrlStruct *cvs){
    cvs->actuator->message_array[7] = 0;
    cvs->actuator->message_array[8] = 0;
}

void BrasEntonnoirSeq(CtrlStruct *cvs){
    cvs->actuator->message_array[7] = 1;
    cvs->actuator->message_array[8] = 1;
}

void BrasPanneauEnHaut(CtrlStruct *cvs){
    cvs->actuator->message_array[9] = 0;
}

void BrasPanneauEnBas(CtrlStruct *cvs){
    cvs->actuator->message_array[9] = 1;
}

void RouePanneauBleu(CtrlStruct *cvs){
    cvs->actuator->message_array[10] = 1;
    cvs->actuator->message_array[11] = 0;
}

void RouePanneauJaune(CtrlStruct *cvs){
    cvs->actuator->message_array[10] = 0;
    cvs->actuator->message_array[11] = 1;
}

void RouePanneauSTOP(CtrlStruct *cvs){
    cvs->actuator->message_array[10] = 0;
    cvs->actuator->message_array[11] = 0;
}

void TapisRentrant(CtrlStruct *cvs){
    cvs->actuator->message_array[12] = 1;
    cvs->actuator->message_array[13] = 0;
}

void TapisRentrant_Pot(CtrlStruct *cvs){
    cvs->actuator->message_array[12] = 0;
    cvs->actuator->message_array[13] = 1;
}

void TapisSortant(CtrlStruct *cvs){
    cvs->actuator->message_array[12] = 1;
    cvs->actuator->message_array[13] = 1;
}

void TapisStop(CtrlStruct *cvs){
    cvs->actuator->message_array[12] = 0;
    cvs->actuator->message_array[13] = 0;
}

void free_Actuator(CtrlStruct *cvs)
{
	free(cvs->actuator->message_array);
}

