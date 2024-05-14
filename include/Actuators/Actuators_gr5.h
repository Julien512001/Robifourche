#ifndef ACTUATOR_GR5_H
#define ACTUATOR_GR5_H

#include "../../include/main/CtrlStruct_gr5.h"
#include "../../include/Protocoles/SPI_gr5.h"

typedef struct Actuator{
    uint32_t *message_array;
} Actuator;

void CtrlActuators(CtrlStruct *cvs);

void ResetActuator(CtrlStruct *cvs);

void SortirBlocFourche(CtrlStruct *cvs);

void RentrerBlocFourche(CtrlStruct *cvs);

void StopBlocFourche(CtrlStruct *cvs);

void FourcheEnHaut(CtrlStruct *cvs);

void FourcheEnBas(CtrlStruct *cvs);

void FeuilleOUT(CtrlStruct *cvs);

void FeuilleIN(CtrlStruct *cvs);

void FeuilleSTOP(CtrlStruct *cvs);

void EntonnoirOUT(CtrlStruct *cvs);

void EntonnoirIN(CtrlStruct *cvs);

void EntonnoirSTOP(CtrlStruct *cvs);

void BrasEntonnoirFermé(CtrlStruct *cvs);

void BrasEntonnoirOuvert(CtrlStruct *cvs);

void BrasEntonnoirMiddle(CtrlStruct *cvs);

void BrasEntonnoirSeq(CtrlStruct *cvs);

void BrasPanneauEnHaut(CtrlStruct *cvs);

void BrasPanneauEnBas(CtrlStruct *cvs);

void RouePanneauBleu(CtrlStruct *cvs);

void RouePanneauJaune(CtrlStruct *cvs);

void RouePanneauSTOP(CtrlStruct *cvs);

void TapisRentrant(CtrlStruct *cvs);

void TapisRentrant_Pot(CtrlStruct *cvs);

void TapisSortant(CtrlStruct *cvs);

void TapisStop(CtrlStruct *cvs);

void free_Actuator(CtrlStruct *cvs);


#endif