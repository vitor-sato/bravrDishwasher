/************************************************************************/
/*                                                                      */
/*  Actuators.cpp --  Library to control the actuators of a dishwasher  */
/*                          Version - 2.1                               */
/*                                                                      */
/************************************************************************/
/*  Author:   Bravr - Vitor Sato Eschholz                               */
/*  July 26, 2018                                                       */
/************************************************************************/
/*  Module Description:                                                 */
/*                                                                      */
/*  This library contains the implementation of the routines which 	    */
/*  manage the necessary control signals to activate or deactivate the  */
/*  composants of the dishwasher, taking in to account the context      */
/*	where they are used. Meaning that an error management system is 	*/
/*  also implemented in order to avoid a mistaken activation.           */
/*                                                                      */
/*                                                                      */
/************************************************************************/
/*  Revision History:                                                   */
/*                                                                      */
/*  07/26/2018(VitorSE): Created										*/
/*  08/03/2018(VitorSE): Changes related to the functions that should 	*/
/*                       be implemented in the main program.            */
/*  Todo:                                                               */
/*                                                                      */
/*    - Standardize the headers of all routines                         */
/*                                                                      */
/************************************************************************/

#include "Arduino.h"
#include "Actuators.h"
#include <setjmp.h>

// ----------------------------- Constructor -----------------------------
Actuators::Actuators(jmp_buf context){
	configuration();
  	memcpy(_env, context, sizeof(context));
}

// ---------------------------- Configuration ----------------------------
// Description : rotine to set the pin configuration of the microcontroler
void Actuators::configuration(void) {
	pinMode(relayOnPin, OUTPUT);
	pinMode(comCyclagePin, OUTPUT);
	pinMode(comChauffagePin, OUTPUT);
	pinMode(comRemplissagePin, OUTPUT);
	pinMode(comRegenerationPin, OUTPUT);
	pinMode(comDiverterPin, OUTPUT);
	pinMode(comVidangePin, OUTPUT);
	pinMode(comSechagePin, OUTPUT);
	pinMode(comDoseurPin, OUTPUT);
	pinMode(pompeCyc1Pin, OUTPUT);
	pinMode(pompeCyc2Pin, OUTPUT);
	pinMode(diverter1Pin, OUTPUT);
	pinMode(diverter2Pin, OUTPUT);

  return;
}

// ---------------------- Initialization variables ----------------------
// Description : rotine to initialize the desired initial state of the
// actuadors
void Actuators::init_variables(void) {
  stateRelayOn = LOW;
  stateCyclage = LOW;
  stateChauffage = LOW;
  stateRemplissage = LOW;
  stateRegeneration = LOW;
  stateDiverter = LOW;
  stateVidange = LOW;
  stateSechage = LOW;
  stateDoseur = LOW;
  flagTimer3 = LOW;
  return;
}
	
// --------------------- RelayOn Off ---------------------
// Description : rotine to deactivate the relayOn.
void Actuators::relayOn_off(void) {
  stateRelayOn = LOW;
  digitalWrite(relayOnPin, stateRelayOn);
}

// --------------------- RelayOn On ---------------------
// Description : rotine to activate the relayOn.
void Actuators::relayOn_on(void) {
  stateRelayOn = HIGH;
  digitalWrite(relayOnPin, stateRelayOn);
  return;              
}

// --------------------- Cyclage Off ---------------------
// Description : rotine to deactivate the cycle pump.
void Actuators::cyclage_off(void) {
  stateCyclage = LOW;
  digitalWrite(comCyclagePin, stateCyclage);
  return;
}

// ----------------------- Cyclage On -----------------------
// Description : rotine to activate the cycle pump. For this,
// it checks if the relayOn is activated and if there's water
// in the tank.
void Actuators::cyclage_on(void) {
  int errorCode = -1;
  if (stateRelayOn == LOW) {
    errorCode = 1;               // error code, relayOn is off
    error_management(errorCode);
  }
  else {
    stateCyclage = HIGH;
    digitalWrite(comCyclagePin, stateCyclage);
    error_management(errorCode);            // no error has occured
  }
}

// ----------------------- Chauffage Off -----------------------
// Description : rotine to deactivate the heating.
void Actuators::chauffage_off(void) {
  stateChauffage = LOW;
  digitalWrite(comChauffagePin, stateChauffage);
  return;
}

// ----------------------- Chauffage On -----------------------
// Description : rotine to activate the heating. For this, it
// checks if there is an lavage cycle being executed at the
// moment
void Actuators::chauffage_on(void) {
  int errorCode = -1;
  if (stateCyclage == LOW) {
    errorCode = 3;               // error code, cyclage is off
    error_management(errorCode);
  }
  else {
    stateChauffage = HIGH;
    digitalWrite(comChauffagePin, stateChauffage);
    error_management(errorCode);            // no error has occured
  }
}

// ----------------------- Remplissage Off ---------------------
// Description : rotine to deactivate the filling valve.
void Actuators::remplissage_off(void) {
  stateRemplissage = LOW;
  digitalWrite(comRemplissagePin, stateRemplissage);
  return;
}

// ----------------------- Remplissage On -----------------------
// Description : rotine to activate the filling valve. For this,
// it checks if the relayOn is activated.
void Actuators::remplissage_on(void) {
  int errorCode = -1;
  if (stateRelayOn == LOW) {
    errorCode = 1;               // error code, relayOn is off
    error_management(errorCode);
  }
  else {
    stateRemplissage = HIGH;
    digitalWrite(comRemplissagePin, stateRemplissage);
    error_management(errorCode);                     // no error has occured
  }
}

// -------------------- Regeneration Off -----------------------
// Description : rotine to deactivate the regeneration valve.
void Actuators::regeneration_off(void) {
  stateRegeneration = LOW;
  digitalWrite(comRegenerationPin, stateRegeneration);
  return;
}


// -------------------- Regeneration On -----------------------
// Description : rotine to activate the regeneration valve.
// For this, it checks if the relayOn is activated.
void Actuators::regeneration_on(void) {
  int errorCode = -1;
  if (stateRelayOn == LOW) {
    errorCode = 1;               // error code, relayOn is off
    error_management(errorCode);
  }
  else {
    stateRegeneration = HIGH;
    digitalWrite(comRegenerationPin, stateRegeneration);
    error_management(errorCode);                     // no error has occured
  }
}

// -------------------- Diverter Off -----------------------
// Description : rotine to deactivate the diverter valve.
void Actuators::diverter_off(void) {
  stateDiverter = LOW;
  digitalWrite(comDiverterPin, stateDiverter);
  TIMSK3 &= (0 << TOIE3);
  flagTimer3 = LOW;
  return;
}

// -------------------- Diverter On -----------------------
// Description : rotine to activate the diverter valve. For
// this, it checks if the relayOn is activated and if there
// is an lavage cycle being executed at the moment. Besides,
// it also changes the direction of the water flow each X minutes
// that is sent to the routine.
void Actuators::diverter_on(void) {
  int errorCode = -1;
  if (stateRelayOn == LOW) {
    errorCode = 1;               // error code, relayOn is off
    error_management(errorCode);
  }
  else if (stateCyclage == LOW) {
    errorCode = 3;               // error code, cyclage is off
    error_management(errorCode);
  }
  else {
    stateDiverter = HIGH;
    digitalWrite(comDiverterPin, stateDiverter);
    TIMSK3 |= (1 << TOIE3);
    flagTimer3 = HIGH;
    error_management(errorCode);            // no error has occured
  }
}

// -------------------- Vidange Off -----------------------
// Description : rotine to deactivate the drain pump.
void Actuators::vidange_off(void) {
  stateVidange = LOW;
  digitalWrite(comVidangePin, stateVidange);
  return;
}

// -------------------- Vidange On -----------------------
// Description : rotine to activate the drain pump. For
// this, it checks if the relayOn is activated.
void Actuators::vidange_on(void){
  int errorCode = -1;
  if (stateRelayOn == LOW) {
    errorCode = 1;               // error code, relayOn is off
    error_management(errorCode);
  }
  else {
    stateVidange = HIGH;
    digitalWrite(comVidangePin, stateVidange);
    error_management(errorCode);            // no error has occured
  }
}

// -------------------- Sechage Off -----------------------
// Description : rotine to deactivate the fan dry.
void Actuators::sechage_off(void) {
  stateSechage = LOW;
  digitalWrite(comSechagePin, stateSechage);
  return;
}

// -------------------- Sechage On -----------------------
// Description : rotine to activate the fan dry. For
// this, it checks if the relayOn is activated and if there
// is a washing phase being executed at the moment.
void Actuators::sechage_on(void) {
  int errorCode = -1;
  if (stateRelayOn == LOW) {
    errorCode = 1;               // error code, relayOn is off
    error_management(errorCode);
  }
  else if (stateCyclage == HIGH) {
    errorCode = 4;               // error code, washing is not over
    error_management(errorCode);
  }
  else {
    stateSechage = HIGH;
    digitalWrite(comSechagePin, stateSechage);
    error_management(errorCode);                     // no error has occured
  }
}

// ----------------------- Doseur Off -------------------------
// Description : rotine to deactivate the solenoid valve which
// releases the detergent and the rinse aid in the tank.
void Actuators::doseur_off(void) {
  stateDoseur = LOW;
  digitalWrite(comDoseurPin, stateDoseur);
  return;
}

// --------------------- Doseur On ---------------------------
// Description : rotine to activate the solenoid valve related
// to the dispenser. For this, it checks if the relayOn is
// activated and if there is an lavage cycle being executed
// at the moment. Note that to really activate the dispenser
// it has to be in HIGH for 45 seconds.
void Actuators::doseur_on(void) {
  int errorCode = -1;
  if (stateRelayOn == LOW) {
    errorCode = 1;               // error code, relayOn is off
    error_management(errorCode);
  }
  else if (stateCyclage == LOW) {
    errorCode = 3;               // error code, cyclage is off
    error_management(errorCode);
  }
  else {
    digitalWrite(comDoseurPin, HIGH);
    stateDoseur = HIGH;
    error_management(errorCode);                     // no error has occured
  }
}

/******************************************************************************/
/*                              Error Management                              */
/******************************************************************************/
/*                                                                            */
/*  Routine dedicated to concentrate all the "longjmp" calls in order to      */
/*  manage some errors that may occur during the program execution. After     */
/*  being called, its only function is to redirect the program execution      */
/*  where the errors will be treated.                                         */
/*                                                                            */
/******************************************************************************/
void Actuators::error_management(int code){
  switch(code){
    case 1:                                   // The relayOn state is off when it shoul be on
      longjmp(_env, code);
    case 2:                                   // Prewash
      longjmp(_env, code);
      break;
    case 3:                                   // Mainwash
      longjmp(_env, code);
      break;
    case 4:                                   // Intermediate rinse
      longjmp(_env, code);
      break;
    case 5:                                   // Final rinse
      longjmp(_env, code);
      break;
    Serial.println("OKAY");
  }
}