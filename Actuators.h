/************************************************************************/
/*																		*/
/*  Actuators.h	--	Library to control the actuators of a dishwasher    */
/*                          Version - 2.1                               */
/*																		*/
/************************************************************************/
/*  Author:   Bravr - Vitor Sato Eschholz                               */
/*  July 26, 2018                                                       */
/************************************************************************/
/*  Module Description:                                                 */
/*                                                                      */
/*  This library contains the implementation of the routines which      */
/*  manage the necessary control signals to activate or deactivate the  */
/*  composants of the dishwasher, taking in to account the context      */
/*  where they are used. Meaning that an error management system is     */
/*  also implemented in order to avoid a mistaken activation.           */
/*                                                                      */
/*                                                                      */
/************************************************************************/
/*  Revision History:                                                   */
/*                                                                      */
/*  07/26/2018(VitorSE): Created                                        */
/*  08/03/2018(VitorSE): Changes related to the functions that should   */
/*                       be implemented in the main program.            */
/*                                                                      */
/*  Todo:                                                               */
/*    - Standardize the headers of all routines                         */
/*                                                                      */
/************************************************************************/

#ifndef Actuators_h
#define Actuators_h

#include "Arduino.h"
#include <setjmp.h>

	class Actuators {
	public:
        Actuators(jmp_buf context);

        const byte diverter1Pin = 12;         // Digital Pin
        const byte diverter2Pin = 13;         // Digital Pin
        const int timeDoseur = 10;            // Necessary time to activate doseur [s]

        void init_variables();
        void relayOn_off();
        void relayOn_on();
        void cyclage_off();
        void cyclage_on();
        void chauffage_off();
        void chauffage_on();
        void remplissage_off();
        void remplissage_on();
        void regeneration_off();
        void regeneration_on();
        void diverter_off();
        void diverter_on();
        void vidange_off();
        void vidange_on();
        void sechage_off();
        void sechage_on();
        void doseur_off();
        void doseur_on();
        void error_management(int code);

        bool stateRelayOn;                 // Actual state of the relayOn
        bool stateCyclage;                 // Actual state of the cycle pump
        bool cntrlCyclage1;                // Actual state of the pins which controle the cycle pump 
        bool cntrlCyclage2;                // 
        bool stateChauffage;               // Actual state of the heater
        bool stateRemplissage;             // Actual state of the filling valve
        bool stateRegeneration;            // Actual state of the regeneration valve
        bool stateDiverter;                // Actual state of the diverter valte
        bool stateVidange;                 // Actual state of the drain pump
        bool stateSechage;                 // Actual state of the fan dryer
        bool stateDoseur;                  // Actual state of the dispenser
        bool flagTimer3;


    private:
        void configuration();
        // Pins definition
        const byte relayOnPin = 8;            // Digital Pin
        const byte comCyclagePin = 14;         // Digital Pin
        const byte comChauffagePin = 15;       // Digital Pin
        const byte comRemplissagePin = 22;    // Digital Pin
        const byte comRegenerationPin = 24;   // Digital Pin
        const byte comDiverterPin = 4;        // Digital Pin
        const byte comVidangePin = 5;         // Digital Pin
        const byte comSechagePin = 6;         // Digital Pin
        const byte comDoseurPin = 7;          // Digital Pin
        const byte pompeCyc1Pin = 9;          // Digital Pin
        const byte pompeCyc2Pin = 10;         // Digital Pin
        // Intrinsic component characteristics
        jmp_buf _env; 
  };
	
#endif
