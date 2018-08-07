/************************************************************************/
/*                                                                      */
/*    Sensors.h --  Library to control the actuators of a dishwasher    */
/*                          Version - 2.1                               */
/*                                                                      */
/************************************************************************/
/*  Author:   Bravr - Vitor Sato Eschholz                               */
/*  July 26, 2018                                                       */
/************************************************************************/
/*  Module Description:                                                 */
/*                                                                      */
/*  This library contains the implementation of the routines which      */
/*  manage the interface of the microprocessor with the sensors of the  */
/*  dishwasher. It mainly allows the main program to receive the data   */
/*  in the way they will be used, which means that the routines are     */
/*  responsible for converting voltage and pulses, in numbers that have */
/*  a useful meaning, such as temperature in ºC, turbidity level in NTU */
/*  or water flow in L/m^3.                                             */
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
/*    - Measure tempScale, dirtyScale, dirtyLevel, flowScale			*/
/*                                                                      */
/************************************************************************/

#ifndef Sensors_h
#define Sensors_h

#include "Arduino.h"

	class Sensors{
	public:
		Sensors();

		void configuration();
		void niveau_sel();
		void niveau_rincage();
		double mesure_temperature();
		int niveau_turbidite();
		double calculate_flow();
		int counting_pulses();

		bool nvSelState;                   // Actual state of the salt level sensor
		bool nvRinState;                   // Actual state of the rinse aid level sensor

	private:
		// Pins definition
		const byte nvSelPin = 26 ;            // Digital Pin
		const byte nvRinPin = 28;             // Digital Pin
		const byte thermPin = A0;             // Analog Pin 0
		const byte turbPin = A1;              // Analog Pin 1
		const byte flowPin = 47;              // Pin T5 to make an external clock of the time
		// Intrinsic component characteristics
		const int samplePeriod = 500;         // How long debitmetre pulses are going to be counted
		const double tempScale = 0.025;       // Conversion scale of the thermistor [V/ºC]
		const double dirtyScale = -0.001;     // Conversion scale of the turbidity sensor [V/NTU]
		const int dirtyLevel = 500;           // Minimum accepted dirty level [NTU]
		const double flowScale = 0.00025;     // Coversion scale of the flow sensor (L/s*Hz)
		const double resADC = 5/1024;         // Resolution ADC 10 bits
	};
	
#endif