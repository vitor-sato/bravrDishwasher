/************************************************************************/
/*                                                                      */
/*  Sensors.cpp --  Library to control the actuators of a dishwasher    */
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
/*                                                                      */
/************************************************************************/

#include "Arduino.h"
#include "Sensors.h"

// ----------------------------- Constructor -----------------------------
Sensors::Sensors(void){
	configuration();
}

// ---------------------------- Configuration ----------------------------
// Description : rotine to set the pin configuration of the microcontroler
void Sensors::configuration(void) {
	pinMode(nvSelPin, INPUT);
	pinMode(nvRinPin, INPUT);
	pinMode(thermPin, INPUT);
	pinMode(turbPin, INPUT);
	pinMode(flowPin, INPUT);
  return;
}

// --------------------- Salt Level ---------------------
// Description : rotine that reads the level of salt in the
// reservoir an updates the nvSelState
void Sensors::niveau_sel() {
  nvSelState = digitalRead(nvSelPin);
}

// ------------------ Rinse aid Level -------------------
// Description : rotine that reads the level of rinse aid
// in the reservoir an updates the nvRinState
void Sensors::niveau_rincage() {
  nvRinState = digitalRead(nvRinPin);
}

// ----------------- Mesure Temperature ------------------
// Description : rotine that reads the voltage from the
// thermistor and returns the temperature of the water
// in that moment.
double Sensors::mesure_temperature() {
  int thermistor;
  double temperature;
  double tension;

  thermistor = analogRead(thermPin);
  tension = (thermistor*5.0)/1024.0;
  temperature = ((tension-2.5) / tempScale);
  return temperature;
}

// ----------------- Dirty Level ------------------
// Description : rotine that reads the voltage from the
// turbidity sensor and returns if the water is dirty and
// the dishes must be washed for more time.
int Sensors::niveau_turbidite() {
  int sensorRead;
  double actualLevel;
  double tension;

  sensorRead = analogRead(turbPin);
  tension = (sensorRead*5.0)/1024.0;
  actualLevel = ((tension-3.0) / dirtyScale) + 1500;
  Serial.print("The turbidity level of the water is: ");
  Serial.print(actualLevel);
  Serial.println(" NTU");
  if (actualLevel > dirtyLevel) {
    return 1;                // the water is dirty
  }
  else {
    return -1;							// the turbidity level of the water is okay
  }
}

// ----------------- Calculate Flow ------------------
// Description : caculate the flow rate of the water
double Sensors::calculate_flow() {
  int numberTimes = 3;
  int mesuresPulses[numberTimes];
  int gap;
  int sum = 0;
  double flow;
  int i;

  for (i = 0; i < numberTimes; i++) {
    mesuresPulses[i] = counting_pulses();                 // Save the number of pulses in a vector
    if (i == 1) {                                         // Checks if the flow is already steady
      gap = abs(mesuresPulses[0] - mesuresPulses[1]);     // Calculate the gap of the first and second mesure
      if (gap > mesuresPulses[1] * 0.1){                   // Check if the first mesure is too different from the second
        mesuresPulses[0] = mesuresPulses[1];              // If yes, discards the first one
      	i--;
      }
    }
  }

  for (i = 0; i < numberTimes; i++) {
    sum = sum + mesuresPulses[i];
  }

  flow = (2 * sum / 3) * flowScale;
  return flow;

}

// ----------------- Counting Pulses ------------------
// Description : configurates the timer 5 in order to count
// the number of pulses are capted in the pin 47 for a sample
// period time.
int Sensors::counting_pulses(void) {
  int nPulses;                  // Variable to store the number of pulses counted

  cli();                        // Desactivate the interruptions to guarantee the right configuration
  bitSet(TCCR5B , CS52);        // Counter Clock source is external pin
  bitSet(TCCR5B , CS51);        //
  bitSet(TCCR5B , CS50);        // Clock on rising edge
  TCNT5 = 0;                    // Guarantees the counting will start on zero
  sei();                        // Reactivate the interruptions

  delay(samplePeriod);          // Period to count the number of pulses

  cli();                        // Desactivate the interruptions to not counting pulses out of the interval
  nPulses = TCNT5;              // Get the number of pulses occurred in the sample period
  sei();                        // Reactivate the interruptions
  return nPulses;
}