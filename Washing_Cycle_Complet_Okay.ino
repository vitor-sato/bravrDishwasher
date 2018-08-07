/******************************************************************************/
/*                                                                            */
/*  Washing_cycle.ino -- Program to execute a normal cycle in a dishwasher    */
/*                               Version - Teste                              */
/*                                                                            */
/******************************************************************************/
/*  Author:   Vitor Sato Eschholz                                             */
/*  July 26, 2018                                                             */
/*                                                                            */
/******************************************************************************/
/*  Program Description:                                                      */
/*                                                                            */
/*  This program contains the implementation of the routines necessary to     */
/*  execute the different phases of a dishwasher washing cycle.It includes a  */
/*  simple mechanism to detect some software errors and avoid damaging the    */
/*  the components of the controlled machine. Finally it prevents a sudden    */
/*  interruption of the cycle providing the necessaries treatments to         */
/*  restart the cycle without any problem.                                    */
/*                                                                            */
/******************************************************************************/
/*  Revision History:                                                         */
/*                                                                            */
/*  07/20/2018(VitorSE): Created with phases routines                         */
/*  07/25/2018(VitorSE): Addition of erroe treatment and pause system         */
/*  07/26/2018(VitorSE): Modularization of the program with the creation of   */
/*                       Actuators.h, Sensors.h and Timer.h                   */
/*	07/27/2018(VitorSE): Error management changed to the Actuators library	  */
/*                       Changed program to test in the protoboard            */
/*                                                                            */
/*  Todo:                                                                     */
/*    - Measure tankCapacity, timeRegeneration ;                              */
/*    - Review the routine continue_cycle() ;                                 */
/*                                                                            */
/******************************************************************************/

#include <setjmp.h>
#include <Actuators.h>
#include <Sensors.h>

jmp_buf env;
Actuators a(env);
Sensors s;

/******************************************************************************/
/***************************** Defining Constants *****************************/
/******************************************************************************/
const byte overfPin = 3;              // Pin INT1, external interrupt
const byte doorPin = 2;               // Pin INT0, external interrupt with higher priority
const int tankCapacity = 2;           // Tank capacity [L]
const int initialVidangeTime = 10;    // Time to the initial drain [s]
const int timeRegeneration = 15;       // Time to necessary to the water of the regeneration reservatory drains [s]
const int timeNaturalDry = 30;        // Time to let the dishes naturally drying [s]
const int temperatureInitial = 65;    // Initial temperature of the final rinse phase [ºC]
const int timeVidange = 20;

/******************************************************************************/
/***************************** Defining Variables *****************************/
/******************************************************************************/
// Time's variables
int timePhase;                                // Chosen time to execute the actual phase [min]
int timeDiverter;                             // Period of changing the state of the diverter valve [min]
int count1_Sec;
int count1_Min;
int count3_Sec;
int count3_Min;
int count4_Sec;
int count4_Min;
int timeToFill;
// State's variables
bool overfState;                            // Actual state of the floating sensor
bool doorState;                             // Actual state of the door sensor
bool cycleLavage;                           // Indicates if there is any cycle running at the moment
bool cyclePaused;                           // Indicates if the cycle was paused by any reason
bool stateWater;                            // Indicates if more time of clean rinse is necessary
bool cuveFilled;
bool cntrlDiverter1;                        // Actual state of the pins which controle the direction 
bool cntrlDiverter2;                        // of the water flux
bool flagTemperature;                       // If High use Temperature Phase in the final rinse
bool flagDoor;
bool flagTimer1;
int temperaturePhase;                       // Indicates the necessary temperature of the water to the actual phase
int lavagePhase;                            // Indicates the actual phase of the washing cycle : 0- Begin, 1- Back rinse, 2- Prewash,
                                            // 3- Mainwash, 4- Intermediate rinse, 5- Final rinse, 6- Regeneration, 7- Drying, 8- Finish
int stateDirection;                         // Actual direction of the water : 0- upper sprinkle, 1- middle arm, 2- lower arm
int countAux;                               // Indicates how many times the loop of the actual phase has been executed
int waterDirty;                             // Indicates if the water is clean or not
// Auxiliar variables when pause occurs
bool tempVidange;                  
bool tempRemplissage;
bool tempCyclage;
bool tempDiverter;
bool tempDirection;
bool tempSechage;
bool tempRegeneration;
bool tempChauffage;
bool tempDoseur;
double tempFlow;
int tempCount;
int tempDirty;
int tempTimer1;
int tempTimer3;
double tempTemperature;
// Mesures
double waterFlow;                        // Indicates the water inlet flow in m^3/s
double temperatureRead;               // Indicates the precise temperature read by the thermistor in ºC
// For test
boolean cancel;
boolean enter;

void capt_overflow(void);
void capt_door(void);
void error_treatment(int code);
void get_filling_back(void);
void get_draining_back(void);
int back_rinse(void);
void prewash(int);

/******************************************************************************/
/*                                  Setup                                     */
/******************************************************************************/
/*                                                                            */
/*  Special routine of Arduino which is run only once and before any other    */
/*  It's in charged of the external interrupt pins configuration, used for    */
/*  the security door and the anti overflow sensor.                           */
/*                                                                            */
/******************************************************************************/
void setup() {
  pinMode(overfPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(overfPin), capt_overflow, CHANGE);   
  pinMode(doorPin, INPUT);
  attachInterrupt(digitalPinToInterrupt(doorPin), capt_door, CHANGE);       
  Serial.begin(9600);

  // Configuração do timer1s
  TCCR1A = 0;                        //confira timer para operação normal pinos OC1A e OC1B desconectados
  TCCR1B = 0;                        //limpa registrador
  TCCR1B |= (1<<CS10)|(1 << CS12);   // configura prescaler para 1024: CS12 = 1 e CS10 = 1
  TCNT1 = 0xC2F7;

  TCCR3A = 0;                             // configurates timer 3 to normal operation
  TCCR3B = 0;                             // clean register
  TCCR3B |= (1 << CS30) | (1 << CS32);    // configurates prescaler 2 to 1024, allowing to obtain 1 second interruption
  TCNT3 = 0xC2F7;                         // initialize to count 1 second

//  TCCR4A = 0;                             // configurates timer 3 to normal operation
//  TCCR4B = 0;                             // clean register
//  TCCR4B |= (1 << CS40) | (1 << CS42);    // configurates prescaler 2 to 1024, allowing to obtain 1 second interruption
//  TCNT4 = 0xC2F7;                         // initialize to count 1 second
}

/******************************************************************************/
/*                                  Loop                                      */
/******************************************************************************/
/*                                                                            */
/*  Special routine of Arduino which corresponds to the main function of the  */
/*  program. Running in an infinite loop, this routine is responsible  for    */
/*  treating detected errors and specially for calling the phases functions   */
/*  in the appropriate sequence and configuration in order to execute a       */
/*  washing cycle.                                                            */
/*                                                                            */
/******************************************************************************/
void loop() {
  int errorCode = setjmp(env);
  error_treatment(errorCode);
  init_variables();
  Serial.println("The machine is ready to start");
                                        
  delay(500);                                                 // debouncing door
  s.niveau_sel();                                             // checks the salt level
  if(s.nvSelState == LOW){
    Serial.println("The salt level is insufficient, please add to continue");
    do{
      wait_sec(2);
      s.niveau_sel();
    }while(s.nvSelState == LOW);
  }
  Serial.println("The salt level is okay");
  s.niveau_rincage();                                         // checks the rinse aid level
  if(s.nvRinState == LOW){
    Serial.println("The rinse aid level is insufficient, please add to continue");
    do{
      wait_sec(2);
      s.niveau_rincage();
    }while(s.nvRinState == LOW);
  }
  Serial.println("The rinse aid level is okay");

  do{                                                         // loop to wait an enter of the user
    enter = digitalRead(36);
  }while(enter != HIGH);
  doorState = digitalRead(doorPin);
  if(doorState == LOW){
    Serial.println("Close the door please");
  }

  do{
    doorState = digitalRead(doorPin);
  }while(doorState != HIGH);  

    a.relayOn_on(); 

  cycleLavage = HIGH;                                       // indicates the begining of a washing cycle
//  timeToFill = back_rinse();                                // execute the back rinse and save the time necessary to fil the tank
  timeToFill = 16;
//  Serial.print("The time to fill the tank is: ");
//  Serial.println(timeToFill);
//  timePhase = 3;                                            // define that the prewash will last around 5 minutes
//  prewash(timePhase);                                       // execute the prewash
//  timePhase = 3;                                            // define that the mainwash will last around 10 minutes
//  temperaturePhase = 65;                                    // define that the mainwash will be done at 65ºC
//  mainwash(timePhase, temperaturePhase);                    // execute the mainwash
//  timePhase = 3;                                            // define that the intermediate rinse will last around 3 minutes
//  intermediate_rinse(timePhase);                            // execute the intermediate rinse
  timePhase = 3;                                            // define that the final rinse will last around 3 minutes
  temperaturePhase = 75;                                    // define that the final rinse will be done at 75ºC
  final_rinse(timePhase, temperaturePhase);                 // execute the final rinse
//  regeneration();                                           // execute the regeneration phase
//  timePhase = 1;                                           // define that the drying fan will be on for 10 minutes
//  drying(timePhase);                                        // execute the drying phase
  a.relayOn_off();                                          // turn off the relayOn
  cycleLavage = LOW;                                        // indicates the end of a washing cycle
}

/******************************************************************************/
/*                          Variable Initialization                           */
/******************************************************************************/
/*                                                                            */
/*  Routine dedicated to set the initial state of the program to assure its   */
/*  correct functioning.                                                      */
/*                                                                            */
/******************************************************************************/
void init_variables(void) {
  a.init_variables();                                       // initializate the variables related to the actuators
  overfState = LOW;
  doorState = LOW;
  cycleLavage = LOW;
  cyclePaused = LOW;
  stateWater = LOW;
  flagTemperature = LOW;
  waterFlow = 0;
  lavagePhase = 0;
  countAux = 0;
  temperatureRead = 0.0;
  waterDirty = 0;
  count1_Sec = 0;
  count1_Min = 0;
  count3_Sec = 0;
  count3_Min = 0;
  flagTimer1 = LOW;
  flagDoor = LOW;
}

/******************************************************************************/
/*                                Back Rinse                                  */
/******************************************************************************/
/*                                                                            */
/*  Phase routine responsible to control the dishwasher components in order   */
/*  to execute the back rinse. Present in all programs, its execution does    */
/*  not depend on any configuration parameter. Being the first phase, it also */ 
/*  reads the flowmeter and return the necessary time to fill the tank.       */
/*                                                                            */
/******************************************************************************/
int back_rinse(void){
  Serial.println("The back rinse started");
  int fillingTime;                              // variable to store the approximated time to fill the tank
  double timePrecision;                         // variable to store the exact time to fill the tank

  lavagePhase = 1;                              // indentify that the back rinse has started
  a.vidange_on();                               // activates the drain pump
  Serial.println("Vidange - LED 6 On");
  wait_sec(initialVidangeTime);                 // wait until the tank is sure to be empty
  a.remplissage_on();                           // turn the filling valve on
  Serial.println("Remplissage - LED 1 On");
  waterFlow = s.calculate_flow();               // reads the water flow
  Serial.print("Water flow of ");
  Serial.print(waterFlow);
  Serial.println(" L/s");
  timePrecision = (1)/waterFlow;                // calculate the exact time to fill 1 L of water
  fillingTime = (int)(timePrecision + 0.5);     // aproximates the time to fill 1L of water
  Serial.print("It will take ");
  Serial.print(fillingTime);
  Serial.println("s to fill 1 Liter");
  countAux = 0;                                 // guarantees the counter starts at zero
  do{                                           // loop to be executed 3 times
    if(countAux != 0){
      a.vidange_on();                   		    // turn on the drain pump
      Serial.println("Vidange - LED 6 On");
      a.remplissage_on();               		    // and the filling valve at the same time
      Serial.println("Remplissage - LED 1 On");
    }
    wait_sec(fillingTime);                    // wait until one liter of water circulates through the tank
    a.remplissage_off();                        // turne the filling valve off
    Serial.println("Remplissage - LED 6 Off");
    a.vidange_off();                            // turn the drain pump off
    Serial.println("Vidange - LED 1 Off");
    wait_sec(3);                              // wait 3 seconds 
    countAux = countAux + 1;                    // update how many times the loop has already been done 
  }while(countAux < 3);
  
  a.vidange_on();                               // it empties the tank
  Serial.println("Vidange - LED 6 On");
  wait_sec(fillingTime);
  a.vidange_off();
  Serial.println("VIdange - LED 6 Off");
  Serial.println("The back rinse has finished");
  
  timePrecision = (tankCapacity)/waterFlow;                   // calculate time to completly fill the tank [s]
  fillingTime = (int)(timePrecision + 0.2);                   // aproximates the time to fill
  return fillingTime;
}

/******************************************************************************/
/*                                  Prewash                                   */
/******************************************************************************/
/*                                                                            */
/*  Phase routine responsible to control the dishwasher components in order   */
/*  to execute the prewash. Its duration is determined by the parameter       */
/*  timePrewash.                                                              */
/*                                                                            */
/******************************************************************************/
void prewash(int timePrewash){
  Serial.println("The prewash started");
  lavagePhase = 2;                                  // indentify that the prewash has started
  fill_tank(timeToFill);                          // fill the tank with water
  timeDiverter = 1;                                 // set the time Diverter to 2 min
  stateDirection = 0;                               // set in which way the Diverter must start
  Serial.println("Direction set to 0");
  a.diverter_on();                          		    // turn the diverter motor on
  Serial.println("Diverter - LED 3 On");
  Serial.println("Waiting Prewash");
  wait_min(timePrewash);                          // wait the time to complete the prewashing
  drain_tank();                                   // drain out the tank
  Serial.println("The prewash ended");
  return;  
}

/******************************************************************************/
/*                                Mainwash                                    */
/******************************************************************************/
/*                                                                            */
/*  Phase routine responsible to control the dishwasher components in order   */
/*  to execute the mainwash. Its duration is determined by the parameter      */
/*  timeMainwash while its temperature by temperatureWash                     */
/*                                                                            */
/******************************************************************************/
void mainwash(int timeMainwash, int temperatureWash){
  int halfTime;
  int onlyOnce = 0;
  Serial.println("The mainwash started");
  lavagePhase = 3;                                        // indentify that the prewash has started
  temperatureRead = 0.0;                                  // reset the temperature reading
  fill_tank(timeToFill);                                          // fill the tank with water
  timeDiverter = 1;                                       // set the time Diverter to 3 min
  stateDirection = 0;                                     // set in which way the Diverter must start
  Serial.println("Direction set to 0");
  a.diverter_on();                                		    // turn the diverter motor on
  Serial.println("Diverter - LED 3 On");
  a.doseur_on();                                 		      // release the detergent in the tank
  Serial.println("Doseur - LED 5 On");
  wait_sec(10);
  Serial.println("Detergent released");
  a.doseur_off();
  Serial.println("Doseur - LED 5 Off");

  Serial.println("Waiting mainwash time");    
  for(countAux = 0; countAux < 2; countAux++){            // loop to be executed 2 times trying to ensure the high temperature 
    a.chauffage_on();                              		    // turn the heating on
    Serial.println("Chauffage - LED 4 On");
    Serial.println("Waiting until reach the temperature");
    do{                                                   // waits until the temperature reaches the desired value
      temperatureRead = s.mesure_temperature();
      Serial.print(temperatureRead);
      Serial.println("ºC");     
      if(onlyOnce != 0){
        wait_sec(2);
        onlyOnce++;
      }
    }while(temperatureRead < temperatureWash);
    Serial.print("Water is at: ");
    Serial.print(temperatureRead);
    Serial.println("ºC");
    a.chauffage_off();                                    // turn the heating off
    Serial.println("Chauffage - LED 4 Off");
    halfTime = (int)(timeMainwash/2.0 + 0.5);
    wait_min(halfTime);                                   // wait the time to complete the mainwash
  }
  Serial.println("The mainwash has ended");
  drain_tank();                                         // drain out the tank 
  return;  
}

/******************************************************************************/
/*                           Intermediate Rinse                               */
/******************************************************************************/
/*                                                                            */
/*  Phase routine responsible to control the dishwasher components in order   */
/*  to execute the intermediate rinse. Its duration is determined by the      */
/*  parameter timeIntRinse. In this phase, the turbidity sensor is checked    */
/*  in order to analyse the need of increasing the washing cycle time by      */
/*  increasing the time of the Intermediate Rinse by 3 minutes, and if still  */
/*  needed, increasing the duration of the final rinse in 2 minutes.          */
/*                                                                            */
/******************************************************************************/
void intermediate_rinse(int timeIntRinse){
  Serial.println("The intermediate rinse has started");
  lavagePhase = 4;                                  // indentify that the intermediate rinse has started
  fill_tank(timeToFill);                            // fill the tank with water
  timeDiverter = 1;                                 // set the time Diverter to 1 min
  stateDirection = 0;                               // set in which way the Diverter must start
  temperatureRead = 0.0;                                  // reset the temperature reading
  Serial.println("Diverter direction set to 0");
  a.diverter_on();                          		    // turn the diverter motor on
  Serial.println("Diverter - LED 3 On");
  Serial.println("Waiting time to complete the intermediate rinse");
  wait_min(timeIntRinse);                           // wait the time to complete the intermediate rinse
  waterDirty = s.niveau_turbidite();                // verifies the turbidity level of the water
  if(waterDirty == 1){                              // if needed, increase the intermediate rinse time by 3 minutes
    Serial.println("The water is dirty, the intermediate rinse will continue");
    wait_min(3);
    waterDirty = s.niveau_turbidite();              // recheck the turbidity level of the water
    if(waterDirty == 1){                            // if needed, indicates that the water is still dirty
      Serial.println("The water is still dirty, the final rinse will be longer");
      stateWater = HIGH;
    }
    else{
      Serial.println("The water is clean now");
    }
  }
  else{
    Serial.println("The water is clean");
  }
  Serial.println("The intermediate rinse has finished");
  drain_tank();                                   // drain out the tank
  return;  
}

/******************************************************************************/
/*                               Final Rinse                                  */
/******************************************************************************/
/*                                                                            */
/*  Phase routine responsible to control the dishwasher components in order   */
/*  to execute the final rinse. Its duration is determined by the parameter   */
/*  timeFianlRinse while its temperature by temperatureMax.                   */
/*                                                                            */
/******************************************************************************/
void final_rinse(int timeFinalRinse, int temperatureMax){
  Serial.println("The final rinse has started");
  lavagePhase = 5;                                        // indentify that the final rinse has started
  int onlyOnce = 0;
  fill_tank(timeToFill);                                  // fill the tank with water
  timeDiverter = 1;                                       // set the time Diverter to 1 min
  stateDirection = 0;                                     // set in which way the Diverter must start
  Serial.println("Diverter direction set to 0");
  a.diverter_on();                                		    // turn the diverter motor on
  Serial.println("Diverter - LED 3 On");
  a.chauffage_on();                                		    // turn the heating on
  Serial.println("Chauffage - LED 4 On");
  Serial.println("Waiting until reach the initial temperature");
  do{                                                     // waits until the temperature reaches the initial temperature
    temperatureRead = s.mesure_temperature();
    Serial.print(temperatureRead);
    Serial.println("ºC");              
    if(onlyOnce != 0){
        wait_sec(2);
        onlyOnce++;
    }
  }while(temperatureRead < temperatureInitial);
  Serial.print("Water is at: ");
  Serial.print(temperatureRead);
  Serial.println("ºC");
  a.doseur_on();                                           // release the detergent in the tank
  Serial.println("Doseur - LED 5 On");
  wait_sec(a.timeDoseur);
  Serial.println("Rinse aid released");
  a.doseur_off();
  Serial.println("Doseur - LED 5 Off");
  wait_sec(30);                                           // wait 30s
  Serial.println("Waiting until reach the final temperature");
  flagTemperature = HIGH;
  do{                                                     // waits until the temperature reaches the maximum temperature of the phase
    temperatureRead = s.mesure_temperature();
    Serial.print(temperatureRead);
    Serial.println("ºC");             
    wait_sec(2);
  }while(temperatureRead < temperatureMax);
  Serial.print("Water is at: ");
  Serial.print(temperatureRead);
  Serial.println("ºC");
  a.chauffage_off();                                      // turn the heating off
  Serial.println("Chauffage - LED 4 Off");
  Serial.println("Waiting time to complete the final rinse");
  wait_min(timeFinalRinse);                             // wait the time to complete the Final Rinse
  if(stateWater == HIGH){
    Serial.println("As the water was too dirty, the final rinse will last longer");
    wait_sec(2);
  }
  Serial.println("The final rinse has finished");
  drain_tank();                                         // drains out the tank
  flagTemperature = LOW;
  return;  
}

/******************************************************************************/
/*                               Regeneration                                 */
/******************************************************************************/
/*                                                                            */
/*  Phase routine responsible to control the dishwasher components in order   */
/*  to execute the regeneration. Present in all programs, its execution does  */
/*  not depend on any configuration parameter, since its duration is allways  */
/*  the same.                                                                 */
/*                                                                            */
/******************************************************************************/
void regeneration(void){
  Serial.println("The regeneration has started");
  lavagePhase = 6;                                          // indentify that the regeneration has started
  a.regeneration_on();                               		    //allow the water in the regeneration reservatory to flow
  Serial.println("Regeneration - LED 7 On");
  Serial.println("Waiting time for regeneration");
  wait_sec(timeRegeneration);                             // wait the necessary time
  a.remplissage_on();                               		// turn the inlet valve on to circulate water
  Serial.println("Remplissage - LED 1 On");
  wait_sec(3);
  a.remplissage_off();
  Serial.println("Remplissage - LED 1 Off");
  wait_sec(15);
  a.remplissage_on();
  Serial.println("Remplissage - LED 1 On");
  wait_sec(3);
  a.regeneration_off();                                     // finishes the regeneration process
  Serial.println("Regeneration - LED 7 Off");
  a.remplissage_off();
  Serial.println("Remplissage - LED 1 Off");
  a.vidange_on();
  Serial.println("Vidange - LED 6 On");
  wait_sec(15);
  Serial.println("Vidange - LED 6 On");
  a.vidange_off();

  Serial.println("The regeneration has finished");
  return;
}

/******************************************************************************/
/*                                  Drying                                    */
/******************************************************************************/
/*                                                                            */
/*  Phase routine responsible to control the dishwasher components in order   */
/*  to execute the dish drying.                                               */
/*                                                                            */
/******************************************************************************/
void drying(int timeSechage){
  Serial.println("The drying has started");
  lavagePhase = 7;                                          // indentify that the drying has started
  Serial.println("Waiting for natural dry");
  wait_sec(timeNaturalDry);                               // with all components off, wait for the dishes start naturally drying
  a.sechage_on();
  Serial.println("Fan Drying - LED 8 On");
  Serial.println("Waiting for drying the dishes");
  wait_min(timeSechage);                                  // wait for the dishes to dry
  a.sechage_off();                                          // the drying processes is ended
  Serial.println("Fan Drying - LED 8 Off");
  a.vidange_on();
  Serial.println("Vidange - LED 6 On");
  wait_sec(15);
  Serial.println("Vidange - LED 6 Off");
  a.vidange_off();

  Serial.println("The drying has finished");
  return;
}

/******************************************************************************/
/*                              Pause Cycle                                   */
/******************************************************************************/
/*                                                                            */
/*  Routine activated when the door is opened in the middle of a washing      */
/*  cycle. It saves the actual state of the machine to make it possible to    */
/*  return to the cycle in the point where it was paused.                     */
/*                                                                            */
/******************************************************************************/
void pause_cycle() {
  tempTimer1 = TCNT1;                               // Save current counting time of Timer 1
  tempTimer3 = TCNT3;                               // Save current counting time of Timer 3
  TIMSK1 &= (0 << TOIE1);                           // Stop the timer1 counting
  TIMSK3 &= (0 << TOIE3);                           // Stop the timer3 counting
  cycleLavage = LOW;                                // Update the washing cycle state
  cyclePaused = HIGH;                           
  tempChauffage = a.stateChauffage;                 // Save the actual state of the heater
  a.chauffage_off();                                // Make sure the heater if off 
  a.relayOn_off();                                  // Turn off all the components
  int tempCount1Sec = count1_Sec;
  int tempCount1Min = count1_Min;
  int tempCount3Sec = count3_Sec;
  int tempCount3Min = count3_Min;
  tempVidange = a.stateVidange;                     // Save the actual state of the washing cycle
  tempRemplissage = a.stateRemplissage;
  tempCyclage = a.stateCyclage;
  tempDiverter = a.stateDiverter;
  tempDirection = stateDirection;
  tempRegeneration = a.stateRegeneration;
  tempSechage = a.stateSechage;
  tempDoseur = a.stateDoseur;
  tempFlow = waterFlow;                             // Save the last reading of the flowmeter
  tempDirty = waterDirty;                           // Save the last reading of the turbidity sensor
  turn_off();                             

  // Part dedicated to the communication with the user
  Serial.println("The actual washing cycle was stopped");
  Serial.println("If you really want to cancel it, press CANCEL. If you want to continue it, press ENTER.");
  delay(500);
  int onlyOnce = 0;
  do{
    cancel = digitalRead(34);
    enter = digitalRead(36);
    if (cancel == HIGH) {
      if(onlyOnce == 0){
        Serial.println("CANCEL pressed -> Close the door and the washing cycle will be finished");
        onlyOnce++;
      }
      lavagePhase = 8;
    }
    else if (enter == HIGH) {
      if(onlyOnce == 0){
        Serial.println("ENTER pressed -> The dishwashing will continue after closing the door.");  ;
        onlyOnce++;
      }                                
    }
    doorState = digitalRead(doorPin);
  }while(doorState == LOW);

  count1_Sec = tempCount1Sec;
  count1_Min = tempCount1Min;
  count3_Sec = tempCount3Sec;
  count3_Min = tempCount3Min;
  TCNT1 = tempTimer1;                               // Save current counting time of Timer 1
  TCNT3 = tempTimer3;                               // Save current counting time of Timer 3
  return;
}

/******************************************************************************/
/*                             Continue Cycle                                 */
/******************************************************************************/
/*                                                                            */
/*  Routine dedicated to continue the execution of a washing cycle from the   */
/*  point where it was stopped. To do so, the first step is to read the       */
/*  variables saved by the rotine pause_cycle. Then, identify in each step of */
/*  washing cycle it was paused, to finally, correctly control the necessary  */
/*  components and get back to the cycle.                                     */
/*                                                                            */
/******************************************************************************/
void continue_cycle() {
  cyclePaused = LOW;
  if(lavagePhase == 8){
    cyclePaused = LOW;  // Finish the cycle by user's choice
    longjmp(env, 50);
  }
  a.relayOn_on();                                               // Turn on the relayOn allowing the command signals to be in charged os the components
  delay(500);

  if(tempRemplissage == HIGH){
    a.remplissage_on();
    Serial.println("The filling was turned on again");
  }
  if(tempCyclage == HIGH){
    a.cyclage_on();
    Serial.println("The cyclage pump was turned on again");
  }
  if(tempDiverter == HIGH){
    stateDirection = tempDirection;
    a.diverter_on();
    Serial.println("The diverter motor was turned on again");
  }
  if(tempChauffage == HIGH){
    a.chauffage_on();
    Serial.println("The heating was turned on again");
  }

  switch (lavagePhase) {
    case 3:                                                           // Mainwash
      if(temperatureRead != 0.0 and tempChauffage == LOW){                                     // Checks if the water temperature had to be in a certain temperature
        Serial.println("It was paused in the mainwash, the temperature will be continually checked");
        do{
          temperatureRead = s.mesure_temperature();                       // It's necessary to check if after the pause the water temperature is stil the same
          Serial.print(temperatureRead);
          Serial.println("ºC");
          delay(2000);
        }while(flagDoor == HIGH);
        
        if(temperatureRead < temperaturePhase){
            Serial.println("The heating was turned on again to reach the needed temperature");
            a.chauffage_on();
            do{                                                           // waits until the temperature reaches the desired value
              temperatureRead = s.mesure_temperature();
              Serial.print(temperatureRead);
              Serial.println("ºC");
              delay(2000);          
            }while(temperatureRead < temperaturePhase);
            Serial.println("The temperature was reached again, heating was turned off");
            a.chauffage_off(); 
        }
      }
      break;
    case 5:                                                           // Final rinse
      if(flagTemperature == HIGH){
          tempTemperature = temperaturePhase;
          Serial.println("HIGH");
        }
      else{
          tempTemperature = temperatureInitial;
          Serial.println("LOW");
      }
      if((temperatureRead != 0.0 and tempChauffage == LOW) or (temperatureRead != 0.0 and tempDoseur == HIGH)){                                     // Checks if the water temperature had to be in a certain temperature
          Serial.println("It was paused in the final rinse, the temperature will be continually checked");
          do{
            temperatureRead = s.mesure_temperature();                       // It's necessary to check if after the pause the water temperature is stil the same
            Serial.print(temperatureRead);
            Serial.println("ºC");
            delay(2000);
          }while(flagDoor == HIGH);
          
          if(temperatureRead < tempTemperature){
              Serial.println("The heating was turned on again to reach the needed temperature");
              a.chauffage_on();
              do{                                                           // waits until the temperature reaches the desired value
                temperatureRead = s.mesure_temperature();
                Serial.print(temperatureRead);
                Serial.println("ºC");
                delay(2000);          
              }while(temperatureRead < tempTemperature);
              Serial.println("The temperature was reached again, heating was turned off");
              a.chauffage_off(); 
          }
      }
      break;
  }
  
  if(tempVidange == HIGH){
    a.vidange_on();
    Serial.println("The drain pump was turned on again");
  }
  if(tempRegeneration == HIGH){
    a.regeneration_on();
    Serial.println("The regeneration valve was turned on again");
  }
  if(tempSechage == HIGH){
    a.sechage_on();
    Serial.println("The drying fan was turned on again");
  }
  if(tempDoseur == HIGH){
    count1_Sec = 0;
    a.doseur_on();
    Serial.println("The dispenser was turned on again");
  }
  if(flagTimer1 == HIGH){
    if(count1_Min == 0){
      Serial.print("The the timer 1 was reiniticialized in ");
      Serial.print(count1_Sec);
      Serial.println(" seconds");
    }
    else{
      Serial.print("The the timer 1 was reiniticialized in ");
      Serial.print(count1_Min);
      Serial.print(" minutes and ");
      Serial.print(count1_Sec);
      Serial.println(" seconds");
    }
    TIMSK1 |= (1 << TOIE1);
  }
  if(a.flagTimer3 == HIGH){
    if(count3_Min == 0){
      Serial.print("The the timer 3 was reiniticialized in ");
      Serial.print(count3_Sec);
      Serial.println(" seconds");
    }
    else{
      Serial.print("The the timer 3 was reiniticialized in ");
      Serial.print(count3_Min);
      Serial.print(" minutes and ");
      Serial.print(count3_Sec);
      Serial.println(" seconds");
    }
    TIMSK3 |= (1 << TOIE3);
  }
  cycleLavage = HIGH;
  Serial.println("");
  Serial.println("---------------------------------------------------------------------");
  Serial.println("");
}

/******************************************************************************/
/*                              Error Treatment                               */
/******************************************************************************/
/*                                                                            */
/*  Routine dedicated to inform the user that an error has occured ant then   */
/*  take the necessaries mesures to deal with it.                             */
/*                                                                            */
/******************************************************************************/
void error_treatment(int code){
   switch(code){
    case 1:
      Serial.println("For some unknown software reason the relayOn was off when it was supposed to be on");
      Serial.println("The machine had to be reinitialized");
      turn_off();
      break;
    case 2:
      Serial.println("For some unknown software reason the tank was not filled as it was supposed");
      Serial.println("The machine had to be reinitialized");
      turn_off();
      break;
    case 3:
      Serial.println("For some unknown software reason the cycle pump was off when it was supposed to be on");
      Serial.println("The machine had to be reinitialized");
      turn_off();
      break;
    case 4:
      Serial.println("For some unknown software reason, a component was tried to be activated in the wrong time");
      Serial.println("The machine had to be reinitialized");
      turn_off();
      break;
    case 5:
      Serial.println("For some reason an overflow was detected, please read the manual instructions to know how to proceed");
      turn_off();
      break;
    case 50:
      turn_off();
      Serial.println("As you wanted, the cycle was stopped");
      if(cuveFilled == HIGH){
          a.relayOn_on();
          delay(200);                                   
          drain_tank();
          a.relayOn_off();
      }
      else{
        a.relayOn_on();
        delay(200);
        a.vidange_on();
        Serial.println("Vidange - LED 1 On");
        wait_sec(15);
        a.vidange_off();
        Serial.println("Vidange - LED 1 Off");
        a.relayOn_off();
      }
      break;
  }
}

/******************************************************************************/
/*                             Capteur Overflow                               */
/******************************************************************************/
/*                                                                            */
/*  Interrupt routine associated with the overflow sensor pin. Checks the     */
/*  actual state of the sensor, and if it's high, pauses the washing cycle.   */
/*                                                                            */
/******************************************************************************/
void capt_overflow(void) {
  overfState = digitalRead(overfPin);
  if (overfState == HIGH) {
    if (cycleLavage == HIGH) {
      Serial.println("An overflow was detected and for safety, the washing cycle will be paused.");
      pause_cycle();
    } 
  }
}

/******************************************************************************/
/*                               Capteur Door                                 */
/******************************************************************************/
/*                                                                            */
/*  Interrupt routine associated with the security dorr sensor pin. Checks    */
/*  the actual state of the sensor, and accordingly to it, pauses or          */
/*  continues the washing cycle.                                              */
/*                                                                            */
/******************************************************************************/
void capt_door(void) {
  doorState = digitalRead(doorPin);
  if (doorState == LOW and cycleLavage == HIGH) {
    flagDoor = HIGH;
    Serial.println("");
    Serial.println("---------------------------------------------------------------------");
    Serial.println("");
    Serial.println("You've opened the door in the middle of a cycle");
    pause_cycle();
  }
  if (doorState == HIGH and cyclePaused == HIGH) {
    Serial.println("The door was closed");
    flagDoor = LOW;
    continue_cycle();
  }
  if(doorState == HIGH){
    flagDoor = LOW;
  }
}

void wait_sec(int seconds){
    flagTimer1 = HIGH;
//    Serial.println("Alterei a flag HIGH");
    int count = count1_Sec;
    TCNT1 = 0xC2F7;
    TIMSK1 |= (1 << TOIE1);               // activates the interruption in the timer 1
    do{
      count = count1_Sec;
      delay(50);
    }while(count < seconds and flagDoor == LOW);
    //Serial.println("Estou Aqui");
    count1_Sec = 0;                       // reset the second counting
    TIMSK1 &= (0 << TOIE1);               // desactivate the interruption
    flagTimer1 = LOW;
//    Serial.println("Alterei a flag LOW");
    return;
}

// --------------------- Wait Minutes ---------------------
// Description : rotine responsible to induce a delay in
// seconds using the timer chosen.
void wait_min(int minutes){
    flagTimer1 = HIGH;
    TIMSK1 |= (1 << TOIE1);               // activates the interruption in the timer 1
    do{
        delay(50);
        if(count1_Sec == 10){             // each time the second counting reaches 60
          count1_Sec = 0;                 // reset the second counting
          count1_Min = count1_Min + 1;    // the minute counter is incremented
        }
    }while(count1_Min < minutes);         // until it reaches the desired counting
    count1_Min = 0;                       // reset the minute counting
    TIMSK1 &= (0 << TOIE1);               // desactivate the interruption
    flagTimer1 = LOW;
    return;
}

/******************************************************************************/
/*                               ISR Timer 1                                  */
/******************************************************************************/
/*                                                                            */
/*  Interrupt routine associated with the overflow of the timer 1. It is used */
/*  to the function waiting seconds and minutes, with the function to add one */
/*  more second to the counter.                                               */
/*                                                                            */
/******************************************************************************/
ISR(TIMER1_OVF_vect)
{
  count1_Sec = count1_Sec + 1;            // increments one second in the counting          
  TCNT1 = 0xC2F7;                         // reset timer
}

/******************************************************************************/
/*                               ISR Timer 3                                  */
/******************************************************************************/
/*                                                                            */
/*  Interrupt routine associated with the overflow of the timer 3. It is used */
/*  to periodically change the direction of the diverter. Each time the       */
/*  ounter reaches the specified timeDiverter, it changes the direction of    */
/*  water flow.                                                               */
/*                                                                            */
/******************************************************************************/
ISR(TIMER3_OVF_vect)
{
  count3_Sec = count3_Sec + 1;                              // increments one second in the counting
  TCNT3 = 0xC2F7;                                               // reset timer
  if(count3_Sec == 10)                                        // one minute has passed
  {
    count3_Sec = 0;
    count3_Min = count3_Min + 1;                            // increments one second in the counting
    if(count3_Min == timeDiverter){                           // verifies if the amout of time set was achieved
       count3_Min = 0;
       stateDirection = (stateDirection + 1) % 3;               // changes the diretion in order
       switch(stateDirection){                                  // send the right command to control the diverter motor
        case 0:
          Serial.println("Direction set to 0");
          cntrlDiverter1 = LOW;
          cntrlDiverter2 = LOW;
          digitalWrite(a.diverter1Pin, cntrlDiverter1);
          digitalWrite(a.diverter2Pin, cntrlDiverter2);
          break;
        case 1:
          Serial.println("Direction set to 1");
          cntrlDiverter1 = LOW;
          cntrlDiverter2 = HIGH;
          digitalWrite(a.diverter1Pin, cntrlDiverter1);
          digitalWrite(a.diverter2Pin, cntrlDiverter2);
          break;
        case 2:
          Serial.println("Direction set to 2");
          cntrlDiverter1 = HIGH;
          cntrlDiverter2 = LOW;
          digitalWrite(a.diverter1Pin, cntrlDiverter1);
          digitalWrite(a.diverter2Pin, cntrlDiverter2);
       }
    }
  }                                 
}

// --------------------- Fill Tank ---------------------
// Description : rotine to execute all the necessary steps
// to completely fill the tank.
void fill_tank(int timeFilling) {
  Serial.println("Filling the tank");
  a.remplissage_on();                                     // turn the filling valve on
  Serial.println("Remplissage - LED 1 On");
  wait_sec(5);                                        // wait 5 seconds
  a.cyclage_on();
  Serial.println("Cyclage - LED 2 On");                            // initiates to circulate the water
  wait_sec(timeFilling-5);                             // wait until the tank is full of water
  a.remplissage_off();
  Serial.println("Remplissage - LED 1 Off");                           // turn the filling valve off
  cuveFilled = HIGH;
  Serial.println("Tank Filled");
}

// --------------------- Drain Tank ---------------------
// Description : rotine to execute all the necessary steps
// to completely drain out the tank.
void drain_tank(void) {
  Serial.println("Draining the tank");
  a.diverter_off();                                       // turn the diverter motor off
  Serial.println("Diverter - LED 3 Off");
  a.cyclage_off();                                        // stop to circulate the water
  Serial.println("Cyclage - LED 2 Off");
  a.vidange_on();                                         // it empties the tank
  Serial.println("Vidange - LED 6 On");
  wait_sec(timeVidange);                      
  a.vidange_off();
  Serial.println("Vidange - LED 6 Off");
  cuveFilled = LOW;
  Serial.println("Tank Empty");
}

// --------------------- Turn off ----------------------
// Description : rotine dedicated to end the current cycle 
// for some unexpected reason.
void turn_off(){
  Serial.println("All components were turned off");
  a.chauffage_off();                                // Make sure the heater if off 
  a.relayOn_off();                                  // Turn off all the components
  delay(200);
  a.remplissage_off();                              // Make sure the control signals are off
  a.vidange_off();                                  
  a.cyclage_off();                                  
  a.diverter_off();
  a.regeneration_off();
  a.sechage_off();    
  a.doseur_off();
  return;
}
