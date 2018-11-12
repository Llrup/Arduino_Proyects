// ############################################################################
// ============================================================================
// ===                    SERVO COMANDED BY POTENCIOMETER                   ===
// ============================================================================
// ############################################################################

// ============================================================================
// AUTHOR: Manuel Ruiz Pérez
// Alumn of Master in Aeronautical Engineering, ETSIAE, UPM.
// DATE: 11 November 2018
// Madrid, Spain.
// ============================================================================

// ============================================================================
// REFERENCES: 
// 1. Example Arduino libraries. 
// 2. Material de Prácticas de la Asignatura Sistemas Aéreos no Tripulados, 
// Master en Ingeniería Aeronáutica; Escuela Técnica Superior de Ingeniería 
// Aeronáutica y del Espacio, Universidad Politécnica de Madrid.
// ============================================================================

// ============================================================================
// USED MATERIAL:
// - Elego Mega2560.R3 (Arduino Compatible). Patch driver needed (CH341SER_MAC).
// - Servo Power HD-1160A 
// - Potenciometer
// - Wires
// - Protoboard
// ============================================================================

// ============================================================================
// PROGRAM DESCRIPTION: 
// - About this code, you're going to obtain a servo comanded by a potenciometer. 
// 
// - To resume the logic the author is going to use in this software it can be said
//   that the structure the lector are going to find it's:
//      - Preliminaries and Description of the Software.
//      - Program Code.
//          - Libraries included.
//          - Function and Variables, General Definitions.
//          - Setup Configuration Block.
//              - Initial Pin activation where servo is attached.
//          - Continuous Loop Block.
//              - Servo control routine, comanded by potenciometer.
//      - Program Ending.
// ============================================================================


// ############################################################################
// ############################################################################
//                             PROGRAM CODE STARTING
// ============================================================================
// From here we're going to develop the code to run for achieve the control of
// the board. It's importan to see that we're programming in C, and it's 
// mandatory to respect the logic behind this type of coding language. For this
// at first we need to determine what libraries we're going to use:

// LIBRARIES:__________________________________________________________________

//Servo control library.
#include <Servo.h> 

// FUNCTION & VARS DEFINITIONS:_______________________________________________
// Here we're to define some functions and vars we could use later in the main 
// program for a better use of code syntax. 

// Defining the servo variables:
Servo servo; // Creating a servo object to be controlled.
int pos = 0; // Var to store the servo position.

// Defining the potenciometer variables:
const int analogPin = A0;
int value; // Var to store the potenciometer raw analog reading. 
int position; // Potenciometer position in %.


// ############################################################################
//                         SETUP CONFIGURATION BLOCK
// ============================================================================
// Put here the setup configuration code to all the program to run once.
void setup() {

  // Servo initial setup:
  servo.attach(9); // To pin 9.

}// End of setup.
// ============================================================================




// ############################################################################
//                            CONTINUOUS LOOP BLOCK
// ============================================================================
// Put here the main code you want to run repeatedly one time and another for years.
void loop() {

  value = analogRead(analogPin); // Reading the potenciometer analog input.
  position = map(value, 0, 1023, 0, 100); // Transform to % the analog reading.

  // Servo control routine:
  servo.write(position);
  delay(150); // Waiting 15 msfor the servo to reach the position.

}// End of Cotinuous Loop.
// ============================================================================
// PROGRAM END.
