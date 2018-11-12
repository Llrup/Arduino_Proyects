// ############################################################################
// ============================================================================
// ===                          SIMPLE BLINK EXAMPLE                         ===
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
// - LED 5mm
// - 100 Ohm Resistance
// - Wires
// - Protoboard
// ============================================================================

// ============================================================================
// PROGRAM DESCRIPTION: 
// - From this code you can obtain a LED blinking with luminosity change and no more. 
// 
// - To resume the logic the author is going to use in this software it can be said
//   that the structure the lector are going to find it's:
//      - Preliminaries and Description of the Software.
//      - Program Code.
//          - Libraries included.
//          - Function and Variables, General Definitions.
//          - Setup Configuration Block.
//              - Initial Control Process LED Activation.
//          - Continuous Loop Block.
//              - LED blink routine.
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


// FUNCTION & VARS DEFINITIONS:_______________________________________________
// Here we're to define some functions and vars we could use later in the main 
// program for a better use of code syntax. 

int led = 7; // Pin where the led is connected.
int brightness = 0; // Initial value for the led brightness
int fadeAmount = 1; // Temporary pass of the brightness, #Coment 1 (Go after PROGRAM END)


// ############################################################################
//                         SETUP CONFIGURATION BLOCK
// ============================================================================
// Put here the setup configuration code to all the program to run once.

void setup() {
  // Setting up that the pin atached to the LED is going to be a OUTPUT.
  pinMode(led, OUTPUT);
}// End of setup.
// ============================================================================



// ############################################################################
//                            CONTINUOUS LOOP BLOCK
// ============================================================================
// Put here the main code you want to run repeatedly one time and another for years.
void loop() {
  
  // Setting the brightness of the led situated at pin declared before. 
  analogWrite(led, brightness);

  // Changing the brightness for next time throught the loop:
  brightness = brightness + fadeAmount;

  // Choosing if the LED go to HIGH brightness or to LOW.
  if (brightness == 0 || brightness == 255){
    fadeAmount = -fadeAmount;
    } // END of If 
    
    // Waiting 30 milliseconds into changes, going HIGH or going LOW.
    delay(30);

}// End of Cotinuous Loop.
// ============================================================================
// PROGRAM END.



// ############################################################################
//                                    COMENTS
// ============================================================================
//
//  # 1 fadeAmount: it's the cuantity your led its going to increase his brightness 
//                  after each loop, if you set a low cuantity the transition are going
//                  to be smoother than if you set a high value. The maximum value to 
//                  this variable is 255, take into account that your arduino can offers
//                  2^8 = 256 values -> 8 bits, in his PWR Digital output.
//
// ============================================================================
