// ############################################################################
// ============================================================================
// ===              SERIAL COMUNICATION IMU & IMU YPR OUTPUTS               ===
// ============================================================================
// ############################################################################


// ============================================================================
// AUTHOR: Manuel Ruiz Pérez
// Alumn of Master in Aeronautical Engineering, ETSIAE, UPM.
// DATE: 8 November 2018
// Madrid, Spain.
// ============================================================================


// ============================================================================
// REFERENCES: 
// 1. Jeff Rowberg IMU example of use, (MPU6050_DMP6)
// 2. Material de Prácticas de la Asignatura Sistemas Aéreos no Tripulados, 
// Master en Ingeniería Aeronáutica; Escuela Técnica Superior de Ingeniería 
// Aeronáutica y del Espacio, Universidad Politécnica de Madrid.
// ============================================================================


// ============================================================================
// USED MATERIAL:
// - Elego Mega2560.R3 (Arduino Compatible). Patch driver needed (CH341SER_MAC).
// - MPU6050. Driver and libraries needed, support on:
//         https://www.prometec.net/usando-el-mpu6050/
// - Wires
// - Protoboard
// ============================================================================

// ============================================================================
// PROGRAM DESCRIPTION: 
// - From this code you can obtain the Yaw, Pitch, and Roll angles in degrees from 
//   the IMU unit used. Really this is not the best result you can obtain from this
//   hardware, but it's a good point to start learning how it works.
// 
// - To resume the logic the author is going to use in this software it can be said
//   that the structure the lector are going to find it's:
//      - Preliminaries and Description of the Software.
//      - Program Code.
//          - Libraries included.
//          - Function and Variables General Definitions.
//          - Code Control Routine Definition.
//          - Setup Configuration Block.
//              - I2C Iclusion decision.
//              - Serial comm protocol call.
//              - Gyro Offset Initial Conditions.
//              - Initial Code Control Routine.
//              - Initial Control Process LED Activation.
//          - Continuous Loop Block.
//              - Active Code Control Protocol.
//              - IMU working and serial printing outputs.
//      - Program Ending.
// ============================================================================


// ============================================================================
// NOTES, READ ME: 
// To see the output the IMU offers you need to open the Monitor Serie placed in
// Arduino Sofware right upper corner, the little magnifying glass. When you have 
// it opened go to the bottom and select the 115400 baudios port. Tipe a character
// and press ENTER, ENJOY!.
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

// Communication BUS control. To use de I2C communication protocol:
#include "I2Cdev.h" 
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE // Doing a IF to include or not 
  #include "Wire.h"                               // this library. The program put
#endif                                           // them only if we use.
//------------------------------------------


// IMU Control:
#include "MPU6050_6Axis_MotionApps20.h"
#include "MPU6050.h" //Not necessary if using MotionApps include file. But it's
                     //good to know we have this to use if we need.
MPU6050 mpu; // Class default address for IMU MPU6050 (0x68), declared here.
//------------------------------------------


// Serial Communications:
#include <SoftwareSerial.h>
//------------------------------------------
//____________________________________________________________________________



// FUNCTION & VARS DEFINITIONS:_______________________________________________
// Here we're to define some functions and vars we could use later in the main 
// program for a better use of code syntax. 

#define OUTPUT_READABLE_YAWPITCHROLL
// For see the yaw/pitch/roll angles (in degrees) calculated from the 
// quaternions coming from the FIFO. Note this also requires gravity vector 
// calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
//------------------------------------------

// Definition of MPU control/Status vars:
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
//------------------------------------------

// Definition of MPU orientation/motion vars:
Quaternion q;           // [w, x, y, z]      quaternion container
VectorInt16 aa;         // [x, y, z]         accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]         gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]         world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]         gravity vector
//VectorInt16 g;         // [x, y, z]        accel sensor measurements to comunication serial port.
float euler[3];         // [psi, theta, phi] Euler angle container
float ypr[3];           // [yaw, pitch, roll] yaw/pitch/roll container and gravity vector
//------------------------------------------

// To have constance if the process is doing or we have some problems, we're
// going to use the LED implemented on the board as a process control.  
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;
//------------------------------------------
//____________________________________________________________________________



// CODE CONTROL ROUTINES DEFINITION:__________________________________________
// Here we're going to define a interrupt detection routine of the IMU, define
// a stoping criteria if there is something going wrong.

// Interrupt Detection Routine:
volatile bool mpuInterrupt = false; // Indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}
//------------------------------------------
//____________________________________________________________________________




// ############################################################################
//                         SETUP CONFIGURATION BLOCK
// ============================================================================
// Put here the setup configuration code to all the program to run once.
void setup() {


    // Join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    //------------------------------------------

    // Initialize Serial Communication Protocol:
    // (115200 chosen because we want, but it's really up to you depending on your project)
    Serial.begin(115200);
  //Serial.begin(38400); // If we want a slower port uncoment this and coment the upper one.
    while (!Serial); 
    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
    // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.
    //------------------------------------------

    // Initialize device (IMU):
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    //------------------------------------------

    // Verify the connection with the IMU:
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    //------------------------------------------

    // Taking into account the IMU reply, wait for ready:
    Serial.println(F("\nSend any character to begin DMP programming and demo: "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again
    //------------------------------------------

    // Load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();
    //------------------------------------------

    // Gyro offsets: 
    mpu.setXGyroOffset(220); // The Initial X position in degrees.
    mpu.setYGyroOffset(76);  // The Initial Y position in degrees.
    mpu.setZGyroOffset(-85); // The Initial Z position in degrees.
    mpu.setZAccelOffset(1788);
    // Here scaled for min sensitivity, the values is needed to put in relation 
    // with the initial position of the IMU when it started the operation to 
    // have good value of the outputs. In the last line of this code 
    // block (mpu.setZAccelOffset(1788);) it's determined the gravity offset, it's 
    // important to check if this value works in your hardware, interesting to do a 
    // little resizing of this value especially.
    //------------------------------------------


    // Making sure up to here all works well (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);
        //------------------

        // Enabling Arduino interrupt detection protocol:
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        //------------------

        // Set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;
        //------------------

        // Get expected DMP packet size for later comparison:
        packetSize = mpu.dmpGetFIFOPacketSize();
        //------------------
        } // IF end, ELSE start
        
        else {
        // ERROR! label make here if something crash.
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
        } // End of If-Else disruption.
    //------------------------------------------

    // Configure LED for output process control:
    pinMode(LED_PIN, OUTPUT);
    //------------------------------------------

} // End of setup.
// ============================================================================




// ############################################################################
//                            CONTINUOUS LOOP BLOCK
// ============================================================================
// Put here the main code you want to run repeatedly one time and another for years.

void loop() {
    
    // ACTIVE CONTROL PROTOCOL:
    // We're saying, if programming failed, don't try to do anything
    if (!dmpReady) return;
    //------------------
    
    // Wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        // Other program behavior stuff here
       
        // If you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data.
     //------------------
     } // WHILE LOOP end.
  
    // Reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    //------------------
    //------------------------------------------ End Control Protocol.


    // FIFO COUNT & IMU DATA PRINTING:

    // Getting the current FIFO count value:
    fifoCount = mpu.getFIFOCount();
    //------------------

    // Checking for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

      // Otherwise, check for DMP data ready interrupt (this should happen frequently)
     }// IF end, ELSEIF start:
     else if (mpuIntStatus & 0x02) {
        
        // Wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
        // End WHILE.
        
        mpu.getFIFOBytes(fifoBuffer, packetSize); // read a packet from FIFO

        // Track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(ypr[2] * 180/M_PI);
        #endif

        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
        
      }// ELSEIF end.
    //------------------
    //------------------------------------------ End FIFO Count and IMU data Print.

    
} // End of Cotinuous Loop.
// ============================================================================
// PROGRAM END.
