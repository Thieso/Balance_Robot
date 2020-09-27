// include relevant libraries
#include <Arduino.h>
#include <SPI.h>
#include <AccelStepper.h>

// variables for SPI communications
byte buff[4];
volatile byte indx;
volatile boolean process;

// Define maximum motor performance parameters
#define MAX_SPEED 2000
#define MAX_ACCELERATION 6000

// Define pin connections
#define DIR_PIN_R 2
#define STEP_PIN_R 3
#define DIR_PIN_L 4
#define STEP_PIN_L 5
#define MS1_L 7
#define MS2_L 8
#define MS3_L 9
#define MS1_R 8
#define MS2_R 11
#define MS3_R 12

// position variables
double pos_L, pos_R;
double prev_pos_L = 0;
double prev_pos_R = 0;

// velocity variables
double vel_L, vel_R, vel_mean;
int vel_send;

// variable to indicate that serial data was received
bool serial_received = false;

// time variables
unsigned long current_time, elapsed_time, prev_time = 0;

// control signal (to be read from other arduino)
int u_a = 0;
int u_a_read = 0;

// Define motor interface type
#define motorInterfaceType 1

// Creates an instance of the stepper motor class
AccelStepper left_motor(motorInterfaceType, STEP_PIN_L, DIR_PIN_L);
AccelStepper right_motor(motorInterfaceType, STEP_PIN_R, DIR_PIN_R);

// helper function to set microstep size
void setMicrosteps(int motor, int ms1, int ms2, int ms3) {
    if (motor == 0) {
        digitalWrite(MS1_L, ms1);
        //digitalWrite(MS2_L, ms2);
        //digitalWrite(MS3_L, ms3);
    } else {
        digitalWrite(MS1_R, ms1);
        //digitalWrite(MS2_R, ms2);
        //digitalWrite(MS3_R, ms3);
    }
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    // start serial
    Serial.begin(9600);

    // SPI setup
    pinMode(MISO, OUTPUT); // have to send on master in so it set as output
    SPCR |= _BV(SPE); // turn on SPI in slave mode
    indx = 0; // buffer empty
    process = false;
    SPI.attachInterrupt(); // turn on interrupt

    // set microstepping for the motors
    pinMode(MS1_L, OUTPUT);
    //pinMode(MS2_L, OUTPUT);
    //pinMode(MS3_L, OUTPUT);
    pinMode(MS1_R, OUTPUT);
    //pinMode(MS2_R, OUTPUT);
    //pinMode(MS3_R, OUTPUT);
    setMicrosteps(0, HIGH, LOW, LOW);
    setMicrosteps(1, HIGH, LOW, LOW);

    // motor setup
    left_motor.setMaxSpeed(MAX_SPEED);
    left_motor.setAcceleration(MAX_ACCELERATION);
    left_motor.setSpeed(0);
    right_motor.setMaxSpeed(MAX_SPEED);
    right_motor.setAcceleration(MAX_ACCELERATION);
    right_motor.setSpeed(0);

    // wait for SPI data to come in
    while (indx == 0) 
        delay(100);
}

// ================================================================
// ===                    SPI interrupt routine                 ===
// ================================================================
//
ISR (SPI_STC_vect) 
{ 
    byte c = SPDR; // read byte from SPI Data Register
    if (indx < sizeof buff) {
        buff[indx] = c; // save data in the next index in the array buff
        indx++; // increment the index
        if (c == 0xCC) //check for the end of the word
            process = true;
    }
}


// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    // receive the control signal from analog input
    //if (Serial.available() >= 3) {
        //startByte = Serial.read();
        //if (startByte == 0xCC) {
            //b1 = Serial.read();
            //b2 = Serial.read();
            //u_a = b1 * 256 + b2;
            //u_a *= 2;
            //serial_received = true;
            //// set the max motor speed to which it accelerates
            //left_motor.setMaxSpeed(u_a);
            //right_motor.setMaxSpeed(u_a);
            //// adapt the microstep setting to have the smallest microstepping
            //// possible at any time
            ////if (MAX_SPEED / abs(u_a) > 4) {
                ////u_a *= 4;
                ////setMicrosteps(0, HIGH, HIGH, LOW);
                ////setMicrosteps(1, HIGH, HIGH, LOW);
            ////} else if (MAX_SPEED / abs(u_a) > 2) {
                ////u_a *= 2;
                ////setMicrosteps(0, LOW, HIGH, LOW);
                ////setMicrosteps(1, LOW, HIGH, LOW);
            ////} else {
                ////setMicrosteps(0, HIGH, LOW, LOW);
                ////setMicrosteps(1, HIGH, LOW, LOW);
            ////}
        //}
    //}
    if (process) {
        process = false; //reset the process
        u_a_read = buff[0] * 256 + buff[1];
        if (abs(u_a_read) <= MAX_SPEED) {
            u_a = abs(u_a_read);
            if (buff[2] != 0x00)
                u_a *= -1;
            left_motor.setMaxSpeed(u_a);
            right_motor.setMaxSpeed(u_a);
        }
        Serial.println(u_a); //print the array on serial monitor
        indx = 0; //reset button to zero
    }

    // set motor spinning direction for left motor, right motor tries to find
    // the same position as left motor to sync them
    if (u_a > 0) {
        left_motor.moveTo(-10000);
    } else {
        left_motor.moveTo(10000);
    }
    right_motor.moveTo(left_motor.currentPosition() * -1);

    // if serial was read, speed can be written to serial
    //if (serial_received == true) {
        //// compute new time variables
        //current_time = millis();
        //elapsed_time = current_time - prev_time;
        //prev_time = current_time;

        //// compute motor speed
        ////pos_L = left_motor.currentPosition();
        ////pos_R = right_motor.currentPosition();
        ////vel_L = (pos_L - prev_pos_L) / elapsed_time;
        ////vel_R = (pos_R - prev_pos_R) / elapsed_time;
        ////vel_mean = 0.5 * (vel_L + vel_R);
        ////vel_send = int(vel_mean);
        ////prev_pos_L = pos_L;
        ////prev_pos_R = pos_R;
        //vel_send = 0;

        //// send velocity over serial
        //Serial.write(0xCC);
        //Serial.write(vel_send / 256);
        //Serial.write(vel_send % 256);
        
        //serial_received = false;
    //}

    // run the motors
    //right_motor.runSpeed();
    //left_motor.runSpeed();
    right_motor.run();
    left_motor.run();
}
