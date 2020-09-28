// include relevant libraries
#include <Arduino.h>
#include <Wire.h>
#include <AccelStepper.h>

// define slave address of arduino
#define SLAVE_ADDR 9

// byte array to send over I2C
byte bytesToSend[2];

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
byte b1, b2;
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
// ===                    I2C receive function                  ===
// ================================================================
void receiveEvent() {
  // read one character from the I2C
    if (Wire.available() >= 2) {
        b1 = Wire.read();
        b2 = Wire.read();
        u_a = b1;
        u_a = (u_a << 8) | b2;
        left_motor.setMaxSpeed(u_a);
        right_motor.setMaxSpeed(u_a);
    }
}

// ================================================================
// ===                    I2C request function                  ===
// ================================================================
void requestEvent() {
    bytesToSend[0] = (vel_send >> 8) & 0xFF;
    bytesToSend[1] = vel_send & 0xFF;
    Wire.write(bytesToSend, 2); // respond with message of 2 bytes
    // as expected by master
}


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
void setup() {
    // start serial
    Serial.begin(9600);

    // I2C setup
    Wire.begin(SLAVE_ADDR);
    Wire.onReceive(receiveEvent);
    Wire.onRequest(requestEvent);

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

    // wait for I2C data to come in
    while (u_a == 0) 
        delay(100);
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
void loop() {

    // set motor spinning direction for left motor, right motor tries to find
    // the same position as left motor to sync them
    if (u_a > 0) {
        left_motor.moveTo(-10000);
    } else {
        left_motor.moveTo(10000);
    }
    right_motor.moveTo(left_motor.currentPosition() * -1);

    // compute new time variables
    current_time = millis();
    elapsed_time = current_time - prev_time;

    // compute motor speed
    if (elapsed_time > 100) {
        prev_time = current_time;
        pos_L = left_motor.currentPosition();
        pos_R = right_motor.currentPosition();
        vel_L = 1000 * (pos_L - prev_pos_L) / elapsed_time;
        vel_R = 1000 * (pos_R - prev_pos_R) / elapsed_time;
        vel_mean = 0.5 * (vel_L + vel_R);
        vel_send = int(vel_mean);
        prev_pos_L = pos_L;
        prev_pos_R = pos_R;
    }

    // run the motors
    right_motor.run();
    left_motor.run();
}
