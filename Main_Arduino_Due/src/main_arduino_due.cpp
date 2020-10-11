// include relevant libraries
#include <Arduino.h>
#include <Wire.h>
#include <PID_v1.h>
#include <SparkFunMPU9250-DMP.h>
#include <AccelStepper.h>
#include <MultiStepper.h>
//#include "eeprom_utils.h"

// max angle to show on LEDs
#define MAX_ANGLE 16
#define LED_ANGLE 5

// Define maximum motor speed
#define MAX_SPEED 2000

// define controller sample rate [ms]
#define CONTROLLER_SAMPLE_RATE 10

// LED pins
#define LED_PIN_1  11
#define LED_PIN_2  12
#define LED_PIN_3  13

// Define pin connections
#define DIR_PIN_R 2
#define STEP_PIN_R 3
#define DIR_PIN_L 4
#define STEP_PIN_L 5
#define MS1_L 22
#define MS2_L 24
#define MS3_L 26
#define MS1_R 23
#define MS2_R 25
#define MS3_R 27


// define controller parameters
// velocity controller
const double Kp_v = 0;
const double Kd_v = 0;
const double Ki_v = 0;
// angle controller
const double Kp_a = 380;
const double Kd_a = 30;
const double Ki_a = 5;
// angle variable
double current_angle = 0;
// setpoint
double sp = 0;
// control signals
double u_v = 0;
double u_a = 0;
int u_a_send = 0;
// bytes for SPI comms
byte bytesToSend[2];
byte b1, b2;
//current speed of vehicle
int speed_read = 0;

// time variables
long current_time, elapsed_time, prev_time = 0;

// define PID controller objects
PID PID_angle(&current_angle, &u_a, &sp, Kp_a, Ki_a, Kd_a, DIRECT);

// create sensor object
MPU9250_DMP imu;

// Define motor interface type
#define motorInterfaceType 1

// Creates an instance of the stepper motor class
AccelStepper left_motor(motorInterfaceType, STEP_PIN_L, DIR_PIN_L);
AccelStepper right_motor(motorInterfaceType, STEP_PIN_R, DIR_PIN_R);
MultiStepper steppers;

// positions of the motors
long positions[2];

// helper function to set microstep size
void setMicrosteps(int motor, int ms1, int ms2, int ms3) {
    if (motor == 0) {
        digitalWrite(MS1_L, ms1);
    } else {
        digitalWrite(MS1_R, ms1);
    }
}

class FilterBuLp1
{
    public:
        FilterBuLp1()
        {
            v[0]=v[1]=0.0;
        }
    private:
        float v[2];
    public:
        float step(float x) //class II 
        {
            v[0] = v[1];
            v[1] = (8.636402701376222346e-2 * x)
                 + (0.82727194597247555308 * v[0]);
            return 
                 (v[0] + v[1]);
        }
};
// filter object
FilterBuLp1 filter;

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
    Serial1.begin(9600);

    // setup LED pins as output and turn them on and off to see if they work
    pinMode(LED_PIN_1, OUTPUT);
    pinMode(LED_PIN_2, OUTPUT);
    pinMode(LED_PIN_3, OUTPUT);
    digitalWrite(LED_PIN_1, HIGH);
    digitalWrite(LED_PIN_2, HIGH);
    digitalWrite(LED_PIN_3, HIGH);
    delay(500);
    digitalWrite(LED_PIN_1, LOW);
    digitalWrite(LED_PIN_2, LOW);
    digitalWrite(LED_PIN_3, LOW);

    // Call imu.begin() to verify communication and initialize
    if (imu.begin() != INV_SUCCESS)
    {
        while (1)
        {
            //Serial.println("Unable to communicate with MPU-9250");
            //Serial.println("Check connections, and try again.");
            //Serial.println();
            delay(5000);
        }
    }

    imu.dmpBegin(DMP_FEATURE_6X_LP_QUAT | // Enable 6-axis quat
            DMP_FEATURE_GYRO_CAL, // Use gyro calibration
            1000/CONTROLLER_SAMPLE_RATE); // Set DMP FIFO rate in hz
    // DMP_FEATURE_LP_QUAT can also be used. It uses the 
    // accelerometer in low-power mode to estimate quat's.
    // DMP_FEATURE_LP_QUAT and 6X_LP_QUAT are mutually exclusive
    // setLPF() can be used to set the digital low-pass filter
    // of the accelerometer and gyroscope.
    // Can be any of the following: 188, 98, 42, 20, 10, 5
    // (values are in Hz).
    imu.setLPF(5); 

    // set PID sample time
    PID_angle.SetSampleTime(CONTROLLER_SAMPLE_RATE);
    // set PID output limits
    PID_angle.SetOutputLimits(-MAX_SPEED, MAX_SPEED);
    // start PID controllers
    PID_angle.SetMode(AUTOMATIC);

    // set microstepping for the motors
    pinMode(MS1_L, OUTPUT);
    pinMode(MS1_R, OUTPUT);
    setMicrosteps(0, HIGH, LOW, LOW);
    setMicrosteps(1, HIGH, LOW, LOW);

    // motor setup
    left_motor.setMaxSpeed(MAX_SPEED);
    //left_motor.setAcceleration(MAX_ACCELERATION);
    left_motor.setSpeed(0);
    right_motor.setMaxSpeed(MAX_SPEED);
    //right_motor.setAcceleration(MAX_ACCELERATION);
    right_motor.setSpeed(0);

    steppers.addStepper(left_motor);
    steppers.addStepper(right_motor);
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    //Serial.println(millis());
    // Check for new data in the FIFO
    if ( imu.fifoAvailable() )
    {
        // Use dmpUpdateFifo to update the ax, gx, mx, etc. values
        if ( imu.dmpUpdateFifo() == INV_SUCCESS)
        {
            // computeEulerAngles can be used -- after updating the
            // quaternion values -- to estimate roll, pitch, and yaw
            imu.computeEulerAngles();
        }
        // get the current angle 
        if (imu.roll > 300) {
            current_angle = 360 - imu.roll;
        } else {
            current_angle = - imu.roll;
        }

        // compute new controller value
        PID_angle.Compute();
        u_a = filter.step(u_a);
        Serial1.println(u_a);

        // set motor spinning direction for the steppers
        if (u_a > 0) {
            positions[0] = -10000;
            positions[1] = 10000;
        } else {
            positions[0] = 10000;
            positions[1] = -10000;
        }
        left_motor.setMaxSpeed(u_a);
        right_motor.setMaxSpeed(u_a);
        steppers.moveTo(positions);
        //
        // set LEDs based on the angle
        analogWrite(LED_PIN_1, min(255, max(0, current_angle / LED_ANGLE * 255)));
        analogWrite(LED_PIN_2, max(0, 255 - abs(current_angle) / LED_ANGLE * 255));
        analogWrite(LED_PIN_3, min(255, max(0, -current_angle / LED_ANGLE * 255)));
    }

    // run the motors
    steppers.run();
}
