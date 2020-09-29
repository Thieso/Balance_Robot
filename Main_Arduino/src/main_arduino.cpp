// include relevant libraries
#include <Arduino.h>
#include <Wire.h>
#include <PID_v1.h>
#include <MPU9250.h>
#include "eeprom_utils.h"

// slave address of other arduino
#define SLAVE_ADDR 9

// max angle to show on LEDs
#define MAX_ANGLE 18
#define LED_ANGLE 5

// Define maximum motor speed
#define MAX_SPEED 1500

// LED pins
#define LED_PIN_1  5
#define LED_PIN_2  6
#define LED_PIN_3  9

// define controller parameters
// velocity controller
const double Kp_v = 0;
const double Kd_v = 0;
const double Ki_v = 0;
// angle controller
const double Kp_a = 860;
const double Kd_a = 3.3;
const double Ki_a = 105;
// angle variable
double current_angle;
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

// define PID controller objects
PID PID_angle(&current_angle, &u_a, &sp, Kp_a, Ki_a, Kd_a, DIRECT);

//Low pass butterworth filter order=1 alpha1=0.01 
class  FilterBuLp1
{
    public:
        FilterBuLp1()
        {
            v[0] = MAX_ANGLE/2;
            v[1] = MAX_ANGLE/2;
        }
    private:
        float v[2];
    public:
        float step(float x) //class II 
        {
            v[0] = v[1];
            v[1] = (2.452160924946589216e-2 * x)
                 + (0.95095678150106821569 * v[0]);
            return (v[0] + v[1]);
        }
};
FilterBuLp1 filter;

// create sensor object
MPU9250 mpu;

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
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

    // start MPU unit
    Wire.begin();
    delay(2000);
    mpu.setup();

    // calibrate MPU unit
    loadCalibration();

    // wait for angle to stabalize
    do {
        mpu.update();
    } while(mpu.getRoll() > 16);

    // set PID sample time
    PID_angle.SetSampleTime(10);
    // set PID output limits
    PID_angle.SetOutputLimits(-MAX_SPEED, MAX_SPEED);
    // start PID controllers
    PID_angle.SetMode(AUTOMATIC);

    // create filter
    filter = FilterBuLp1();
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    // update data from MPU sensor
    mpu.update();

    // get the current angle (low pass filtered)
    current_angle = filter.step(mpu.getRoll());

    // compute new controller value
    PID_angle.Compute();

    // set LEDs based on the angle
    analogWrite(LED_PIN_1, min(255, max(0, current_angle / LED_ANGLE * 255)));
    analogWrite(LED_PIN_2, max(0, 255 - abs(current_angle) / LED_ANGLE * 255));
    analogWrite(LED_PIN_3, min(255, max(0, -current_angle / LED_ANGLE * 255)));

    // set the value to send to other arduino
    u_a_send = (int) u_a;

    // write control signal to other arduino (with 0xCC as start byte)
    Wire.beginTransmission(SLAVE_ADDR);
    bytesToSend[0] = (u_a_send >> 8) & 0xFF;
    bytesToSend[1] = u_a_send & 0xFF;
    Wire.write(bytesToSend, 2);
    Wire.endTransmission();
}
