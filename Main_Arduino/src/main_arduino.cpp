// include relevant libraries
#include <Arduino.h>
#include <Wire.h>
#include <PID_v1.h>
#include <MPU9250.h>
#include "eeprom_utils.h"
#include <SoftwareSerial.h>

// create Software Serial object
SoftwareSerial btSerial(2, 3);

// slave address of other arduino
#define SLAVE_ADDR 9

// max angle to show on LEDs
#define MAX_ANGLE 18

// Define maximum motor speed
#define MAX_SPEED 1500

// LED pins
#define LED_PIN_1  5
#define LED_PIN_2  6
#define LED_PIN_3  9

// define controller parameters
// velocity controller
const double Kp_v = 0.001;
const double Kd_v = 0;
const double Ki_v = 0;
// angle controller
const double Kp_a = 1300;
const double Kd_a = 30;
const double Ki_a = 150;
// angle variable
double current_angle;
// setpoints
double sp_v = 0; // [m/s]
// control signals
double u_v = 0;
double u_a = 0;
int u_a_send = 0;
// bytes for SPI comms
byte bytesToSend[2];
byte b1, b2;
//current speed of vehicle
int speed_read = 0;
double current_speed = 0;

// define PID controller objects
PID PID_velocity(&current_speed, &u_v, &sp_v, Kp_v, Ki_v, Kd_v, DIRECT);
PID PID_angle(&current_angle, &u_a, &u_v, Kp_a, Ki_a, Kd_a, DIRECT);

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
            v[1] = (6.244035046342855111e-3 * x)
                 + (0.98751192990731428978 * v[0]);
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
    // start serial
    Serial.begin(9600);

    // start software serial
    btSerial.begin(9600);

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

    //// calibrate MPU unit
    //mpu.calibrateAccelGyro();
    loadCalibration();
    //mpu.setMagneticDeclination(3.44);
    //delay(5000);
    //digitalWrite(LED_PIN_1, HIGH);
    //digitalWrite(LED_PIN_2, HIGH);
    //digitalWrite(LED_PIN_3, HIGH);
    //mpu.calibrateMag();
    //mpu.printCalibration();

    //// save to eeprom
    //saveCalibration();
    //digitalWrite(LED_PIN_1, LOW);
    //digitalWrite(LED_PIN_2, LOW);
    //digitalWrite(LED_PIN_3, LOW);

    // wait for angle to stabalize
    do {
        mpu.update();
    } while(mpu.getRoll() > 16);

    // set PID sample time
    PID_velocity.SetSampleTime(20);
    PID_angle.SetSampleTime(20);
    // set PID output limits
    PID_velocity.SetOutputLimits(-MAX_SPEED, MAX_SPEED);
    PID_angle.SetOutputLimits(-MAX_SPEED, MAX_SPEED);
    // start PID controllers
    PID_velocity.SetMode(AUTOMATIC);
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
    //current_angle = mpu.getRoll();
    current_angle = filter.step(mpu.getRoll());

    // get current speed 
    Wire.requestFrom(SLAVE_ADDR, 2);
    if(Wire.available() >= 2) {
        b1 = Wire.read();
        b2 = Wire.read();
    }
    speed_read = b1;
    speed_read = (speed_read << 8) | b2;
    current_speed = speed_read;

    // compute new controller values
    PID_velocity.Compute();
    PID_angle.Compute();

    // set LEDs based on the angle
    analogWrite(LED_PIN_1, min(255, max(0, current_angle / MAX_ANGLE * 255)));
    analogWrite(LED_PIN_2, max(0, 255 - abs(current_angle) / MAX_ANGLE * 255));
    analogWrite(LED_PIN_3, min(255, max(0, -current_angle / MAX_ANGLE * 255)));

    // set the value to send to other arduino
    u_a_send = (int) u_a;

    // write control signal to other arduino (with 0xCC as start byte)
    Wire.beginTransmission(SLAVE_ADDR);
    bytesToSend[0] = (u_a_send >> 8) & 0xFF;
    bytesToSend[1] = u_a_send & 0xFF;
    Wire.write(bytesToSend, 2);
    Wire.endTransmission();

    Serial.println(current_angle);
    btSerial.println(u_a);
    
    //Serial.print("\t");
    //Serial.println(speed_read);
    //Serial.print("\t");
    //Serial.print(u_v);
    //Serial.print("\t");
    //if (u_a > 0)
        //Serial.println(u_a_send);
    //else
        //Serial.println(-u_a_send);
}
