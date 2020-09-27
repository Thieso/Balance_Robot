// include relevant libraries
#include <Arduino.h>
#include <SPI.h>
#include <PID_v1.h>
#include <MPU9250.h>
#include "eeprom_utils.h"

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
const double Kp_v = 0;
const double Kd_v = 0;
const double Ki_v = 0;
// angle controller
const double Kp_a = 950;
const double Kd_a = 15;
const double Ki_a = 90;
// angle variable
double current_angle;
// setpoints
double sp_v = 0; // [m/s]
// control signals
double u_v = 0;
double u_a = 0;
int u_a_send = 0;
//current speed of vehicle
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
            //v[1] = (5.919070381840546569e-2 * x)
                 //+ (0.88161859236318906863 * v[0]);
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

    // start SPI communications
    digitalWrite(SS, HIGH); // disable Slave Select
    SPI.begin();
    SPI.setClockDivider(SPI_CLOCK_DIV8);//divide the clock by 8

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
    } while(mpu.getRoll() > 18);

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
    //if (Serial.available() >= 3) {
        //startByte = Serial.read();
        //if (startByte == 0xCC) {
            //b1 = Serial.read();
            //b2 = Serial.read();
            //current_speed = b1 * 256 + b2;
        //}
    //}
    current_speed = 0;

    // compute new controller values
    PID_velocity.Compute();
    PID_angle.Compute();

    // set LEDs based on the angle
    analogWrite(LED_PIN_1, min(255, max(0, current_angle / MAX_ANGLE * 255)));
    analogWrite(LED_PIN_2, max(0, 255 - abs(current_angle) / MAX_ANGLE * 255));
    analogWrite(LED_PIN_3, min(255, max(0, -current_angle / MAX_ANGLE * 255)));

    // set the value to send to other arduino
    u_a_send = (int) u_a;
    u_a_send = abs(u_a_send);


    // write control signal to other arduino (with 0xCC as start byte)
    digitalWrite(SS, LOW); // enable Slave Select
    // send test string
    SPI.transfer(u_a_send / 256);
    SPI.transfer(u_a_send % 256);
    if (u_a > 0)
        SPI.transfer(0x00);
    else
        SPI.transfer(0x01);
    SPI.transfer(0xCC);
    digitalWrite(SS, HIGH); // disable Slave Select

    //Serial.print(current_angle);
    //Serial.print("\t");
    //Serial.print(current_speed);
    //Serial.print("\t");
    //Serial.print(u_v);
    //Serial.print("\t");
    if (u_a > 0)
        Serial.println(u_a_send);
    else
        Serial.println(-u_a_send);
}
