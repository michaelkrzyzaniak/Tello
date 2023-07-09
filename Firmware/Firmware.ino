/*!
 * @author
 *  Written by Michael Krzyzaniak at Arizona State 
 *  University's School of Arts, Media + Engineering
 *  in Fall of 2013.
 *
 *  mkrzyzan@asu.edu
 */

#include <stdint.h>
#include <Wire.h>
#include "MPU_6050_9150.h"
#include "Calibration.h"
#include "Orientation.h"

#define UP_BUTTON_PWR_PIN    12
#define UP_BUTTON_READ_PIN   10
#define DOWN_BUTTON_PWR_PIN  7
#define DOWN_BUTTON_READ_PIN 5

#define TAP_THRESH 2.0

Calibration* calibration;
MadgwickAHRS* orientation;

float filtered_tap = 0;
int tap_can_trigger = 1;
int tap_did_trigger = 0;
int tap_angle       = 0;

Quaternion test_mag;

/*----------------------------------------------------*/
void setup(void)
{
  Serial.begin(115200);
  delay(2000);

  pinMode(UP_BUTTON_PWR_PIN, OUTPUT);
  pinMode(DOWN_BUTTON_PWR_PIN, OUTPUT);
  pinMode(UP_BUTTON_READ_PIN, INPUT_PULLDOWN);
  pinMode(DOWN_BUTTON_READ_PIN, INPUT_PULLDOWN);
  digitalWrite(UP_BUTTON_PWR_PIN, HIGH);
  digitalWrite(DOWN_BUTTON_PWR_PIN, HIGH);

  pinMode(15, INPUT);
  attachInterrupt(digitalPinToInterrupt(15), sensor_ready, RISING);
  
  pinMode(23, OUTPUT);
  digitalWrite(23, LOW); //supplies GND to the sensor

  //pinMode(12, OUTPUT);
  //digitalWrite(12, LOW);

  delay(5); //let the MPU power up;

  calibration = calibration_new();
  orientation = madgwick_ahrs_new();

  /* use MPU_2 if ADC0 is high, else MPU_1 */ 
  //(_2_G, _4_G, _8_G, or _16_G)
  //(_250, _500, _1000, or _2000 deg / sec)
  mpu_setup(MPU_1, MPU_ACCEL_RESOLUTION_8_G, MPU_GYRO_RESOLUTION_1000); 
}

/*----------------------------------------------------*/
void loop(void)
{
  Quaternion orientation_quat;
  Quaternion orientation_euler;

  if(calibration_calibrated(calibration))
    {
      cli();
      madgwick_ahrs_get_orientation(orientation, orientation_quat);
      sei()
      quaternion_to_euler_angles(orientation_quat, orientation_euler);
      int up   = digitalRead(UP_BUTTON_READ_PIN);
      int down = digitalRead(DOWN_BUTTON_READ_PIN);
      int updown = (up & down) ? 2 : up - down;
      //-1 for down, 0 for nothing, +1 for up, 2 for both
      
      int tap = 0;
      if(tap_did_trigger)
        {
          tap = tap_angle;
          tap_did_trigger = 0;
        }

      Serial.print(orientation_euler[1]);
      Serial.print(", ");
      Serial.print(orientation_euler[2]);
      Serial.print(", ");
      Serial.print(orientation_euler[3]);
      Serial.print(", ");
      //Serial.print(up - down);
      Serial.print(updown);
      Serial.print(", ");
      Serial.print(tap);
      Serial.print("\r\n");
/*
      Serial.print(test_mag[1]);
      Serial.print(", ");
      Serial.print(test_mag[2]);
      Serial.print(", ");
      Serial.print(test_mag[3]);
      Serial.print("\r\n");
*/
    }
  delay(100);
}

/*----------------------------------------------------*/
//this whole thing takes a little under 1 ms so sensor sample freq is set to 500Hz (2ms)
void sensor_ready()
{
  mpu_sensor_data_t data;
  float tap;

  //323 microseconds i2c fast mode plus 1000000
  mpu_read_sensors(MPU_1, &data);

  //returns true when sample added
  if(calibration_update(calibration, data.gyro, data.accel, data.mag))
    //Serial.println("Sample Added");
    Serial.println(calibration_calibrated_mask(calibration), BIN);

  quaternion_copy(data.mag, test_mag);

  tap = hypot(data.accel[1], data.accel[2]);
  if(tap > filtered_tap)
    filtered_tap = tap;
  else
    filtered_tap = (0.95 * filtered_tap) + (0.05 * tap);
  //Serial.println(filtered_tap);
  if(filtered_tap > TAP_THRESH)
    {
      if(tap_can_trigger)
        {
          tap_did_trigger = 1;
          //re-use tap variable
          tap = atan2(data.accel[2], data.accel[1]);
          tap += M_PI;
          tap *= 4.0 / (2*M_PI);
          tap = round(tap);
          tap_angle = tap;
          tap_angle %= 4;
          tap_angle += 1;
        }
      tap_can_trigger = false;
    }
  else
    tap_can_trigger = true;

  if(calibration_calibrated(calibration))
    madgwick_ahrs_update_marg   (orientation, 0.002, data.gyro, data.accel, data.mag);
}
