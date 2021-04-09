#include <Arduino_FreeRTOS.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
 
/* This driver uses the Adafruit unified sensor library (Adafruit_Sensor),
   which provides a common 'type' for sensor data and some helper functions.
 
   To use this driver you will also need to download the Adafruit_Sensor
   library and include it in your libraries folder.
 
   You should also assign a unique ID to this sensor for use with
   the Adafruit Sensor API so that you can identify this particular
   sensor in any data logs, etc.  To assign a unique ID, simply
   provide an appropriate value in the constructor below (12345
   is used by default in this example).
 
   Connections
   ===========
   Connect SCL to analog 5
   Connect SDA to analog 4
   Connect VDD to 3-5V DC
   Connect GROUND to common ground
 
   History
   =======
   2015/MAR/03  - First release (KTOWN)
   2015/AUG/27  - Added calibration and system status helpers
 
 
   In this program, we use the Adafruit BNO055 connected to the Arduino to capture
   Roll+Pitch+Yaw data.
 
   @author Addison Sears-Collins
   @version 1.0 2019-04-02
*/
 
Adafruit_BNO055 bno = Adafruit_BNO055(55);
 
// Define two tasks for Blink & IMURead. These are function prototypes.
// The Blink task blinks the built-in LED on the Arduino board.
// IMURead captures Roll+Pitch+Yaw data from the Adafruit BNO055.
void TaskBlink( void *pvParameters );
void TaskIMURead( void *pvParameters );
 
 
/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/
 
void TaskBlink(void *pvParameters)  // This is a task.
{
  (void) pvParameters;
 
/*
  Blink
  Turns on an LED on for one second, then off for one second, repeatedly.
 
  Most Arduinos have an on-board LED you can control. On the UNO, LEONARDO, MEGA, and ZERO 
  it is attached to digital pin 13, on MKR1000 on pin 6. LED_BUILTIN takes care 
  of use the correct LED pin whatever is the board used.
   
  The MICRO does not have a LED_BUILTIN available. For the MICRO board please substitute
  the LED_BUILTIN definition with either LED_BUILTIN_RX or LED_BUILTIN_TX.
  e.g. pinMode(LED_BUILTIN_RX, OUTPUT); etc.
   
  If you want to know what pin the on-board LED is connected to on your Arduino model, check
  the Technical Specs of your board  at https://www.arduino.cc/en/Main/Products
   
  This example code is in the public domain.
 
  modified 8 May 2014
  by Scott Fitzgerald
   
  modified 2 Sep 2016
  by Arturo Guadalupi
*/
 
  // initialize digital LED_BUILTIN on pin 13 as an output.
  pinMode(LED_BUILTIN, OUTPUT);
 
  for (;;) // A Task shall never return or exit.
  {
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    vTaskDelay( 1000 / portTICK_PERIOD_MS ); // wait for one second
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
    vTaskDelay( 1000 / portTICK_PERIOD_MS ); // wait for one second
  }
}
 
void TaskIMURead(void *pvParameters)  // This is a task.
{
  (void) pvParameters;
   
/*
  IMUReadSerial
  Capture the Roll+Pitch+Yaw data
 
*/
 
  for (;;)
  {
    // Get a new sensor event
    sensors_event_t event;
    bno.getEvent(&event);
    
    /* Display the floating point data */
    Serial.print("Yaw: ");
    Serial.print(event.orientation.x, 4);
    Serial.print("\tPitch: ");
    Serial.print(event.orientation.y, 4);
    Serial.print("\tRoll: ");
    Serial.print(event.orientation.z, 4);
 
    /* Optional: Display calibration status */
    displayCalStatus();
 
    /* Optional: Display sensor status (debug only) */
    displaySensorStatus();
 
    /* New line for the next sample */
    Serial.println("");
     
    vTaskDelay(7);  // seven tick delay (105ms) in between reads for stability
  }
}
 
/**************************************************************************/
/*
    Displays some basic information on this sensor from the unified
    sensor API sensor_t type (see Adafruit_Sensor for more information)
*/
/**************************************************************************/
void displaySensorDetails(void)
{
  sensor_t sensor;
  bno.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}
 
/**************************************************************************/
/*
    Display some basic info about the sensor status
*/
/**************************************************************************/
void displaySensorStatus(void)
{
  /* Get the system status values (mostly for debugging purposes) */
  uint8_t system_status, self_test_results, system_error;
  system_status = self_test_results = system_error = 0;
  bno.getSystemStatus(&system_status, &self_test_results, &system_error);
 
  /* Display the results in the Serial Monitor */
  Serial.println("");
  Serial.print("System Status: 0x");
  Serial.println(system_status, HEX);
  Serial.print("Self Test:     0x");
  Serial.println(self_test_results, HEX);
  Serial.print("System Error:  0x");
  Serial.println(system_error, HEX);
  Serial.println("");
  delay(500);
}
 
/**************************************************************************/
/*
    Display sensor calibration status
*/
/**************************************************************************/
void displayCalStatus(void)
{
  /* Get the four calibration values (0..3) */
  /* Any sensor data reporting 0 should be ignored, */
  /* 3 means 'fully calibrated" */
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
 
  /* The data should be ignored until the system calibration is > 0 */
  Serial.print("\t");
  if (!system)
  {
    Serial.print("! ");
  }
 
  /* Display the individual values */
  Serial.print("Sys:");
  Serial.print(system, DEC);
  Serial.print(" G:");
  Serial.print(gyro, DEC);
  Serial.print(" A:");
  Serial.print(accel, DEC);
  Serial.print(" M:");
  Serial.print(mag, DEC);
}
 
/*--------------------------------------------------*/
/*--------------------Setup and Loop----------------*/
/*--------------------------------------------------*/
// The setup function runs once when you press reset or power the board
void setup() {
   
  // Initialize serial communication at 9600 bits per second.
  Serial.begin(9600);
   
  while (!Serial) {
    ; // Wait for serial port to connect. Needed for native USB, on LEONARDO, MICRO, YUN, and other 32u4 based boards.
  }
 
  Serial.println("Orientation Sensor Test"); Serial.println("");
 
  /* Initialize the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
 
  delay(1000);
 
  /* Display some basic information on this sensor */
  displaySensorDetails();
 
  /* Optional: Display current status */
  //displaySensorStatus();
 
  bno.setExtCrystalUse(true);
   
  // Set up two tasks to run independently.
  xTaskCreate(
    TaskBlink
    ,  "Blink"   // A name just for humans
    ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  1  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL );
 
  xTaskCreate(
    TaskIMURead
    ,  "IMURead"
    ,  128  // Stack size
    ,  NULL
    ,  2  // Priority
    ,  NULL );
 
  // Now the task scheduler, which takes over control of scheduling individual tasks, is automatically started.
}
 
void loop()
{
  // Empty. Things are done in Tasks.
}
