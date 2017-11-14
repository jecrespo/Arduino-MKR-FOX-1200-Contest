#include <SigFox.h>
#include <ArduinoLowPower.h>
#include <Timer.h>  //https://github.com/JChristensen/Timer
#include <Adafruit_HMC5883_U.h>
#include <Adafruit_ADXL345_U.h>

#define NUMBER_READINGS 3 //When you get 3 sensor readings with bike fallen, send alert.
#define SENSOR_READING_TIME 10000
#define UPDATE_SIGFOX_TIME 600000

// Set debug to false to enable continuous mode
// and disable serial prints
int debug = false;

boolean fallen = false;
int fallen_counter = 0; //counter number falling readings
int fallen_event;
int led_event;

Timer t;

/* Assign a unique ID to this sensor at the same time */
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(1);
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(2);

float roll, pitch, headingDegrees; //X, Y, Z  -- https://support.pcigeomatics.com/hc/en-us/article_attachments/201813485/image005.png

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  if (debug == true) {

    Serial.begin(115200);
    Serial.println("Starting....");
    while (!Serial) {}
  }

  if (!SigFox.begin()) {
    //something is really wrong, try rebooting
    reboot();
  }

  //Send module to standby until we need to send a message
  SigFox.end();

  if (debug == true) {
    // Enable debug prints and LED indication if we are testing
    SigFox.debug();
  }

  if (!accel.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.println("Ooops, no ADXL345 detected ... Check your wiring!");
    digitalWrite(LED_BUILTIN, HIGH);  //light on builtin led if a problem is detected
    while (1);
  }
  accel.setRange(ADXL345_RANGE_16_G);

  if (!mag.begin())
  {
    /* There was a problem detecting the HMC5883 ... check your connections */
    Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
    digitalWrite(LED_BUILTIN, HIGH);  //light on builtin led if a problem is detected
    while (1);
  }

  //Initialitation
  readSensors();
  updateState();

  if (debug == true) {
    t.every(UPDATE_SIGFOX_TIME / 10, updateState); //Send data to update state to sigfox platform. Like a "keepalive"
    t.every(SENSOR_READING_TIME / 10, readSensors); //Read sensor to check if bike has fallen
  }
  else {
    t.every(UPDATE_SIGFOX_TIME, updateState); //Send data to update state to sigfox platform. Like a "keepalive"
    t.every(SENSOR_READING_TIME, readSensors);  //Read sensor to check if bike has fallen
  }
}

void loop()
{
  t.update();
}

void reboot() {
  NVIC_SystemReset();
  while (1);
}

void updateState() {  //Send axis inclination to sigfox
  SigFox.begin();

  delay(100);

  if (debug == true) {
    Serial.print("Send Data: ");
  }
  delay(100);

  // Data snet to sigfox platform
  // D (data) + roll (3 bytes) + # + pitch (3 bytes) + # +heading (3 bytes) = 12 bytes
  String to_be_sent = "D" + String(int(roll)) + "#" + String(int(pitch)) +  "#" + String(int(headingDegrees));
  if (debug == true) {
    Serial.println(to_be_sent);
  }

  SigFox.beginPacket();
  SigFox.print(to_be_sent);
  int ret = SigFox.endPacket();

  if (debug == true) {
    Serial.println(SigFox.status(SIGFOX));
    Serial.println(SigFox.status(ATMEL));
  }

  // shut down module, back to standby
  SigFox.end();

  if (debug == true) {
    if (ret > 0) {
      Serial.println("No transmission");
    } else {
      Serial.println("Transmission ok");
    }
  }
}

void readSensors () { //read sensors and check bike state
  /* Get a new sensor event */
  sensors_event_t event_accel;
  accel.getEvent(&event_accel);

  sensors_event_t event_mag;
  mag.getEvent(&event_mag);

  if (debug == true) {
    /* Display the results (acceleration is measured in m/s^2) */
    Serial.print("X: "); Serial.print(event_accel.acceleration.x); Serial.print("  ");
    Serial.print("Y: "); Serial.print(event_accel.acceleration.y); Serial.print("  ");
    Serial.print("Z: "); Serial.print(event_accel.acceleration.z); Serial.print("  "); Serial.println("m/s^2 ");
  }

  float anguloX = atan(event_accel.acceleration.y / sqrt((event_accel.acceleration.x * event_accel.acceleration.x) + (event_accel.acceleration.z * event_accel.acceleration.z))); //En radianes
  roll = anguloX * 180 / M_PI;

  float anguloY = atan(event_accel.acceleration.x / sqrt((event_accel.acceleration.y * event_accel.acceleration.y) + (event_accel.acceleration.z * event_accel.acceleration.z))); //En radianes
  pitch = anguloY * 180 / M_PI;

  if (debug == true) {
    Serial.print("Angle X: "); Serial.print(" ");
    Serial.print(roll); Serial.println(" degrees");
    Serial.print("Angle Y: "); Serial.print(" ");
    Serial.print(pitch); Serial.println(" degrees");
    Serial.println("-----------------------------------");
  }

  if (debug == true) {
    /* Display the results (magnetic vector values are in micro-Tesla (uT)) */
    Serial.print("X: "); Serial.print(event_mag.magnetic.x); Serial.print("  ");
    Serial.print("Y: "); Serial.print(event_mag.magnetic.y); Serial.print("  ");
    Serial.print("Z: "); Serial.print(event_mag.magnetic.z); Serial.print("  "); Serial.println("uT");
  }

  // Hold the module so that Z is pointing 'up' and you can measure the heading with x&y
  // Calculate heading when the magnetometer is level, then correct for signs of axis.
  float heading = atan2(event_mag.magnetic.y, event_mag.magnetic.x);

  // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
  // Find yours here: http://www.magnetic-declination.com/
  // Mine is: -0* 28' W, which is 0.47 Degrees, or (which we need) 0.0081 radians, Calculator: https://es.planetcalc.com/71/
  // If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
  float declinationAngle = 0.0081;
  heading += declinationAngle;

  // Correct for when signs are reversed.
  if (heading < 0)
    heading += 2 * PI;

  // Check for wrap due to addition of declination.
  if (heading > 2 * PI)
    heading -= 2 * PI;

  // Convert radians to degrees for readability.
  headingDegrees = heading * 180 / M_PI;

  if (debug == true) {
    Serial.print("Heading (degrees): "); Serial.println(headingDegrees);
    Serial.println("-----------------------------------");
    Serial.println("-----------------------------------");
  }

  //Checks if bike is straight or fallen
  //Due to IMU position stright state: roll (x) == 90 && pitch (y) == 0
  //Fallen state: (roll (x) < 30 || roll (x) > 150) || (pitch (y) > 60|| pitch (y) < -60)
  //heading (z) only direcction

  if (((roll < 30) || (roll > 150)) || ((pitch > 60) || (pitch < -60))) {
    fallen_counter++;
  }
  else {
    fallen_counter = 0; //reset counter
    if (fallen == true) {
      fallen = false;
      t.stop(led_event);
      t.stop(fallen_event);
      digitalWrite(LED_BUILTIN, LOW); //in case timer stopped and led on
    }
  }

  if ((fallen_counter > NUMBER_READINGS) && (fallen == false)) {
    fallen = true;
    led_event = t.oscillate(LED_BUILTIN, 1000, HIGH); //blink builtin led with new timer
    fallen_event = t.every(300000, sendAlarm, 3); //repeat 3 times send alarm every 5 minutes
  }
}

void  sendAlarm() {
  //send alarm to sigfox

  SigFox.begin();

  if (debug == true) {
    Serial.print("Alarm event on sensor: ");
  }
  delay(100);

  // A (alarm) + roll (3 bytes) + # + pitch (3 bytes) + # +heading (3 bytes) = 12 bytes
  String to_be_sent = "A" + String(int(roll)) + "#" + String(int(pitch)) +  "#" + String(int(headingDegrees));
  if (debug == true) {
    Serial.println(to_be_sent);
  }

  SigFox.beginPacket();
  SigFox.print(to_be_sent);
  int ret = SigFox.endPacket();

  if (debug == true) {
    Serial.println(SigFox.status(SIGFOX));
    Serial.println(SigFox.status(ATMEL));
  }

  // shut down module, back to standby
  SigFox.end();

  if (debug == true) {
    if (ret > 0) {
      Serial.println("No transmission");
    } else {
      Serial.println("Transmission ok");
    }
  }
}
