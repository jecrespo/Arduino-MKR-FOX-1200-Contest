#include <SigFox.h>
#include <ArduinoLowPower.h>
#include <Timer.h>
#include <Adafruit_HMC5883_U.h>
#include <Adafruit_ADXL345_U.h>

// Set debug to false to enable continuous mode
// and disable serial prints
int debug = false;

Timer t;

/* Assign a unique ID to this sensor at the same time */
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(1);
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(2);

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);

  if (debug == true) {

    Serial.begin(9600);
    while (!Serial) {}
  }

  Serial.println("Starting....");

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
    digitalWrite(LED_BUILTIN, HIGH);
    while (1);
  }
  accel.setRange(ADXL345_RANGE_16_G);

  if (!mag.begin())
  {
    /* There was a problem detecting the HMC5883 ... check your connections */
    Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
    while (1);
  }

  t.every(600000, takeReading);
  //t.every(5000, readSensors);
}

void loop()
{
  t.update();
  readSensors();
  delay(100);
}

void reboot() {
  NVIC_SystemReset();
  while (1);
}

void takeReading() {
  // if we get here it means that an event was received

  SigFox.begin();

  if (debug == true) {
    Serial.println("Alarm event on sensor");
  }
  delay(100);

  // 3 bytes (ALM) + 8 bytes (ID as String) + 1 byte (source) < 12 bytes
  String to_be_sent = "ALM" + SigFox.ID() +  "1";

  SigFox.beginPacket();
  SigFox.print(to_be_sent);
  int ret = SigFox.endPacket();

  Serial.println(SigFox.status(SIGFOX));
  Serial.println(SigFox.status(ATMEL));

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

void readSensors () {
  /* Get a new sensor event */
  sensors_event_t event_accel;
  accel.getEvent(&event_accel);

  sensors_event_t event_mag;
  mag.getEvent(&event_mag);

  /* Display the results (acceleration is measured in m/s^2) */
  Serial.print("X: "); Serial.print(event_accel.acceleration.x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(event_accel.acceleration.y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(event_accel.acceleration.z); Serial.print("  "); Serial.println("m/s^2 ");

  /* Display the results (magnetic vector values are in micro-Tesla (uT)) */
  Serial.print("X: "); Serial.print(event_mag.magnetic.x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(event_mag.magnetic.y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(event_mag.magnetic.z); Serial.print("  "); Serial.println("uT");

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
  float headingDegrees = heading * 180 / M_PI;

  Serial.print("Heading (degrees): "); Serial.println(headingDegrees);
}

