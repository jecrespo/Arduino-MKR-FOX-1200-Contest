#include <SigFox.h>
#include <ArduinoLowPower.h>
#include <Timer.h>
#include <Adafruit_HMC5883_U.h>
#include <Adafruit_ADXL345_U.h>

// Set debug to false to enable continuous mode
// and disable serial prints
int debug = false;

Timer t;

void setup() {

  if (debug == true) {

    Serial.begin(9600);
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

  t.every(3600000, takeReading);
}

void loop()
{
  t.update();
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

