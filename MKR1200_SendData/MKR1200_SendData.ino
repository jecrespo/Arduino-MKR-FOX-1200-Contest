#include <SigFox.h>
#include <ArduinoLowPower.h>
#include <Timer.h>
#include <Wire.h>

#define SENSOR 0x53 // Device add in which is also included the 8th bit for selectting the mode, read in this case
#define Power_Register 0x2D
#define X_Axis_Register_DATAX0 0x32 // Hexadecima address for the DATAX0 internal register.
#define X_Axis_Register_DATAX1 0x33 // Hexadecima address for the DATAX1 internal register.
#define Y_Axis_Register_DATAY0 0x34
#define Y_Axis_Register_DATAY1 0x35
#define Z_Axis_Register_DATAZ0 0x36
#define Z_Axis_Register_DATAZ1 0x37

// Set debug to false to enable continuous mode
// and disable serial prints
int debug = true;

Timer t;

void setup() {

  if (debug == true) {

    Serial.begin(9600);
    while (!Serial) {}
  }

  Serial.println("Comienza...");

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

  //t.every(3600000, takeReading);
  t.every(10000, takeReading);
  t.every(5000, accelReading);
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
  Serial.println("manda datos...");
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

void accelReading() {
  int  DataReturned_x0, DataReturned_x1, DataModified_x_out;
  float x;

  Serial.println("lee acel...");

  Wire.begin(9600);
  Wire.beginTransmission(SENSOR);
  Wire.write(Power_Register); // Power-saving features control: Register 0x2D POWER_CTL is 8 bits: [unused:unused:Link:AUTO_SLEEP:Measure:Sleep:Wakeup1:Wakeup0]
  Wire.write(8);  // bit D3 high for measuring enable: 8d - 00001000b
  Wire.write(X_Axis_Register_DATAX0);   // Requiring Register DATA_0
  Wire.write(X_Axis_Register_DATAX1);   // Requiring Register DATA
  Wire.endTransmission(); // end Transmission
  Wire.requestFrom(SENSOR, 2);
  while (Wire.available() < 2) { //wait response
    DataReturned_x0 = Wire.read();
    DataReturned_x1 = Wire.read();
    DataReturned_x1 = DataReturned_x1 << 8;
    DataModified_x_out = DataReturned_x0 + DataReturned_x1;
    x = DataModified_x_out / 256.0;
  }
  // Prints the data on the Serial Monitor
  Serial.print("x = ");
  Serial.println(x);
  //poner en sleep
}

