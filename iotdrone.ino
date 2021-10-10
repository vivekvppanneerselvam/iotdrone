#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <BlynkSimpleEsp8266.h>
#include <Wire.h>

// For MPU-6050
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

#ifndef APSSID
#define APSSID "VIVEK"
#define APPSK  "9840271370"
#endif
WidgetLCD lcd(V1);
const char *ssid = APSSID;
const char *password = APPSK;
char auth[] = "Ecwbk-hxKME1p5O7abvXmaBxImKeeiFE";

// Pin declaration
#define SCL D1  //D6
#define SDA D2  //D5
#define INTERRUPT_PIN D5  //D1

/*------------------------------------------------Global Var--------------------------------------------------------*/
ESP8266WebServer server(80);
MPU6050 mpu;
MPU6050 mpu2(0x69); // <-- use for AD0 high

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vectorC:\Users\vivek\OneDrive\Documents\Arduino
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

int pitch_roll[] = { 0, 0}; int throttle_yaw[] = { 0, 0};
int pitchVal[] = {0, 0, 0, 0}; int rollVal[] = {0, 0, 0, 0}; int yawVal[] = {0, 0, 0, 0}; int throttleVal[] = {0, 0, 0, 0}; int calibVal[] = {0, 0, 0, 0};
int yawAngle = 0; int pitchAngle = 0; int rollAngle = 0; int resetVal = 0;


void setup() {

  Serial.begin(115200);
  lcd.clear();
  lcd.print(4, 0, "Drone");
  lcd.print(4, 1, "online");
  pinMode(INTERRUPT_PIN, INPUT);
  Blynk.begin(auth, ssid, password); // Connect to Blynk App

  do {
    Wire.begin(SDA, SCL); //Wire.begin (<SDA>,<SCL>)
    //Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    Serial.println(F("Initializing Gyroscope and Accelerometer..."));
    mpu.initialize();    
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
    Serial.println(mpu2.testConnection() ? F("MPU6050-2 connection successful") : F("MPU6050-2 connection failed"));
    Serial.println(F("\tInitializing DMP..."));
    devStatus = mpu.dmpInitialize();
    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(0);  // Default 220. Use 0 (value confirmed)
    mpu.setYGyroOffset(0 );   // Default 76. Use 0 (value confirmed)
    mpu.setZGyroOffset(0);  // Default -85. Use 0 (value confirmed)
    mpu.setZAccelOffset(1024); // Default 1788. Use 1024 (value confirmed)
    delay (1);
  }
  while (devStatus != 0);

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    dmpReady = true;  

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("\tDMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
  Serial.println(F("Gyroscope and Accelerometer --------------------------- OK"));


}

void loop() {
  Blynk.run();

  // Protection from unintetional disconnect from server. Attempt to reconnect after that.
  if (!Blynk.connected()) {
    Serial.println ("\nDisconnected from Blynk App");
    Serial.println ("Attempting to reconnect...");
    Blynk.begin(auth, ssid, password);
    delay (1);
  }

  // Read some Gyroscope values here....

  delay (1);
}


/**********************************************************************************************INTERRUPT DETECTION ROUTINE*********************************
   Interrupt pin for MPU6050
*/
void dmpDataReady() {
  mpuInterrupt = true;
}


/**********************************************************************************************************PITCH/ROLL joystick*********************************
   Pitch/Roll Joystick
*/
BLYNK_WRITE(V20) {
  int x = param[0].asInt();
  int y = param[1].asInt();
  (y < 384 || y > 640) ? pitch_roll[1] = map(y, 0, 1023, -20, 20) : pitch_roll[1] = 0; // Pitch
  (x < 384 || x > 640) ? pitch_roll[2] = map(x, 0, 1023, -20, 20) : pitch_roll[2] = 0; //Roll
}

/**********************************************************************************************************Throttle/Yaw joystick*********************************
   Throttle/Yaw Joystick
*/
BLYNK_WRITE(V21) {
  int x = param[0].asInt();
  int y = param[1].asInt();
  (y > 100) ? throttle_yaw[1] = map(y, 0, 1023, 0, 70) : throttle_yaw[1] = 0; //Throttle. y > 100 to prevent motor burnout under for low PWM values. Default: map(y, 0, 1023, 0, 70)
  (x < 312 || x > 712) ? throttle_yaw[2] = map(x, 0, 1023, -35, 35) : throttle_yaw[2] = 0; //Yaw. Default: map(x, 0, 1023, -35, 35)
}

BLYNK_CONNECTED() {
  Blynk.syncVirtual(V20, V21, V22);  // Sync values
}

/**********************************************************************************************************MANUAL RESET*********************************
   Reset button on app
*/
BLYNK_WRITE(V22) {
  resetVal = param.asInt();
}
