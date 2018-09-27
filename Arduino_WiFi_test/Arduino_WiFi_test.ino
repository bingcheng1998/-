#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif
// ================================================================
// ===                   You can change here                    ===
// ================================================================
//#define OUTPUT_SERIALR           //串口调试输出
//#define DEBUG
#define OUTPUT_PROCESSING                            //UDP高级防损失打包发送
#define WiFiHOTSPOTS                            //是否自身发送热点
//#define TIME_COUNTER                           //Count the times of reading from all the mpu6050s
//#define SERIAL_START                           //正式使用UDP需要注释掉这条, 仅限调试使用
#define SerialPort 115200
#define mpuNum 2                               //mpuNum is the number of elements inside of bootPin[mpuNum]
#define mpuNum1 6
static int bootPin[mpuNum] = {4, 5};  //Available ports are 4 ,5, 9, 10, 12, 13, 14, 15.
//Run IMU_Zero at Example-MPU6050 and you can get these numbers:
static int XGyroOffset[mpuNum1] = {48, 198, 53, 102, 16, 17};
static int YGyroOffset[mpuNum1] = { 49, 79, 1, -12, 61, 63};
static int ZGyroOffset[mpuNum1] = { 11, 16, -129, 15, 121, 116};
static int ZAccelOffset[mpuNum1] = {1283, 577, 924, 745, 2007, 2583};
unsigned int localPort = 8266;
const char WiFiAPPSW[] = "12345678";            // Your password of Wifi hotspots
const char* ssid     = "bingcheng";             // The SSID of your router
const char* password = "963852741";         // The passWd of your router
// ================================================================
// ===                   Do NOT change below                    ===
// ================================================================
WiFiUDP Udp;
char packetBuffer[255]; //buffer to hold incoming packet
char  ReplyBuffer[] = "I received n";
//                                                                    循环  MPU序号
uint8_t teapotPacket[15] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, 0x00, '\r', '\n' };

IPAddress remoteIp;
int remotePORT;
// AD0 low = 0x68 (default)
// AD0 high = 0x69
MPU6050 mpu[mpuNum];
//bool mpuInitSuccess[mpuNum];
#define LED_PIN 16
bool blinkState = false;

// MPU control/status vars
bool dmpReady[mpuNum];  // set true if DMP init was successful
uint8_t mpuIntStatus[mpuNum];   // holds actual interrupt status byte from MPU
uint8_t devStatus[mpuNum];      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize[mpuNum];    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount[mpuNum];     // count of all bytes currently in FIFO
uint8_t fifoBuffer[mpuNum][64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// ================================================================
// ===                      BOOT                                ===
// ================================================================

void boot(int pin) {
  for (int i = 0; i < mpuNum; ++i)
  {
    digitalWrite(bootPin[i], HIGH);
  }
  digitalWrite(pin, LOW);
}

#ifdef TIME_COUNTER
int iCount = 0;
#endif

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin(0, 2);
  Wire.setClock(400000); 
  // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif
  // configure LED for output
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, false);
  for (int i = 0; i < mpuNum; ++i)
  {
    pinMode(bootPin[i], OUTPUT);
  }
  int bootSuccess = 0;
  Serial.begin(SerialPort);
  while (!Serial);
  delay(500);
  Serial.println();
  for (int i = 0; i < mpuNum; i++) {
    boot(bootPin[i]);
    delay(200);
    // initialize device
    Serial.print(F("Initializing MPU6050 ic = "));
    Serial.println(i);
    mpu[i].initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu[i].testConnection() ? F("MPU6050 connection --------successful") : F("MPU6050 connection --------failed"));
    Serial.println(F("Initializing DMP..."));
    devStatus[i] = mpu[i].dmpInitialize();

    mpu[i].setXGyroOffset(XGyroOffset[i]);
    mpu[i].setYGyroOffset(YGyroOffset[i]);
    mpu[i].setZGyroOffset(ZGyroOffset[i]);
    mpu[i].setZAccelOffset(ZAccelOffset[i]);
    if (devStatus[i] == 0) {
      // turn on the DMP, now that it's ready
      Serial.println(F("Enabling DMP..."));
      mpu[i].setDMPEnabled(true);
      mpuIntStatus[i] = mpu[i].getIntStatus();
      dmpReady[i] = true;
      packetSize[i] = mpu[i].dmpGetFIFOPacketSize();
      bootSuccess++;
    } else {
      // ERROR!
      // 1 = initial memory load failed
      // 2 = DMP configuration updates failed
      // (if it's going to break, usually the code will be 1)
      Serial.print(F("DMP Initialization failed (code "));
      Serial.print(devStatus[i]);
      Serial.println(F(")"));
    }
  }
  //————————————————————————————Test the MPU 6050 is connected successfully————————————————————————————//
#ifdef OUTPUT_SERIALR
  Serial.print(F("["));
  for (int i = 0; i < mpuNum; ++i)
  {
    Serial.print("\tP-");
    Serial.print(i);
  }
  Serial.println(F("\t]"));

  Serial.print(F("[\t"));
  for (int i = 0; i < mpuNum; ++i)
  {
    if (devStatus[i] == 0) {
      Serial.print(" o\t");
    } else {
      Serial.print(" x\t");
    }
  }
  Serial.println(F("]"));

  if (bootSuccess == mpuNum) {
    Serial.println(F("Congratulations!\tAll of the MPU6050 chips are successfully connected and work well"));
  } else {
    Serial.println(String("Initialization error!\t" + String(mpuNum - bootSuccess) + " of the " + mpuNum + " MPU6050 chips are disconnected or broken"));
  }
#endif
  if (bootSuccess == mpuNum) {
    for (int j = 0; j < 6; j++) {
      blinkState = !blinkState;
      digitalWrite(LED_PIN, blinkState);
      delay(60);
    }
  } else {
    while (1) {
      blinkState = !blinkState;
      digitalWrite(LED_PIN, blinkState);
      delay(1000);
    }
  }
  //————————————————————-----————————init the WiFi connection-----————————————————————————————//
#ifndef  OUTPUT_SERIALR
  delay(100);

  Udp.begin(localPort);
#ifdef WiFiHOTSPOTS
  setupHotspots();
#endif
#ifndef WiFiHOTSPOTS
  WiFiSetup();
#endif
  setupUdp();
  delay(100);
#endif

#ifdef SERIAL_START
  Serial.println(F("\nSend any character to begin DMP: "));
  while (Serial.available() && Serial.read()); // empty buffer
  while (!Serial.available());                 // wait for data
  while (Serial.available() && Serial.read()); // empty buffer again
#endif
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {

  for (int i = 0; i < mpuNum; ++i)
  {
    if (!dmpReady[i]) return;
    // if programming failed, don't try to do anything
  }

  for (int i = 0; i < mpuNum; ++i)
  {
    boot(bootPin[i]);
    // reset interrupt flag and get INT_STATUS byte
    //mpuInterrupt = false;
    mpuIntStatus[i] = mpu[i].getIntStatus();

    // get current FIFO count
    fifoCount[i] = mpu[i].getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus[i] & 0x10) || fifoCount[i] == 1024) {
      // reset so we can continue cleanly
      mpu[i].resetFIFO();
      //Serial.println(F("FIFO0 overflow!"));
      Serial.print("ypr");
      Serial.print(i);
      Serial.print("\tFIFO\tover\tflow");
      // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus[i] & 0x02) {
      // wait for correct available data length, should be a VERY short wait

      while (fifoCount[i] < packetSize[i]) fifoCount[i] = mpu[i].getFIFOCount();

      // read a packet from FIFO
      mpu[i].getFIFOBytes(fifoBuffer[i], packetSize[i]);

      // track FIFO count here in case there is > 1 packet available
      // (this lets us immediately read more without waiting for an interrupt)
      fifoCount[i] -= packetSize[i];

#ifdef OUTPUT_SERIALR
      // display Euler angles in degrees
      mpu[i].dmpGetQuaternion(&q, fifoBuffer[i]);
      mpu[i].dmpGetGravity(&gravity, &q);
      mpu[i].dmpGetYawPitchRoll(ypr, &q, &gravity);
      Serial.print("ypr");
      Serial.print(i);
      Serial.print("\t");
      Serial.print(ypr[0] * 180 / M_PI);
      Serial.print("\t");
      Serial.print(ypr[1] * 180 / M_PI);
      Serial.print("\t");
      Serial.print(ypr[2] * 180 / M_PI);
#endif

#ifdef OUTPUT_PROCESSING
      Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
      // display quaternion values in InvenSense Teapot demo format:
      teapotPacket[2] = fifoBuffer[i][0];
      teapotPacket[3] = fifoBuffer[i][1];
      teapotPacket[4] = fifoBuffer[i][4];
      teapotPacket[5] = fifoBuffer[i][5];
      teapotPacket[6] = fifoBuffer[i][8];
      teapotPacket[7] = fifoBuffer[i][9];
      teapotPacket[8] = fifoBuffer[i][12];
      teapotPacket[9] = fifoBuffer[i][13];
      teapotPacket[12] = i;
      //Serial.write(teapotPacket, 14);
      for (int j = 0; j < 15; j++) {
        Udp.write(teapotPacket[j]);
      }
      teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
      Udp.endPacket();
#endif
      // blink LED to indicate activity
      blinkState = !blinkState;
      digitalWrite(LED_PIN, blinkState);

    } else {
      Serial.print("ypr");
      Serial.print(i);
      Serial.print("\t    \t    \t    ");
    }
    Serial.print("\t");
#ifdef TIME_COUNTER
    iCount ++;
#endif
  }
#ifdef TIME_COUNTER
  int millisCounter = millis();
  Serial.println(String("Count:\t" + String(iCount) + "\tmillis:\t" + String(millisCounter)));
#endif
#ifndef TIME_COUNTER
  Serial.println();
#endif
}


// ================================================================
// ===                     setup Hotspots                       ===
// ================================================================



void setupHotspots() {
  WiFi.mode(WIFI_AP);
  uint8_t mac[WL_MAC_ADDR_LENGTH];
  WiFi.softAPmacAddress(mac);

  String macID = String(mac[WL_MAC_ADDR_LENGTH - 3], HEX) +
                 String(mac[WL_MAC_ADDR_LENGTH - 2], HEX) +
                 String(mac[WL_MAC_ADDR_LENGTH - 1], HEX);
  macID.toUpperCase();

  String AP_NameString = "MPU6050" + macID;
  Serial.println("WiFi Host Name: " + AP_NameString);

  char AP_NameChar[AP_NameString.length() + 1];
  memset(AP_NameChar, AP_NameString.length() + 1, 0);

  for (int i = 0; i < AP_NameString.length(); i++)
    AP_NameChar[i] = AP_NameString.charAt(i);

  WiFi.softAP(AP_NameChar, WiFiAPPSW);

  Serial.println();
  Serial.print ( "IP address: " );
  Serial.println ( WiFi.softAPIP() );
}

// ================================================================
// ===                  setup Wifi connection                   ===
// ================================================================

void WiFiSetup() {
#ifdef DEBUG
  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
#endif
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
#ifdef DEBUG
    Serial.print(".");
#endif
  }

#ifdef DEBUG
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
#endif
}


// ================================================================
// ===                       setup Udp                          ===
// ================================================================


void setupUdp() {
  bool udpSetup = 0;
  Serial.println("UDP connecting...");
  while (!udpSetup) {
    int packetSize = Udp.parsePacket();
    if (packetSize) {
      Serial.println("UDP connected!!!");
      Serial.print("Received packet of size ");
      Serial.println(packetSize);
      Serial.print("From ");
      IPAddress remoteIp = Udp.remoteIP();
      Serial.print(remoteIp);
      Serial.print(", port ");
      Serial.println(Udp.remotePort());
      // read the packet into packetBufffer
      int len = Udp.read(packetBuffer, 255);
      if (len > 0) {
        packetBuffer[len] = 0;
      }
      Serial.println("Contents:");
      Serial.println(packetBuffer);
      udpSetup = !udpSetup;
    }
  }
}


