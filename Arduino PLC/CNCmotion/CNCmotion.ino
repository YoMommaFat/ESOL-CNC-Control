#include <SPI.h>
#include <Ethernet.h>
#include <ArduinoJson.h>        // Arduino IDE-ből "ArduinoJson by Benoit Blanchon"
#include <AccelStepper.h>       // https://www.airspayce.com/mikem/arduino/AccelStepper/classAccelStepper.html
//#include <Arduino_FreeRTOS.h>   // Arduino IDE-ből "FreeRTOS by Richard Barry"

// Ratio: X, Y: 2,5 mm/rev, Z: 0.4 mm/rev (5/2, 4/10)
// 2.5 mm = 1,280,000, 1 mm = 512,000  /
// 0.4 mm = 1,280,000, 1 mm = 3,200,000
// 1280000/16000=80, 200/80=2.5, 4/10=2.5
// 16000/24*30=20000, 16000/30*24=12800 (Max: 25600)

// Define IOs
#define STP_X Q0_0
#define DIR_X Q0_1
#define SON_X Q0_2
#define STP_Y Q0_3
#define DIR_Y Q0_4
#define SON_Y Q1_0
#define STP_Z Q1_1
#define DIR_Z Q1_2
#define SON_Z Q1_3
#define END_X I0_0
#define END_Y I0_1
#define END_Z I0_2
#define RDY_X I1_0
#define RDY_Y I1_1
#define RDY_Z I1_2

#define COIL_ON Q1_4
#define COIL_OK I1_3
#define END_IND I0_3
#define END_OPT I0_4

// Define states
#define READY     1     // Ready
#define RUNNING   2     // Running
#define HOMING    3     // Homing
#define STOP      4     // Stopping
#define E_STOP    5     // Emergency stop
#define PINGG     6     // Ping

// Define axes
#define X         0
#define Y         1
#define Z         2
#define N         3

byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
IPAddress ip(10, 10, 0, 100);
IPAddress myDns(192, 168, 1, 1);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);
EthernetServer server(100);
char buf[100];

const int inEnd[3] = { END_X, END_Y, END_Z };
const int son[3] = { SON_X, SON_Y, SON_Z };
const int limitAxis[3] = { 200, 200, -300 }; // [mm]
const long positionHoming[3] = { 200, 200, 10 }; // [mm]
const long destinationHoming = 0; // [mm]
const int pulseWidth = 10; // [us]
const float speedDefault = 50; // [mm/s]
const float speedHoming = 10; // [mm/s]
const float accelDefault = 5; // [mm/s^2]
const float accelHoming = 10; // [mm/s^2]

unsigned long timePrevious = 0; // Heartbeat
unsigned long timePassed = 0; // Heartbeat
unsigned long timeHeartbeat = 0; // Heartbeat
unsigned long prevBlinkTime = 0; // Blink
unsigned long periodBlink = 1000; // Blink
unsigned long prevSendTime = 0; // Send
unsigned long periodSend = 500; // Send
//unsigned long prevSendTime0 = 100;
//unsigned long prevSendTime1 = 200;
//unsigned long prevSendTime2 = 300;

byte state = READY;
byte axisHoming = N;
byte enabled = false;
byte positioned = false;
byte blinkCon = false;
byte homed[4] = { false, false, false, false};

AccelStepper stepper[3] = { {AccelStepper::DRIVER, STP_X, DIR_X}, {AccelStepper::DRIVER, STP_Y, DIR_Y}, {AccelStepper::DRIVER, STP_Z, DIR_Z} };

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(COIL_ON, OUTPUT);
  Serial.begin(9600);
  while (!Serial) { ; }
  Serial.println("");
  Ethernet.init(10);
  Ethernet.begin(mac, ip);
//  Ethernet.begin(mac, ip, myDns, gateway, subnet);
  delay(1000);
  if (Ethernet.hardwareStatus() == EthernetNoHardware) { Serial.println("Ethernet error!"); while (true) { delay(1); } }
  if (Ethernet.linkStatus() == Unknown) { Serial.println("Ethernet link status unknown. Link status detection is only available with W5200 and W5500."); }
  else if (Ethernet.linkStatus() == LinkON) { Serial.println("Ethernet link status: On"); }
  else if (Ethernet.linkStatus() == LinkOFF) { Serial.println("Ethernet link status: Off"); }
  server.begin();
  Serial.print("IP address: "); Serial.println(Ethernet.localIP());
  for(int i = 0; i < 3; i++) {
    stepper[i].setEnablePin(son[i]);
    stepper[i].setPinsInverted(false, false, false);
    stepper[i].setAcceleration(accelDefault * 200);
    stepper[i].setMaxSpeed(speedDefault * 200);
    stepper[i].setSpeed(speedDefault * 200);
    stepper[i].setMinPulseWidth(pulseWidth);
    stepper[i].enableOutputs();
  }
  delay(1000);
  Serial.println("System ready!");
  sprintf(buf, "{\"cmd\":\"homed\",\"val\":[%d]}", axisHoming);
  server.print(buf);
}

void loop(void) {
  switch (state) {
    case READY: // Ready
      // Rest...
      break;
    case RUNNING: // Running
      if (stepper[X].isRunning() || stepper[Y].isRunning() || stepper[Z].isRunning()) {
        stepper[X].run(); stepper[Y].run(); stepper[Z].run();
      }
      else {
        homed[axisHoming] = true;
        if (homed[X] == true && homed[Y] == true && homed[Z] == true) { enabled = true; }
        else { enabled = false; }
        sprintf(buf, "{\"cmd\":\"homed\",\"val\":[%d]}", axisHoming);
        server.print(buf);
        axisHoming = N;
        positioned = true;
        state = READY;
      }
      break;
    case HOMING: // Homing
      stepper[axisHoming].setSpeed(speedHoming * 200);
      stepper[axisHoming].runSpeed();
      if (!digitalRead(inEnd[axisHoming])) { // If end switch pressed
        stepper[axisHoming].setAcceleration(accelHoming * 200); 
        stepper[axisHoming].stop();
        stepper[axisHoming].run();
        state = STOP;
      }
      break;
    case STOP: // Stopping
      stepper[axisHoming].run();
      if (!stepper[axisHoming].isRunning()) { // If stopped after end switch pressed
        stepper[axisHoming].setSpeed(speedDefault * 200);
        stepper[axisHoming].setAcceleration(accelDefault * 200);
        stepper[axisHoming].setCurrentPosition(positionHoming[axisHoming] * 200);
        stepper[axisHoming].moveTo(destinationHoming * 200);
        stepper[axisHoming].run();
        state = RUNNING;
        Serial.print("Axis homed: "); Serial.println(axisHoming);
      }
      break;
    case PINGG: // Ping
      Serial.print("Ping!\n");
      break;
    case E_STOP: // Emergency stop
      stepper[X].run(); stepper[Y].run(); stepper[Z].run();
      break;
    default:
      Serial.print("Unknown State!\n");
      break;
  }
 
  #ifdef HEARTBEAT_ENABLED
    timePassed = millis() - timePrevious;
    timePrevious = millis();
    if (timeHeartbeat >= periodHeartbeat) {
//      stepperLeft.stop();
//      stepperRight.stop();
      state = E_STOP;
      }
    else { timeHeartbeat += timePassed; }
  #endif

  processCommand();
  sendAllData();
  blinkLED();
}

void processCommand() { // TASK: fogadja a json csomagokat
  int axis;
  long dist, dist2;
  String command;
  StaticJsonDocument<100> doc; // Allocate the JSON document
  EthernetClient client = server.available(); // Wait for a new client, returns first client which has data to read or a 'false' client
  
/*  if (client) { // client is true only if it is connected and has data to read
    String s = client.readStringUntil('\n'); // read the message incoming from one of the clients
    s.trim(); // trim eventual \r
    Serial.println(s); // print the message to Serial Monitor
    client.print("echo: "); // this is only for the sending client
    server.println(s); // send the message to all connected clients
  }*/

//  {\"sen\":\"gps\",\"time\":13,\"data\":48.7}
//  192.168.1.100:100
//  {\"cmd\":\"home\",\"val\":[2]}
//  {\"cmd\":\"move\",\"val\":[0,1000]}

  if (client) { // Check if the sender is transmitting
    DeserializationError err = deserializeJson(doc, client); // Read the JSON document
    command = doc["cmd"].as<String>();
    if (err == DeserializationError::Ok) {
      Serial.print("cmd = ");
      Serial.print(command);
      if (command == "move") {
        axis = doc["val"][0].as<int>();
        dist = doc["val"][1].as<int>();
        Serial.print(", axis = "); Serial.print(axis);
        Serial.print(", dist = "); Serial.println(dist);
        if (axis < X || axis > Z) { Serial.println("Wrong axis."); }
        else if (axis == X  && abs(dist) > limitAxis[X]) { Serial.println("Wrong X axis range."); }
        else if (axis == Y  && abs(dist) > limitAxis[Y]) { Serial.println("Wrong Y axis range."); }
        else if (axis == Z  && (dist > 0 || dist < limitAxis[Z])) { Serial.println("Wrong Z axis range."); }
        else { 
          if (enabled) { stepper[axis].moveTo(dist * 200); state = RUNNING; Serial.println("Moving axis."); }
          else { Serial.println("Movement disabled, try homing first."); }
        }
      }
      else if (command == "moveXY") {
        dist = doc["val"][0].as<int>();
        dist2 = doc["val"][1].as<int>();
        Serial.print(", distX = "); Serial.print(dist);
        Serial.print(", distY = "); Serial.println(dist2);
        if (abs(dist) > limitAxis[X]) { Serial.println("Wrong X axis range."); }
        else if (abs(dist2) > limitAxis[Y]) { Serial.println("Wrong Y axis range."); }
        else { 
          if (enabled) { stepper[X].moveTo(dist * 200); stepper[Y].moveTo(dist2 * 200); state = RUNNING; positioned = false; Serial.println("Moving axes."); }
          else { Serial.println("Movement disabled, try homing first."); }
        }
      }
      else if (command == "moveZ") {
        dist = doc["val"][0].as<int>();
        Serial.print(", distZ = "); Serial.print(dist);
        if (dist > 0 || dist < limitAxis[Z]) { Serial.println("Wrong Z axis range."); }
        else { 
          if (enabled) { stepper[Z].moveTo(dist * 200); state = RUNNING; positioned = false; Serial.println("Moving axis."); }
          else { Serial.println("Movement disabled, try homing first."); }
        }
      }
      else if (command == "home") {
        axis = doc["val"][0].as<int>();
        Serial.print(", axis = "); Serial.println(axis);
        if (axis < X || axis > Z) { Serial.println("Wrong axis."); }
        else { axisHoming = axis; state = HOMING; Serial.println("Homing axis."); }
      }
      else if (command == "enable") {
        axis = doc["val"][0].as<int>();
        Serial.print(", state = "); Serial.println(axis);
        if (axis == 0) { for(int i = 0; i < 3; i++) { homed[i] = false; }; enabled = false; Serial.println("Movement disabled."); }
        else if (axis == 1) { enabled = true; Serial.println("Movement enabled."); }
        else { Serial.println("Wrong enable state."); }
      }
      else if (command == "coil") {
        axis = doc["val"][0].as<int>();
        Serial.print(", state = "); Serial.println(axis);
        if (axis == 0) { digitalWrite(COIL_ON, LOW); Serial.println("Charging disabled."); }
        else if (axis == 1) { digitalWrite(COIL_ON, HIGH); Serial.println("Charging enabled."); }
        else { Serial.println("Wrong charging state."); }
      }
      else if (command == "estop") {
        axis = doc["val"][0].as<int>();
        Serial.print(", state = "); Serial.println(axis);
        if (axis == 1) {
          digitalWrite(COIL_ON, LOW); Serial.println("Charging disabled.");
          for(int i = 0; i < 3; i++) {
            stepper[i].setAcceleration(accelHoming * 200);
            stepper[i].stop();
            stepper[i].run();
            homed[i] = false;
          }
          enabled = false; positioned = false;
          state = E_STOP;
          Serial.println("Emergency stop activated.");
        }
        else if (axis == 0) {
          for(int i = 0; i < 3; i++) {
            stepper[i].setAcceleration(accelDefault * 200);
            stepper[i].stop();
            stepper[i].run();
          }
//          state = RUNNING;
          Serial.println("Emergency stop released.");
        }
        else { Serial.println("Wrong state."); }
      }
      else if (command == "heartbeat") {
        axis = doc["val"][0].as<int>();
        Serial.print(", value = "); Serial.println(axis);
        timeHeartbeat = 0;
        Serial.println("Heartbeat received.");
      }
      else {
        Serial.println("Unknown command received!");
      }
    }
    else {
      Serial.print("deserializeJson() returned "); // Print error to the "debug" serial port
      Serial.println(err.c_str());
    }
  }
}

void blinkLED() {
  if (millis() - prevBlinkTime >= periodBlink) {
    prevBlinkTime += periodBlink;
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
}

/*
void sendData() {
    sendServoDriveData();
    sendEndSwitchData();
    sendHomingStateData();
}

void sendServoDriveData() {
  if (millis() - prevSendTime0 >= periodSend) {
    prevSendTime0 += periodSend;
    int intRDY_X = digitalRead(RDY_X);
    int intRDY_Y = digitalRead(RDY_Y);
    int intRDY_Z = digitalRead(RDY_Z);
    sprintf(buf, "{\"cmd\":\"servoDriveData\",\"val\":[%d,%d,%d]}", intRDY_X, intRDY_Y, intRDY_Z);
    int len = server.print(buf);
  }
//  Serial.print("Servo drive data sent. Size in bytes: "); Serial.println(len);
}

void sendEndSwitchData() {
  if (millis() - prevSendTime1 >= periodSend) {
    prevSendTime1 += periodSend;
    int intEND_X = digitalRead(END_X);
    int intEND_Y = digitalRead(END_Y);
    int intEND_Z = digitalRead(END_Z);
    sprintf(buf, "{\"cmd\":\"endSwitchData\",\"val\":[%d,%d,%d]}", intEND_X, intEND_Y, intEND_Z);
    int len = server.print(buf);
  }
//  Serial.print("End switch data sent. Size in bytes: "); Serial.println(len);
}

void sendHomingStateData() {
  if (millis() - prevSendTime2 >= periodSend) {
    prevSendTime2 += periodSend;
    int homed_X = 0;
    int homed_Y = 0;
    int homed_Z = 0;
    sprintf(buf, "{\"cmd\":\"homingStateData\",\"val\":[%d,%d,%d]}", homed_X, homed_Y, homed_Z);
    int len = server.print(buf);
  }
//  Serial.print("Homing state data sent. Size in bytes: "); Serial.println(len);
}
*/
void sendAllData() {
  if (millis() - prevSendTime >= periodSend) {
    prevSendTime += periodSend;
    blinkCon = !blinkCon;
    int intRDY_X = digitalRead(RDY_X);
    int intRDY_Y = digitalRead(RDY_Y);
    int intRDY_Z = digitalRead(RDY_Z);
    int intEND_X = digitalRead(END_X);
    int intEND_Y = digitalRead(END_Y);
    int intEND_Z = digitalRead(END_Z);
    int charg_ON = digitalRead(COIL_ON);
    int charg_OK = digitalRead(COIL_OK);
    int vert_IND = digitalRead(END_IND);
    int vert_OPT = digitalRead(END_OPT);
    sprintf(buf, "{\"cmd\":\"allData\",\"val\":[%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d]}",
            intRDY_X, intRDY_Y, intRDY_Z, intEND_X, intEND_Y, intEND_Z, homed[X], homed[Y], homed[Z],
            enabled, positioned, charg_ON, blinkCon, charg_OK, vert_IND, vert_OPT);
    server.print(buf);
  } 
}
