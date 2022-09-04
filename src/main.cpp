  /*Use this web gui to control. The commands are forwarded to the hoverboard and responses are forwareded too.
  https://candas1.github.io/Hoverboard-Web-Serial-Control/
  Make sure to set the the log to "Binary" and connect to the com port of the Arduino!
  Switch to Control and set the mode to Uart. Then the tripele beeps should stop.
  */

#include <Arduino.h>

//#define USE_BLE

//ble stuff
//for some reason we cannot ifdef this because pio somehow automatically installs the ble dependencies and 
//the code will not work without these includes, we would need to completely remove the ble includes.
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#ifdef USE_BLE


// #define SERVICE_UUID           "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define SERVICE_UUID           "FFE0"
// #define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "FFE2"
// #define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "FFE1"

#endif

//servo input to allow rc remote input (currently not working)
#include <ServoInput.h>

//CAN module to inerface with ebike battery
// MCP2515 - ESP32
//    SCK  - IO18
//    SI   - IO23
//    SO   - IO19
//    CS   - IO5
//    Vcc  - 5V ! ESP32 inputs are 5V tolerant
//    GND  - GND
//
// http://henrysbench.capnfatz.com/henrys-bench/arduino-projects-tips-and-more/arduino-can-bus-module-1st-network-tutorial/
// CAN Receive Example der Bibliothek https://github.com/coryjfowler/MCP_CAN_lib
#include <mcp_can.h>
#include <SPI.h>
#include "SimpleTimer.h"

MCP_CAN CAN0(5);                // Set CS pin
char msgString[128];            // Serial Output String Buffer
// timer for sending the enable battery command
SimpleTimer can_enable_battery_timer;

//CAN module to inerface with ebike battery


// *******************************************************************
//  Arduino Nano 5V example code
//  for   https://github.com/EmanuelFeru/hoverboard-firmware-hack-FOC
//
//  Copyright (C) 2019-2020 Emanuel FERU <aerdronix@gmail.com>
//
// *******************************************************************
// INFO:
// • This sketch uses the the Serial Software interface to communicate and send commands to the hoverboard
// • The built-in (HW) Serial interface is used for debugging and visualization. In case the debugging is not needed,
//   it is recommended to use the built-in Serial interface for full speed perfomace.
// • The data packaging includes a Start Frame, checksum, and re-syncronization capability for reliable communication
//
// The code starts with zero speed and moves towards +
//
// CONFIGURATION on the hoverboard side in config.h:
// • Option 1: Serial on Right Sensor cable (short wired cable) - recommended, since the USART3 pins are 5V tolerant.
//   #define CONTROL_SERIAL_USART3
//   #define FEEDBACK_SERIAL_USART3
//   // #define DEBUG_SERIAL_USART3
// • Option 2: Serial on Left Sensor cable (long wired cable) - use only with 3.3V devices! The USART2 pins are not 5V tolerant!
//   #define CONTROL_SERIAL_USART2
//   #define FEEDBACK_SERIAL_USART2
//   // #define DEBUG_SERIAL_USART2
// *******************************************************************

// ########################## DEFINES ##########################
#define HOVER_SERIAL_BAUD   115200      // [-] Baud rate for HoverSerial (used to communicate with the hoverboard)
#define SERIAL_BAUD         115200      // [-] Baud rate for built-in Serial (used for the Serial Monitor)
#define START_FRAME         0xABCD     	// [-] Start frme definition for reliable serial communication
#define TIME_SEND           100         // [ms] Sending time interval
#define SPEED_MAX_TEST      300         // [-] Maximum speed for testing
#define SPEED_STEP          20          // [-] Speed step
// #define DEBUG_RX                        // [-] Debug received data. Prints all bytes to serial (comment-out to disable)
#include <HardwareSerial.h>
HardwareSerial HoverSerial(2);        // RX, TX

// Global variables
uint8_t idx = 0;                        // Index for new data pointer
uint16_t bufStartFrame;                 // Buffer Start Frame
byte *p;                                // Pointer declaration for the new received data
byte incomingByte;
byte incomingBytePrev;

typedef struct{
   uint16_t start;
   int16_t  steer;
   int16_t  speed;
   uint16_t checksum;
} SerialCommand;
SerialCommand Command;

typedef struct{
   uint16_t start;
   int16_t  cmd1;
   int16_t  cmd2;
   int16_t  speedR_meas;
   int16_t  speedL_meas;
   int16_t  batVoltage;
   int16_t  boardTemp;
   uint16_t cmdLed;
   uint16_t checksum;
} SerialFeedback;
SerialFeedback Feedback;
SerialFeedback NewFeedback;


//RC input signal setup
// Steering Setup
const int SteeringSignalPin = 32;  // MUST be interrupt-capable!
const int SteeringPulseMin = 1277;  // microseconds (us)
const int SteeringPulseMax = 1720;  // Ideal values for your servo can be found with the "Calibration" example

ServoInputPin<SteeringSignalPin> steering(SteeringPulseMin, SteeringPulseMax);

//Throttle Setup
const int ThrottleSignalPin = 35;  // MUST be interrupt-capable!
const int ThrottlePulseMin = 1196;  // microseconds (us)
const int ThrottlePulseMax = 1807;  // Ideal values for your servo can be found with the "Calibration" example

ServoInputPin<ThrottleSignalPin> throttle(ThrottlePulseMin, ThrottlePulseMax);


//ble stuff
#ifdef USE_BLE
BLECharacteristic *pCharacteristic_tx;
bool deviceConnected;

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string rxValue = pCharacteristic->getValue();

      Serial.print("Received via ble:");
      Serial.println(rxValue.c_str());

      HoverSerial.write(rxValue.c_str());
    }
};


// ########################## SETUP ##########################

void setup_btle(){

  BLEDevice::init("BT05");
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  pCharacteristic_tx = pService->createCharacteristic(
                        CHARACTERISTIC_UUID_TX,
                        BLECharacteristic::PROPERTY_NOTIFY
                      );

  pCharacteristic_tx->addDescriptor(new BLE2902());

  BLECharacteristic *pCharacteristic_rx = pService->createCharacteristic(
                                          CHARACTERISTIC_UUID_RX,
                                          BLECharacteristic::PROPERTY_WRITE
                                        );

  pCharacteristic_rx->setCallbacks(new MyCallbacks());

  pService->start();

  pServer->getAdvertising()->start();
  Serial.println("Started bluetooth le service.");

}

#endif

void can_send_enable_battery(){
    byte data[4] = {0x04, 0x00, 0x00, 0x00};

    // send data:  ID = 0x09a, Standard CAN Frame, Data length =  bytes, 'data' = array of data bytes to send
    byte sndStat = CAN0.sendMsgBuf(0x09a, 0, 4, data);
    if(sndStat == CAN_OK){
      //Serial.println("Message Sent Successfully!");
    } else {
      Serial.println("Error Sending enablet battery CAN message...");
    }
  }

void setup_can()
{
  while (CAN_OK != CAN0.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ))  // init can bus : masks and filters disabled, baudrate, Quarz vom MCP2551
  {
    Serial.println("CAN BUS Shield init fail");
    Serial.println(" Init CAN BUS Shield again");
    delay(100);
  }
  Serial.println("CAN BUS Shield init ok!");
  CAN0.setMode(MCP_NORMAL);   // Change to normal mode to allow messages to be transmitted

  can_enable_battery_timer.setInterval(1000, can_send_enable_battery);
}

void setup_rc_servo_input()
{
  //attach interupts to servoinput lib as a workaround, as the internal interrupt setup doesn't work.
  pinMode(SteeringSignalPin, INPUT_PULLUP);
  pinMode(ThrottleSignalPin, INPUT_PULLUP);

	//attach interupts to servoinput lib as a workaround, as the internal interrupt setup doesn't work.
  attachInterrupt(digitalPinToInterrupt(SteeringSignalPin), steering.isr, CHANGE);
	attachInterrupt(digitalPinToInterrupt(ThrottleSignalPin), throttle.isr, CHANGE);
}

void setup()
{
  Serial.begin(SERIAL_BAUD);
  Serial.println("Hoverboard Serial v1.0");

  HoverSerial.begin(HOVER_SERIAL_BAUD);
  pinMode(LED_BUILTIN, OUTPUT);

  #ifdef USE_BLE
  setup_btle();
  #endif

  setup_can();
  setup_rc_servo_input();
}

// ########################## SEND ##########################
void Send(int16_t uSteer, int16_t uSpeed)
{
  // Create command
  Command.start    = (uint16_t)START_FRAME;
  Command.steer    = (int16_t)uSteer;
  Command.speed    = (int16_t)uSpeed;
  Command.checksum = (uint16_t)(Command.start ^ Command.steer ^ Command.speed);

  // Write to Serial
  HoverSerial.write((uint8_t *) &Command, sizeof(Command));
}

// ########################## RECEIVE ##########################
void Receive()
{
    // Check for new data availability in the Serial buffer
    if (HoverSerial.available()) {
        incomingByte 	  = HoverSerial.read();                                   // Read the incoming byte
        bufStartFrame	= ((uint16_t)(incomingByte) << 8) | incomingBytePrev;       // Construct the start frame
    }
    else {
        return;
    }

  // If DEBUG_RX is defined print all incoming bytes
  #ifdef DEBUG_RX
        Serial.print(incomingByte);
        return;
    #endif

    // Copy received data
    if (bufStartFrame == START_FRAME) {	                    // Initialize if new data is detected
        p       = (byte *)&NewFeedback;
        *p++    = incomingBytePrev;
        *p++    = incomingByte;
        idx     = 2;
    } else if (idx >= 2 && idx < sizeof(SerialFeedback)) {  // Save the new received data
        *p++    = incomingByte;
        idx++;
    }

    // Check if we reached the end of the package
    if (idx == sizeof(SerialFeedback)) {
        uint16_t checksum;
        checksum = (uint16_t)(NewFeedback.start ^ NewFeedback.cmd1 ^ NewFeedback.cmd2 ^ NewFeedback.speedR_meas ^ NewFeedback.speedL_meas
                            ^ NewFeedback.batVoltage ^ NewFeedback.boardTemp ^ NewFeedback.cmdLed);

        // Check validity of the new data
        if (NewFeedback.start == START_FRAME && checksum == NewFeedback.checksum) {
            // Copy the new data
            memcpy(&Feedback, &NewFeedback, sizeof(SerialFeedback));

            // Print data to built-in Serial
            Serial.print("1: ");   Serial.print(Feedback.cmd1);
            Serial.print(" 2: ");  Serial.print(Feedback.cmd2);
            Serial.print(" 3: ");  Serial.print(Feedback.speedR_meas);
            Serial.print(" 4: ");  Serial.print(Feedback.speedL_meas);
            Serial.print(" 5: ");  Serial.print(Feedback.batVoltage);
            Serial.print(" 6: ");  Serial.print(Feedback.boardTemp);
            Serial.print(" 7: ");  Serial.println(Feedback.cmdLed);
        } else {
          Serial.println("Non-valid data skipped");
        }
        idx = 0;    // Reset the index (it prevents to enter in this if condition in the next cycle)
    }

    // Update previous states
    incomingBytePrev = incomingByte;
}

void can_receive(){

  unsigned long rxId = 0;
  byte len = 0;
  byte rxBuf[8];

  if (CAN_MSGAVAIL == CAN0.checkReceive())           // check if data coming
  {
    CAN0.readMsgBuf(&rxId, &len, rxBuf);      // Read data: len = data length, buf = data byte(s)
    Serial.println("-----------------------------");
    if ((rxId & 0x80000000) == 0x80000000)            // Determine if ID is standard (11 bits) or extended (29 bits)
      sprintf(msgString, "Extended ID: 0x%.8lX  DLC: %1d  Data:", (rxId & 0x1FFFFFFF), len);
    else
      sprintf(msgString, "Standard ID: 0x%.3lX       DLC: %1d  Data:", rxId, len);

    Serial.print(msgString);

    if ((rxId & 0x40000000) == 0x40000000) {          // Determine if message is a remote request frame.
      sprintf(msgString, " REMOTE REQUEST FRAME");
      Serial.print(msgString);
    } else {
      for (byte i = 0; i < len; i++) {
        sprintf(msgString, " 0x%.2X", rxBuf[i]);
        Serial.print(msgString);
      }
    }
    Serial.println();
  }
}


// ########################## LOOP ##########################

unsigned long iTimeSend = 0;
int iTest = 0;
int iStep = SPEED_STEP;

void send_motor_test_command(){
  unsigned long timeNow = millis();
  // Send commands
  if (iTimeSend > timeNow) return;
  iTimeSend = timeNow + TIME_SEND;
  Send(0, iTest);

  // Calculate test command signal
  iTest += iStep;

  // invert step if reaching limit
  if (iTest >= SPEED_MAX_TEST || iTest <= -SPEED_MAX_TEST)
    iStep = -iStep;


  // Blink the LED
  digitalWrite(LED_BUILTIN, (timeNow%2000)<1000);
}

void send_rc_servo_input_values()
{
  if(ServoInput.available()){

    /*full range values with default speed / torque settings in firmware
      #define I_MOT_MAX       15              // [A] Maximum single motor current limit
      #define I_DC_MAX        17              // [A] Maximum stage2 DC Link current limit for Commutation and Sinusoidal types (This is the final current protection. Above this value, current chopping is applied. To avoid this make sure that I_DC_MAX = I_MOT_MAX + 2A)
      #define N_MOT_MAX       1000            // [rpm] Maximum motor speed limit

      int steerPermil = steering.map(-1000, 1000);  // remap to a percentage both forward and reverse
      int throttlePermil = throttle.map(1000, -1000);  // remap to a percentage both forward and reverse
    */

    /*
    /*values for slower speed / torque settings in firmware
      #define I_MOT_MAX       6              // [A] Maximum single motor current limit
      #define I_DC_MAX        8              // [A] Maximum stage2 DC Link current limit for Commutation and Sinusoidal types (This is the final current protection. Above this value, current chopping is applied. To avoid this make sure that I_DC_MAX = I_MOT_MAX + 2A)
      #define N_MOT_MAX       100 
    */

    Serial.print("RC - ");

    int steerPermil = steering.map(-500, 500);  // remap to a percentage both forward and reverse
    Serial.print("steerPermil: ");
    Serial.print(steerPermil);
    Serial.print("% ");

    int steeringPulse = steering.getPulse();
    Serial.print(" Pulseln: ");
    Serial.print(steeringPulse);

    Serial.print(" | ");  // separator

    int throttlePermil = throttle.map(200, -200);  // remap to a percentage both forward and reverse
    Serial.print("Throttle: ");
    Serial.print(throttlePermil);
    Serial.print("% ");

    int throttlePulse = throttle.getPulse();
    Serial.print(" Pulseln: ");
    Serial.print(throttlePulse);

    if (throttlePermil >= 0) {
      Serial.print("(Forward)");
    }
    else {
      Serial.print("(Reverse)");
    }

    Serial.println();

    Send(steerPermil,throttlePermil);
  }
}

void loop(void)
{
  can_enable_battery_timer.run();

  send_rc_servo_input_values();

  // Check for new received serial data from hoverboard and print in in clear text
  //Receive();


  /*the following two forwards enable the usage of the web gui
  https://candas1.github.io/Hoverboard-Web-Serial-Control/
  Make sure to set the the log to "Binary" and connect to the com port of the Arduino!
  */

  //Forward built in usb serial (0) to hoverboard
  if (Serial.available()) {
    HoverSerial.write(Serial.read());
  }

  //Forward hoverboard data to build in serial
  if (HoverSerial.available()) {
    char value = HoverSerial.read();
    Serial.write(value);

    // #ifdef USE_BLE
    // std::string value_str{value};
    //
    // if (deviceConnected) { 
    //   pCharacteristic_tx->setValue(value_str); 
    //   pCharacteristic_tx->notify();
    // }
    //#endif

  }

  //send_motor_test_command();
}



// ########################## END ##########################