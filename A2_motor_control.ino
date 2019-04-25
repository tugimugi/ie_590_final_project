/*
   IE590 Final Project: Automatic Watering of Crops

   Components needed:
   2 Arduinos Unos (2 way comm.)

   Arduino #1 (Plant Monitor + HMI):
   1 Transceiver module (NRF24L01)
   1 RGB LED + 3 resistors (220 Ohms for each channel)
   1 LCD
   1 Soil Moisture Sensor (Optional: 2)
   1 Push Button (for enabling manual watering) + 1 resistor

   Arduino #2 (Motor Control):
   1 Transceiver module (NRF24L01)
   2 LEDs (1 Green, 1 Red) + 2 resistors (220 Ohms each)
   1 L293D IC Motor Driver (instead of a Relay)
   1 DC Motor (instead of Pump Motor)
   1 Push Button (for starting manual watering) + 1 resistor

   Ps. This code is for Arduino #2
   
   By: Can Tugal & Rahul Taneja
*/

//----------Libraries----------//
// Transceiver module
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include "RF24_config.h"
#include "printf.h"

//----------I/O Pins----------//
// LEDs
#define GreenLEDPin A0
#define RedLEDPin   A1

// Relay + Motor
#define EnableA 3
#define MotorA1 5
#define MotorA2 6
//#define PinPot1 A0            // speed

// Push Button
#define pushButtonPin 7

//----------Global Variables---------//
//int potVal1;                  // speed
int motor_speed = 130;
boolean motorOn = false;
boolean motorEnableOn = false;

// Push Button
int pushButtonState;

// Soil Moisture Sensor
int moistureVal;
int tooDry = 150; //set low parameter for plant
int tooWet = 400; //set high parameter for plant

// Transceiver module (NRF24L01)
int vals[] = {100,0};     // 0 - Moisture Level, 1 - Remote Watering Enable
String message;           // Message transmitted to Arduino #2
char message_char[31];

const uint64_t pipes[2] = { 0xDEDEDEDEE7LL, 0xDEDEDEDEE9LL };

boolean stringComplete = false;  // whether the string is complete
static int dataBufferIndex = 0;
boolean stringOverflow = false;
char charOverflow = 0;

char SendPayload[31] = "";
char RecvPayload[31] = "";
char serialBuffer[31] = "";


//-------------Functions------------//
// Transceiver module (NRF24L01)

//LED
void LED(boolean motorOn){
  if (motorOn == 1) {
    digitalWrite(GreenLEDPin, HIGH); //Red LED
    digitalWrite(RedLEDPin, LOW);
  }
  else{
    digitalWrite(GreenLEDPin, LOW);
    digitalWrite(RedLEDPin, HIGH); //Blue LED
  }
}

// Motor + Motor Driver
void motor(boolean motorOn, int motor_speed){
  if(motorOn == true){
    digitalWrite(MotorA1, HIGH);
    digitalWrite(MotorA2, LOW);
    analogWrite(EnableA, motor_speed);
  }
  else{
    digitalWrite(MotorA1, LOW);
    digitalWrite(MotorA2, LOW);
    analogWrite(EnableA, 0);
  }
}

// Transceiver (Radio) setup 
void radio_setup(){
  printf_begin();
  radio.begin();
  
  radio.setDataRate(RF24_250KBPS);
  radio.setPALevel(RF24_PA_MAX);
  radio.setChannel(70);
  
  radio.enableDynamicPayloads();
  radio.setRetries(15,15);
  radio.setCRCLength(RF24_CRC_16);

  radio.openWritingPipe(pipes[0]);
  radio.openReadingPipe(1,pipes[1]);  
  
  radio.startListening();
  radio.printDetails();
}

void setup() {                                    // Setup Code (Run once)
  Serial.begin(57600);
  Serial.println("Automatic Crop Watering System");
  Serial.println("Arduino #2 - Motor Control");
  radio_setup(); Serial.println("Radio Setup completed");
  pinMode(pushButtonPin, INPUT);
  pinMode(EnableA, OUTPUT);
  pinMode(MotorA1, OUTPUT);
  pinMode(MotorA2, OUTPUT);
  delay(500);
  lcd.clear();
}

void loop() {                                     // Main Code (Repeated Run)
  // Read push button
  pushButtonState = digitalRead(pushButtonPin);
  //potVal1 = analogRead(PinPot1);
  //motor_speed = map(potVal1, 0, 1023, 0, 255);
  
  // Receive message
  nRF_receive();                                    

  // Retrieve message
  for (int i = 0; i < 2; i = i + 1) {
    vals_rec[i] = RecvPayload[i]-'0';
  }
  moistureVal = vals_rec[0];
  motorEnableOn = vals_rec[1];
  RecvPayload[0] = 0;

  //Motor logic
  if (pushButtonState == 1 && motorEnableOn == 1){
    motorOn = true;
    manualModeOn = true;
  }
  else{
   if (moistureVal <= tooDry){
     motorOn = true;
     wateringOn = true;
    }
    else if (moistureVal > tooDry && moistureVal < tooWet && wateringOn == false){
      motorOn = false;
    }
    else if (moistureVal > tooWet){
      motorOn = false;
      wateringOn = false;
    }
    manualModeOn = false;
  }
  
  motor(motorOn,motor_speed);               // Run motor
  LED(motorOn);                             // Light LEDs

  // Prepare message
  vals_send[0] = motorOn; vals_send[1] = manualModeOn;
  for (int i = 0; i < 2; i = i + 1) {
    message += vals_send[i];
  }
  message.toCharArray(message_char, 31);    // Convert message type (string to char)
  // Send message
  serial_receive();
  delay(50);
}

void serialEvent() {
  while (Serial.available() > 0 ) {
    char incomingByte = Serial.read();

    if (stringOverflow) {
      serialBuffer[dataBufferIndex++] = charOverflow;  // Place saved overflow byte into buffer
      serialBuffer[dataBufferIndex++] = incomingByte;  // saved next byte into next buffer
      stringOverflow = false;                          // turn overflow flag off
    } else if (dataBufferIndex > 31) {
      stringComplete = true;        // Send this buffer out to radio
      stringOverflow = true;        // trigger the overflow flag
      charOverflow = incomingByte;  // Saved the overflow byte for next loop
      dataBufferIndex = 0;          // reset the bufferindex
      break;
    }
    else if (incomingByte == '\n') {
      serialBuffer[dataBufferIndex] = 0;
      stringComplete = true;
    } else {
      serialBuffer[dataBufferIndex++] = incomingByte;
      serialBuffer[dataBufferIndex] = 0;
    }
  } // end while()
} // end serialEvent()

void nRF_receive(void) {                          // Receive message
  int len = 0;
  if ( radio.available() ) {
    bool done = false;
    while ( !done ) {
      len = radio.getDynamicPayloadSize();
      done = radio.read(&RecvPayload, len);
      delay(5);
    }

    RecvPayload[len] = 0; // null terminate string

    //lcd.setCursor(0, 0);
    //lcd.print("R:");
    Serial.print("R:");
    //lcd.setCursor(2, 0);
    //lcd.print(RecvPayload);
    Serial.print(RecvPayload);
    Serial.println();
    //RecvPayload[0] = 0;  // Clear the buffers
  }

} // end nRF_receive()

void serial_receive(void) {                       // Send message
  if (stringComplete) {
  //if(1){
    //serialBuffer = message_char;
    //Serial.print(serialBuffer); Serial.print("//"); Serial.println(message_char);
    //Serial.print(charOverflow); Serial.println("//");
    strcat(SendPayload, message_char); //message_char //charOverflow
    // swap TX & Rx addr for writing
    radio.openWritingPipe(pipes[1]);
    radio.openReadingPipe(0, pipes[0]);
    radio.stopListening();
    bool ok = radio.write(&SendPayload, strlen(SendPayload));

    //lcd.setCursor(0, 1);
    //lcd.print("S:");
    Serial.print("S:");
    //lcd.setCursor(2, 1);
    //lcd.print(SendPayload);
    Serial.print(SendPayload);
    Serial.println();
    stringComplete = false;

    // restore TX & Rx addr for reading
    radio.openWritingPipe(pipes[0]);
    radio.openReadingPipe(1, pipes[1]);
    radio.startListening();
    SendPayload[0] = 0;
    dataBufferIndex = 0;
  } // endif
} // end serial_receive()
