/*
   IE590 Final Project: Automatic Watering of a Plant

   Components needed:
   2 Arduinos Unos

   Arduino #1 (Plant Monitor + HMI):
   1 Transceiver module (NRF24L01)
   1 RGB LED + 3 resistors (220 Ohms each channel)
   1 I2C LCD
   1 Soil Moisture Sensor (Optional: 2)
   1 Push Button (for enabling manual watering) + 1 resistor

   Arduino #2 (Motor Control):
   1 Transceiver module (NRF24L01)
   2 LEDs (1 Green, 1 Red) + 2 resistors (220 Ohms each)
   1 L293D IC Motor Driver (instead of a Relay)
   1 DC Motor (instead of Pump Motor)
   1 Push Button (for starting manual watering) + 1 resistor

   Ps. This code is for Arduino #1
   
   By: Can Tugal & Rahul Taneja
*/

//----------Libraries----------//
// Transceiver module
#include <SPI.h>
#include <printf.h>
#include <nRF24L01.h>
#include <RF24_config.h>
#include <RF24.h>

// LCD
#include <Wire.h>
#include <LiquidCrystal_I2C.h>


//----------I/O Pins----------//
// Arduino #1
// Transceiver module (NRF24L01)
#define ce  9
#define csn 10
// #define sck 13
// #define mosi 11
// #define miso 12

// RGB LED
#define RGBLEDRedPin   A0
#define RGBLEDGreenPin A1
#define RGBLEDBluePin  A2

// Soil Moisture Sensor
#define moistPin A3

// LCD
#define sda A4
#define scl A5

// Push Button
#define pushButtonPin 7


//----------Global Variables---------//
// Push Button
int pushButtonState = 0;
int pressCount = 0;

// Soil Moisture Sensor
int moistureVal;
int moisPercent;
int maxMois = 160;
int minMois = 1023;

// Motor
int manualEnable = 0;
int previousManualEnable;
boolean motorOn = false;
boolean manualModeOn = false;
boolean previousMotorState = false;
boolean previousManualMode = false;

// Transceiver module (NRF24L01)
int sendVals[] = {100,0};                 // 0 - Moisture Level, 1 - Manual Watering Enable
char recvVals;                            // 0 - Motor status, 1 - Motor Mode
String sendMessage;                       // Message transmitted to Arduino #2
char message_char[31];

const uint64_t pipes[2] = { 0xDEDEDEDEE7LL, 0xDEDEDEDEE9LL };

boolean stringComplete = false;           // whether the string is complete
static int dataBufferIndex = 0;
boolean stringOverflow = false;
char charOverflow = 0;

char SendPayload[31] = "";
char RecvPayload[31] = "";
char serialBuffer[31] = "";


//-------------Functions------------//
// Transceiver module (NRF24L01)
RF24 radio(ce,csn);

// LCD
// Set the LCD address to 0x27 for a 16 chars and 2 line display
LiquidCrystal_I2C lcd(0x27, 16, 2);

//RGB LED
void RGB_LED(int moisPercent) {
  if (moisPercent <= 20) {
    digitalWrite(RGBLEDRedPin, HIGH); //Red LED
    digitalWrite(RGBLEDGreenPin, LOW);
    digitalWrite(RGBLEDBluePin, LOW);
  }
  else if (moisPercent >= 80) {
    digitalWrite(RGBLEDRedPin, LOW);
    digitalWrite(RGBLEDGreenPin, HIGH); //Blue LED
    digitalWrite(RGBLEDBluePin, LOW);
  }
  else {
    digitalWrite(RGBLEDRedPin, LOW);
    digitalWrite(RGBLEDGreenPin, LOW);
    digitalWrite(RGBLEDBluePin, HIGH); //Green LED
  }
}

// LCD initialization
void LCD_init(){
  lcd.begin();
  lcd.setBacklight((uint8_t)1);                       // Turn on the blacklight
  lcd.clear();
  lcd.setCursor(0,0); lcd.print("Automatic Crop  ");
  lcd.setCursor(0,1); lcd.print("Watering System ");
}

// LCD main
void LCD_main(boolean motorOn, boolean manualModeOn, int moisPercent, int manualEnable){
  // First line 
  if (manualEnable != previousManualEnable && manualEnable == 1){
    lcd.setCursor(0,0); lcd.print("Manual Mode On   ");
    delay(500);
    lcd.clear();
  }
  else if (manualEnable != previousManualEnable && manualEnable == 0){
    lcd.setCursor(0,0); lcd.print("Auto Mode On     ");
    delay(500);
    lcd.clear();
  }
  
  lcd.setCursor(0,0); lcd.print("Moisture: ");
  if (moisPercent < 10){
    lcd.print(moisPercent); lcd.print("%   ");
  }
  else if (moisPercent > 99){
    lcd.print(moisPercent); lcd.print("% ");
  }
  else lcd.print(moisPercent); lcd.print("%  ");
  
  if (manualModeOn == true){
    lcd.setCursor(15,0); lcd.print("M");
  }
  else if (manualModeOn == false){
    lcd.setCursor(15,0); lcd.print("A");
  }

  // Second line 
  if (motorOn == true){
    lcd.setCursor(0,1); lcd.print("Watering plant  ");
  }
  else if (motorOn == false){
    lcd.setCursor(0,1); lcd.print("                ");
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
  Serial.println("Arduino #1 - Plant Monitor + HMI");
  LCD_init(); Serial.println("LCD Setup completed");
  radio_setup(); Serial.println("Radio Setup completed");
  pinMode(pushButtonPin, INPUT);
  delay(2000);
  lcd.clear();
}

void loop() {                                     // Main Code (Repeated Run)
  moistureVal = analogRead(moistPin) - maxMois;
  if (moistureVal < 0){
    moistureVal = 0;
    moisPercent = 100;
  }
  else moisPercent = map(moistureVal,0,1023 - maxMois,100,0);
  //Serial.print(moistureVal); Serial.print(" "); Serial.print(percent); Serial.println("%");
  
  pushButtonState = digitalRead(pushButtonPin);
  if (pushButtonState == HIGH) {
    pressCount += 1;
    if (pressCount == 1) {
      manualEnable = 1;
      Serial.println("Manual Mode On");
    }
    else {
      manualEnable = 0;
      pressCount = 0;
    }
    delay(50);
  }

  // Prepare message
  sendVals[0] = moistureVal; sendVals[1] = manualEnable;
  for (int i = 0; i < 2; i = i + 1) {
    sendMessage += sendVals[i];
  }
  sendMessage.toCharArray(message_char, 31);    // Convert message type (string to char)

  // Receive message
  nRF_receive();
                      
  // Retrieve message
  if (RecvPayload[0] == '1'){
    motorOn = true;
    previousMotorState = motorOn;
  }
  else if (RecvPayload[0] == '0'){
    motorOn = false;
    previousMotorState = motorOn;
  }
  else{
    motorOn = previousMotorState;
  }
  
  if (RecvPayload[1] == '1'){
    manualModeOn = true;
    previousManualMode = manualModeOn;
  }
  else if (RecvPayload[1] == '0'){
    manualModeOn = false;
    previousManualMode = manualModeOn;
  }
  else{
    manualModeOn = previousManualMode;
  }

  RecvPayload[0] = 0; RecvPayload[1] = 0;       // Clear buffer

  // Send message
  stringComplete = true;
  serial_receive();                             // Send message function
  stringComplete = false;
  sendMessage = "";                             // Clear string
  
  RGB_LED(moisPercent);                         // Light RBG LED
  LCD_main(motorOn, manualModeOn, moisPercent, manualEnable); // LCD
  previousManualEnable = manualEnable;
  delay(50);
}

// Ps. Functions below were provided in class
/*
  SerialEvent occurs whenever a new data comes in the hardware serial RX. This
  routine is run between each time loop() runs, so using delay inside loop can
  delay response. Multiple bytes of data may be available.
*/
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

    Serial.print("R:");
    Serial.print(RecvPayload);
    Serial.println();
  }
} // end nRF_receive()

void serial_receive(void) {                       // Send message
  if (stringComplete) {
    strcat(SendPayload, message_char); //message_char //charOverflow
    // swap TX & Rx addr for writing
    radio.openWritingPipe(pipes[1]);
    radio.openReadingPipe(0, pipes[0]);
    radio.stopListening();
    bool ok = radio.write(&SendPayload, strlen(SendPayload));
    
    Serial.print("S:");
    Serial.print(SendPayload);
    Serial.println();
    stringComplete = false;

    // restore TX & Rx addr for reading
    radio.openWritingPipe(pipes[0]);
    radio.openReadingPipe(1, pipes[1]);
    radio.startListening();
    SendPayload[0] = 0;
    dataBufferIndex = 0; // endif
  } // end serial_receive()
}
