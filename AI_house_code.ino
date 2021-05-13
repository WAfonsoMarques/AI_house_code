#include <SPI.h>
#include <MFRC522.h>

#include <Servo.h>

#include <dht11.h>

#include <LiquidCrystal_I2C.h>
#include <Wire.h>

#include <Keypad.h>


#include <IRremote.h>

const byte ROWS = 4; 
const byte COLS = 4; 

char hexaKeys[ROWS][COLS] = {
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'},
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}
};

struct Light {
   boolean  sound;
   boolean  sensor;
   boolean  state;
   boolean  activeServoDist;
   boolean  activeServoClothes;
   boolean  activeMotor;
   boolean  ir;
};

struct Ir {
  boolean clotheLine;
  boolean garageOpen;
  boolean garageClose;
};

byte rowPins[ROWS] = {33, 31, 29, 27}; 
byte colPins[COLS] = {32, 30, 28, 26}; 

Keypad customKeypad = Keypad(makeKeymap(hexaKeys), rowPins, colPins, ROWS, COLS);

const unsigned long zero = 16738455;
const unsigned long one = 16724175;
const unsigned long two = 16718055;
const unsigned long three = 16743045;
const unsigned long four = 16716015;
const unsigned long five = 16726215;
const unsigned long six = 16734885;
const unsigned long seven = 16728765;
const unsigned long eight = 16730805;
const unsigned long nine = 16732845;
const unsigned long less = 16769055;
const unsigned long plus = 16754775;

struct Light l;
struct Ir ir;

static volatile unsigned long time, elapsedGas, prev_timeGas;//GAS
static volatile unsigned long elapsedGasSensor, prev_timeGasSensor;//GAS
static volatile unsigned long elapsedWater, prev_timeWater;//Water
static volatile unsigned long elapsedLight, prev_timeLight;//Light
static volatile unsigned long elapsedLcd, prev_timeLcd;//Lcd
static volatile unsigned long elapsedDht11, prev_timeDht11;//Dht11
static volatile unsigned long elapsedRfid, prev_timeRfid;//Rfid
static volatile unsigned long elapsedDistance, prev_timeDistance;//Distance
static volatile unsigned long elapsedDistServ, prev_timeDistServ;//Distance
static volatile unsigned long elapsedSound, prev_timeSound;//Sound
static volatile unsigned long elapsedPrint, prev_timePrint;//Print
static volatile unsigned long elapsedIR, prev_timeIR;//IR

const int lightA0 = A0;
const int smokeA1 = A1;
const int waterA2 = A2;
const int soundA3 = A3;

const byte ledsPin = 2;
const byte distanceTrigPin = 3;
const byte distanceEchoPin = 4;
const byte motorPin = 5;
const byte servoDistPin = 6;
const byte waterPin = 22;
const byte pirPin = 23;
const byte dhtPin = 25;
//-RGB
const byte redPin = 34;
const byte bluePin = 35;
const byte greenPin = 36;
//-Piezo
const byte melodyPin = 38;
//-SERVO Clothes
const byte servoClothesPin = 39;

const byte recvPin = 42;

//-Rfid
const byte rfindRstPin = 49;
const byte rfidSsPi = 53;

Servo servoClothes;

Servo servoDistance;
#define timeOpen 5000

dht11 DHT11; // create object of DHT11

LiquidCrystal_I2C lcd(0x27, 16, 2);

IRrecv irrecv(recvPin);

decode_results results;

MFRC522 mfrc522(rfidSsPi, rfindRstPin);

//Initial Values and thresholds
int speedMotor = 170;// Speed of gas motor

int waterValue = 0;//Water level

int lightValue = 1000;//Init value of light sensor

int humidity = 0;//DHT11 humidity Value
float temperature = 0;//DHT11 temperature Value

int angleServClothesRain = 170;
int angleServClothes = 20;

int angleServDistClosed = 105;
int angleServDistOpen = 10;

long duration = -1; // variable for the duration of sound wave travel
int distance = -1; // variable for the distance measurement

const String knownKeyRfid = "5323383182";
String atualKey = "";

boolean activatedBySound = false;

int soundValue = 0;
int countOutlierSound = 0;

int inputGas = 0;

char profile = '*';
char prevProf ='*';

int counterLed = 0;

boolean isAuthenticated = false;

int pirState = LOW;
int pirVal = 0;

//-----Threshold-----
int minSound = 545;
int maxSound = 565;

int sensorThresGas = 350;// Threshold value GAS

int ledThres = 90;// Threshold value Leds

void setup() {
  l = {false, false, false, true, true, true, false};//Struct light
  ir = {false, false, false};
  
  lcd.init();//initialize lcd screen
  lcd.backlight();// turn on the backlight
  lcd.setBacklight((uint8_t)1);// Turn on the blacklight  
  lcd.print("WELCOME!");// First row
  lcd.setCursor(0,1);// Second row
  lcd.print("SMART HOME");

  
  servoClothes.attach(servoClothesPin);
  servoClothes.write(angleServClothes);

  servoDistance.attach(servoDistPin);
  servoDistance.write(angleServDistClosed);
  
  SPI.begin();
  mfrc522.PCD_Init();
  irrecv.enableIRIn();
   
  pinMode(lightA0, INPUT);
  pinMode(smokeA1, INPUT);
  pinMode(waterA2, INPUT);
  pinMode(soundA3, INPUT);
  
  pinMode(motorPin, OUTPUT);
  pinMode(ledsPin, OUTPUT);
  
  pinMode(distanceTrigPin, OUTPUT);//Distance
  pinMode(distanceEchoPin, INPUT);//Distance
  
  pinMode(waterPin, OUTPUT);   // Configure A2 pin as an OUTPUT
  digitalWrite(waterPin, LOW); // turn the water sensor OFF

  pinMode(redPin,   OUTPUT);
  pinMode(bluePin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  
  pinMode(melodyPin,  OUTPUT);

  pinMode(pirPin, INPUT);
  
  Serial.begin(9600);
}

void loop() {
  
  time = millis();
  elapsedGas = time - prev_timeGas;
  elapsedGasSensor = time - prev_timeGasSensor;
  elapsedWater = time - prev_timeWater;
  elapsedLight = time - prev_timeLight;
  elapsedDht11 = time - prev_timeDht11;
  elapsedRfid = time - prev_timeRfid;
  elapsedDistance = time - prev_timeDistance;
  elapsedDistServ = time - prev_timeDistServ;
  elapsedSound = time - prev_timeSound;
  elapsedPrint = time - prev_timePrint;
  elapsedIR = time - prev_timeIR;

  checkProfile();
  checkMovement();
  changeRgb();
  
// RFID
  if(elapsedRfid > 1000)
  {    
    prev_timeRfid = time;
    if(mfrc522.PICC_IsNewCardPresent())
    {
      atualKey = "";
      if(mfrc522.PICC_ReadCardSerial()){
        Serial.print("Tag UID:");
        for(byte i = 0; i < mfrc522.uid.size; i++){
            Serial.print(mfrc522.uid.uidByte[i] < 0x10 ? " 0" : "");
            atualKey+=mfrc522.uid.uidByte[i];
            
        }

        if (knownKeyRfid == atualKey) {
          
          lcd.clear();
          lcd.setCursor(4,0);
          lcd.print("WELCOME");
        
          // Second row
          lcd.setCursor(0,1);
          lcd.print("AFONSO MARQUES");
          isAuthenticated = true;
        }else {
          lcd.clear();
          lcd.setCursor(5,0);
          lcd.print("SORRY");
        
          // Second row
          lcd.setCursor(2,1);
          lcd.print("UNKNOWN KEY"); 

          if(isAuthenticated == true) {
            profile = '*';
            prevProf = '*';
          }
          
          isAuthenticated = false;              
        }
        mfrc522.PICC_HaltA();
      }
    } 
  }
 

// TEMPERATURE AND HUMIDITY FUNCTION
  if(elapsedDht11 >= 15000)
  {
    prev_timeDht11 = time;
    DHT11.read(dhtPin);
    humidity = DHT11.humidity;// get humidity
    temperature = (float)DHT11.temperature; 
    lcd.clear();
    lcd.setCursor(5,0);
    lcd.print(temperature);
    lcd.print(" C");
  
    // Second row
    lcd.setCursor(0,1);
    lcd.print("HUMIDITY = ");
    lcd.print(humidity);
    lcd.print("%");
  }
  
//GAS FUNCTION
  if(elapsedGas >= 1000)
  {
    prev_timeGas = time;
    inputGas = analogRead(smokeA1);
    if(inputGas >= sensorThresGas ){//Check if atual value is highier than 400
      prev_timeGasSensor = time;   
      l.activeMotor=true;
      analogWrite(motorPin,speedMotor);//Start the motor
      
  
      tone(melodyPin, 500);
      
      digitalWrite(redPin, HIGH);
      
      delay(500);
      
      noTone(melodyPin);
      
      digitalWrite(redPin, LOW);
    
    }else if(elapsedGasSensor >= 2000){
      noTone(melodyPin);
      
      analogWrite(motorPin, LOW);//Turn off the motor
      l.activeMotor=false;
    }
  }

 //CLOTHESLINE
 if(elapsedWater >= 1000)
  {
    prev_timeWater = time;

    digitalWrite(waterPin, HIGH);  // turn the sensor ON
    delay(10);                      // wait 10 milliseconds
    waterValue = analogRead(waterA2); // read the analog value from sensor
    digitalWrite(waterPin, LOW);   // turn the sensor OFF

    if(waterValue > 50 || ir.clotheLine) {
      if(angleServClothesRain != servoClothes.read()) { 
        servoClothes.attach(servoClothesPin);
        servoClothes.write(angleServClothesRain);
        l.activeServoClothes=true;
      }else {
        servoClothes.detach();
        l.activeServoClothes=false;
      }
        
    }else {
      
      if(angleServClothes != servoClothes.read()) {
        servoClothes.attach(servoClothesPin);
        servoClothes.write(angleServClothes);
        l.activeServoClothes=true;
      }else {
        servoClothes.detach();
        l.activeServoClothes=false;
      }     
    }    
  }

//LEDS
  if(elapsedLight > 1000){
    
    prev_timeLight = time;
    lightValue = analogRead(lightA0);
    
    if(lightValue <= ledThres && l.sound == false && l.ir == false) {
      l.state = true; 
      l.sensor = true;
      digitalWrite(ledsPin, l.state);
      
    }else if(l.sound == false && l.ir == false) {
      l.state = false;
      l.sensor = false;
      digitalWrite(ledsPin, l.state);
    } else if (l.ir) {
      digitalWrite(ledsPin, l.state);
    }
  }

//SOUND
  soundValue = analogRead(soundA3);

//  Serial.print("| SOM: ");
//  Serial.println(soundValue);
// 
  if(!l.sensor && !l.activeServoDist && !l.activeServoClothes && !l.activeMotor && !l.ir && (soundValue < minSound || soundValue > maxSound)){
    if(countOutlierSound >= 2 || elapsedSound > 2000) {
      countOutlierSound = 0;
    }
    
    if(elapsedSound > 100){
      prev_timeSound = time;
      elapsedSound = 0;
      countOutlierSound++;
    }
    
    if(countOutlierSound >= 2 && elapsedSound <= 2000) {
      
      l.state = !l.state;
      
      if(l.state){
        l.sound = true;
      }else{
        l.sound = false;
      }
      
      digitalWrite(ledsPin, l.state);
    }

  }

//CAR GATE
  if(elapsedDistance > 500){
    prev_timeDistance = time;
    
    digitalWrite(distanceTrigPin, LOW);// Clears the trigPin condition
    delayMicroseconds(2);
    digitalWrite(distanceTrigPin, HIGH); // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
    delayMicroseconds(10);
    digitalWrite(distanceTrigPin, LOW);
    duration = pulseIn(distanceEchoPin, HIGH);
    distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
    
    if(ir.garageClose || (distance > 15 && !ir.garageOpen) && elapsedDistServ >= timeOpen){
      ir.garageClose = false;
      if(angleServDistClosed != servoDistance.read()) { 
        
        servoDistance.attach(servoDistPin);
        servoDistance.write(angleServDistClosed);
        l.activeServoDist=true;
      }else {
        servoDistance.detach();
        l.activeServoDist=false;  
      }
     
    }else if((distance <= 15 || ir.garageOpen) && isAuthenticated == true){
      ir.garageOpen = false;
      prev_timeDistServ = time;

      if(angleServDistOpen != servoDistance.read()) { 
        
        servoDistance.attach(servoDistPin);
        servoDistance.write(angleServDistOpen);
        l.activeServoDist=true;
      }else {
        servoDistance.detach();
        l.activeServoDist=false;
        
      }
    }
  }

  if(elapsedIR > 300) {
    prev_timeIR = time;
    if (irrecv.decode(&results)) {

      switch(results.value) {
        case nine:
          if(!l.sound && !l.sensor) {
            l.ir = !l.ir;
            l.state = !l.state;  
          }
          break;
        case zero:
          if(isAuthenticated) {
            prevProf = '0';
            profile = '0';
          }
          break;
        case one:
          if(isAuthenticated) {
            prevProf = '1';
            profile = '1';
          }
          break;
        case two:
          if(isAuthenticated) {
            prevProf = '2';
            profile = '2';
          }
          break;
        case three:
          if(isAuthenticated) {
            profile = '3';
          }
          break;
        case four:
          prevProf = '0';
          profile = '0';
          isAuthenticated = true;
          break;
        case five:
          prevProf = '*';
          profile = '*';
          isAuthenticated = false;
          break;
        case six:
          if(isAuthenticated) {
            ir.garageOpen = true;
          } 
        break;
        case seven:
          if(isAuthenticated) {
            ir.garageClose = true;
          } 
        break;
        case less:
          if(isAuthenticated) {
            ir.clotheLine = !ir.clotheLine;
          } 
        break;           
      }
      irrecv.resume(); // Receive the next value
    }
    changeRgb();
  }

  if(elapsedPrint > 1000) {
    prev_timePrint = time;
    printInfo();
  }
}

void printInfo() {
  Serial.println();
  Serial.println("------------ New Round ------------");
  Serial.println();

  Serial.print("| Light: ");
  Serial.println(lightValue);

  Serial.print("| Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  Serial.print("| Water: ");
  Serial.println(waterValue);

  Serial.print("| Input GAS: ");
  Serial.print(inputGas);
  Serial.print(" | MaxValue GAS: ");
  Serial.println(sensorThresGas);

  Serial.print("| SOM: ");
  Serial.println(soundValue);
}

void checkMovement() {
  if(isAuthenticated) {
      if(prevProf == '*') {
        profile = '0';
      } else {
        profile = prevProf;  
      }   
    }else {
      pirVal = digitalRead(pirPin);  // read input value
      if (pirVal == HIGH) {            // check if the input is HIGH
        profile = '#';
        if (pirState == LOW) {
          // Motion detected!
          pirState = HIGH;
        }
      } else {
        if (pirState == HIGH){
          //Motion ended!
          pirState = LOW;
        }
      }
    }
}
void checkProfile() {
  char customKey = customKeypad.getKey();
  if (customKey && isAuthenticated){
    Serial.println(customKey);
    switch (customKey) {
      case '0':
        prevProf = customKey;
        profile = customKey;
        break;
      case '1':
        prevProf = customKey;
        profile = customKey;
        break;
      case '2':
        prevProf = customKey;
        profile = customKey;
        break;
      case '3':
        profile = customKey;
        break;
      default:
        break;
    }
  } 
}
void changeRgb() {
  
  if (l.activeMotor == true || profile == '*') {
    analogWrite(redPin,   0);
    analogWrite(greenPin, 0);
    analogWrite(bluePin,  0);
    
  } else if (profile == '#') {
    analogWrite(redPin, 255);
    analogWrite(greenPin, 0);
    analogWrite(bluePin,  0);
    
  } else if(profile == '0') {
    analogWrite(redPin,   52);
    analogWrite(greenPin, 168);
    analogWrite(bluePin,  83);
    
  } else if (profile == '1') {
    analogWrite(redPin,   0);
    analogWrite(greenPin, 201);
    analogWrite(bluePin,  204);
    
  }else if (profile == '2' ) {
    analogWrite(redPin,   247);
    analogWrite(greenPin, 120);
    analogWrite(bluePin,  198);
    
  }else if (profile == '3') {
    firstSection();
    profile = prevProf;
    irrecv.enableIRIn();
  }
}
const int c = 261;
const int d = 294;
const int e = 329;
const int f = 349;
const int g = 391;
const int gS = 415;
const int a = 440;
const int aS = 455;
const int b = 466;
const int cH = 523;
const int cSH = 554;
const int dH = 587;
const int dSH = 622;
const int eH = 659;
const int fH = 698;
const int fSH = 740;
const int gH = 784;
const int gSH = 830;
const int aH = 880;

void beep(int note, int duration)
{
  //Play tone on melodyPin
  tone(melodyPin, note, duration);
 
  //Play different LED depending on value of 'counterLed'
  if(counterLed % 2 == 0)
  {
    digitalWrite(bluePin, HIGH);
    delay(duration);
    digitalWrite(bluePin, LOW);
  }else
  {
    digitalWrite(greenPin, HIGH);
    delay(duration);
    digitalWrite(greenPin, LOW);
  }
 
  //Stop tone on melodyPin
  noTone(melodyPin);
 
  delay(50);
 
  //Increment counterLed
  counterLed++;
}

void firstSection()
{
  beep(a, 500);
  beep(a, 500);    
  beep(a, 500);
  beep(f, 350);
  beep(cH, 150);  
  beep(a, 500);
  beep(f, 350);
  beep(cH, 150);
  beep(a, 650);
 
  delay(500);
 
  beep(eH, 500);
  beep(eH, 500);
  beep(eH, 500);  
  beep(fH, 350);
  beep(cH, 150);
  beep(gS, 500);
  beep(f, 350);
  beep(cH, 150);
  beep(a, 650);
}
