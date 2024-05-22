#include <Adafruit_NeoPixel.h>

//4 for number of pixels, PIN for pin number
Adafruit_NeoPixel pixels(4, PIN, NEO_GRB + NEO_KHZ800);


const int LB = 9; //motor left - going backwards
const int LF = 5; // motor left - going forward
const int RB = 6; //motor right - going backwards
const int RF = 11; //motor right - going forward

const int servoPin = 3;

const int distance;

const int openPulseWidth = 2400;  // Pulse width for open position
const int closePulseWidth = 544;  // Pulse width for close position

int IR1;
int IR2;
int IR3;
int IR4;
int IR5;
int IR6;
int IR7;
int IR8;

volatile int ticksLeft;  
volatile int ticksRight;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pixels.begin();
  setupMotors();
  setupIRSensors();
  setupGripper();
  setupRotarySensors();
  setupDistanceSensor();
  setupLEDs();

  bool raceStarted = false;
  bool objectAttached = false;
}

void loop() {
  // put your main code here, to run repeatedly:
  if (raceStarted) {
    pixels.setPixelColor(1, pixels.Color(255, 0, 0));
    pixels.setPixelColor(2, pixels.Color(0, 255, 0));
    pixels.setPixelColor(3, pixels.Color(0, 0, 255));
    pixels.setPixelColor(4, pixels.Color(0, 0, 255));
    pixels.show();
    followLine();
  } else {
    checkDistance();
    while(distance < 8) {
      checkDistance();
    }

    driveForward(50, 255);
    raceStarted = true;
  }
}

void followLine() {
//pixels color change!!
  checkDistance();
  if(!objectAttached && distance < 10) {
    //pixels color change!!
    driveForward(20, 130);
    closeGripper();
    objectAttached = true;
  } else {
    //pixels color change!!
    readIRSensors();
    if(IR2 > 600 || IR3 > 600 || IR4 > 600 || IR5 > 600 || IR6 > 600 || IR7 > 600){
      int leftValue = (IR5 + IR6 + IR7) / 3;
      int rightValue = (IR2 + IR3 + IR4) / 3;

      if(leftValue > rightValue) { // check which part of the sensors is reading the line
        adjustRight();
      } else {
        adjustLeft();
      }
    }
  }
}

void adjustLeft() {
  //pixels color change!!
  rightSpeed = map(rightValue, 0, 1023, 0, 255);
  analogWrite(LF, 255);
  analogWrite(RF, rightSpeed);
}

void adjustRight() {
  //pixels color change!!
  leftSpeed = map(leftValue, 0, 1023, 0, 255);
  analogWrite(LF, leftSpeed);
  analogWrite(RF, 255);
}

void checkDistance() {
  digitalWrite(trigPin, LOW); //initialize trigPin on low
  delayMicroseconds(2); // wait 2ms before sending pulse
  digitalWrite(trigPin,HIGH); //send pulse to trigPin to send UltraSonic wave for detection
  delayMicroseconds(10); //keep trigPin HIGH for 10ms
  digitalWrite(trigPin,LOW); //stop sending pulse
  //calculating distance
  duration = pulseIn(echoPin, HIGH); //storing how much time it took to get the pulse back
  distance = duration * 0.034 / 2; //calculating distance based on time from pulseIn
}

void setupMotors() {
  pinMode( , OUTPUT); // MOTORS
  pinMode( , OUTPUT);
  pinMode( , OUTPUT);
  pinMode( , OUTPUT);
}

void setupIRSensors() {
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);
  pinMode(A6, INPUT);
  pinMode(A7, INPUT);
}

void setupGripper() {
  pinMode(servoPin, OUTPUT);
}

void setupDistanceSensor() {
  pinMode(echoPin, INPUT);
  pinMode(trigPin, OUTPUT);
}

void setupRotarySenors() {
  pinMode(rotaryR, INPUT);
  pinMode(rotaryL, INPUT);
  pinMode(LF, OUTPUT);
  pinMode(RF, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(rotaryL), tickLeft, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rotaryR), tickRight, CHANGE);
}

void forwardDistance(int ticksRemaining, int speed) {
  if (ticksLeft > ticksRemaining) {
    idle();
  } else {
    Serial.println(ticksLeft);
    analogWrite(leftF, speed);
    analogWrite(rightF, speed);
  }
}

void tickLeft() {
  noInterrupts();
  ticksLeft++;
  interrupts();
}

void tickRight() {
  noInterrupts();
  ticksRight++;
  interrupts();
}

void idle() {
  analogWrite(leftF, 0);
  analogWrite(rightF,0);
}