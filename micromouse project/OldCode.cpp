#include <PID_v1.h>
#include <MapFloat.h>
#define MotEnable 5 // Motor Enamble pin Runs on PWM signal
#define MotFwd A1   // Motor Forward pin
#define MotRev A0   // Motor Reverse pin

#define MotEnable2 6 // Motor Enamble pin Runs on PWM signal
#define MotFwd2 12   // Motor Forward pin
#define MotRev2 13   // Motor Reverse pin

#define IrLeft A4
#define IrRight A5
#define IrFront 4

String readString;  // This while store the user input data
int User_Input = 0; // This while convert input string into integer

int encoderPin1 = 2;   // Encoder Output 'A' must connected with intreput pin of arduino.
int encoderPin2 = A2;  // Encoder Otput 'B' must connected with intreput pin of arduino.
int encoderPin12 = 3;  // Encoder Output 'A' must connected with intreput pin of arduino.
int encoderPin22 = A3; // Encoder Otput 'B' must connected with intreput pin of arduino.

volatile int lastEncoded = 0;   // Here updated value of encoder store.
volatile long encoderValue = 0; // Raw encoder value
int PPR = 1600;                 // Encoder Pulse per revolution.
int angle = 360;                // Maximum degree of motion.
int REV = 0;                    // Set point REQUIRED ENCODER VALUE
int lastMSB = 0;
int lastLSB = 0;

volatile int lastEncoded2 = 0;   // Here updated value of encoder store.
volatile long encoderValue2 = 0; // Raw encoder value
int REV2 = 0;

double kp = 5, ki = 1, kd = 0.01; // modify for optimal performance
double input = 0, output = 0, setpoint = 0;
double input2 = 0, output2 = 0, setpoint2 = 0;
PID myPID(&input, &output, &setpoint, kp, ki, kd, DIRECT);
PID myPID2(&input2, &output2, &setpoint2, kp, ki, kd, DIRECT);

int maxi = 100;
// 2 is working

unsigned long c = 0;

unsigned long currentTime = 0;
unsigned long lastTime = 0;

// int trigPinFront = 4;
// int echoPinFront = 10;

int trigPinRight = 8;
int echoPinRight = 9;

int trigPinLeft = 7;
int echoPinLeft = 11;

float readingFront = 0;
float readingRight = 0;
float readingLeft = 0;

boolean isWallFront = false;
boolean isWallRight = false;
boolean isWallLeft = false;

boolean calibrationEnabled = true;
float calibrationThreshold = 3.5;
float minusCalibrationThreshold = -1 * calibrationThreshold;

int speed_1 = maxi;
int speed_2 = maxi;
int turn_number = 180;

int center_counter = 0;
void setup()
{
    // IR Pins
    pinMode(IrLeft, INPUT);
    pinMode(IrRight, INPUT);
    pinMode(IrFront, INPUT);

    pinMode(MotEnable, OUTPUT);
    pinMode(MotFwd, OUTPUT);
    pinMode(MotRev, OUTPUT);
    Serial.begin(9600); // initialize serial comunication

    pinMode(encoderPin1, INPUT_PULLUP);
    pinMode(encoderPin2, INPUT_PULLUP);

    digitalWrite(encoderPin1, HIGH); // turn pullup resistor on
    digitalWrite(encoderPin2, HIGH); // turn pullup resistor on

    pinMode(MotEnable2, OUTPUT);
    pinMode(MotFwd2, OUTPUT);
    pinMode(MotRev2, OUTPUT);

    pinMode(encoderPin12, INPUT_PULLUP);
    pinMode(encoderPin22, INPUT_PULLUP);

    digitalWrite(encoderPin12, HIGH); // turn pullup resistor on
    digitalWrite(encoderPin22, HIGH); // turn pullup resistor on

    // call updateEncoder() when any high/low changed seen
    // on interrupt 0 (pin 2), or interrupt 1 (pin 3)
    attachInterrupt(0, updateEncoder, CHANGE);
    attachInterrupt(1, updateEncoder2, CHANGE);

    myPID.SetMode(AUTOMATIC);                // set PID in Auto mode
    myPID.SetSampleTime(1);                  // refresh rate of PID controller
    myPID.SetOutputLimits(-1 * maxi, maxi);  // this is the MAX PWM value to move motor, here change in value reflect change in speed of motor.
    myPID2.SetMode(AUTOMATIC);               // set PID in Auto mode
    myPID2.SetSampleTime(1);                 // refresh rate of PID controller
    myPID2.SetOutputLimits(-1 * maxi, maxi); // this is the MAX PWM value to move motor, here change in value reflect change in speed of motor.

    //    pinMode(trigPinFront, OUTPUT);
    //    pinMode(echoPinFront, INPUT);
    pinMode(trigPinRight, OUTPUT);
    pinMode(echoPinRight, INPUT);
    pinMode(trigPinLeft, OUTPUT);
    pinMode(echoPinLeft, INPUT);
}

void loop()
{
    moveForward();
    if (center_counter >=3){
      stopMotors();
      delay(10000);
    }
    //    reverseIfNeeded();
    // turnLeft();
    //  turnRight();
    //   stopMotors();
//      getUltrasonicReadings();
//      c = millis();
//      while (millis() - c <= 3000);
}

void calibrate()
{
    speed_1 = maxi;
    speed_2 = maxi;
    getUltrasonicReadings();
    float diff = readingRight - readingLeft;
    //    Serial.println(diff);
    float absDiff = abs(diff);
    float map_value = mapFloat(absDiff, calibrationThreshold, 10, 3, 11); // 2,9 working well with 80 speed
    if (wallRight() && wallLeft())
    {
        if (absDiff > calibrationThreshold)
        {
            if (diff > 0)
            {
                speed_2 = maxi - map_value;
            }
            else
            {
                speed_1 = maxi - map_value;
            }
        }
    }
    else
    {
        if (wallRight() && readingRight > 9)
        {
            float map_value = mapFloat(readingRight, 9.5, 14, 2, 5.5);
            speed_2 = maxi - map_value;
        }
        else if(wallLeft() && readingLeft < 7.5) {
            float map_value = mapFloat(readingLeft, 3, 9, 2, 5.5);
            speed_2 = maxi - map_value;
        }
        else if (wallLeft() && readingLeft > 8)
        {
            float map_value = mapFloat(readingLeft, 9.5, 14, 2, 5.5);
            speed_1 = maxi - map_value;
        }
        else if(wallRight() && readingRight < 7.5) {
            float map_value = mapFloat(readingRight, 3, 9, 2, 5.5);
            speed_1 = maxi - map_value;
        }
    }
}

void getUltrasonicReadings()
{
    //    readingFront = readUltrasonicDistance(trigPinFront, echoPinFront, -1);
    readingRight = readUltrasonicDistance(trigPinRight, echoPinRight, 0);
    readingLeft = readUltrasonicDistance(trigPinLeft, echoPinLeft, 0);
//        if (readingFront >= 500 || readingFront <= 0) {
//          Serial.println("Front Out of range");
//        }
//        else {
//          Serial.print("Sensor Front  ");
//          Serial.print(readingFront);
//          Serial.println("cm");
//        }
//        if (readingRight >= 500 || readingRight <= 0) {
//          Serial.print("Right Out of range");
//          Serial.println(readingRight);
//        }
//        else {
//          Serial.print("Sensor Right  ");
//          Serial.print(readingRight);
//          Serial.println("cm");
//        }
//        if (readingLeft >= 500 || readingLeft <= 0) {
//          Serial.print("Left Out of range");
//          Serial.println(readingLeft);
//        }
//        else {
//          Serial.print("Sensor Left  ");
//          Serial.print(readingLeft);
//          Serial.println("cm");
//        }
}

float readUltrasonicDistance(int triggerPin, int echoPin, int subValue)
{
    pinMode(triggerPin, OUTPUT);
    digitalWrite(triggerPin, LOW);
    long lastRead = micros();
    while (micros() - lastRead < 2)
        ;
    digitalWrite(triggerPin, HIGH);
    lastRead = micros();
    while (micros() - lastRead < 15)
        ;
    digitalWrite(triggerPin, LOW);
    pinMode(echoPin, INPUT);
    float duration = pulseIn(echoPin, HIGH);
    float distance = (duration / 2) / 29.1;
    return distance - subValue;
}

void moveForward()
{
    detectWalls();
    calibrate();
    myPID.Compute(); // calculate new output
    pwmOut(speed_1);
    myPID2.Compute(); // calculate new output
    pwmOut2(speed_2);
    //    delay(50);
}

void changeDirections()
{
    encoderValue2 = 0;
    encoderValue = 0;
    int avg = 0;
    while (avg < turn_number)
    {
        REV = maxi;
        REV2 = -1 * maxi;
        myPID.Compute();
        myPID2.Compute();
        pwmOut(REV);
        pwmOut2(REV2);
        avg = (encoderValue + encoderValue2) / 2;
    }

    encoderValue2 = 0;
    encoderValue = 0;
    avg = 0;
    while (avg < turn_number)
    {
        REV = maxi;
        REV2 = -1 * maxi;
        myPID.Compute();
        myPID2.Compute();
        pwmOut(REV);
        pwmOut2(REV2);
        avg = (encoderValue + encoderValue2) / 2;
    }
}

void turnRight()
{
    encoderValue2 = 0;
    encoderValue = 0;
    int avg = 0;
    while (avg < turn_number)
    {
        REV = maxi;
        REV2 = -1 * maxi;
        myPID.Compute();
        myPID2.Compute();
        pwmOut(REV);
        pwmOut2(REV2);
        avg = (encoderValue + encoderValue2) / 2;
    }
    // moveSlightlyForward();
    stopMotors();
    c = millis();
    while (millis() - c <= 500)
        ;
}

void turnLeft()
{
    encoderValue2 = 0;
    encoderValue = 0;
    int avg = 0;
    while (avg < turn_number)
    {
        REV = -1 * maxi;
        REV2 = maxi;
        myPID.Compute();
        myPID2.Compute();
        pwmOut(REV);
        pwmOut2(REV2);
        avg = (encoderValue + encoderValue2) / 2;
    }
    stopMotors();
    c = millis();
    while (millis() - c <= 500);
}

void moveSlightlyForward()
{
    speed_1 = maxi;
    speed_2 = maxi;
    myPID.Compute(); // calculate new output
    pwmOut(speed_1);
    myPID2.Compute(); // calculate new output
    pwmOut2(speed_2);
    //delay is the worst case scenario here !!
    unsigned long startTime = millis();
    while(millis() - startTime < 1500) {
      calibrate();
      if (digitalRead(IrFront) == 0){
        break;
      }
    }
}

void stopMotors()
{
    finish(MotFwd, MotRev);
    finish(MotFwd2, MotRev2);
}

void pwmOut(int out)
{
    if (out > 0)
    {                                // if REV > encoderValue motor move in forward direction.
        analogWrite(MotEnable, out); // Enabling motor enable pin to reach the desire angle
        forward(MotFwd, MotRev);     // calling motor to move forward
    }
    else
    {
        analogWrite(MotEnable, abs(out)); // if REV < encoderValue motor move in forward direction.
        reverse(MotFwd, MotRev);          // calling motor to move reverse
    }
}

void pwmOut2(int out)
{
    if (out > 0)
    {                                 // if REV > encoderValue motor move in forward direction.
        analogWrite(MotEnable2, out); // Enabling motor enable pin to reach the desire angle
        forward(MotFwd2, MotRev2);    // calling motor to move forward
    }
    else
    {
        analogWrite(MotEnable2, abs(out)); // if REV < encoderValue motor move in forward direction.
        reverse(MotFwd2, MotRev2);         // calling motor to move reverse
    }
}
void updateEncoder()
{
    encoderValue++;
}

void updateEncoder2()
{
    encoderValue2++;
}

void forward(int MotF, int MotR)
{
    digitalWrite(MotF, HIGH);
    digitalWrite(MotR, LOW);
}

void reverse(int MotF, int MotR)
{
    digitalWrite(MotF, LOW);
    digitalWrite(MotR, HIGH);
}
void finish(int MotF, int MotR)
{
    digitalWrite(MotF, LOW);
    digitalWrite(MotR, LOW);
}

void detectWalls()
{
    isWallFront = wallFront();
    isWallRight = wallRight();
    isWallLeft = wallLeft();
    if (isWallFront && isWallLeft && !isWallRight)
    {
        stopMotors();
        delay(500);
        turnRight();
        center_counter ++;
    }
    else if (isWallFront && !isWallLeft && isWallRight)
    {
        stopMotors();
        delay(500);
        turnLeft();
        center_counter = 0;
    }
    else if (isWallFront && !isWallLeft && !isWallRight)
    {
        stopMotors();
        delay(500);
        turnLeft();
        center_counter = 0;
    }
    else if (isWallFront && isWallLeft && isWallRight)
    {
        stopMotors();
        delay(500);
        changeDirections();
        center_counter = 0;
    }
    else if(!isWallFront && !isWallLeft) {
      delay(410);
      stopMotors();
      delay(500);
      turnLeft();
      moveSlightlyForward();
      center_counter = 0;
    }
}

void reverseIfNeeded()
{
    //  readingFront = readUltrasonicDistance(trigPinFront, echoPinFront, -1);
    //  if(readingFront <= 3) {
    //    speed_1 = -1 * maxi;
    //    speed_2 = -1 * maxi;
    //    myPID.Compute(); // calculate new output
    //    pwmOut(speed_1);
    //    myPID2.Compute(); // calculate new output
    //    pwmOut2(speed_2);
    delay(400);
    //  }
}

boolean wallFront()
{
    //    int readingFronts = readUltrasonicDistance(trigPinFront, echoPinFront, -1);
    //    if (readingFronts >= 0 && readingFronts <= 9)
    //        return true;
    //    else
    //        return false;
    return digitalRead(IrFront) == 0;
}

boolean wallLeft()
{
    return digitalRead(IrLeft) == 0;
}

boolean wallRight()
{
    return digitalRead(IrRight) == 0;
}
