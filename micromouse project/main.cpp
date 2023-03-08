#include <MapFloat.h>
#define Motor1Enable 5 // Motor Enamble pin Runs on PWM signal
#define Motor1Forward A1   // Motor Forward pin
#define Motor1Reverse A0   // Motor Reverse pin

#define Motor2Enable 6 // Motor Enamble pin Runs on PWM signal
#define Motor2Forward 12   // Motor Forward pin
#define Motor2Reverse 13   // Motor Reverse pin

#define IrLeft A4
#define IrRight A5
#define IrFront 4

int encoderPin11 = 2;   // Encoder Output 'A' must connected with interrupt pin of arduino
int encoderPin12 = A2;  // Encoder Otput 'B' must connected with intreput pin of arduino
int encoderPin21 = 3;  // Encoder Output 'A' must connected with intreput pin of arduino
int encoderPin22 = A3; // Encoder Otput 'B' must connected with intreput pin of arduino

volatile long encoderValue = 0; // Raw encoder value

int trigPinRight = 8;
int echoPinRight = 9;
int trigPinLeft = 7;
int echoPinLeft = 11;

boolean isWallFront = false;
boolean isWallRight = false;
boolean isWallLeft = false;

//variable to control the max speed the motor can move
int maxi = 100;

int motor1Speed = maxi;
int motor2Speed = maxi;

volatile int lastEncoded = 0;   // Here updated value of encoder store.
int REV = 0;                    // Set point REQUIRED ENCODER VALUE
volatile long encoderValue2 = 0; // Raw encoder value
int REV2 = 0;

unsigned long c = 0;
unsigned long justTurnedLeft;

float readingRight = 0;
float readingLeft = 0;

float calibrationThreshold = 3.8;

int turn_number = 180;

int centerCounter = 0;

void setup()
{
    Serial.begin(9600); // initialize serial comunication

    // define pin mode for IR sensors
    pinMode(IrLeft, INPUT);
    pinMode(IrRight, INPUT);
    pinMode(IrFront, INPUT);

    //define trigger and echo pins for the ultrasonic sensors
    pinMode(trigPinRight, OUTPUT);
    pinMode(echoPinRight, INPUT);
    pinMode(trigPinLeft, OUTPUT);
    pinMode(echoPinLeft, INPUT);

    //define pinmode for motor 1
    pinMode(Motor1Enable, OUTPUT);
    pinMode(Motor1Forward, OUTPUT);
    pinMode(Motor1Reverse, OUTPUT);

    //define pinmode for motor 1 encoders
    pinMode(encoderPin11, INPUT_PULLUP);
    pinMode(encoderPin12, INPUT_PULLUP);
    digitalWrite(encoderPin11, HIGH); // turn pullup resistor on
    digitalWrite(encoderPin12, HIGH); // turn pullup resistor on

    //define pinmode for motor 2
    pinMode(Motor2Enable, OUTPUT);
    pinMode(Motor2Forward, OUTPUT);
    pinMode(Motor2Reverse, OUTPUT);

    //define pinmode for motor 2 encoders
    pinMode(encoderPin21, INPUT_PULLUP);
    pinMode(encoderPin22, INPUT_PULLUP);
    digitalWrite(encoderPin21, HIGH); // turn pullup resistor on
    digitalWrite(encoderPin22, HIGH); // turn pullup resistor on


    //physical interrupt
    //call updateEncoder when any change happens to pin 2 or pin 3
    attachInterrupt(0, updateEncoder, CHANGE);
    attachInterrupt(1, updateEncoder2, CHANGE);

}

void loop()
{
    moveForward();
    if (centerCounter >=3)
    {
        stopMotors();
        delay(10000);
    }
}


void moveForward()
{
    calibrate();
    detectWalls();
    pwmOut(motor1Speed);
    pwmOut2(motor2Speed);
}


void detectWalls()
{
    isWallFront = wallFront();
    isWallRight = wallRight();
    isWallLeft = wallLeft();
    if(isWallFront && isWallRight && isWallLeft ){
        stopMotors();
        delay(500);
        changeDirections();
        centerCounter = 0; //not possible center cells - reset      
    }
    else if (isWallFront && !isWallRight && isWallLeft){
       stopMotors();
        delay(500);
        turnRight();
        centerCounter++; //possible cneter cells - increment counter
    }
    else if(isWallFront && isWallRight && !isWallLeft){
      stopMotors();
      delay(500);
      turnLeft();
      centerCounter = 0; //not possible center cells - reset
    }
    else if(isWallFront && !isWallRight && !isWallLeft){
      stopMotors();
      delay(500);
      turnLeft();
      centerCounter = 0; //not possible center cells - reset
    }
    else if(!isWallFront && isWallRight && !isWallLeft && (millis() - justTurnedLeft > 1150)){
      delay(410);
      stopMotors();
      delay(500);
      turnLeft();
      moveSlightlyForward();
      centerCounter = 0; //not possible center cells - reset
    }
    calibrate();

    // if (isWallFront && isWallLeft && !isWallRight)
    // {
    //     stopMotors();
    //     delay(500);
    //     turnRight();
    //     centerCounter++; //possible cneter cells - increment counter
    // }
    // else if (isWallFront && !isWallLeft && isWallRight)
    // {
    //     stopMotors();
    //     delay(500);
    //     turnLeft();
    //     centerCounter = 0; //not possible center cells - reset
    // }
    // else if (isWallFront && !isWallLeft && !isWallRight)
    // {
    //     stopMotors();
    //     delay(500);
    //     turnLeft(); //following the left hand follower algorithm
    //     centerCounter = 0; //not possible center cells - reset
    // }
    // else if (isWallFront && isWallLeft && isWallRight)
    // {
    //     stopMotors();
    //     delay(500);
    //     changeDirections();
    //     centerCounter = 0; //not possible center cells - reset
    // }
    // else if(!isWallFront && !isWallLeft) //TODO: Get back to this case
    // {
    //     delay(410);
    //     stopMotors();
    //     delay(500);
    //     turnLeft();
    //     moveSlightlyForward();
    //     centerCounter = 0; //not possible center cells - reset
    // }
}

void stopMotors()
{
    digitalWrite(Motor1Forward, LOW);
    digitalWrite(Motor1Reverse, LOW);
    digitalWrite(Motor2Forward, LOW);
    digitalWrite(Motor2Reverse, LOW);
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
        pwmOut(REV);
        pwmOut2(REV2);
        avg = (encoderValue + encoderValue2) / 2;
    }
    justTurnedLeft = millis();
    stopMotors();
    c = millis();
    while (millis() - c <= 500);
}

void changeDirections() //turn right twice
{
    encoderValue2 = 0;
    encoderValue = 0;
    int avg = 0;
    while (avg < turn_number)
    {
        REV = maxi;
        REV2 = -1 * maxi;
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
        pwmOut(REV);
        pwmOut2(REV2);
        avg = (encoderValue + encoderValue2) / 2;
    }
}


void moveSlightlyForward()
{
    motor1Speed = maxi;
    motor2Speed = maxi;
    pwmOut(motor1Speed);
    pwmOut2(motor2Speed);
    //delay is the worst case scenario here !!
    unsigned long startTime = millis();
    while(millis() - startTime < 800) //!
    {
        calibrate();
        if (digitalRead(IrFront) == 0)
        {
            break;
        }
    }
}

void calibrate()
{
    motor1Speed = maxi;
    motor2Speed = maxi;
    getUltrasonicReadings();
    float diff = readingRight - readingLeft;
    float absDiff = abs(diff);
    float map_value = mapFloat(absDiff, calibrationThreshold, 10, 3, 11); // 2,9 working well with 80 speed
    if (wallRight() && wallLeft())
    {
        if (absDiff > calibrationThreshold)
        {
            if (diff > 0)
            {
                motor2Speed = maxi - map_value;
            }
            else
            {
                motor1Speed = maxi - map_value;
            }
        }
    }
    else
    {
        if (wallRight() && readingRight > 9)
        {
            float map_value = mapFloat(readingRight, 9.5, 14, 2, 5.5);
            motor2Speed = maxi - map_value;
        }
        else if(wallLeft() && readingLeft < 7.5)
        {
            float map_value = mapFloat(readingLeft, 4, 9, 2, 5.5);
            motor2Speed = maxi - map_value;
        }
        else if (wallLeft() && readingLeft > 8)
        {
            float map_value = mapFloat(readingLeft, 9.5, 14, 2, 5.5);
            motor1Speed = maxi - map_value;
        }
        else if(wallRight() && readingRight < 7.5)
        {
            float map_value = mapFloat(readingRight, 4, 9, 2, 5.5);
            motor1Speed = maxi - map_value;
        }
    }
    Serial.print("Left: ");
    Serial.print(motor1Speed);
    Serial.print("  , right: ");
    Serial.println(motor2Speed);    
}

void getUltrasonicReadings()
{
    readingRight = readUltrasonicDistance(trigPinRight, echoPinRight);
    readingLeft = readUltrasonicDistance(trigPinLeft, echoPinLeft);
}

float readUltrasonicDistance(int triggerPin, int echoPin) //TODO: Verify the reading is done correctly
{
    int subValue = 0;
    // pinMode(triggerPin, OUTPUT);
    digitalWrite(triggerPin, LOW);
    long lastRead = micros();
    while (micros() - lastRead < 2)
        ;
    digitalWrite(triggerPin, HIGH);
    lastRead = micros();
    while (micros() - lastRead < 15)
        ;
    digitalWrite(triggerPin, LOW);
    // pinMode(echoPin, INPUT);
    float duration = pulseIn(echoPin, HIGH);
    float distance = (duration / 2) / 29.1;
    return distance - subValue;
}


void pwmOut(int outSpeed)
{
    if (outSpeed > 0) //move forward
    {
        analogWrite(Motor1Enable, outSpeed); // Enabling motor enable pin to reach the desire angle
        forward(Motor1Forward, Motor1Reverse); //move motor forward
    }
    else //move backward
    {
        analogWrite(Motor1Enable, abs(outSpeed)); // Enabling motor enable pin to reach the desire angle
        reverse(Motor1Forward, Motor1Reverse); //move motor backwards
    }
}

void pwmOut2(int outSpeed)
{
    if (outSpeed > 0) //move forward
    {
        // if REV > encoderValue motor move in forward direction.
        analogWrite(Motor2Enable, outSpeed); // Enabling motor enable pin to reach the desire angle
        forward(Motor2Forward, Motor2Reverse); //move motor forward
    }
    else //move backward
    {
        analogWrite(Motor2Enable, abs(outSpeed)); // if REV < encoderValue motor move in forward direction.
        reverse(Motor2Forward, Motor2Reverse);  //move motor backwards
    }
}

//called when physical interrupt occurs to pin 2 from motor 1
void updateEncoder()
{
    encoderValue++;
}

//called when physical interrupt occurs to pin 2 from motor 2
void updateEncoder2()
{
    encoderValue2++;
}

//move the motor forward
void forward(int MotF, int MotR)
{
    digitalWrite(MotF, HIGH);
    digitalWrite(MotR, LOW);
}

//move the motor backwards
void reverse(int MotF, int MotR)
{
    digitalWrite(MotF, LOW);
    digitalWrite(MotR, HIGH);
}

//detect if there is a wall to the front
boolean wallFront()
{
    return digitalRead(IrFront) == 0;
}

//detect if there is a wall to the left
boolean wallLeft()
{
    return digitalRead(IrLeft) == 0;
}

//chack if there is a wall to the right
boolean wallRight()
{
    return digitalRead(IrRight) == 0;
}
