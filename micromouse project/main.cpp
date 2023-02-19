#include <PID_v1.h>

#define Motor1Enable 5
#define Motor1Forward A1
#define Motor1Backward A0

#define Motor2Enable 6
#define Motor2Forward 12
#define Motor2Backward 13

//TODO: make better names of encoder pins when connecting them
#define encoderPin1 2 //encoder output 'A' must be connected with interrupt pin of arduino
#define encoderPin2 A2 //Encoder Output 'B' must connected with interrupt pin of arduino
#define encoderPin12 3 //encoder output 'A' must be connected with interrupt pin of arduino
#define encoderPin22 A3 //encoder output 'B' must be connected with interrupt pin of arduino

volatile int encoderValue1 = 0;
volatile int encoderValue2 = 0;

int maxi = 100; //TODO: why is this used?

int speed1 = maxi;
int speed2 = maxi;

void setup()
{
    Serial.begin(9600);

    //pinMode for motors
    pinMode(Motor1Enable, OUTPUT);
    pinMode(Motor1Forward, OUTPUT);
    pinMode(Motor1Backward, OUTPUT);

    pinMode(Motor2Enable, OUTPUT);
    pinMode(Motor2Forward, OUTPUT);
    pinMode(Motor2Backward, OUTPUT);

    pinMode(encoderPin1, INPUT_PULLUP);
    pinMode(encoderPin2, INPUT_PULLUP);

    //TODO: See how this part works exactly
    digitalWrite(encoderPin1, HIGH); //turn pullup resistor on
    digitalWrite(encoderPin2, HIGH); //turn pullup resistor on

    pinMode(encoderPin12, INPUT_PULLUP);
    pinMode(encoderPin22, INPUT_PULLUP);

    //TODO: See how this part works exactly
    digitalWrite(encoderPin12, HIGH); //turn pullup resistor on
    digitalWrite(encoderPin22, HIGH); //turn pullup resistor on

    attachInterrupt(0, updateEncoder1, CHANGE);
    attachInterrupt(1, updateEncoder2, CHANGE);

    myPID.SetMode(AUTOMATIC);                // set PID in Auto mode
    myPID.SetSampleTime(1);                  // refresh rate of PID controller
    myPID.SetOutputLimits(-1 * maxi, maxi);  // this is the MAX PWM value to move motor, here change in value reflect change in speed of motor.
    myPID2.SetMode(AUTOMATIC);               // set PID in Auto mode
    myPID2.SetSampleTime(1);                 // refresh rate of PID controller
    myPID2.SetOutputLimits(-1 * maxi, maxi); // this is the MAX PWM value to move motor, here change in value reflect change in speed of motor.
}

void loop()
{
    analogWrite(Motor1Enable, 30); // TODO: change speed in here
    makeMotorMoveForward(Motor1Forward, Motor1Backward);

    delay(10000);

    makeMotorMoveBackward(Motor1Forward, Motor1Backward);
    //moveForward();
}

void moveForward()
{
    //how is this working?
    myPID.compute();
    motor1PwmOut(speed_1);
    myPID2.compute();
    motor2PwmOut(speed_2);
}

void motor1PwmOut(int out)
{
    if(out > 0)
    {
        analogWrite(Motor1Enable, out);
        makeMotorMoveForward(Motor1Forward, Motor1Backward);
    }
    else
    {
        analogWrite(Motor1Enable, abs(out));
        makeMotorMoveBackward(Motor1Forward, Motor1Backward);
    }
}

void motor2PwmOut(int out)
{
    if(out > 0)
    {
        analogWrite(Motor2Enable, out);
        makeMotorMoveForward(Motor2Forward, Motor2Backward);
    }
    else
    {
        analogWrite(Motor2Enable, abs(out));
        makeMotorMoveBackward(Motor2Forward, Motor2Backward);
    }
}

//used in motor1PwmOut and motor2PwmOut
void makeMotorMoveForward(int motFor, int motBack)
{
    digitalWrite(motFor, HIGH);
    digitalWrite(motBack, LOW);
}

//used in motor1PwmOut and motor2PwmOut
void makeMotorMoveBackward(int motFor, int motBack)
{
    digitalWrite(motFor, LOW);
    digitalWrite(motBack, HIGH);
}

// used with attaching the interrupt
void updateEncoder1()
{
    encoderValue1++;
}

// used with attaching the interrupt
void updateEncoder2()
{
    encoderValue2++;
}






