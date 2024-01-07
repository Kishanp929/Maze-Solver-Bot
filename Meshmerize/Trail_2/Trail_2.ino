
#include <SparkFun_TB6612.h>  // driver library

#define AIN1 4
#define BIN1 6
#define AIN2 3
#define BIN2 7
#define PWMA 9
#define PWMB 10
#define STBY 5

// these constants are used to allow you to make your motor configuration
// line up with function names like forward.  Value can be 1 or -1
const int offsetA = 1;
const int offsetB = 1;

// Initializing motors.  The library will allow you to initialize as many
// motors as you have memory for.  If you are using functions like forward
// that take 2 motors as arguements you can either write new functions or
// call the function more than once.

Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY);


int P, D, I, previousError, PIDvalue, error;
int lsp, rsp;
int lfspeed = 200;  // line following speed 
int tuspeed = 50;  // turn taking speed

float Kp = 0;      // positional constant
float Kd = 0;      // differential constant
float Ki = 0 ;     // integral constant


int minValues[5], maxValues[5], threshold[5];

void setup()
{
  Serial.begin(9600);
  pinMode(11, INPUT_PULLUP);   //after pressing this pin calibration will start
  pinMode(12, INPUT_PULLUP);   //after pressing this pin the bot will start moving
}


void loop()
{


  MazeSolve();


}


void MazeSolve();
  while (digitalRead(11)) {}
  delay(1000);
  calibrate();
  while (digitalRead(12)) {}
  delay(1000);

  while (1)
  {
    if (analogRead(1) > threshold[1] && analogRead(5) < threshold[5] )
    {
      lsp = 0; rsp = lfspeed;
      motor1.drive(0);
      motor2.drive(lfspeed);
    }

    else if (analogRead(5) > threshold[5] && analogRead(1) < threshold[1])
    { lsp = lfspeed; rsp = 0;
      motor1.drive(lfspeed);
      motor2.drive(0);
    }
    else if (analogRead(3) > threshold[3])
    {
      Kp = 0.0006 * (1000 - analogRead(3));
      Kd = 10 * Kp;
      //Ki = 0.0001;
      linefollow();
    }
  }
}






void calibrate()
{
  for ( int i = 1; i < 6; i++)
  {
    minValues[i] = analogRead(i);
    maxValues[i] = analogRead(i);
  }
  
  for (int i = 0; i < 3000; i++)
  {
    motor1.drive(50);
    motor2.drive(-50);

    for ( int i = 1; i < 6; i++)
    {
      if (analogRead(i) < minValues[i])
      {
        minValues[i] = analogRead(i);
      }
      if (analogRead(i) > maxValues[i])
      {
        maxValues[i] = analogRead(i);
      }
    }
  }

  for ( int i = 1; i < 6; i++)
  {
    threshold[i] = (minValues[i] + maxValues[i]) / 2;
    Serial.print(threshold[i]);
    Serial.print("   ");
  }
  Serial.println();
  
  motor1.drive(0);
  motor2.drive(0);
}







void linefollow()
{ while(1)
  {
    int error = (analogRead(4) - analogRead(2)); 

    P = error;                // here we don't know the value of I and previousError, i think it should be initiallized 
    I = I + error;
    D = error - previousError;

    PIDvalue = (Kp * P) + (Ki * I) + (Kd * D);
    previousError = error;

    lsp = lfspeed - PIDvalue;
    rsp = lfspeed + PIDvalue;

    if (lsp > 255) {
      lsp = 255;
    }
    if (lsp < 0) {
      lsp = 0;
    }
    if (rsp > 255) {
      rsp = 255;
    }
    if (rsp < 0) {
      rsp = 0;
    }
    motor1.drive(lsp);
    motor2.drive(rsp);


    if (analogRead(1) > threshold || analogRead(5) > threshold)      //If there is a left turn, right turn or an intersection of left+right, or left+right+straight 
    { 
    motor1.drive(lfspeed);         // to move a motor a bit ahead
    motor2.drive(lfspeed);
    // delay(100);
    // motor1.drive(0);
    // motor2.drive(0);
    return; 
    }

    // if there is a dead end
    if (analogRead(1) < threshold[1] && analogRead(2) < threshold[2] && analogRead(3) < threshold[3] && analogRead(4) < threshold[4] && analogRead(5) < threshold[5])
    {    
    motor1.drive(lfspeed);
    motor2.drive(lfspeed);
    // delay(100);
    // motor1.drive(0);
    // motor2.drive(0);
    return;
    }
  }
}




void turn(char dir)
{
  switch (dir)
  { case 'L':
    motor1.drive(tuspeed);
    motor2.drive(-tuspeed);
      while (analogRead(1) < threshold[1])
      {
        
      }

      while (analogRead(1) < threshold[1] && analogRead(2) < threshold[2] && analogRead(3) < threshold[3] ) // wait for 1,2,3 sensors to find the line
      {
        
      }
      
      
      digitalLow( leftMotor1 );
      digitalHigh( leftMotor2 );
      digitalHigh( rightMotor2 );
      digitalHigh( rightMotor1 );
      analogWrite(rightMotorPWM, 255);
      break;

    // Turn right 90deg
    case 'R':
      digitalLow( rightMotor1 );
      digitalHigh( rightMotor2 );
      digitalLow( leftMotor1 );
      digitalHigh( leftMotor2 );
      analogWrite(rightMotorPWM, speedturn);
      analogWrite(leftMotorPWM, speedturn);
      line_position = qtrrc.readLine(sensorValues);

      while (sensorValues[6] < threshold)  // wait for outer most sensor to find the line
      {
        line_position = qtrrc.readLine(sensorValues);
      }
      line_position = qtrrc.readLine(sensorValues);

      while (sensorValues[4] < threshold || sensorValues[3] < threshold) // wait for outer most sensor to find the line
      {
        line_position = qtrrc.readLine(sensorValues);
      }

      digitalLow( rightMotor1 );
      digitalHigh( rightMotor2 );
      digitalLow( leftMotor1 );
      digitalHigh( leftMotor2 );
      analogWrite(leftMotorPWM, 255);
      break;

    // Turn right 180deg to go back
    case 'B':
     digitalLow( rightMotor1 );
      digitalHigh( rightMotor2 );
      digitalLow( leftMotor1 );
      digitalHigh( leftMotor2 );
      analogWrite(rightMotorPWM, speedturn);
      analogWrite(leftMotorPWM, speedturn);
      line_position = qtrrc.readLine(sensorValues);

      while (sensorValues[6] < threshold)  // wait for outer most sensor to find the line
      {
        line_position = qtrrc.readLine(sensorValues);
      }
      line_position = qtrrc.readLine(sensorValues);

      while (sensorValues[4] < threshold || sensorValues[3] < threshold) // wait for outer most sensor to find the line
      {
        line_position = qtrrc.readLine(sensorValues);
      }

      digitalLow( rightMotor1 );
      digitalHigh( rightMotor2 );
      digitalLow( leftMotor1 );
      digitalHigh( leftMotor2 );
      analogWrite(leftMotorPWM, 255);
      break;
     


    case 'S':

      break;
  }
}


// This function decides which way to turn during the learning phase of
// maze solving.  It uses the variables found_left, found_straight, and
// found_right, which indicate whether there is an exit in each of the
// three directions, applying the "left hand on the wall" strategy.
char select_turn(unsigned char found_left, unsigned char found_straight, unsigned char found_right)
{


  if (found_left)
    return 'L';
  else if (found_straight)
    return 'S';
  else if (found_right)
    return 'R';
  else
    return 'B';
}