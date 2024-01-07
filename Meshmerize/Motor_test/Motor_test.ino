/*
Sample code to test your LF-2 Robot motor connections. 
This code will make your robot go forward for 1 second and stop for 1 second. 
In case of a different behaviour you will have to switch the motor connections. 
*/

#include <SparkFun_TB6612.h>

#define AIN1 4
#define BIN1 6
#define AIN2 3
#define BIN2 7
#define PWMA 9
#define PWMB 10
#define STBY 5

const int offsetA = 1;
const int offsetB = 1;

Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY);


void setup() {

}

void loop() {
  motor1.drive(100);
  motor2.drive(100);
  delay(1000);

  motor1.drive(0);
  motor2.drive(0);
  delay(2000);
}
