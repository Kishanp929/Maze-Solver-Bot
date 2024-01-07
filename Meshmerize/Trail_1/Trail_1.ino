// IR Sensor
// IR Sensor ka reading ranges between (0-1023)
// If white surface is detected then it outputs 1023 ~5v
// If Black surface is detected then it outputs 0 value ~0v






#define leftFarSensor      A0      // We are defining leftfarsensor as A0 so we will use analogRaed(leftFarSensor) instead of analogRaed(A0)
#define leftNearSensor     A1
#define centerSensor       A2
#define rightNearSensor    A3
#define rightFarSensor     A4


// IR Sensor readings
int leftFarReading;
int leftNearReading;
int centerReading;
int rightNearReading;
int rightFarReading;

int leftNudge;
int replaystage;
int rightNudge;

// Enable Pins
#define enl 9   // LEFT
#define enr 10  // RIGHT

// speed of motor
#define spd 150

#define leapTime 200

// Motors
#define leftMotor1  5
#define leftMotor2  2
#define rightMotor1 4
#define rightMotor2 3

//for storing path details
char path[30] = {};
int pathLength;
int readLength;

void readSensors()	//accepts input from sensors
{
  leftFarReading = analogRead(leftFarSensor);
  leftNearReading = analogRead(leftNearSensor);
  centerReading = analogRead(centerSensor);
  rightFarReading = analogRead(rightFarSensor);
  rightNearReading = analogRead(rightNearSensor);
  
}


void loop(){                       // IR Sensor ka reading ranges between (0-1023) 
	readSensors();
  	//if only the middle two sensors can see the black line(white paper pe black line)
	if(leftFarReading<200 && rightFarReading<200 && (leftNearReading>200 && rightNearReading>200 && )) 
	{ 
		straight();                                                                                    
	}
  	//otherwise goes to the leftHandWall method
	else
	{                                                                                              
		leftHandWall();                                                                                   
	}
}



























