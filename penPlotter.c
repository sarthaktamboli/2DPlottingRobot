//	MTE 1A Final Project - 2D Plotter
//	Simeng Yang, Jeremy Afoke, Zong Xu, Sarthak Tamboli

#define ASIZE 25
#include "NXT_FileIO.c"

//	Lifts the pen from a lowered position
//	Returns the Boolean state of the pen (true is lifted)
bool liftHoist(int &encoderCountZ){
	nMotorEncoder[motorA] = 0;
	motor[motorA] = -10;
	//	While the touch sensor has not been activated and the ultrasonic sensor
	//	does not detect an object closer than 20 cm length-wise along the horizontal axis
	while (SensorValue[S4] == 0 && SensorValue[S3] > 20){}
	//	Update the encoder count
	encoderCountZ = nMotorEncoder[motorA];
	//	Turn off the motor
	motor[motorA] = 0;
	return true;
}

//	Lowers the pen from a raised position
//	Returns the Boolean state of the pen (false is lowered)
bool lowerHoist(int encoderCountZ){
	nMotorEncoder[motorA] = 0;
	motor[motorA] = 10;
	//	While the pen is not restored to 0-height and the ultrasonic sensor
	//	does not detect an object closer than 20 cm length-wise along the horizontal axis
	while (nMotorEncoder[motorA] < abs(encoderCountZ) && SensorValue[S3] > 20){}
	//	Turn off the motor
	motor[motorA] = 0;
	return false;
}

//	Resets the hoist to the initial horizontal position (x = 0)
//	Returns 0 for motor encoder to reset the horizontal position in main
float resetHorizontal(int &encoderCountX){
	nMotorEncoder[motorC] = 0;
	motor[motorC] = -30;
	//	While touch sensor has not been actovated and the ultrasonic sensor
	//	does not detect an object closer than 20 cm length-wise along the horizontal axis
	while(SensorValue[S2] == 0 && SensorValue[S3] > 20){}
	//	Reverse motor temporarily to relieve pressure on hoist
	motor[motorC] = 10;
	wait1Msec(500);
	//	Turn off motor
	motor[motorC] = 0;
	encoderCountX = nMotorEncoder[motorC];
	//	Reset motor encoder
	nMotorEncoder[motorC] = 0;
	return 0;
}

//	Resets the hoist to the initial vertical position (y = 0)
//	Returns 0 to reset the vertical position in main
float resetVertical(){
	motor[motorB] = 20;
	//	While the colour sensor detects white (paper) and the ultrasonic sensor
	//	does not detect an object closer than 20 cm length-wise along the horizontal axis
	while(SensorValue[S1] != 6 && SensorValue[S3] > 20){}
	//	Wait a bit to ensure the paper is sufficiently fed in
	wait1Msec(1000);
	//	Turn off motor
	motor[motorB] = 0;
	//	Reset motor encoder
	nMotorEncoder[motorB] = 0;
	return 0;
}

//	Moves the pen horizontally and vertically
void translateDiagonal(float distX, float distY, float &x, float &y, int encoderCountX){
	//	Default motor power should be supplied to motor (x or y) travelling greater distance
	//	The other motor power will be defined relative to the default power, by a distance ratio,
	//	so both motors cover the required distance over the same time.
	//	The motor powers must be constrained so that the pen does not travel too quickly in either direction.
	//	We have constrained the default motor power to be an upper limiting bound.
	float const MOTORPOW = 25;
	float distRatio;
	//	Horizontal distance is less than vertical distance
	//	Horizontal motor will have to rotate slower than vertical motor
	if (abs(distX) < abs(distY)){
		//	Distance ratio between x and y (less than 1)
		distRatio = abs(distX / distY);
		//	Assigns sign for direction of motion (positive or negative)
		if (distY < 0)
			motor[motorB] = -MOTORPOW;	//	Y
	  else
	  	motor[motorB] = MOTORPOW;	//	Y
	 //	Adjust motor power by distance ratio
	  if (distX < 0)
			motor[motorC] = -distRatio * MOTORPOW;	//	X
	  else
	  	motor[motorC] = distRatio * MOTORPOW;	//	X
	}
	//	Vertical distance is less than horizontal distance
	//	Vertical motor will have to rotate slower than horizontal motor
	else{
		//	Distance ratio between y and x (less than or equal to 1)
		distRatio = abs(distY / distX);
		//	Adjust motor power by distance ratio
		if (distY < 0)
			motor[motorB] = -distRatio * MOTORPOW;	//	Y
	  else
	  	motor[motorB] = distRatio * MOTORPOW;	//	Y
	  if (distX < 0)
			motor[motorC] = -MOTORPOW;	//	X
	  else
	  	motor[motorC] = MOTORPOW;	//	X
	}

	//	Reset motor encoders
	nMotorEncoder[motorB] = 0;
	nMotorEncoder[motorC] = 0;
	// There are ~1350 ticks for 15.65 cm, with each tick being ~ 0.116 cm
	int numTicksX = distX / 15.65 * encoderCountX;
	// 1000 ticks is ~12.03 cm, with each tick being ~ 0.012 cm
	int numTicksY = distY / 12.03 * 1000;
	//	While the distance to be traverse is not met and the ultrasonic sensor
	//	does not detect an object closer than 20 cm length-wise along the horizontal axis
	while(abs(nMotorEncoder[motorC]) <= abs(numTicksX) && SensorValue[S2] == 0
		&& abs(nMotorEncoder[motorB]) <= abs(numTicksY) && SensorValue[S3] > 20){}
  //	Turn off motors
	motor[motorB] = 0;
	motor[motorC] = 0;
	//	Update x and y position of hoist in main
	x += distX;
	y += distY;
}

//	Feeds in the paper
void feedPaper(){
	motor[motorB] = 20;
	//	Motor active while the colour sensor does not detect white (paper) and the ultrasonic sensor
	//	does not detect an object closer than 20 cm length-wise along the horizontal axis
	while(SensorValue[S1] != 6 && SensorValue[S3] > 20){}
	motor[motorB] = 0;
}

//	Ejects paper from tray
void ejectPaper(){
	motor[motorB] = 20;
	//	Motor active while the colour sensor detects white (paper) and the ultrasonic sensor
	//	does not detect an object closer than 20 cm length-wise along the horizontal axis
	while(SensorValue[S1] == 6 && SensorValue[S3] > 20){}
	motor[motorB] = 0;
}

//	point struct for x, y and z coordinates
struct point {
	float x[ASIZE];
	float y[ASIZE];
	bool z[ASIZE];
};

//	Initialize x, y and z for a given point
void initArray(point a, int index, float x, float y, bool z){
	a.x[index] = x;
	a.y[index] = y;
	a.z[index] = z;
}

task main()
{
	//	Sensor configuration
	SensorType[S1] = sensorColorNxtFULL;
	SensorType[S2] = sensorTouch;
	SensorType[S3] = sensorSONAR;
	SensorType[S4] = sensorTouch;
	int encoderCountZ = 0, encoderCountX = 0;
	int index = 0;
	float x = 0, y = 0;
	//	State of z (lifted or lowered)
	bool z = false;
	//	Distance along z axis
	int zValue;

	point a;

	//	Read in point arrays from a text file
	//	Point data is generated from an external C++ graphics application
	TFileHandle fin;
	bool fileOkay = openReadPC(fin, "data.txt");
	if (!fileOkay)
	{
		displayString(0, "Infile error");
	}
	//	If file is opened
	else{
		//	Read file data into point struct arrays while data is available
		while (readFloatPC(fin, x)){
			readFloatPC(fin, y);
			readIntPC(fin, zValue);
			initArray(a, index, x, y, zValue);
			index++;
		};
	}

	//	Initialization
	//	Lift pen from lowered position and update Z state
	z = liftHoist(encoderCountZ);
	//	Feed in paper
	feedPaper();
	wait1Msec(1000);
	//	Resets paper horizontally and vertically to (0,0) starting location
	x = resetHorizontal(encoderCountX);
	y = resetVertical();
	int counter = 0;

	//	Location from array
	while(counter < index){
		//	Need to be lifted and not already lifted
		if (a.z[counter] == 1 && z == false)
			z = liftHoist(encoderCountZ);
		//	Need to be lowered and not already lowered
		else if (a.z[counter] == 0 && z == true)
			z = lowerHoist(encoderCountZ);
		wait1Msec(300);
		//	Translates the pen horizontally and vertically
		displayString(0, "dx at [%d] = %f", counter, a.x[counter] - x);
		displayString(1, "dy at [%d] = %f", counter, a.y[counter] - y);
		translateDiagonal(a.x[counter] - x, a.y[counter] - y, x, y, encoderCountX);
		counter++;
	}

	//	Lift pen from lowered position
	liftHoist(encoderCountZ);
	//	Push end of paper to colour sensor
	feedPaper();
	//	Push rest of paper off colour sensor
	ejectPaper();
	//	Lower hoist
	lowerHoist(encoderCountZ);
	displayString(0, "%s", "GROUP 47 :)");
	while (nNxtButtonPressed == -1){}
	while (nNxtButtonPressed != -1){}
}
