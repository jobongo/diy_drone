/*
 Name:		Controller.ino
 Created:	3/1/2017 10:06:02 PM
 Author:	Matthew
*/
/*
X Potentiometer to Arduino A0
Y Potentiometer to Arduino A1
*/

/*-----( Import needed libraries )-----*/

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

/*-----( Declare Constants and Pin Numbers )-----*/

#define radioCEPin 9
#define radioCSNPin 10

#define rollPin A0
#define pitchPin A1
#define throttlePin A2
#define yawPin A3
/*
#define RP_P_PIN A9
#define RP_I_PIN A8
#define RP_D_PIN A7

#define Y_P_PIN A6
#define Y_I_PIN A5
#define Y_D_PIN A4
*/



// NOTE: the "LL" at the end of the constant is "LongLong" type
/*

NRF24L01 Setup

GND	-> GND
VCC	-> 3.3v Nrf24L01 is rated for up to 3.6v
CSN	-> Pin 10
CE	-> Pin 9
SCK	-> Pin 13
MOSI-> Pin 11
MISO-> Pin 12
IRG	-> Not Used
*/
const uint64_t pipe = 0xFFFFLL; // Define the transmit pipe
RF24 radio(radioCEPin, radioCSNPin); // Create a Radio
							 /*-----( Declare Variables )-----*/

//float controller[10];  // 5 element array holding Joystick Readings, Throttle, and X, Y Zeroized positions
float controller[4];  // 5 element array holding Joystick Readings, Throttle, and X, Y Zeroized positions


//Define Joystick Variables
int xInputZero;
int yInputZero;
int xInputValue;
int yInputValue;
int yawZero;

float RP_P_Corr;
float RP_I_Corr;
float RP_D_Corr;

float Y_P_Corr;
float Y_I_Corr;
float Y_D_Corr;


float pRollFactor;
float iRollFactor;
float dRollFactor;

float pYawFactor;
float iYawFactor;
float dYawFactor;

float yawPinValue;
float yawValue;

int minThrottle = 650;
int maxThrottle = 2000;

int minRoll = -30;
int maxRoll = 30;

int minPitch = minRoll;
int maxPitch = maxRoll;

int minYawAdj = -75; // Raise this to get higher resolution of tuning for yaw adjustmet.
int maxYawAdj = 75;
//----------------------        Setup       --------------------------//
//----------------------********************--------------------------//

void setup()
{
	analogReadRes(12);
	analogReadAveraging(40);
	//Serial.begin(115200); Uncomment if using arduino. Not needed with Teensy.
	radio.begin();
	radio.setDataRate(RF24_2MBPS);
	radio.setRetries(15, 15);
	radio.setPALevel(RF24_PA_LOW);
	radio.setChannel(52);
	radio.openWritingPipe(pipe);

	//
	//Reads the initial X and Y analog readings and assigns
	//it as the zeroized (Joystick center) position.
	//

	xInputZero = analogRead(rollPin);
	delay(200);
	yInputZero = analogRead(pitchPin);
	delay(200);
	yawZero = analogRead(yawPin);
	delay(200);

}//--(end setup )---

 //----------------------      Main Loop     --------------------------//
 //----------------------********************--------------------------//

void loop()
{
	int throttle;
	int joystickX_RawValue = analogRead(rollPin);
	int joystickY_RawValue = analogRead(pitchPin);
	int joystickYaw_RawValue = analogRead(yawPin);
	
	yawPinValue = setYaw();
	xInputValue = setRoll();
	yInputValue = setPitch();
	throttle = setThrottle();
	//calibratePID();

	controller[0] = throttle;
	controller[1] = yawPinValue;
	controller[2] = xInputValue;
	controller[3] = yInputValue;
	/*
	controller[4] = pRollFactor;
	controller[5] = iRollFactor;
	controller[6] = dRollFactor;

	controller[7] = pYawFactor;
	controller[8] = iYawFactor;
	controller[9] = dYawFactor;
	*/

	radio.write(controller, sizeof(controller));

	Serial.print("X-Analog Reading: \t");
	Serial.print(joystickX_RawValue);
	Serial.print(" ");
	Serial.print("Y-Analog Reading: \t");
	Serial.print(joystickY_RawValue);
	Serial.print(" ");
	Serial.print("Yaw Reading: \t");
	Serial.print(joystickYaw_RawValue);
	Serial.print(" ");
	Serial.print("X-Position: \t");
	Serial.println(xInputValue);
	Serial.print("Y-Position:\t");
	Serial.print(yInputValue);
	Serial.print(" ");
	Serial.print("Throttle Position: \t");
	Serial.print(throttle);
	Serial.print(" ");
	Serial.print("Yaw Adjust: \t");
	Serial.println(yawPinValue);
	Serial.println(" ");
	Serial.println("**********************************************************************************************************************");
	Serial.println(" ");
	Serial.print("Roll/Pitch P Correction: \t");
	Serial.print(pRollFactor, 4);
	Serial.print(" ");
	Serial.print("Roll/Pitch I Correction: \t");
	Serial.print(iRollFactor, 4);
	Serial.print(" ");
	Serial.print("Roll/Pitch D Correction: \t");
	Serial.println(dRollFactor, 4);
	Serial.print("Yaw P Correction: \t\t");
	Serial.print(pYawFactor, 4);
	Serial.print(" ");
	Serial.print("Yaw I Correction: \t\t");
	Serial.print(iYawFactor, 4);
	Serial.print(" ");
	Serial.print("Yaw D Correction: \t\t");
	Serial.println(dYawFactor, 4);
	Serial.println(" ");
	Serial.println("**********************************************************************************************************************");

}
/*
//--(end main loop )---
void calibratePID() {
	pRollFactor = analogRead(RP_P_PIN);
	pRollFactor = mapFloat(pRollFactor, 0, 4095, 0, 15);

	iRollFactor = analogRead(RP_I_PIN);
	iRollFactor = mapFloat(iRollFactor, 0, 4095, 0, 4);

	dRollFactor = analogRead(RP_D_PIN);
	dRollFactor = mapFloat(dRollFactor, 0, 4095, 0, 50);

	pYawFactor = analogRead(Y_P_PIN);
	pYawFactor = mapFloat(pYawFactor, 0, 4095, 0, 50);

	iYawFactor = analogRead(Y_I_PIN);
	iYawFactor = mapFloat(iYawFactor, 0, 4095, 0, 4);

	dYawFactor = analogRead(Y_D_PIN);
	dYawFactor = mapFloat(dYawFactor, 0, 4095, 0, 50);

}
*/
 //----------------------      Functions     --------------------------//
 //----------------------********************--------------------------//

int setThrottle() {
	int throttle = analogRead(throttlePin);
	throttle = map(throttle, 0, 4095, 420, 2250);
	throttle = constrain(throttle, minThrottle, maxThrottle);
	return throttle;
}

float setRoll() {
	int xInputValue = analogRead(rollPin);
	if (xInputValue <= xInputZero) xInputValue = map(xInputValue, 0, xInputZero, minRoll, 0); 
	else if (xInputValue >= xInputZero)	xInputValue = map(xInputValue, xInputZero, 4095, 0, maxRoll);
	if ((xInputValue > -10) && (xInputValue < 10)) xInputValue = 0;
	else if (xInputValue > 0) xInputValue -= 10;
	else if (xInputValue < 0) xInputValue += 10;
	return xInputValue;
}

float setPitch() {
	int yInputValue = analogRead(pitchPin);
	if (yInputValue <= yInputZero) yInputValue = map(yInputValue, 0, yInputZero, maxPitch, 0); 
	else if (yInputValue >= yInputZero) yInputValue = map(yInputValue, yInputZero, 4095, 0, minPitch);
	if ((yInputValue > -10) && (yInputValue < 10)) yInputValue = 0;  //controller deadzone 
	else if (yInputValue > 0) yInputValue -= 10; 
	else if (yInputValue < 0)  yInputValue += 10; 
	return yInputValue;
}

float setYaw(){
	yawPinValue = analogRead(yawPin);

	if (yawPinValue <= yawZero) yawPinValue = mapFloat(yawPinValue, 0, yawZero, maxYawAdj, 0);
	else if (yawPinValue >= yawZero) yawPinValue = mapFloat(yawPinValue, yawZero, 4095, 0, minYawAdj);
	if ((yawPinValue < 5) && (yawPinValue > -5)) {
		yawPinValue = 0;
	}
	return yawPinValue;
}

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}