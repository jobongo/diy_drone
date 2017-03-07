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

#define CE_PIN 9
#define CSN_PIN 10

#define joystick_X_Pin A0
#define joystick_Y_Pin A1
#define throttle_Pin A2
#define yawAdjust_Pin A3



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



RF24 radio(CE_PIN, CSN_PIN); // Create a Radio

							 /*-----( Declare Variables )-----*/

int16_t controller[4];  // 5 element array holding Joystick Readings, Throttle, and X, Y Zeroized positions


						//Define Joystick Variables

int xInputZero;
int yInputZero;
int xInputValue;
int yInputValue;
int yawZero;
int yawValue;

//----------------------        Setup       --------------------------//
//----------------------********************--------------------------//

void setup()
{
	//Serial.begin(115200); Uncomment if using arduino. Not needed with Teensy.
	radio.begin();
	radio.setDataRate(RF24_2MBPS);
	radio.setRetries(15, 15);
	radio.setPALevel(RF24_PA_LOW);
	radio.setChannel(52);
	//radio.setPayloadSize(64);
	radio.openWritingPipe(pipe);

	//
	//Reads the initial X and Y analog readings and assigns
	//it as the zeroized (Joystick center) position.
	//

	xInputZero = analogRead(joystick_X_Pin);
	delay(200);
	yInputZero = analogRead(joystick_Y_Pin);
	delay(200);
	yawZero = analogRead(yawAdjust_Pin);
	delay(200);
}//--(end setup )---


 //----------------------      Main Loop     --------------------------//
 //----------------------********************--------------------------//

void loop()
{
	int throttle;
	int joystickX_RawValue = analogRead(joystick_X_Pin);
	int joystickY_RawValue = analogRead(joystick_Y_Pin);
	int joystickYaw_RawValue = analogRead(yawAdjust_Pin);
	int yawValue = Define_Yaw_Adjust();

	xInputValue = Define_X_Value();
	yInputValue = Define_Y_Value();
	throttle = Define_Throttle();

	Serial.print("X-Analog Reading: \t");
	Serial.print(joystickX_RawValue);
	Serial.print(" ");
	Serial.print("Y-Analog Reading: \t");
	Serial.print(joystickY_RawValue);
	Serial.print(" ");
	Serial.print("Yaw Reading: \t");
	Serial.print(joystickYaw_RawValue);
	Serial.print(" ");
	Serial.print("X-Position:\t");
	Serial.print(xInputValue);
	Serial.print(" ");
	Serial.print("Y-Position:\t");
	Serial.print(yInputValue);
	Serial.print(" ");
	Serial.print("Throttle Position: \t");
	Serial.print(throttle);
	Serial.print(" ");
	Serial.print("Yaw Adjust: \t");
	Serial.print(yawValue);
	Serial.println(" ");





	controller[0] = throttle;
	controller[1] = yawValue;
	controller[2] = xInputValue;
	controller[3] = yInputValue;

	radio.write(controller, sizeof(controller));
	delay(50);

}//--(end main loop )---


 //----------------------      Functions     --------------------------//
 //----------------------********************--------------------------//



 //*--------------------- Throttle Function  -------------------------*//


int Define_Throttle() {
	int throttle = analogRead(throttle_Pin);
	throttle = map(throttle, 0, 1023, 420, 2250);
	throttle = constrain(throttle, 650, 2000);

	return throttle;
}

int Define_X_Value() {
	int xInputValue = analogRead(joystick_X_Pin);
	if (xInputValue <= xInputZero)
	{
		xInputValue = map(xInputValue, 0, xInputZero, -55, 0); // 55 to compensate for controller dead zone  of +-10 below
	}
	else if (xInputValue >= xInputZero) {
		xInputValue = map(xInputValue, xInputZero, 1023, 0, 55);
	}
	if ((xInputValue > -10) && (xInputValue < 10)) //controller deadzone becuase joystick doesn't always return to exactly 0
	{
		xInputValue = 0;
	}
	else if (xInputValue > 0)
	{
		xInputValue -= 10;
	}
	else if (xInputValue < 0)
	{
		xInputValue += 10;
	}
	return xInputValue;
}

int Define_Y_Value() {
	int yInputValue = analogRead(joystick_Y_Pin);


	if (yInputValue <= yInputZero) {
		yInputValue = map(yInputValue, 0, yInputZero, 55, 0); // 55 to compensate for controller dead zone  of +-10 below
	}
	else if (yInputValue >= yInputZero) {
		yInputValue = map(yInputValue, yInputZero, 1023, 0, -55); // 55 to compensate for controller dead zone  of +-10 below
	}
	if ((yInputValue > -10) && (yInputValue < 10)) //controller deadzone becuase joystick doesn't always return to 0
	{
		yInputValue = 0;
	}
	else if (yInputValue > 0)
	{
		yInputValue -= 10;
	}
	else if (yInputValue < 0)
	{
		yInputValue += 10;
	}
	return yInputValue;
}

float Define_Yaw_Adjust()
{

	yawValue = analogRead(yawAdjust_Pin);
	if (yawValue <= yawZero) {
		yawValue = map(yawValue, 0, yawZero, -115, 0);
	}
	else if (yawValue >= yawZero) {
		yawValue = map(yawValue, yawZero, 1023, 0, 115);
	}
	if ((yawValue > -10) && (yawValue < 10))
	{
		yawValue = 0;
	}
	else if (yawValue > 0)
	{
		yawValue -= 10;
	}
	else if (yawValue < 0)
	{
		yawValue += 10;
	}
	return yawValue;
}

