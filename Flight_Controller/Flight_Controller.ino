/*
 Name:		Flight_Controller.ino
 Created:	3/1/2017 10:08:38 PM
 Author:	Matthew
*/

//----------------------********************--------------------------//
//----------------------********************--------------------------//
//----------------------********************--------------------------//
//----------------------********************--------------------------//

// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class using DMP (MotionApps v2.0)
// 6/21/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib
//
// Changelog:
// 2013-05-08 - added seamless Fastwire support
//            - added note about gyro calibration
// 2012-06-21 - added note about Arduino 1.0.1 + Leonardo compatibility error
// 2012-06-20 - improved FIFO overflow handling and simplified read process
// 2012-06-19 - completely rearranged DMP initialization code and simplification
// 2012-06-13 - pull gyro and accel data from FIFO packet instead of reading directly
// 2012-06-09 - fix broken FIFO read sequence and change interrupt detection to RISING
// 2012-06-05 - add gravity-compensated initial reference frame acceleration output
//            - add 3D math helper file to DMP6 example sketch
//            - add Euler output and Yaw/Pitch/Roll output formats
// 2012-06-04 - remove accel offset clearing for better results (thanks Sungon Lee)
// 2012-06-01 - fixed gyro sensitivity to be 2000 deg/sec instead of 250
// 2012-05-30 - basic DMP initialization working

/* ============================================
I2Cdev device library code is placed under the MIT license
Copyright (c) 2012 Jeff Rowberg

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
===============================================
*/

/******( Import needed libraries )******/
//#include <SPI.h> //<----Using RF24.h
//#include <I2Cdev.h> //<----Using the Teensy Arduino Wire Implementation
//#include <nRF24L01.h> // <----Another SPI implementation for the NRF24L01

#include <RF24.h>
#include <Servo.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <Wire.h>

MPU6050 mpu;

/* =================== NRF24L01 SETUP ===============*/

/*
GND	-> GND
VCC	-> 3.3v Nrf24L01 is rated for up to 3.6v without a voltage regulator. 
CSN	-> Pin 10
CE	-> Pin 9
SCK	-> Pin 13
MOSI-> Pin 11
MISO-> Pin 12
IRQ	-> Not Used

-----( Declare Constants, Pin Numbers, and setup radio )-----
*/

#define CE_PIN 9
#define CSN_PIN 10
RF24 radio(CE_PIN, CSN_PIN); // Create Radio Instance

/*
******************************************************************************************
Make sure that in the servo library in the servo header file that the default time is set
to 0 microseconds (If this is not done, the esc's will not calibrate correctly). To get
better response from the ESCs, set the minimum refresh time is set to 2500(400hz) (default
is 20000).
******************************************************************************************
*/
Servo FL_MOTOR, FR_MOTOR, RL_MOTOR, RR_MOTOR; //Separate instances for each of the ESC's

const uint64_t pipe = 0xFFFFLL; // The radio transmit pipe. This can be whatever value but 
								// must be the same on the controller and the drone.

/* 
Parts of the following MPU Code taken from
https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050/examples/MPU6050_DMP6
*/

bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
Quaternion q;           // [w, x, y, z] quaternion container
VectorInt16 aa;         // [x, y, z] accel sensor measurements
VectorInt16 aaReal;     // [x, y, z] gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z] world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z] gravity vector
float euler[3];         // [psi, theta, phi]  Euler angle container
float ypr[3], yaw, pitch, roll;         // [yaw, pitch, roll] yaw/pitch/roll container and gravity vector
int16_t gyro[3];			// [Gyro-X, Gyro-Y, Gyro-Z] Gyro measurements for rate of rotation
uint8_t intPin = 2;

/* ================= CONTROLLER VARIABLES ================== */

int16_t controller[4]; // Array to hold data sent from controller.
int throttle;			//Throttle position from controller.			<---- initialize variables here?
int cRoll;		// Position of Joystick X Axis (ROLL).
int cPitch;	// Position of Joystick Y (PITCH).            
float controllerYaw;	// Yaw position from controller. 

/* ====================== MPU 6050 ========================= */

int flMotorSpeed; //Variable to hold front left ESC speed.
int frMotorSpeed; //Variable to hold front right ESC speed.
int rlMotorSpeed; //Variable to hold rear left ESC speed.
int rrMotorSpeed; //Variable to hold rear left ESC speed

				  
/* ================ CORRECTION VARIABLES =================== */

float flRollAdj = 0, frRollAdj = 0, rlRollAdj = 0, rrRollAdj = 0;
float flPitchAdj = 0, frPitchAdj = 0, rlPitchAdj = 0, rrPitchAdj = 0;
float flYawAdj, frYawAdj, rlYawAdj, rrYawAdj;
float flYawCorr, frYawCorr, rlYawCorr, rrYawCorr;
float flYPRCorr, frYPRCorr, rlYPRCorr, rrYPRCorr;
float xGyroCorr, yGyroCorr, xAccelCorr, yAccelCorr, zAccelCorr;
float yawAutoCorr = 2;
float yawAdjust;

/* ===================== MISCELLANEOUS ===================== */

bool rotorLowerCal = false;
bool rotorUpperCal = false;
bool rotorsCalibrated = false;
bool unstableYaw = true;
float heading;

/* ======================== MPU 6050 ======================= */

float xaReal;
float yaReal;
float zaReal;
float xGyro;
float yGyro;
float zGyro;
float aRes = 2.0 / 16384.0;
float gRes = 250.0 / 16384.0;
int16_t ax, ay, az, gx, gy, gz;

/* ================== RADIO READ RATE ====================== */

unsigned long currentTime;
unsigned long radioTime;
unsigned long oldTime1;

/* ===================== DEBUG OPTIONS ===================== */
/*
				Set the following to true to enable 
				serial output of various parameters.
*/

bool serialDebugEn = true;
bool aDebug = false; // Output Accelerometer Values
bool gDebug = false; // Ouput Gyroscope Values
bool yprDebug = false; // Output Yaw, Pitch, Roll Values
bool dmpDebug = false; // Output DMP Values Only
bool controlDebug = false; //Output values received from controller
bool finalThrottle = false;
bool droneDebug = true;
bool correctionFactors = true;

unsigned long debugTime;
unsigned long oldTime2;
unsigned long debugPollTime = 10; //Time in milliseconds to output to serial.

/* ================== INTERRUPT DETECTION  ================= */

volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high

/* ======================= SETUP ============================ */

void setup() {
	pinMode(0, INPUT);
	//Serial.begin(115200); //Not needed if using Teensy 3.1/2
	delay(500);
	
	Serial.println("Nrf24L01 Receiver Starting");
	radio.begin();
	radio.setDataRate(RF24_2MBPS); //Specify transmission speed. Accepted values are RF24_2MBPS, RF24_1MBPS, and RF24_250KBPS
	radio.setPALevel(RF24_PA_LOW); //Tranmit Power. Accepted values are RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX, RF24_PA_ERROR
	radio.setChannel(52);
	radio.setRetries(15, 15);
	radio.openReadingPipe(1, pipe);
	radio.startListening();

	FL_MOTOR.attach(3, 600, 2000); // Assigns each ESC to pins 3-6 respectively. 
	FR_MOTOR.attach(4, 600, 2000);
	RL_MOTOR.attach(5, 600, 2000);
	RR_MOTOR.attach(6, 600, 2000);

	// join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
	Wire.begin(); // Still testing the difference between Wire and Jeff Rowberg's Libraries.
	TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz). Comment this line if having compilation difficulties with TWBR.
	//#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
#endif

	Serial.println(F("Initializing MPU..."));
	mpu.initialize();
	mpu.setSleepEnabled(false); //This is important to ensure that Sleep is disabled.

	// verify connection
	Serial.println(F("Testing Connection to MPU..."));
	Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
	delay(2000);
	setMPUScale(); //Changes the scale (resolution) of the MPU. Should not normally need to be changed. 

	// load and configure the DMP
	Serial.println(F("Initializing DMP..."));
	devStatus = mpu.dmpInitialize();

	setMPUOffset(); //Set Gyro and Accel Offsets to calibrate the MPU.

	if (devStatus == 0) { // make sure it worked (returns 0 if so)
		Serial.println(F("Enabling DMP..."));
		mpu.setDMPEnabled(true); // turn on the DMP, now that it's ready
		Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));// enable Arduino interrupt detection
		pinMode(intPin, INPUT);
		attachInterrupt(intPin, dmpDataReady, RISING);
		mpuIntStatus = mpu.getIntStatus();
		Serial.println(F("DMP ready! Waiting for first interrupt..."));
		dmpReady = true; // set our DMP Ready flag so the main loop() function knows it's okay to use it
		packetSize = mpu.dmpGetFIFOPacketSize(); // get expected DMP packet size for later comparison
	}
	else {
		// ERROR!
		// 1 = initial memory load failed
		// 2 = DMP configuration updates failed
		// (if it's going to break, usually the code will be 1)
		Serial.print(F("DMP Initialization failed (code "));
		Serial.print(devStatus);
		Serial.println(F(")"));
	}
	//uint8_t rate = mpu.getRate();
	//Serial.print("Rate is ");
	//Serial.println(rate);

	escCalibration();			//Run ESC Calibration before flying. Checks minimum and maximum throttle positions. 
								//This is written for SimonK firmwares with a pulsewidth range of ~600 - 2000
	detectStableYaw();
}

/* ========================== MAIN ========================= */

void loop() {
	readMPUData();
	fly();

	currentTime = millis();
	debugTime = currentTime - oldTime2;

	if (serialDebugEn && debugTime > debugPollTime)
	{
		serialDebug();
		oldTime2 = currentTime;
	}
}

/* ======================= FUNCTIONS ======================== */

void fly() {
	currentTime = millis();
	radioTime = currentTime - oldTime1;
	if (radioTime > 10)
	{
		if (radio.available())
		{
			radio.read(controller, sizeof(controller));
			//Serial.print("Radio is Available! ");
		}
		oldTime1 = currentTime;
	}

	throttle = controller[0];
	controllerYaw = controller[1];
	cRoll = controller[2];
	cPitch = controller[3];

	autoLeveling();
	calcGyroCorr();
	calcAccelCorr();

	/* Working Code. Code below is test. 
	flMotorSpeed = throttle + flYPRCorr - xGyroCorr + yGyroCorr + xAccelCorr + yAccelCorr - zAccelCorr;
	frMotorSpeed = throttle + frYPRCorr + xGyroCorr + yGyroCorr + xAccelCorr - yAccelCorr - zAccelCorr;
	rlMotorSpeed = throttle + rlYPRCorr - xGyroCorr - yGyroCorr - xAccelCorr + yAccelCorr - zAccelCorr;
	rrMotorSpeed = throttle + rrYPRCorr + xGyroCorr - yGyroCorr - xAccelCorr - yAccelCorr - zAccelCorr;
	*/
	/* Testing different solutions for autobalancing.*/
	flMotorSpeed = throttle + flYPRCorr;// - xGyroCorr + yGyroCorr + xAccelCorr + yAccelCorr - zAccelCorr;
	frMotorSpeed = throttle + frYPRCorr;// + xGyroCorr + yGyroCorr + xAccelCorr - yAccelCorr - zAccelCorr;
	rlMotorSpeed = throttle + rlYPRCorr;// - xGyroCorr - yGyroCorr - xAccelCorr + yAccelCorr - zAccelCorr;
	rrMotorSpeed = throttle + rrYPRCorr;// + xGyroCorr - yGyroCorr - xAccelCorr - yAccelCorr - zAccelCorr;

	flMotorSpeed = constrain(flMotorSpeed, 650, 2000);
	frMotorSpeed = constrain(frMotorSpeed, 650, 2000);
	rlMotorSpeed = constrain(rlMotorSpeed, 650, 2000);
	rrMotorSpeed = constrain(rrMotorSpeed, 650, 2000);

	if (throttle >700)
	{
		setRotorSpeed(flMotorSpeed, frMotorSpeed, rlMotorSpeed, rrMotorSpeed);
	}
	else
	{
		flMotorSpeed = throttle;
		frMotorSpeed = throttle;
		rlMotorSpeed = throttle;
		rrMotorSpeed = throttle;
		setRotorSpeed(flMotorSpeed, frMotorSpeed, rlMotorSpeed, rrMotorSpeed);
	}
}
void setRotorSpeed(int flMotorSpeed, int frMotorSpeed, int rlMotorSpeed, int rrMotorSpeed) {
	//currentTime = millis();

	FL_MOTOR.write(flMotorSpeed);
	FR_MOTOR.write(frMotorSpeed);
	RL_MOTOR.write(rlMotorSpeed);
	RR_MOTOR.write(rrMotorSpeed);
}
void escCalibration() {

	unsigned long currentTime;
	unsigned long deltaTime;
	unsigned long oldTime = 0;

	while (rotorsCalibrated == false)
	{
		currentTime = millis();
		deltaTime = currentTime - oldTime;
		if (deltaTime > 10)
		{
			if (radio.available())
			{
				radio.read(controller, sizeof(controller));
				//Serial.print("Radio is Available! ");
			}
			oldTime = currentTime;
		}

		throttle = controller[0]; //Checks throttle from controller. 

		if (rotorUpperCal == false)
		{
			if (throttle < 2000)
			{
				Serial.println("Throttle is not at the maximum setting.");
				delay(500);
				//Add beep or LED functions to say that the throttle is not at the maximum setting. 
				//break;
			}
			else
			{
				flMotorSpeed = throttle;
				frMotorSpeed = throttle;
				rlMotorSpeed = throttle;
				rrMotorSpeed = throttle;
				Serial.print("Calibrating ESC!! ");
				Serial.print("Current Throttle is ");
				Serial.println(throttle);
				setRotorSpeed(flMotorSpeed, frMotorSpeed, rlMotorSpeed, rrMotorSpeed);
				rotorUpperCal = true;
				delay(2000);
			}
		}
		if ((rotorUpperCal == true) && (rotorLowerCal != true)) //Checks to see if upper calibration is done while lower calibration has not. 
		{
			if (throttle > 650) // Waits until the throttle is 651 or less.
			{
				Serial.println("Throttle is not at the lowest setting!");
				delay(200);
			}
			else
			{
				Serial.print("Calibrating ESC!! ");
				Serial.print("Current Throttle is ");
				Serial.println(throttle);
				//Write the lowest throttle setting to the ESC's.
				flMotorSpeed = throttle;
				frMotorSpeed = throttle;
				rlMotorSpeed = throttle;
				rrMotorSpeed = throttle;
				setRotorSpeed(flMotorSpeed, frMotorSpeed, rlMotorSpeed, rrMotorSpeed);
				rotorLowerCal = true;
				delay(2000);
			}
		}
		if ((rotorLowerCal != false) && (rotorUpperCal != false))
		{
			rotorsCalibrated = true;
			for (int i = 0; i < 4; i++)
			{
				Serial.println("ESC's have been calibrated!"); // Possibly create a routine the flashes lights/beeps 3 times to indicate that rotors are calibrated. 
				i = i + 1;
				delay(500);

			}
		}
	}
}
void autoLeveling() {
	double zCorrFact = 8;
	double yawFact = 2.2;
	
	int minYPRAdj = (-(throttle - 650));
	int maxYPRAdj = (2000 - throttle);
	float rollAdjFact = 15;		// Y Adjust Factor. Decrease to increase rate of adjust.
	float pitchAdjFact = 15;	// Y Adjust Factor. Decrease to increase rate of adjust.
	//float yprMinMaxFactor = 20;
	float minAngleAdj = -(throttle * .2);	//Minimum amount that roll/pitch can adjust. % of the current throttle
	float maxAngleAdj = throttle * .2;		//Maximum amount that the roll/pitch can adjust. % of the current throttle

	//float minAngleAdj = -(abs(yprMinMaxFactor * (roll)));
	//float maxAngleAdj = abs(yprMinMaxFactor * (roll));
	//float minAngleAdj = -(abs(yprMinMaxFactor * (pitch)));
	//float maxAngleAdj = abs(yprMinMaxFactor * (pitch));
	float rollAdj = (cRoll - (roll)) / rollAdjFact;
	float pitchAdj = (cPitch - (pitch)) / pitchAdjFact;
	float xGyroFactor = 7;
	float yGyroFactor = 7;
	
	/*
	int cRollMin = cRoll - 25;  //Set to 25
	int cRollMax = cRoll + 25;
	int cPitchMin = cPitch - 25;
	int cPitchMax = cPitch + 25;
	int minRotorAdjust = -25; //Set to 25
	int maxRotorAdjust = 25;

	*/

	if (controllerYaw == 0) //Yaw AutoStabilize
	{
		yawAdjust = 0;
		yawAutoCorr = yawAutoCorr + (zGyro * zCorrFact);
	}

	yawAdjust = controllerYaw * yawFact;

	heading = heading + (controllerYaw / 20);
	yawAutoCorr = constrain(yawAutoCorr, -150, 150);
	yawAdjust = constrain(yawAdjust, -200, 200);

	flYawAdj = -yawAdjust;
	frYawAdj = yawAdjust;
	rlYawAdj = yawAdjust;
	rrYawAdj = -yawAdjust;

	flYawCorr = -yawAutoCorr;
	frYawCorr = yawAutoCorr;
	rlYawCorr = yawAutoCorr;
	rrYawCorr = -yawAutoCorr;
	/*
	flRollAdj = -(map(roll, cRollMin, cRollMax, minRotorAdjust, maxRotorAdjust));
	rlRollAdj = -(map(roll, cRollMin, cRollMax, minRotorAdjust, maxRotorAdjust));
	frRollAdj = map(roll, cRollMin, cRollMax, minRotorAdjust, maxRotorAdjust);
	rrRollAdj = map(roll, cRollMin, cRollMax, minRotorAdjust, maxRotorAdjust);

	flPitchAdj = -(map(pitch, cPitchMin, cPitchMax, minRotorAdjust, maxRotorAdjust));
	frPitchAdj = -(map(pitch, cPitchMin, cPitchMax, minRotorAdjust, maxRotorAdjust));
	rlPitchAdj = map(pitch, cPitchMin, cPitchMax, minRotorAdjust, maxRotorAdjust);
	rrPitchAdj = map(pitch, cPitchMin, cPitchMax, minRotorAdjust, maxRotorAdjust);
	
	flYPRCorr = flRollAdj + flPitchAdj; // + flYawAdj + flYawCorr;
	frYPRCorr = frRollAdj + frPitchAdj; // + frYawAdj + frYawCorr;
	rlYPRCorr = rlRollAdj + rlPitchAdj; // + rlYawAdj + rlYawCorr;
	rrYPRCorr = rrRollAdj + rrPitchAdj; // + rrYawAdj + rrYawCorr;
	*/

	flRollAdj = flRollAdj + rollAdj - (xGyro*xGyroFactor);	
	rlRollAdj = rlRollAdj + rollAdj - (xGyro*xGyroFactor);
	frRollAdj = frRollAdj - rollAdj + (xGyro*xGyroFactor);
	rrRollAdj = rrRollAdj - rollAdj + (xGyro*xGyroFactor);

	flRollAdj = constrain(flRollAdj, minAngleAdj, maxAngleAdj);
	rlRollAdj = constrain(rlRollAdj, minAngleAdj, maxAngleAdj);
	frRollAdj = constrain(frRollAdj, minAngleAdj, maxAngleAdj);
	rrRollAdj = constrain(rrRollAdj, minAngleAdj, maxAngleAdj);

	flPitchAdj = flPitchAdj + pitchAdj + (yGyro*yGyroFactor);
	frPitchAdj = frPitchAdj + pitchAdj + (yGyro*yGyroFactor);
	rlPitchAdj = rlPitchAdj - pitchAdj - (yGyro*yGyroFactor);
	rrPitchAdj = rrPitchAdj - pitchAdj - (yGyro*yGyroFactor);

	flPitchAdj = constrain(flPitchAdj, minAngleAdj, maxAngleAdj);
	frPitchAdj = constrain(frPitchAdj, minAngleAdj, maxAngleAdj);
	rlPitchAdj = constrain(rlPitchAdj, minAngleAdj, maxAngleAdj);
	rrPitchAdj = constrain(rrPitchAdj, minAngleAdj, maxAngleAdj);


	flYPRCorr = flRollAdj + flPitchAdj;// +flYawAdj + flYawCorr; ?? Possible try and get the average????
	flYPRCorr = constrain(flYPRCorr, minYPRAdj, maxYPRAdj);
	frYPRCorr = frRollAdj + frPitchAdj;// +frYawAdj + frYawCorr;
	frYPRCorr = constrain(frYPRCorr, minYPRAdj, maxYPRAdj);
	rlYPRCorr = rlRollAdj + rlPitchAdj;// +rlYawAdj + rlYawCorr;
	rlYPRCorr = constrain(rlYPRCorr, minYPRAdj, maxYPRAdj);
	rrYPRCorr = rrRollAdj + rrPitchAdj;// +rrYawAdj + rrYawCorr;
	rrYPRCorr = constrain(rrYPRCorr, minYPRAdj, maxYPRAdj);




}
void calcGyroCorr() {
	float gyroCorrFact = 75;
	xGyroCorr = constrain(xGyro * gyroCorrFact, -200, 200);
	yGyroCorr = constrain(yGyro * gyroCorrFact, -200, 200);
}
void calcAccelCorr() {
	float accelCorrFact = 50;
	float zaccelCorrFact = 50;
	xAccelCorr = constrain(xaReal * accelCorrFact, -200, 200);
	yAccelCorr = constrain(yaReal * accelCorrFact, -200, 200);
	zAccelCorr = constrain(zaReal * zaccelCorrFact, -500, 500);
}
void getGyro() {
	mpu.dmpGetGyro(gyro, fifoBuffer);
	xGyro = gyro[0] * gRes;
	yGyro = gyro[1] * gRes;
	zGyro = gyro[2] * gRes;
}
void getAccel() {
	// display real acceleration, adjusted to remove gravity
	mpu.dmpGetQuaternion(&q, fifoBuffer);
	mpu.dmpGetAccel(&aa, fifoBuffer);
	mpu.dmpGetGravity(&gravity, &q);
	mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
	xaReal = aaReal.x * aRes;
	yaReal = aaReal.y * aRes;
	zaReal = aaReal.z * aRes;
}
void getYPR() {
	mpu.dmpGetQuaternion(&q, fifoBuffer);
	mpu.dmpGetGravity(&gravity, &q);
	mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
	yaw = (ypr[0] * 180 / M_PI), pitch = (ypr[1] * 180 / M_PI), roll = (ypr[2] * 180 / M_PI);
}
void setMPUOffset() {
	mpu.setXAccelOffset(-1425);
	mpu.setYAccelOffset(1228);
	mpu.setZAccelOffset(1620); // 1688 factory default for my test chip
	mpu.setXGyroOffset(145); //Default 220
	mpu.setYGyroOffset(97); // Default 76
	mpu.setZGyroOffset(60); // Default -85
}
void getDMPMotion6() {
	mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
	int start = millis(); Serial.print(start); Serial.print("\t");
	Serial.print(ax); Serial.print("\t");
	Serial.print(ay); Serial.print("\t");
	Serial.print(az); Serial.print("\t");
	Serial.print(gx); Serial.print("\t");
	Serial.print(gy); Serial.print("\t");
	Serial.println(gz);
}
void setMPUScale() {
	mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_4);
	mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_500);
}
void dmpDataReady() {
	mpuInterrupt = true;
}
void detectStableYaw() {
	unsigned long prevTime = 0;
	float oldYaw = 0;
	readMPUData();
	unsigned long currentTime;
	unsigned long deltaTime;
	while (unstableYaw)
	{
		readMPUData();
		currentTime = millis();
		deltaTime = currentTime - prevTime;
		readMPUData();
		if (deltaTime > 1000)
		{
			Serial.println(yaw);
			float yawDiff = abs(yaw - oldYaw);
			Serial.println("Old Yaw is ");
			Serial.println(oldYaw);
			Serial.println(deltaTime);
			if (yawDiff > .05)
			{
				Serial.println("Yaw is not stable. Waiting 1 second.");
			}
			if (yawDiff < .05)
			{
				Serial.println("Yaw is stable. ");
				heading = yaw;
				Serial.print("Heading is: ");
				Serial.print(heading);
				unstableYaw = false;
			}
			prevTime = currentTime;
			oldYaw = yaw;
		}
	}
}
void readMPUData() {
	// if programming failed, don't try to do anything
	if (!dmpReady) return;

	// wait for MPU interrupt or extra packet(s) available
	while (!mpuInterrupt && fifoCount < packetSize) {
		// other program behavior stuff here
		// .
		// .
		// .
		// if you are really paranoid you can frequently test in between other
		// stuff to see if mpuInterrupt is true, and if so, "break;" from the
		// while() loop to immediately process the MPU data
		// .
		// .
		// .
	}

	// reset interrupt flag and get INT_STATUS byte
	mpuInterrupt = false;
	mpuIntStatus = mpu.getIntStatus();

	// get current FIFO count
	fifoCount = mpu.getFIFOCount();

	// check for overflow (this should never happen unless our code is too inefficient)
	if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
		// reset so we can continue cleanly
		mpu.resetFIFO();
		Serial.println(F("FIFO overflow!"));

		// otherwise, check for DMP data ready interrupt (this should happen frequently)
	}
	else if (mpuIntStatus & 0x02) {
		// wait for correct available data length, should be a VERY short wait
		while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

		// read a packet from FIFO
		mpu.getFIFOBytes(fifoBuffer, packetSize);

		// track FIFO count here in case there is > 1 packet available
		// (this lets us immediately read more without waiting for an interrupt)
		fifoCount -= packetSize;

		getGyro();
		getAccel();
		getYPR();
	}
}
void serialDebug()
{
	Serial.println("##################################################################################################################################################");
	Serial.println("##################################################################################################################################################");
	Serial.println();
	//DMP Motion 6
	if (dmpDebug) {
		Serial.println("-------------------------------------DMP6 Values------------------------------------------------");
		Serial.print(ax); Serial.print("\t");
		Serial.print(ay); Serial.print("\t");
		Serial.print(az); Serial.print("\t");
		Serial.print(gx); Serial.print("\t");
		Serial.print(gy); Serial.print("\t");
		Serial.println(gz);
		Serial.println("*************************************************************************************************");
		Serial.println();
	}
	//YPR */

	if (yprDebug) {
		Serial.println("--------Yaw, Pitch, Roll Values--------");
		Serial.println("Yaw: ");
		Serial.print(yaw);
		Serial.print("Pitch: ");
		Serial.print(pitch);
		Serial.print("Roll: ");
		Serial.println(roll);
	}

	//Real Acceleration
	if (aDebug)
	{
		Serial.println("---------------------Accelerometer Values---------------------");
		Serial.println("Real X Accel\t\tReal Y Accel\t\tReal Z Accel");
		Serial.print(xaReal);
		Serial.print("\t\t\t");
		Serial.print(yaReal);
		Serial.print("\t\t\t");
		Serial.println(zaReal);
		Serial.println("**************************************************************");
		Serial.println();
	}
	//Gyroscope
	if (gDebug)
	{
		Serial.println("------------Gyroscope Values-----------");
		Serial.println("Gyro X:\t\tGyro Y:\t\tGyro Z:\t\t");
		Serial.print(xGyro);
		Serial.print("\t\t");
		Serial.print(yGyro);
		Serial.print("\t\t");
		Serial.println(zGyro);
		Serial.println("***************************************");
		Serial.println();
	}
	//Controller
	if (controlDebug)
	{
		Serial.println("---------------------------Controller Values---------------------------------");
		Serial.println("Throttle:\t\tController X:\t\tController Y:\t\tYaw Position: ");
		Serial.print(throttle);
		Serial.print("\t\t\t\t");
		Serial.print(cRoll);
		Serial.print("\t\t\t");
		Serial.print(cPitch);
		Serial.print("\t\t\t");
		Serial.println(controller[1]);
		Serial.println("*****************************************************************************");
		Serial.println();
	}
	//Throttle being applied to the rotors
	if (finalThrottle)
	{
		Serial.println("-----------------------------------------------Final Throttle Values--------------------------------------------");
		Serial.println("Front Left ESC Throttle:\tFront Right ESC Throttle:\tRear Left ESC Throttle:\tRear Rigth ESC Throttle:");
		Serial.print("\t");
		Serial.print(flMotorSpeed);
		Serial.print("\t\t\t\t");
		Serial.print(frMotorSpeed);
		Serial.print("\t\t\t\t");
		Serial.print(rlMotorSpeed);
		Serial.print("\t\t\t");
		Serial.println(rrMotorSpeed);
		Serial.println("*****************************************************************************************************************");
		Serial.println();
	}
	if (droneDebug)
	{
		Serial.println("-----------------------------------------------Drone Summary Values------------------------------------------------");
		Serial.print("Throttle: ");
		Serial.print(throttle);
		Serial.print("\tYaw: ");
		Serial.print(yaw);
		Serial.print("\tPitch: ");
		Serial.print(pitch);
		Serial.print("\tRoll: ");
		Serial.print(roll);
		Serial.print("\tx Gyro: ");
		Serial.print(xGyro);
		Serial.print("\ty Gyro: ");
		Serial.print(yGyro);
		Serial.print("\tz Gyro: ");
		Serial.print(zGyro);
		Serial.print("\tx Accel: ");
		Serial.print(xaReal);
		Serial.print("\ty Accel: ");
		Serial.print(yaReal);
		Serial.print("\tz Accel: ");
		Serial.println(zaReal);
		Serial.print("FL ESC: ");
		Serial.print(flMotorSpeed);
		Serial.print("\tFR ESC: ");
		Serial.print(frMotorSpeed);
		Serial.print("\tRL ESC: ");
		Serial.print(rlMotorSpeed);
		Serial.print("\tRR ESC: ");
		Serial.println(rrMotorSpeed);
		Serial.println("*********************************************************************************************************************");
		Serial.println();
	}
	if (correctionFactors)
	{
		Serial.println("-----------------------------------------------Correction Values being applied--------------------------------------------------------");
		Serial.print("Throttle: ");
		Serial.print(throttle);
		Serial.print("\tYaw:\t");
		Serial.println(yawAutoCorr);
		Serial.print("FL Roll:\t");
		Serial.print(flRollAdj);
		Serial.print("\tFR Roll:\t");
		Serial.print(frRollAdj);
		Serial.print("\tRL Roll:\t");
		Serial.print(rlRollAdj);
		Serial.print("\tRR Roll:\t");
		Serial.println(rrRollAdj);
		Serial.print("FL Pitch:\t");
		Serial.print(flPitchAdj);
		Serial.print("\tFR Pitch:\t");
		Serial.print(frPitchAdj);
		Serial.print("\tRL Pitch:\t");
		Serial.print(rlPitchAdj);
		Serial.print("\tRR Pitch:\t");
		Serial.println(rrPitchAdj);
		Serial.print("X Gyro:\t\t");
		Serial.print(xGyroCorr);
		Serial.print("\tY Gyro:\t");
		Serial.println(yGyroCorr);
		Serial.print("X Accel:\t");
		Serial.print(xAccelCorr);
		Serial.print("\tY Accel:\t");
		Serial.print(yAccelCorr);
		Serial.print("\tZ Accel:\t");
		Serial.println(zAccelCorr);
		Serial.println("***************************************************************************************************************************************");
		Serial.println();
	}
}
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}