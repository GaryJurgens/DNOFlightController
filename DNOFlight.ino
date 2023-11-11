/*
 Name:		DNOFlight.ino
 Created:	11/3/2023 7:16:00 AM
 Author:	garyj
*/

// the setup function runs once when you press reset or power the board






#include <PID_v1.h>
#include <Wire.h>
#include <units.h>
#include <mpu9250.h>
#include <eigen.h>
#define SDA_PIN 32
#define SCL_PIN 33
// Define rate limiter parameters (adjust as needed)
float rateLimiterAlpha = 0.02; // Smoothing factor (0 < alpha < 1)
// motor Pins
const int FRONT_LEFT_PIN = 15, FRONT_RIGHT_PIN = 17, BACK_LEFT_PIN = 4, BACK_RIGHT_PIN = 16;
// Constants for the ESC
// Constants for LEDC
const int LEDC_BASE_FREQ = 100; // Use 300Hz for the desired PWM frequency
const int LEDC_TIMER_BIT = 16; // Using 16-bit timers (you can increase this if supported)
const int MAX_DUTY_CYCLE = (1 << LEDC_TIMER_BIT) - 1; // Maximum duty cycle based on timer bit resolution

// Constants for PWM signal
const float MIN_PULSE_WIDTH_MS = 1.3; // Minimum pulse width in milliseconds (adjust as needed)
const float MAX_PULSE_WIDTH_MS = 1.8; // Maximum pulse width in milliseconds (adjust as needed)

// For a 300Hz frequency, the period is approximately 3.33 milliseconds (1/LEDC_BASE_FREQ * 1000).
// To calculate the duty cycle:
const float PERIOD_MS = 1000.0 / LEDC_BASE_FREQ; // Period of PWM signal in mill

// Calculate duty cycle counts for adjusted pulse widths
const int minDuty = (int)((MIN_PULSE_WIDTH_MS / PERIOD_MS) * MAX_DUTY_CYCLE);
const int maxDuty = (int)((MAX_PULSE_WIDTH_MS / PERIOD_MS) * MAX_DUTY_CYCLE);
// Assign channels to your motors
const int FRONT_LEFT_CHANNEL = 1;
const int FRONT_RIGHT_CHANNEL = 2;
const int BACK_LEFT_CHANNEL = 3;
const int BACK_RIGHT_CHANNEL = 4;

//Servo RightFrontMotor, LeftFrontMotor, RightBackMotor, LeftBackMotor;

//radio Channels

const int RADIO_INPUT_PIN_CH1 = 5;  // Change to the pin you're using
unsigned long pwmValueCH1 = 0;
const int RADIO_INPUT_PIN_CH2 = 18;  // Change to the pin you're using
unsigned long pwmValueCH2 = 0;
const int RADIO_INPUT_PIN_CH3 = 19;  // Change to the pin you're using
unsigned long pwmValueCH3 = 0;
const int RADIO_INPUT_PIN_CH4 = 21;  // Change to the pin you're using
unsigned long pwmValueCH4 = 0;

// Radio input values for the different channels

float CH1_Roll_Left = 980, CH1_Roll_Right = 2012, CH1_Roll_Center = 1500;
float CH2_Pitch_Forward = 980, CH2_Pitch_Backward = 2012, CH2_Pitch_Center = 1500;
float CH3_Throttle_Min = 980, CH3_Throttle_Max = 2012, CH3_Throttle_Center = 1500;
float CH4_Yaw_Left = 980, CH4_Yaw_Right = 2012, CH4_Yaw_Center = 1500;

/// throttel settings

int CurrentThrottle = 0;
const int deadzone = 10; // Adjust this value as needed
const int PitchDeadzone = 10; // Adjust this value as needed

// sensors

bfs::Mpu9250 imu;

/// Tuning parameters

// Define global variables
int currentMotorSpeed = 0;      // Current speed of the motor
int targetMotorSpeed = 0;       // Target speed to reach
unsigned long lastUpdate = 0;   // Last update time
int updateInterval = 10;        // Time interval for speed updates in milliseconds
int rampStep = 40;			   // Step size for ramping up or down

// Define global variables
int PitchcurrentMotorSpeed = 0;      // Current speed of the motor
int PitchtargetMotorSpeed = 0;       // Target speed to reach
unsigned long PitchlastUpdate = 0;   // Last update time
int PitchupdateInterval = 10;        // Time interval for speed updates in milliseconds
int PitchrampStep = 40;			   // Step size for ramping up or down



float PitchKp = 1.0; // Proportional gain
float PitchKi = 1; // Integral gain
float PitchKd = 0.0; // Derivative gain

float Pitchintegral = 0;
float Pitchprevious_error = 0;

String RollDirection = "center"; 
String PitchDirection = "center";


int GetRollFromTransmitter()
{
	int pwmValue = pulseIn(RADIO_INPUT_PIN_CH1, HIGH, 25000);
	int thrust = 0;
	Serial.println("Roll PWM: " + String(pwmValue));

	if (pwmValue < CH1_Roll_Center)
	{
		RollDirection = "left";
		Serial.println("Roll Direction Left");

	}
	else if (pwmValue > CH1_Roll_Center)
	{
		RollDirection = "right";
		Serial.println("Roll Direction Right");
	}
	else
	{
		RollDirection = "center";
	}

	if (abs(pwmValue - CH1_Roll_Center) <= 10) {
		// Get the current roll angle or rate
		float rollError = imu.gyro_x_radps() * 180 / PI; // Error in degrees

		// Proportional term
		//float Pout = RollKp * rollError;

		// Integral term
		//Rollintegral += rollError;
		//float Iout = RollKi * Rollintegral;

		// Derivative term
		//float derivative = rollError - Rollprevious_error;
		//float Dout = RollKd * derivative;

		// Calculate total output
		//int output = Pout + Iout + Dout;

		Serial.println("Roll Error");
		//Serial.println(output);

		//if (output < 0)
		//{
		//	RollDirection = "left";
		//}
		//else if (output > 0)
		//{
		//	RollDirection = "right";
		//}
		//else
		//{
		RollDirection = "center";
		//}

		// Restrict to max/min
		//int correctedThrottle = constrain(CurrentThrottle - output, 0, 100);

		// Save error for next loop
		//Rollprevious_error = rollError;

		Serial.println("Roll Corrected Throttle");
		//Serial.println(correctedThrottle);
		//return correctedThrottle;
	}
	else
	{
		// Manual control

		
		if (RollDirection == "left")
		{
			// Map low pwmValue to high thrust when rolling left
			thrust = map(pwmValue, CH1_Roll_Left, CH1_Roll_Center, 100, 0);
		}
		else if (RollDirection == "right")
		{
			// Map low pwmValue to high thrust when rolling right
			thrust = map(pwmValue, CH1_Roll_Left, CH1_Roll_Right, 0, 100);
		}
		

	}
		int constrained = constrain(thrust, 0, 100);
		Serial.println("Roll Manual Throttle");
		Serial.println(constrained);
		return constrained;
	

}
int GetPitchFromTransmitter()
{
	int pwmValue = pulseIn(RADIO_INPUT_PIN_CH2, HIGH, 25000);
	Serial.println("Pitch PWM: " + String(pwmValue));

	int thrust = 0; // Correct the variable name from "thurst" to "thrust"

	if (pwmValue < CH2_Pitch_Center)
	{
		PitchDirection = "forward";
		Serial.println("Pitch Direction Forward");
	}
	else if (pwmValue > CH2_Pitch_Center)
	{
		PitchDirection = "backward";
		Serial.println("Pitch Direction Backward");
	}
	else
	{
		PitchDirection = "center";
	}

	// Check if the stick is within the deadzone around the center
	if (abs(pwmValue - CH2_Pitch_Center) <= 10)
	{
		// Automatic pitch correction
		float PitchError = imu.gyro_y_radps() * 180 / PI; // Error in degrees

		// Proportional term
		float Pout = PitchKp * PitchError;

		// Integral term
		Pitchintegral += PitchError;
		float Iout = PitchKi * Pitchintegral;

		// Derivative term
		float derivative = PitchError - Pitchprevious_error;
		float Dout = PitchKd * derivative;

		// Calculate total output
		int output = Pout + Iout + Dout;

		if (output < 0)
		{
			PitchDirection = "forward";
		}
		else if (output > 0)
		{
			PitchDirection = "backward";
		}
		else
		{
			PitchDirection = "center";
		}

		// Map output to thrust
		thrust = map(output, -100, 100, 0, 100); // Map output to thrust in the range [0, 100]

		// Save error for the next loop
		Pitchprevious_error = PitchError;

		Serial.println("Pitch Corrected Throttle");
		Serial.println(thrust);
		return thrust;
	}
	else
	{
		// Manual control
		if (PitchDirection == "forward")
		{
			// Map low pwmValue to high thrust when pitching forward
			thrust = map(pwmValue, CH2_Pitch_Forward, CH2_Pitch_Backward, 0, CurrentThrottle);
		}
		else if (PitchDirection == "backward")
		{
			// Map low pwmValue to high thrust when pitching backward
			thrust = map(pwmValue, CH2_Pitch_Forward, CH2_Pitch_Backward, CurrentThrottle, 0);
		}

		Serial.println("Pitch Manual Throttle");
		Serial.println(thrust);
		return thrust;
	}
}
int GetThrottleFromTransmitter()
{
	int pwmValue = pulseIn(RADIO_INPUT_PIN_CH3, HIGH, 25000);
	int thurst = map(pwmValue, CH3_Throttle_Min, CH3_Throttle_Max, 0, 100);

	Serial.println("throttle Thrust" + String(thurst));
	return thurst;
}
int GetYawFromTransmitter()
{
	// 10% of current throttle
	int TenPercent = CurrentThrottle / 10;
	int Lowend = CurrentThrottle - TenPercent;
	int Highend = CurrentThrottle + TenPercent;
	int pwmValue = pulseIn(RADIO_INPUT_PIN_CH4, HIGH, 25000);
	int thrust = map(pwmValue, CH4_Yaw_Left, CH4_Yaw_Right, Lowend, Highend);
	int constrained = constrain(thrust, 0, 100);

	Serial.println("Yaw Thrust " + String(constrained));
	return constrained;
}

void SetPitchMotorSpeeds(int Speed)
{
	Serial.println("pitch speed");
	Serial.println(Speed);

	if (PitchDirection == "forward")
	{
		Serial.println("pitch forwards");
		setESCPower(Speed, BACK_LEFT_CHANNEL);
		setESCPower(Speed, BACK_RIGHT_CHANNEL);
		setESCPower(CurrentThrottle, FRONT_LEFT_CHANNEL);
		setESCPower(CurrentThrottle, FRONT_RIGHT_CHANNEL);
	}
	else if (PitchDirection == "backward")
	{
		Serial.println("pitch backwards");
		setESCPower(Speed, FRONT_LEFT_CHANNEL);
		setESCPower(Speed, FRONT_RIGHT_CHANNEL);
		setESCPower(CurrentThrottle, BACK_LEFT_CHANNEL);
		setESCPower(CurrentThrottle, BACK_RIGHT_CHANNEL);
	}
	else if (PitchDirection == "center")
	{
		Serial.println("Pitch - center");
		setESCPower(CurrentThrottle, FRONT_LEFT_CHANNEL);
		setESCPower(CurrentThrottle, FRONT_RIGHT_CHANNEL);
		setESCPower(CurrentThrottle, BACK_LEFT_CHANNEL);
		setESCPower(CurrentThrottle, BACK_RIGHT_CHANNEL);
	}
}

void SetMotorThrottle()
{
setESCPower(CurrentThrottle, BACK_LEFT_CHANNEL);
setESCPower(CurrentThrottle, BACK_RIGHT_CHANNEL);
setESCPower(CurrentThrottle, FRONT_RIGHT_CHANNEL);
setESCPower(CurrentThrottle, FRONT_LEFT_CHANNEL);

}

void SetRollMotorSpeeds(int Speed)
{
	Serial.println("roll speed");
	Serial.println(Speed);

	

	if (RollDirection == "left")
	{
		Serial.println("Roll - Left");
		setESCPower(Speed, BACK_LEFT_CHANNEL);
		setESCPower(Speed, FRONT_LEFT_CHANNEL);
		setESCPower(CurrentThrottle, BACK_RIGHT_CHANNEL);
		setESCPower(CurrentThrottle, FRONT_RIGHT_CHANNEL);
	}
	else if (RollDirection == "right")
	{
		Serial.println("Roll - Right");
		setESCPower(Speed, BACK_RIGHT_CHANNEL);
		setESCPower(Speed, FRONT_RIGHT_CHANNEL);
		setESCPower(CurrentThrottle, BACK_LEFT_CHANNEL);
		setESCPower(CurrentThrottle, FRONT_LEFT_CHANNEL);
	}
	else if (RollDirection == "center")
	{
		Serial.println("Roll - Center");
		setESCPower(CurrentThrottle, FRONT_LEFT_CHANNEL);
		setESCPower(CurrentThrottle, FRONT_RIGHT_CHANNEL);
		setESCPower(CurrentThrottle, BACK_LEFT_CHANNEL);
		setESCPower(CurrentThrottle, BACK_RIGHT_CHANNEL);
	}
}

void SetYawMotorSpeeds(int Speed)
{
	// set all motors to the same speed
	

	// 10% of current throttle
	int reducedSpeed = CurrentThrottle / 20;
	if (Speed < reducedSpeed / 2)
	{
		//setESCPower(reducedSpeed, LeftFrontMotor);
		//setESCPower(reducedSpeed, RightBackMotor);
	}
	else
	{
		//setESCPower(Speed, RightFrontMotor);
		//setESCPower(Speed, LeftBackMotor);
	}
}


// Function to calibrate one ESC
void calibrateESC(int channel) {

	

	// Set the speed to 100
	setESCPower(100, channel);

	// Wait for 2 seconds
	delay(3000);

	// Set the speed to 0
	setESCPower(0, channel);

	// Wait for 2 seconds
	delay(3000);
}

void initSensors() {
	Wire.begin(SDA_PIN, SCL_PIN);
	Wire.setClock(400000);
	/* I2C bus,  0x68 address */
	imu.Config(&Wire, bfs::Mpu9250::I2C_ADDR_PRIM);
	if (!imu.Begin()) {
		Serial.println("Error initializing IMU");
		while (1) {}
	}

	Serial.println("Calibrating IMU... Keep it level and still.");
	delay(1000);

	Serial.println("Done!");
}
// Function to set the speed of the ESC
void setESCPower(int speed, int channel) {

	// Map the speed from (0-100) to (minDuty-maxDuty)
	int duty = map(speed, 0, 100, minDuty, maxDuty);
	duty = constrain(duty, minDuty, maxDuty);
	ledcWrite(channel, duty);

}
void updateIMUData() {
	imu.Read();
	imu.new_imu_data();
	imu.new_mag_data();
}

// The setup() function runs once each time the micro-controller starts
void setup()
{
	Serial.begin(115200);
	while (!Serial) { delay(10); }  // wait for serial port to connect
	Serial.println("Started");
	delay(1000);

	// define pinmodes for radio input pins

	pinMode(RADIO_INPUT_PIN_CH1, INPUT);
	pinMode(RADIO_INPUT_PIN_CH2, INPUT);
	pinMode(RADIO_INPUT_PIN_CH3, INPUT);
	pinMode(RADIO_INPUT_PIN_CH4, INPUT);

	pinMode(FRONT_LEFT_PIN, OUTPUT);
pinMode(FRONT_RIGHT_PIN, OUTPUT);
pinMode(BACK_LEFT_PIN, OUTPUT);
pinMode(BACK_RIGHT_PIN, OUTPUT);

// Setup timer and channel for each motor
ledcSetup(FRONT_LEFT_CHANNEL, LEDC_BASE_FREQ, LEDC_TIMER_BIT);
ledcSetup(FRONT_RIGHT_CHANNEL, LEDC_BASE_FREQ, LEDC_TIMER_BIT);
ledcSetup(BACK_LEFT_CHANNEL, LEDC_BASE_FREQ, LEDC_TIMER_BIT);
ledcSetup(BACK_RIGHT_CHANNEL, LEDC_BASE_FREQ, LEDC_TIMER_BIT);

// Attach the channel to the GPIO to be controlled
ledcAttachPin(FRONT_LEFT_PIN, FRONT_LEFT_CHANNEL);
ledcAttachPin(FRONT_RIGHT_PIN, FRONT_RIGHT_CHANNEL);
ledcAttachPin(BACK_LEFT_PIN, BACK_LEFT_CHANNEL);
ledcAttachPin(BACK_RIGHT_PIN, BACK_RIGHT_CHANNEL);
	
	calibrateESC(FRONT_LEFT_CHANNEL);
	calibrateESC(FRONT_RIGHT_CHANNEL);
	calibrateESC(BACK_LEFT_CHANNEL);
	calibrateESC(BACK_RIGHT_CHANNEL);
	delay(2000);

	

	//initSensors();
}

// Add the main program code into the continuous loop() function
void loop() {

	

	Serial.println("Testing Motors");

	//calibrateESC(LeftBackMotor);
	//setESCPower(50, BACK_LEFT_CHANNEL);
	
	//delay(2000);  // short delay to allow the ESC to recognize the signal

	Serial.println("Right Back Motor should be running");

	// Once confirmed, you can uncomment the next line, and so on.
	
	CurrentThrottle = GetThrottleFromTransmitter();
	int rollspeed = SmoothThrottleRollAdjustment(GetRollFromTransmitter());
	int pitchspeed = SmoothThrottlePitchAdjustment(GetPitchFromTransmitter());
	if (rollspeed > 0)
	{
		SetRollMotorSpeeds(rollspeed);
	}
	else if(pitchspeed > 0)
	{ 
		SetPitchMotorSpeeds(pitchspeed);
	}
	else
	{
		SetMotorThrottle();
	}


	

	//delay(2000);

	// Add more as you confirm each one works.
}



int SmoothThrottleRollAdjustment(int thrustvalue)
{
   targetMotorSpeed = thrustvalue;
 unsigned long currentTime = millis();
    if (currentTime - lastUpdate > updateInterval) {
        lastUpdate = currentTime;

        if (currentMotorSpeed < targetMotorSpeed) {
            currentMotorSpeed += rampStep; // Increment speed
            if (currentMotorSpeed > targetMotorSpeed) {
                currentMotorSpeed = targetMotorSpeed; // Avoid overshooting
            }
        } else if (currentMotorSpeed > targetMotorSpeed) {
            currentMotorSpeed -= rampStep; // Decrement speed
            if (currentMotorSpeed < targetMotorSpeed) {
                currentMotorSpeed = targetMotorSpeed; // Avoid undershooting
            }
        }

        // Set motor speed
		return currentMotorSpeed; // Replace with your motor control function
    }
	
}

int SmoothThrottlePitchAdjustment(int thrustvalue)
{
	PitchtargetMotorSpeed = thrustvalue;
	unsigned long currentTime = millis();
	if (currentTime - PitchlastUpdate > PitchupdateInterval) {
		PitchlastUpdate = currentTime;

		if (PitchcurrentMotorSpeed < PitchtargetMotorSpeed) {
			PitchcurrentMotorSpeed += PitchrampStep; // Increment speed
			if (PitchcurrentMotorSpeed > PitchtargetMotorSpeed) {
				PitchcurrentMotorSpeed = PitchtargetMotorSpeed; // Avoid overshooting
			}
		}
		else if (PitchcurrentMotorSpeed > PitchtargetMotorSpeed) {
			PitchcurrentMotorSpeed -= PitchrampStep; // Decrement speed
			if (PitchcurrentMotorSpeed < PitchtargetMotorSpeed) {
				PitchcurrentMotorSpeed = PitchtargetMotorSpeed; // Avoid undershooting
			}
		}

		// Set motor speed
		return PitchcurrentMotorSpeed; // Replace with your motor control function
	}

}


