/*
 Name:		DNOFlight.ino
 Created:	11/3/2023 7:16:00 AM
 Author:	garyj
*/

// the setup function runs once when you press reset or power the board

#define SDA_PIN 32
#define SCL_PIN 33

// motor Pins
const int FRONT_LEFT_PIN = 15, FRONT_RIGHT_PIN = 17, BACK_LEFT_PIN = 4, BACK_RIGHT_PIN = 16;
// Constants for the ESC
const int ESC_FREQ = 50; // ESCs usually use a frequency of 50Hz
const int ESC_RESOLUTION = 16; // 16-bit resolution
const int minDuty = 3276; // Minimum duty cycle for ESC (usually around 1ms pulse)
const int maxDuty = 6553; // Maximum duty cycle for ESC (usually around 2ms pulse)

const int ChFrontRight = 0;
const int ChFrontLeft = 1;
const int ChBackRight = 4;
const int ChBackLeft = 5;

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

float RollKp = 1.0; // Proportional gain
float RollKi = 0.1; // Integral gain
float RollKd = 0.05; // Derivative gain

float Rollintegral = 0;
float Rollprevious_error = 0;

// pitch tuning parameters

float PitchKp = 1.0; // Proportional gain
float PitchKi = 0.1; // Integral gain
float PitchKd = 0.05; // Derivative gain

float Pitchintegral = 0;
float Pitchprevious_error = 0;



int GetRollFromTransmitter()
{
	int pwmValue = pulseIn(RADIO_INPUT_PIN_CH1, HIGH, 25000);

	Serial.println("Roll PWM: " + String(pwmValue));

	if (abs(pwmValue - CH1_Roll_Center) <= deadzone) {
		// Get the current roll angle or rate
		float rollError = imu.gyro_x_radps() * 180 / PI; // Error in degrees

		// Proportional term
		float Pout = RollKp * rollError;

		// Integral term
		Rollintegral += rollError;
		float Iout = RollKi * Rollintegral;

		// Derivative term
		float derivative = rollError - Rollprevious_error;
		float Dout = RollKd * derivative;

		// Calculate total output
		int output = Pout + Iout + Dout;

		// Restrict to max/min
		int correctedThrottle = constrain(CurrentThrottle - output, 0, 100);

		// Save error for next loop
		Rollprevious_error = rollError;

		Serial.println("Roll Corrected Throttle");
		Serial.println(correctedThrottle);
		return correctedThrottle;
	}
	else
	{
		// 10% of current throttle
		int TenPercent = CurrentThrottle / 10;
		int Lowend = CurrentThrottle - TenPercent;
		int Highend = CurrentThrottle + TenPercent;

		int thrust = map(pwmValue, CH1_Roll_Left, CH1_Roll_Right, Lowend, Highend);

		int constrained = constrain(thrust, 0, 100);
		Serial.println("Roll Manual Throttle");
		Serial.println(constrained);
		return constrained;
	}
}
int GetPitchFromTransmitter()
{
	int pwmValue = pulseIn(RADIO_INPUT_PIN_CH2, HIGH, 25000);
	Serial.println("Pitch PWM: " + String(pwmValue));

	// Check if the stick is within the deadzone around the center
	if (abs(pwmValue - CH2_Pitch_Center) <= PitchDeadzone) {
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

		// Restrict to max/min
		int correctedThrottle = constrain(CurrentThrottle - output, 0, 100);

		// Save error for next loop
		Pitchprevious_error = PitchError;

		Serial.println("Pitch Corrected Throttle");
		Serial.println(correctedThrottle);
		return correctedThrottle;
	}
	else {
		// Manual control
		int TenPercent = CurrentThrottle / 10;
		int Lowend = CurrentThrottle - TenPercent;
		int Highend = CurrentThrottle + TenPercent;

		int thrust = map(pwmValue, CH2_Pitch_Forward, CH2_Pitch_Backward, Lowend, Highend);
		Serial.println("Pitch Manual Throttle");
		Serial.println(thrust);
		return constrain(thrust, 0, 100);
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
	// set all motors to the same speed

	setESCPower(CurrentThrottle, ChFrontLeft);
	setESCPower(CurrentThrottle, ChFrontRight);
	setESCPower(CurrentThrottle, ChBackLeft);
	setESCPower(CurrentThrottle, ChBackRight);

	// 10% of current throttle
	int TenPercent = CurrentThrottle / 20;

	if (Speed < TenPercent / 2)
	{
		setESCPower(Speed, ChFrontLeft);
		setESCPower(Speed, ChBackRight);
	}
	else
	{
		setESCPower(Speed, ChFrontRight);
		setESCPower(Speed, ChBackLeft);
	}
}

void SetRollMotorSpeeds(int Speed)
{
	// set all motors to the same speed
	setESCPower(CurrentThrottle, ChFrontLeft);
	setESCPower(CurrentThrottle, ChFrontRight);
	setESCPower(CurrentThrottle, ChBackLeft);
	setESCPower(CurrentThrottle, ChBackRight);

	// 10% of current throttle
	int TenPercent = CurrentThrottle / 20;

	if (Speed < TenPercent / 2)
	{
		setESCPower(Speed, ChFrontRight);
		setESCPower(Speed, ChBackLeft);
	}
	else
	{
		setESCPower(Speed, ChFrontLeft);
		setESCPower(Speed, ChBackRight);
	}
}

void SetYawMotorSpeeds(int Speed)
{
	// set all motors to the same speed
	setESCPower(CurrentThrottle, ChFrontLeft);
	setESCPower(CurrentThrottle, ChFrontRight);
	setESCPower(CurrentThrottle, ChBackLeft);
	setESCPower(CurrentThrottle, ChBackRight);

	// 10% of current throttle
	int TenPercent = CurrentThrottle / 20;
	if (Speed < TenPercent / 2)
	{
		setESCPower(Speed, ChFrontLeft);
		setESCPower(Speed, ChBackRight);
	}
	else
	{
		setESCPower(Speed, ChFrontRight);
		setESCPower(Speed, ChBackLeft);
	}
}

void SetMotorSpeed(int motor, int speed)
{
	ledcWrite(motor, speed);
}
// Function to calibrate one ESC
void calibrateESC(int LedcMotorChannel) {
	// Send max throttle
	setESCPower(0, LedcMotorChannel);
	delay(500);
	setESCPower(100, LedcMotorChannel); // 255 for 8-bit resolution (full throttle)
	Serial.println("Set to max throttle for calibration. Connect the battery now. Wait for beeps.");
	delay(2000); // Wait 5 seconds, adjust timing as needed

	// Send min throttle
	setESCPower(0, LedcMotorChannel); // 0 for min throttle
	Serial.println("Set to min throttle. Wait for confirmation beeps.");
	delay(2000); // Wait for confirmation beeps, adjust timing as needed

	// Calibration done
	Serial.println("ESC Calibration done.");
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
	int dutyCycle = map(speed, 0, 100, minDuty, maxDuty);
	ledcWrite(channel, dutyCycle);
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

	// Attach the ESC signal pin to the LEDC
	ledcSetup(ChBackLeft, ESC_FREQ, ESC_RESOLUTION);
	ledcSetup(ChBackRight, ESC_FREQ, ESC_RESOLUTION);
	ledcSetup(ChFrontLeft, ESC_FREQ, ESC_RESOLUTION);
	ledcSetup(ChFrontRight, ESC_FREQ, ESC_RESOLUTION);

	// attach the pins to the Ledc Chnnels

	ledcAttachPin(FRONT_RIGHT_PIN, ChFrontRight);
	ledcAttachPin(FRONT_LEFT_PIN, ChFrontLeft);
	ledcAttachPin(BACK_RIGHT_PIN, ChBackRight);
	ledcAttachPin(BACK_LEFT_PIN, ChBackLeft);
	delay(1000); // to allow the ESC to initialize
	calibrateESC(ChFrontRight);
	calibrateESC(ChFrontLeft);
	calibrateESC(ChBackRight);
	calibrateESC(ChBackLeft);

	initSensors();
}

// Add the main program code into the continuous loop() function
void loop()
{
	updateIMUData();

	CurrentThrottle = GetThrottleFromTransmitter();

	int Roll = GetRollFromTransmitter();
	int Pitch = GetPitchFromTransmitter();
	int Yaw = GetYawFromTransmitter();

	SetPitchMotorSpeeds(Pitch);
	SetRollMotorSpeeds(Roll);
	SetYawMotorSpeeds(Yaw);
}