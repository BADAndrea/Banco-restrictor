/*
OBJ: Execution of ModBus commands
	- control of stepper
	- control of DC motor
	- monitor Volage, current and power consumed by DC motor
*/

#define DC_PUMP_MOTOR  //enable/disable PWM control of pump (pump v2)
//#define DC_PUMP_MOTOR_SENSE //enable/disable sensing of pump speed via hall effect sensor
#define DC_PUMP_POWER_MEASURE //enable/disable measuring of voltage, current, power of pump

#include <Arduino.h>
#include <ModbusRtu.h>
#include <AccelStepper.h>
#include <PWM/PWM.h>
#ifdef DC_PUMP_POWER_MEASURE
#include <SPI.h>
#include <Adafruit_INA219.h>
#endif

//general config paramters
constexpr int CHIP_ID = 5; //modbus ID 
constexpr int VECTOR_LENGHT = 11;
constexpr int MICROSTEPPING = 8;
constexpr int STEPPER_LOW_CURRENT = 50; //approx 1v -> 150mA
constexpr int STEPPER_HIGH_CURRENT = 150; //127 approx 2.5v -> 400mA
constexpr int PWM_frequency = 15000; //Hz

//data array positions for modbus network sharing
constexpr int STEPPER_SPEED_VECTOR_POSITION = 0;
constexpr int STEPPER_ACCELERATION_VECTOR_POSITION = 1;

constexpr int STEPPER_STEPS_VECTOR_POSITION = 2;
constexpr int STEPPER_POS1_VECTOR_POSITION = 3;
constexpr int STEPPER_POS2_VECTOR_POSITION = 4;

constexpr int MODBUS_IN_COUNT_VECTOR_POSITION = 5;
constexpr int MODBUS_OUT_COUNT_VECTOR_POSITION = 6;
constexpr int MODBUS_ERROR_COUNT_VECTOR_POSITION = 7;

constexpr int DC_PUMP_SPEED_POSITION = 8;
constexpr int DC_PUMP_RPM_FROM_SENS_POSITION = 9;
constexpr int DC_PUMP_CURRENT_POSITION = 10;

//arduino pin configuration
constexpr int U8TXENPIN = 6;

constexpr int STEPPER_CURRENT_PIN_MOVING = 3;
constexpr int STEPPER_DIRECTION_PIN = 8;
constexpr int STEPPER_STEP_PIN = 7;

//stepper variabiles
bool stepper_resetting = true;
unsigned int stepper_old_speed, stepper_old_acceleration;
unsigned int stepper_current_delay = 300; //to make sure stepper driver reaches new current conditions
unsigned long stepper_move_command_time = 0;

#ifdef DC_PUMP_MOTOR
constexpr int DC_PUMP_SPEED_CONTROL_PIN = 9;
#endif // DC_PUMP_MOTOR

#ifdef DC_PUMP_MOTOR_SENSE
constexpr int DC_PUMP_SPEED_SENSE_PIN = 2;
volatile unsigned int rpm = 0;
unsigned int rpm_out = 0;
unsigned long last_rpm_reset_time = 0;
constexpr int RPM_UPDATE_TIME = 10; //in seconds
#endif // DC_PUMP_MOTOR_SENSE

#ifdef DC_PUMP_MOTOR
unsigned int pump_speed_percent = 0;
#endif // DC_PUMP_MOTOR

#ifdef DC_PUMP_POWER_MEASURE
unsigned int current_mA = 0;
unsigned long last_motor_current_update = 0;
constexpr int CURRENT_UPDATE_TIME = 200; //in milliseconds

// Define the number of samples to keep track of. The higher the number, the
// more the readings will be smoothed, but the slower the output will respond to
// the input. Using a constant rather than a normal variable lets us use this
// value to determine the size of the readings array.
const int numReadings = 20;
int readIndex = 0;              	// the index of the current reading

int current_readings[numReadings];	// the readings from the analog input
int current_total = 0;            	// the running total
int current_average = 0;     			// the average
#endif

//data array for modbus network sharing
uint16_t au16data[VECTOR_LENGHT] = {
 100, //stepper speed
 10, //stepper acceleration
 0, //stepper1 steps to do
 0, //stepper1 current position (16bit), part1
 0, //stepper1 current position (16bit), part2, NOT USED in this configuration
 0, //IN count
 0, //OUT count
 0, //ERROR count
 0, //DC pump motor speed
 0, //DC pump motor speed read from sensor
 0 //DC pump current
};

/*
 *  Modbus object declaration
 *  u8id : node id = 0 for master, = 1..247 for slave
 *  u8serno : serial port (use 0 for Serial)
 *  u8txenpin : 0 for RS-232 and USB-FTDI
 *               or any pin number > 1 for RS-485
 */

Modbus slave(CHIP_ID, 0, U8TXENPIN);
AccelStepper stepper(1, STEPPER_STEP_PIN, STEPPER_DIRECTION_PIN); //step, dir
#ifdef DC_PUMP_POWER_MEASURE
Adafruit_INA219 ina219;
#endif

void set_holding_current(int stepper_number) {
	analogWrite(STEPPER_CURRENT_PIN_MOVING, STEPPER_LOW_CURRENT);
}

void set_moving_current(int stepper_number) {
	analogWrite(STEPPER_CURRENT_PIN_MOVING, STEPPER_HIGH_CURRENT);
	stepper_move_command_time = millis();
}

void set_stepper_SA() {
	stepper.setMaxSpeed(au16data[STEPPER_SPEED_VECTOR_POSITION]);
	stepper.setAcceleration(au16data[STEPPER_ACCELERATION_VECTOR_POSITION]);

	stepper_old_speed = au16data[STEPPER_SPEED_VECTOR_POSITION];
	stepper_old_acceleration = au16data[STEPPER_ACCELERATION_VECTOR_POSITION];
}

void reset_vapour_position() {
	//stepper motors aren't damaged by stalling
	//valve has 500 steps maximum, 480 in regulation, migth be a good idea to full open valve and then close the desired amount of steps
	//in order to maintain a known reference
	set_stepper_SA();
	set_moving_current(0);
	stepper.move(-510 * MICROSTEPPING); //set an open move to stall
}

unsigned int l_2uint_int1(long long_number) { //split the long and return first unsigned integer
	union l_2uint {
		long l;
		uint16_t i[2];
	};
	union l_2uint l_number;
	l_number.l = long_number;
	return l_number.i[0];
}

unsigned int l_2uint_int2(long long_number) { //split the long and return second unsigned integer
	union l_2uint {
		long l;
		uint16_t i[2];
	};
	union l_2uint l_number;
	l_number.l = long_number;
	return l_number.i[1];
}

void update_modbus_data() {
	au16data[STEPPER_POS1_VECTOR_POSITION] = l_2uint_int1(stepper.currentPosition() / 8); //split the long into 2 unsigned integers
	au16data[STEPPER_POS2_VECTOR_POSITION] = l_2uint_int2(stepper.currentPosition());

	#ifdef DC_PUMP_MOTOR_SENSE
	au16data[DC_PUMP_RPM_FROM_SENS_POSITION] = rpm_out;
	#endif // DC_PUMP_MOTOR_SENSE

	au16data[MODBUS_IN_COUNT_VECTOR_POSITION] = slave.getInCnt(); //important to update for stepper move logic
	au16data[MODBUS_OUT_COUNT_VECTOR_POSITION] = slave.getOutCnt();
	au16data[MODBUS_ERROR_COUNT_VECTOR_POSITION] = slave.getErrCnt();

	au16data[STEPPER_POS1_VECTOR_POSITION] = 500 - au16data[STEPPER_STEPS_VECTOR_POSITION];

	#ifdef DC_PUMP_POWER_MEASURE
	au16data[DC_PUMP_CURRENT_POSITION] = current_average;
	#endif
}

#ifdef DC_PUMP_MOTOR_SENSE
void addrev() {
	rpm += 1;
}
#endif // DC_PUMP_MOTOR_SENSE

void setup() {
	//TCCR1B = TCCR1B & B11111000 | B00000010; // for PWM frequency of 3921.16 Hz
	//TCCR1B = TCCR1B & B11111000 | B00000001; // set timer 1 divisor to 1 for PWM frequency of 31372.55 Hz

	InitTimersSafe(); 
	SetPinFrequencySafe(DC_PUMP_SPEED_CONTROL_PIN, PWM_frequency);

	pinMode(STEPPER_CURRENT_PIN_MOVING, OUTPUT);
	analogWrite(STEPPER_CURRENT_PIN_MOVING, STEPPER_LOW_CURRENT);

	slave.begin(9600); //9600 baud, 8-bits, 1-bit stop
	reset_vapour_position();

	#ifdef DC_PUMP_MOTOR
	pinMode(DC_PUMP_SPEED_CONTROL_PIN, OUTPUT);
	#endif // DC_PUMP_MOTOR

	#ifdef DC_PUMP_MOTOR_SENSE
	attachInterrupt(digitalPinToInterrupt(DC_PUMP_SPEED_SENSE_PIN), addrev, RISING);
	#endif // DC_PUMP_MOTOR_SENSE

	#ifdef DC_PUMP_POWER_MEASURE
	ina219.begin();
	for (int thisReading = 0; thisReading < numReadings; thisReading++) {
		current_readings[thisReading] = 0;
	}
	#endif
}

void loop() {
	slave.poll(au16data, VECTOR_LENGHT); //Modbus magic
	
	if (millis() - stepper_move_command_time >= stepper_current_delay) {
		stepper.run(); //Stepper magic
	}

	if (!stepper_resetting) {
		//if new values of stepper speed / acceleration are provided, update accelstepper values
		if (stepper_old_speed != au16data[STEPPER_SPEED_VECTOR_POSITION] || stepper_old_acceleration != au16data[STEPPER_ACCELERATION_VECTOR_POSITION]) {
			set_stepper_SA();
		}

		if (au16data[STEPPER_STEPS_VECTOR_POSITION] != stepper.currentPosition() && au16data[MODBUS_IN_COUNT_VECTOR_POSITION] != slave.getInCnt()) {
			//if stepper is not where i want and just received a command, set new position to reach
			set_moving_current(0);
			stepper.moveTo(au16data[STEPPER_STEPS_VECTOR_POSITION] * MICROSTEPPING);
		}
	} 
	else {
		if (stepper.distanceToGo() == 0) {
			stepper.setCurrentPosition(0); //reset position, side effect of setting stepper speed to 0
			update_modbus_data();
			set_stepper_SA(); //setCurrentPosition has side effect of setting stepper speed to 0, re-issue update of speed and acceleration
			stepper_resetting = false;
		}
	}

	#ifdef DC_PUMP_MOTOR_SENSE
	if (millis() - last_rpm_reset_time > RPM_UPDATE_TIME * 1000) { //if ended acquisition time, "save" value and reset counter
		rpm_out = round(rpm * 60 / RPM_UPDATE_TIME);
		rpm = 0;
	}
	#endif // DC_PUMP_MOTOR_SENSE
	
	update_modbus_data();

	if (stepper.distanceToGo() == 0) { //reduce power to stepper if move is complete
		set_holding_current(0);
	}

	#ifdef DC_PUMP_MOTOR
	if (pump_speed_percent != au16data[DC_PUMP_SPEED_POSITION]) {
		pump_speed_percent = au16data[DC_PUMP_SPEED_POSITION];
		//analogWrite(DC_PUMP_SPEED_CONTROL_PIN, (int)map(pump_speed_percent, 0, 100, 0, 255));
		pwmWrite(DC_PUMP_SPEED_CONTROL_PIN, (int)map(pump_speed_percent, 0, 100, 0, 255));
	}
	#endif // DC_PUMP_MOTOR

	#ifdef DC_PUMP_POWER_MEASURE
	if (millis() - last_motor_current_update > CURRENT_UPDATE_TIME) {
		current_mA = ina219.getCurrent_mA();

		//a bit of rolling average
		current_total -= current_readings[readIndex];
		current_readings[readIndex] = current_mA;
		current_total += current_readings[readIndex];

		readIndex = readIndex + 1;
		if (readIndex == numReadings) {
			readIndex = 0;
		}

		current_average = current_total / numReadings;
	}
	#endif
}
