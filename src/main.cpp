/*
OBJ: Execution of ModBus commands
	- control of stepper
	- control of DC motor
*/

#define DC_PUMP_MOTOR  //enable/disable PWM control of pump (pump v2)
#define DC_PUMP_MOTOR_SENSE //enable/disable sensing of pump speed via hall effect sensor
#define DC_PUMP_POWER_MEASURE //enable/disable measuring of voltage, current, power of pump

#include <Arduino.h>
#include <ModbusRtu.h>
#include <AccelStepper.h>
#ifdef DC_PUMP_POWER_MEASURE
#include <SPI.h>
#include <Adafruit_INA219.h>
#endif

//general config paramters
constexpr int CHIP_ID = 5; //modbus ID 
constexpr int VECTOR_LENGHT = 12;
constexpr int MICROSTEPPING = 8;
constexpr int STEPPER_LOW_CURRENT = 50; //approx 1v -> 150mA
constexpr int STEPPER_HIGH_CURRENT = 127; //approx 2.5v -> 400mA

//data array positions for modbus network sharing
constexpr int STEPPER_SPEED_VECTOR_POSITION = 0;
constexpr int STEPPER_ACCELERATION_VECTOR_POSITION = 1;

constexpr int STEPPER1_STEPS_VECTOR_POSITION = 2;
constexpr int STEPPER1_POS1_VECTOR_POSITION = 3;
constexpr int STEPPER1_POS2_VECTOR_POSITION = 4;

constexpr int MODBUS_IN_COUNT_VECTOR_POSITION = 5;
constexpr int MODBUS_OUT_COUNT_VECTOR_POSITION = 6;
constexpr int MODBUS_ERROR_COUNT_VECTOR_POSITION = 7;

constexpr int DC_PUMP_SPEED_POSITION = 8;
constexpr int DC_PUMP_RPM_FROM_SENS_POSITION = 9;
constexpr int DC_PUMP_VOLTAGE_POSITION = 10;
constexpr int DC_PUMP_CURRENT_POSITION = 11;

//arduino pin configuration
constexpr int U8TXENPIN = 6;

constexpr int STEPPER1_CURRENT_PIN_MOVING = 3;
constexpr int STEPPER1_DIRECTION_PIN = 8;
constexpr int STEPPER1_STEP_PIN = 7;

//stepper variabiles
bool stepper_resetting = true;
unsigned int stepper_old_speed, stepper_old_acceleration;
unsigned int stepper_current_delay = 200; //to make sure stepper driver reaches new current conditions

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
float current_mA = 0;
float loadvoltage_mV = 0;
unsigned long last_motor_power_update = 0;
constexpr int POWER_UPDATE_TIME = 2; //in seconds
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
 0, //DC pump voltage
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
AccelStepper stepper(1, STEPPER1_STEP_PIN, STEPPER1_DIRECTION_PIN); //step, dir
Adafruit_INA219 ina219;

void set_holding_current(int stepper_number) {
	analogWrite(STEPPER1_CURRENT_PIN_MOVING, STEPPER_LOW_CURRENT);
}

void set_moving_current(int stepper_number) {
	analogWrite(STEPPER1_CURRENT_PIN_MOVING, STEPPER_HIGH_CURRENT);
	delay(stepper_current_delay); //delay to possibly give enought time to driver to reach new current (probably doesn't matter)
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

	/* CHECK NEW RESET POSITION IS WORKING
	while (stepper_vapour.distanceToGo() != 0) {
		stepper_vapour.run();
	}
	stepper_vapour.setCurrentPosition(0); //reset position, side effect of setting stepper speed to 0
	update_modbus_data();
	set_stepper_SA(); //setCurrentPosition has side effect of setting stepper speed to 0, re-issue update of speed and acceleration
	*/
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
	au16data[STEPPER1_POS1_VECTOR_POSITION] = l_2uint_int1(stepper.currentPosition() / 8); //split the long into 2 unsigned integers
	au16data[STEPPER1_POS2_VECTOR_POSITION] = l_2uint_int2(stepper.currentPosition());

	#ifdef DC_PUMP_MOTOR_SENSE
	au16data[DC_PUMP_RPM_FROM_SENS_POSITION] = rpm_out;
	#endif // DC_PUMP_MOTOR_SENSE

	au16data[MODBUS_IN_COUNT_VECTOR_POSITION] = slave.getInCnt(); //important to update for stepper move logic
	au16data[MODBUS_OUT_COUNT_VECTOR_POSITION] = slave.getOutCnt();
	au16data[MODBUS_ERROR_COUNT_VECTOR_POSITION] = slave.getErrCnt();

	au16data[STEPPER1_POS1_VECTOR_POSITION] = 500 - au16data[STEPPER1_STEPS_VECTOR_POSITION];

	#ifdef DC_PUMP_POWER_MEASURE
	au16data[DC_PUMP_VOLTAGE_POSITION] = loadvoltage_mV;
	au16data[DC_PUMP_CURRENT_POSITION] = current_mA;
	#endif
}

#ifdef DC_PUMP_MOTOR_SENSE
void addrev() {
	rpm += 1;
}
#endif // DC_PUMP_MOTOR_SENSE

void setup() {
	pinMode(STEPPER1_CURRENT_PIN_MOVING, OUTPUT);
	analogWrite(STEPPER1_CURRENT_PIN_MOVING, STEPPER_LOW_CURRENT);

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
	#endif
}

void loop() {
	slave.poll(au16data, VECTOR_LENGHT); //Modbus magic
	stepper.run(); //Stepper magic

	if (!stepper_resetting) {
		//if new values of stepper speed / acceleration are provided, update accelstepper values
		if (stepper_old_speed != au16data[STEPPER_SPEED_VECTOR_POSITION] || stepper_old_acceleration != au16data[STEPPER_ACCELERATION_VECTOR_POSITION]) {
			set_stepper_SA();
		}

		if (au16data[STEPPER1_STEPS_VECTOR_POSITION] != stepper.currentPosition() && au16data[MODBUS_IN_COUNT_VECTOR_POSITION] != slave.getInCnt()) {
			//if stepper is not where i want and just received a command, set new position to reach
			set_moving_current(0);
			stepper.moveTo(au16data[STEPPER1_STEPS_VECTOR_POSITION] * MICROSTEPPING);
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
		analogWrite(map(pump_speed_percent, 0, 100, 0, 255), DC_PUMP_SPEED_CONTROL_PIN);
	}
	#endif // DC_PUMP_MOTOR

	#ifdef DC_PUMP_POWER_MEASURE
	if (millis() - last_motor_power_update > POWER_UPDATE_TIME * 1000) {
		current_mA = (int)ina219.getCurrent_mA();
		loadvoltage_mV = (int)ina219.getBusVoltage_V() * 1000 + ina219.getShuntVoltage_mV();
	}
	#endif
}
