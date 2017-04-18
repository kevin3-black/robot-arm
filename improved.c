#pragma config(Sensor, in1,    shoulderPot,    sensorPotentiometer)
#pragma config(Sensor, in2,    elbowPot,       sensorPotentiometer)
#pragma config(Sensor, in3,    wristPot,       sensorPotentiometer)
#pragma config(Sensor, in5,    shoulderFakePot, sensorPotentiometer)
#pragma config(Sensor, in6,    elbowFakePot,   sensorPotentiometer)
#pragma config(Sensor, in7,    wristFakePot,   sensorPotentiometer)
#pragma config(Motor,  port1,           slider,        tmotorVex393_HBridge, openLoop, reversed)
#pragma config(Motor,  port2,           elbow,         tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port3,           shoulder,      tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port5,           wrist,         tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor,  port7,           outtake,       tmotorServoStandard, openLoop)
#pragma config(Motor,  port8,           intake,        tmotorServoStandard, openLoop)
#pragma config(Motor,  port9,           flipper,       tmotorVex393_MC29, openLoop)
//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//

//CONSTANTS
#define INTAKE_OPEN -127
#define INTAKE_CLOSED -17
#define OUTTAKE_OPEN 70
#define OUTTAKE_CLOSED -70
#define FLIP_SPEED 0.17
#define SLIDE_SPEED 0.5
#define WRIST_SENS 0.02
#define ELBOW_SENS 0.01
#define MAX_SPEED 40
#define TICK_DELAY 10

//PID GAIN CONSTANTS
#define P_S 0.15
#define I_S 0.000001
#define D_S 0.5

#define P_E 0.13
#define I_E 0.000001
#define D_E 0.4

#define P_W 0.15
#define I_W 0.000001
#define D_W 0.5

typedef struct {
    float integral; //variable for accumulating integral value
    float error; //variable to store previous error for calculating the derivative
} PIDState;

typedef struct {
		PIDState shoulderState;
		PIDState elbowState;
		PIDState wristState;
} armState;

float PID(PIDState &state, float target, float current, float p, float i, float d);
void move(armState &arm, float sTarget, float eTarget, float wTarget);

task endEffectors() {
    while(true) {
        if(vexRT[Btn6U]) motor[intake] = INTAKE_OPEN;
        else             motor[intake] = INTAKE_CLOSED;

        if(vexRT[Btn5U]) motor[outtake] = OUTTAKE_OPEN;
        else             motor[outtake] = OUTTAKE_CLOSED;

        //motor[flipper] =  (vexRT[Btn7U] - vexRT[Btn7L]) * FLIP_SPEED;

		motor[slider] = SLIDE_SPEED * vexRT[Ch3]; //maps up/down slider to left joystick
		motor[flipper] = FLIP_SPEED * vexRT[Ch2]; //maps flipper to right joystick
    }
}

task main() {
    startTask(endEffectors);
    float wristOffset = 0;
    float elbowOffset = 0;

    PIDState shoulderStart; shoulderStart.integral = 0; shoulderStart.error = SensorValue[shoulderFakePot] - SensorValue[shoulderPot];
    PIDState elbowStart; elbowStart.integral = 0; elbowStart.error = SensorValue[elbowFakePot] - SensorValue[elbowPot];
    PIDState wristStart; wristStart.integral = 0; wristStart.error = SensorValue[wristFakePot] - SensorValue[wristPot];
    armState arm; arm.shoulderState = shoulderStart; arm.elbowState = elbowStart; arm.wristState = wristStart;
    while(true) {
    		if (abs(vexRT[Ch4]) > 20) wristOffset += vexRT[Ch4]*WRIST_SENS;
    		if (abs(vexRT[Ch1]) > 20) elbowOffset -= vexRT[Ch1]*ELBOW_SENS;

       	if (vexRT[Btn7U] || false) move(arm, 3800, 3890+elbowOffset, 350+wristOffset);
       	else if (vexRT[Btn8U] || false) move(arm, 2900, 3160+elbowOffset, 730+wristOffset);
       	else if (vexRT[Btn8L] || true) move(arm, 2020, 2700+elbowOffset, 1770+wristOffset);
       	else if (vexRT[Btn8D] || false) move(arm, 1895, 2187+elbowOffset, 2850+wristOffset);
       	else if (vexRT[Btn8R] || false) move(arm, 1800, 1350+elbowOffset, 3500+wristOffset);
       	else if (vexRT[Btn5D] || false) move(arm, 2815, 2780+elbowOffset, 1085+wristOffset);
    		else {
    			move(arm, SensorValue[shoulderFakePot], SensorValue[elbowFakePot], SensorValue[wristFakePot]);
    			wristOffset = 0;
    			elbowOffset = 0;
    		}
        sleep(TICK_DELAY);
    }
}

void move(armState &arm, float sTarget, float eTarget, float wTarget) {
		motor[shoulder] = PID(arm.shoulderState, sTarget, SensorValue[shoulderPot], P_S, I_S, D_S);
		motor[elbow] = PID(arm.elbowState, eTarget, SensorValue[elbowPot], P_E, I_E, D_E);
		motor[wrist] = PID(arm.wristState, wTarget, SensorValue[wristPot], P_W, I_W, D_W);
}

float PID(PIDState &state, float target, float current, float p, float i, float d) {
    float error = target - current;
    /*if (abs(state.integral) < 500000)*/ state.integral += error * TICK_DELAY;
    float derivative = (error - state.error) / TICK_DELAY;
    state.error = error;

		float speed = p*error + i*state.integral + d*derivative;
   	return speed <= MAX_SPEED ? speed : MAX_SPEED;
}
