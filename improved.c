#pragma config(Sensor, in1, shoulder, sensorPotentiometer)
#pragma config(Sensor, in2, elbow, sensorPotentiometer)
#pragma config(Sensor, in3, wrist, sensorPotentiometer)
#pragma config(Sensor, in5, shoulderFake, sensorPotentiometer)
#pragma config(Sensor, in6, elbowFake, sensorPotentiometer)
#pragma config(Sensor, in7, wristFake, sensorPotentiometer)
#pragma config(Motor, port1, slider, tmotorVex393_HBridge, openLoop, reversed)
#pragma config(Motor, port2, elbow, tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor, port3, shoulder, tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor, port5, wrist, tmotorVex393_MC29, openLoop, reversed)
#pragma config(Motor, port9, flipper, tmotorVex393_MC29, openLoop)
#pragma config(Motor, port7, outtake, tmotorServoStandard, openLoop)
#pragma config(Motor, port8, intake, tmotorServoStandard, openLoop)

//CONSTANTS
#define INTAKE_OPEN -127
#define INTAKE_CLOSED -17
#define OUTTAKE_OPEN 70
#define OUTTAKE_CLOSED -70
#define FLIP_SPEED 20
#define TICK_DELAY 10

//PID GAIN CONSTANTS
#define P_S 0.05
#define I_S 0.0001
#define D_S 0

#define P_E 0.05
#define I_E 0.0001
#define D_E 0

#define P_W 0.05
#define I_W 0.0001
#define D_W 0

typedef struct {
    float integral; //variable for accumulating integral value
    float error; //variable to store previous error for calculating the derivative
} PIDState;

float PID(PIDState &state, float target, float current, float p, float i, float d);

task endEffectors() {
    while(true) {
        if(vexRT[Btn5U]) setMotor(intake, INTAKE_OPEN);
        else             setMotor(intake, INTAKE_CLOSED);

        if(vexRT[Btn6U]) setMotor(outtake, OUTTAKE_OPEN);
        else             setMotor(outtake, OUTTAKE_CLOSED);

        motor[flipper] =  (vexRT[Btn7U] - vexRT[Btn7L]) * FLIP_SPEED;

		motor[slider] = vexRT[Ch3]; //maps up/down slider to left joystick
    }
}

task main() {
    startTask(endEffectors);

    PIDState shoulder; shoulder.integral = 0; shoulder.error = SensorValue[shoulderFake] - SensorValue[shoulder];
    PIDState elbow; elbow.integral = 0; elbow.error = SensorValue[elbowFake] - SensorValue[elbow];
    PIDState wrist; wrist.integral = 0; wrist.error = SensorValue[wristFake] - SensorValue[wrist];
    while(true) {
        motor[shoulder] = PID(shoulder, SensorValue[shoulderFake], SensorValue[shoulder], P_S, I_S, D_S);
        motor[elbow] = PID(elbow, SensorValue[elbowFake], SensorValue[elbow], P_E, I_E, D_E);
        motor[wrist] = PID(wrist, SensorValue[wristFake], SensorValue[wrist], P_W, I_W, D_W);
        sleep(TICK_DELAY);
    }
}

float PID(PIDState &state, float target, float current, float p, float i, float d) {
    float error = target - current;
    state.integral += error * TICK_DELAY;
    float derivative = (error - state.error) / TICK_DELAY;
    state.error = error;

    return p*error + i*state.integral + d*derivative;
}
