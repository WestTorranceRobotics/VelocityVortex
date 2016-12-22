package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="TeleOp", group="WTR")
//@Disabled
public class VelocityVortexIterativeTeleOp extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime stateTime = new ElapsedTime();

    RobotHardware robot = new RobotHardware();
    IterativeFunctions fanctions = new IterativeFunctions(robot);

    private boolean ramIsLeft = true;
    private boolean autoIsRunning = false;
    private int tunaCounter = 0;

    private enum state {
        STATE_SPOOL_UP_SHOOTERS,
        STATE_LOAD_BALL,
        STATE_WAIT_FOR_SHOOTERS,
        STATE_STANDBY,
        STATE_WAIT_FOR_BIG_SHOOT,
        STATE_WAIT_AGAIN,
    }

    private state currentState = state.STATE_STANDBY;

    @Override
    public void init() {
        robot.initRobotHardware(hardwareMap);
    }

    @Override
    public void init_loop() {
        fanctions.anushalizeRobotHardware();
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
/*
        //Driver2, a -> transport up, if not then down
        if(gamepad1.a){
            SetServo(.275);
        } if(!gamepad1.a) {
            SetServo(0.4);
        }
*/
        /*slow moving code
        if(!transportsAreUp() && gamepad2.a){
            SetServo(currentPos - .001);
        } if(!gamepad2.a) {
            SetServo(0.5);
        }
        */

        //driver1, left stick left motor
        if (Math.abs(gamepad1.left_stick_y) < .15) {
            robot.leftMotor.setPower(0);
        } else {
            robot.leftMotor.setPower(-gamepad1.left_stick_y);
        }

        //driver1, right stick right motor
        if (Math.abs(gamepad1.right_stick_y) < .15) {
            robot.rightMotor.setPower(0);
        } else {
            robot.rightMotor.setPower(-gamepad1.right_stick_y);
        }

        //driver1, right bumper -> intake in, left bumper -> intake out
        if(gamepad1.right_bumper) {
            robot.intakeMotor.setPower(1);
        } else {
            robot.intakeMotor.setPower(0);
        }
        if (autoIsRunning){
            if (gamepad1.left_bumper){
                newState(state.STATE_STANDBY);
            }
        }
/*
        //if right bumper on driver2 is pressed then set shooters to 70%
        if(gamepad1.left_bumper) {
            fanctions.setShooterSpeed(0.7);
        } else {
            fanctions.setShooterSpeed(0);
        }
*/
        if(gamepad1.left_trigger > .2) {
            ramIsLeft = true;
        } else if (gamepad1.right_trigger > .2) {
            ramIsLeft = false;
        }

        if(ramIsLeft) {
            fanctions.setRamServoLeft();
        } else {
            fanctions.setRamServoRight();
        }

        switch (currentState) {

            case STATE_SPOOL_UP_SHOOTERS:
                fanctions.setShooterSpeed(0.8);
                fanctions.setTransportDown();
                autoIsRunning = true;
                newState(state.STATE_WAIT_FOR_SHOOTERS);
                break;

            case STATE_WAIT_FOR_SHOOTERS:
                autoIsRunning = true;
                if (stateTime.time() >= 1.5) {
                    newState(state.STATE_LOAD_BALL);
                }
                break;

            case STATE_LOAD_BALL:
                autoIsRunning = true;
                fanctions.setTransportUp();
                tunaCounter++;
                newState(state.STATE_WAIT_FOR_BIG_SHOOT);
                break;

            case STATE_WAIT_FOR_BIG_SHOOT:
                autoIsRunning = true;
                if(tunaCounter > 3) {
                    newState(state.STATE_STANDBY);
                } else if (stateTime.time() >= 1.5) {
                    fanctions.setTransportDown();
                    newState(state.STATE_WAIT_AGAIN);
                }
                break;

            case STATE_WAIT_AGAIN:
                if (stateTime.time() >= .25){
                    newState(state.STATE_LOAD_BALL);
                }
                break;

            case STATE_STANDBY:
                if (gamepad1.left_bumper) {
                    newState(state.STATE_SPOOL_UP_SHOOTERS);
                }
                fanctions.setShooterSpeed(0);
                fanctions.setTransportDown();
                tunaCounter = 0;
                autoIsRunning = false;
                break;
        }
    }

    @Override
    public void stop() {
    }

    public void newState(state newState) {
        currentState = newState;
        stateTime.reset();
    }
}