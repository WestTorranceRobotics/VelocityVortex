/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Iterative Auto", group="WTR")  // @Autonomous(...) is the other common choice
@Disabled
public class IterativeShootAndParkInCenter extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime stateTime = new ElapsedTime();
    RobotHardware robot = new RobotHardware();
    IterativeFunctions Fanctions = new IterativeFunctions();

    private enum state {
        STATE_SPOOL_UP_SHOOTERS,
        //Rev these shooters.
        STATE_DRIVE_TO_VORTEX,
        //Driving to vortex.
        STATE_WAIT_FOR_SHOOTERS,
        //Waiting for shooters to get power.
        STATE_UP,
        //Moving transport ramp up to shoot ball.
        STATE_WAIT,
        //Waiting half a second to move transport ramp back down.
        STATE_SHUT_OFF_SHOOTERS,
        //Turning off shooter motors.
        STATE_TURN_RIGHT,
        //Turning right 90 degrees.
        STATE_MOVE_FORWARD,
        //Move forward two feet.
        STATE_TURN_RIGHT_MORE,
        //Turning right 45 degrees.
        STATE_PARK_IN_CENTER_BACKWARDS,
        //Move two feet backwards to park on center.
    }

    private state currentState;


    @Override
    public void init() {
        robot.initRobotHardware(hardwareMap);
    }

    @Override
    public void init_loop() {
        robot.anushalizeRobotHardware();
    }

    @Override
    public void start() {
        runtime.reset();
        newState(state.STATE_SPOOL_UP_SHOOTERS);
    }

    @Override
    public void loop() {
        switch (currentState) {
            case STATE_SPOOL_UP_SHOOTERS:
                robot.setShooterSpeed(.8);
                newState(state.STATE_DRIVE_TO_VORTEX);
                break;
                //Rev these shooters.
            case STATE_DRIVE_TO_VORTEX:
                Fanctions.setPos(48, .6);
                newState(state.STATE_WAIT_FOR_SHOOTERS);
                break;
                //Driving to vortex.
            case STATE_WAIT_FOR_SHOOTERS:
                if (robot.leftShooterMotor.getPower() >= .8 && robot.rightShooterMotor.getPower() >= .8) {
                    newState(state.STATE_UP);
                }
                break;
                //Waiting for shooters to get power.
            case STATE_UP:
                if (robot.transportsAreUp()) {
                    newState(state.STATE_WAIT);
                }
                break;
                //Moving transport ramp up to shoot ball.
            case STATE_WAIT:
                if (stateTime.time() >= .5) {
                    robot.setTransportsDown();

                    newState(state.STATE_SHUT_OFF_SHOOTERS);
                }
                break;
                //Waiting half a second to move transport ramp back down.
            case STATE_SHUT_OFF_SHOOTERS:
                robot.setShooterSpeed(0);
                newState(state.STATE_TURN_RIGHT);
                Fanctions.setDegrees(90);
                break;
                //Turning off shooter motors.
            case STATE_TURN_RIGHT:
                if(Fanctions.doneTurning()) {
                    Fanctions.endmove();
                    newState(state.STATE_MOVE_FORWARD);
                } else {
                    Fanctions.turn();
                }
                break;
                //Turning right 90 degrees.
            case STATE_MOVE_FORWARD:
                Fanctions.setPos(24, .6);
                Fanctions.setDegrees(45);
                break;
                //Move forward two feet.
            case STATE_TURN_RIGHT_MORE:
                if(Fanctions.doneTurning()){
                    Fanctions.endmove();
                    newState(state.STATE_PARK_IN_CENTER_BACKWARDS);
                } else {
                    Fanctions.turn();
                }
                break;
                //Turning right 45 degrees.
            case STATE_PARK_IN_CENTER_BACKWARDS:
                Fanctions.setPos(-24, .6);
                break;
                //Move two feet backwards to park on center.
        }
    }


    public void stop() {
    }

    private void newState(state newState) {
        // Reset the state time, and then change to next state.
        stateTime.reset();
        currentState = newState;
    }

    public void setPos(double inches, double goes) {

        robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        int ticks = (int) (inches * (1 / 4 * 3.14159265359) * (16 / 24) * (1120));
        int currentleft = robot.leftMotor.getCurrentPosition();
        int currentright = robot.rightMotor.getCurrentPosition();

        robot.leftMotor.setTargetPosition(ticks + currentleft);
        robot.rightMotor.setTargetPosition(ticks + currentright);
        robot.leftMotor.setPower(goes);
        robot.rightMotor.setPower(goes);
    }

    public void endmove() {
        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);
    }

    public void runtoposition(double inches, double speed) {
        setPos(inches, speed);
        while (robot.leftMotor.isBusy() && robot.rightMotor.isBusy()) {

        }
    }

        public void turn(int degrees) {
            int intheading = robot.gyro.getHeading();

            int multiplier = (degrees / Math.abs(degrees));

            robot.leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            while (Math.abs(robot.gyro.getHeading() - intheading) <= Math.abs(degrees)) {

                robot.leftMotor.setPower(multiplier * .6);
                robot.rightMotor.setPower(-.6 * multiplier);

            }
            robot.leftMotor.setPower(0);
            robot.rightMotor.setPower(0);
        }


    }




