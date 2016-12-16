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
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Iterative Auto", group="WTR")  // @Autonomous(...) is the other common choice
//@Disabled
public class ShootandSingleBeacon extends OpMode
{
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime stateTime = new ElapsedTime();
    RobotHardware robot = new RobotHardware();
    IterativeFunctions fanctions = new IterativeFunctions(robot);

    boolean isPressed = false;
    boolean isRed = true;
    boolean beaconIsRed = false;

    byte[] colorCcache;

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
        STATE_TURN_LEFT,
        //Turn left 45 degrees.
        STATE_MOVE_FORTYTWO_INCHES,
        //Move forty two inches forward.
        STATE_WAIT_TO_TURN,
        STATE_TURN_RIGHT,
        //Turn right 45 degrees.
        STATE_SENSE_WHITE_LINE,
        //sense the white line and move on it.
        STATE_MOVE_LEFT_MORE,
        //Move left 90 degrees.
        STATE_SENSE_COLOR,
        //Determine the color of the beacon.
        STATE_WAIT_FOR_RAM,
        //wait a second to have the ram turn.
        STATE_PREPARE_TO_GIT_RAMMED_M8_U_WOT_RAMMING_SPEED,
        //Git rammed. :^)
        STATE_STAND_BACK,
        //Move backwards
    }

    state currentState;

    @Override
    public void init() {
        robot.initRobotHardware(hardwareMap);
        robot.beaconSensor.enableLed(false);
    }

    @Override
    public void init_loop() {
        robot.transportServo.setPosition(.4);
        robot.ramServo.setPosition(.5);
    }

    @Override
    public void start() {
        runtime.reset();
        newState(state.STATE_SPOOL_UP_SHOOTERS);
    }

    @Override
    public void loop() {

        telemetry.addData("state",currentState);

        if(robot.beaconSensor.blue()>robot.beaconSensor.red()) {
            beaconIsRed = false;
        } else {
            beaconIsRed = true;
        }

        switch (currentState) {

            case STATE_SPOOL_UP_SHOOTERS:
                if (robot.leftShooterMotor.getPower() >= 1 && robot.rightShooterMotor.getPower() >= 1){
                    robot.transportServo.setPosition(.35);
                    newState(state.STATE_DRIVE_TO_VORTEX);
                }else{
                    robot.leftShooterMotor.setPower(1);
                    robot.rightShooterMotor.setPower(1);
                }
                break;
            //Rev these shooters.

            case STATE_DRIVE_TO_VORTEX:
                if (stateTime.time() >= .5) {
                    robot.transportServo.setPosition(0.4);
                    newState(state.STATE_WAIT_FOR_SHOOTERS);
                }
                break;
            //Driving to vortex.

            case STATE_WAIT_FOR_SHOOTERS:
                if (stateTime.time() >= .5) {
                    newState(state.STATE_UP);
                }
                break;
            //Waiting for shooters to get power.

            case STATE_UP:
                robot.transportServo.setPosition(.275);
                newState(state.STATE_WAIT);
                break;
            //Moving transport ramp up to shoot ball.

            case STATE_WAIT:
                if (stateTime.time() >= .5) {
                    newState(state.STATE_SHUT_OFF_SHOOTERS);
                }
                break;
            //Waiting half a second to move transport ramp back down.

            case STATE_SHUT_OFF_SHOOTERS:
                if(stateTime.time() >= .5){
                    robot.transportServo.setPosition(.4);
                    fanctions.setDegrees(-45);
                    newState(state.STATE_TURN_LEFT);
                }
                break;
            //Turning off shooter motors.

            case STATE_TURN_LEFT:
                if(fanctions.doneTurning()) {
                    fanctions.endmove();
                    newState(state.STATE_MOVE_FORTYTWO_INCHES);
                } else {
                    fanctions.turn();
                }
                break;
                //Move 45 degrees left.

            case STATE_MOVE_FORTYTWO_INCHES:
                fanctions.setPos(45, .6);
                newState(state.STATE_TURN_RIGHT);
                break;

            case STATE_WAIT_TO_TURN:
                if(!fanctions.driveMotorsAreBusy()){
                    fanctions.setDegrees(45);
                    newState(state.STATE_TURN_RIGHT);
                }

            case STATE_TURN_RIGHT:
                if (fanctions.doneTurning()) {
                    fanctions.endmove();
                    newState(state.STATE_SENSE_WHITE_LINE);
                } else {
                    fanctions.turn();
                }
                break;

            case STATE_SENSE_WHITE_LINE:
                if(robot.lineSensor.getLightDetected() > 0.4) {
                    fanctions.setDegrees(-90);
                    newState(state.STATE_MOVE_LEFT_MORE);
                }else {
                    robot.leftMotor.setPower(.6);
                    robot.rightMotor.setPower(.6);
                }
                break;

            case STATE_MOVE_LEFT_MORE:
                if (fanctions.doneTurning()) {
                    fanctions.endmove();
                    newState(state.STATE_SENSE_COLOR);
                } else {
                    fanctions.turn();
                }
                break;

            case STATE_SENSE_COLOR:
                if (fanctions.sameCola()) {
                    robot.ramServo.setPosition(1);
                } else {
                    robot.ramServo.setPosition(0);
                }
                newState(state.STATE_WAIT_FOR_RAM);
                break;

            case STATE_WAIT_FOR_RAM:
                if(stateTime.time() >= 1) {
                    newState(state.STATE_PREPARE_TO_GIT_RAMMED_M8_U_WOT_RAMMING_SPEED);
                    fanctions.setPos(6,1);
                }
                break;

            case STATE_PREPARE_TO_GIT_RAMMED_M8_U_WOT_RAMMING_SPEED:
                if(!fanctions.driveMotorsAreBusy()){
                    fanctions.endmove();
                    newState(state.STATE_STAND_BACK);
                }



        }
    }

    @Override
    public void stop() {
    }
    private void newState(state newState) {
        // Reset the state time, and then change to next state.
        stateTime.reset();
        currentState = newState;
    }

}
