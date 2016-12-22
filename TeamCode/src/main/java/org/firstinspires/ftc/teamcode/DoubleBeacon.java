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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="DoubleBeacon", group="WTR")  // @Autonomous(...) is the other common choice
//@Disabled
public class DoubleBeacon extends OpMode
{
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime stateTime = new ElapsedTime();
    RobotHardware robot = new RobotHardware();
    IterativeFunctions fanctions = new IterativeFunctions(robot);

    boolean isPressed = false;
    boolean isRed = true;
    boolean beaconIsRed = false;

    private enum state {
        STATE_MOVE_A_BIT,
        STATE_TIMER,
        STATE_TURN,
        STATE_MOVE_A_LOT,
        STATE_TIMER_TWO,
        STATE_TURN_AGAIN,
        STATE_FIND_WHITE_LINE,
        STATE_TURN_ON_WHITE_LINE,
        STATE_CHOOSE_COLOR,
        STATE_RAM,
        STATE_LAST_TIMER_LEL_HEHE_XD,
        STATE_BACK_UP_BOI,
        STATE_TIMER_OPOP,
        STATE_TURN_LEL,
        STATE_FIND_WHITE_LINE_TWO,
        STATE_TURN_ON_WHITE_LINE_TWO,
        STATE_CHOOSE_COLOR_TWO,
        STATE_RAM_TWO,
        STATE_I_PROMISE_THIS_IS_THE_LAST_TIMER_INSERT_LENNY_HEHE_XD,

    }
    private state currentState = null;



    @Override
    public void init() {
        robot.initRobotHardware(hardwareMap);
    }

    @Override
    public void init_loop() {
    robot.ramServo.setPosition(0);
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {

        telemetry.addData("state", currentState);
        telemetry.addData("Len", robot.leftMotor.getCurrentPosition());
        telemetry.addData("Ren", robot.rightMotor.getCurrentPosition());

        if(robot.beaconSensor.blue()>robot.beaconSensor.red()) {
            beaconIsRed = false;
        } else {
            beaconIsRed = true;
        }

        if(gamepad1.a && !isPressed){
            isRed = !isRed; isPressed = true;
        } else if(!gamepad1.a){
            isPressed = false;
        }

        switch (currentState) {

            case STATE_MOVE_A_BIT:
                fanctions.setPos(-10, .9);
                newState(state.STATE_TIMER);
                break;

            case STATE_TIMER:
                if (stateTime.time() >= 1){
                    robot.leftMotor.setPower(0);
                    robot.rightMotor.setPower(0);
                    fanctions.setDegrees(-135);
                    newState(state.STATE_TURN);
                }
                break;

            case STATE_TURN:
                if (fanctions.PIDWithinTolerance()) {
                    fanctions.endmove();
                    newState(state.STATE_MOVE_A_LOT);
                }else{
                    fanctions.turn(fanctions.PIDTurn());
                }
                break;

            case STATE_MOVE_A_LOT:
                fanctions.setPos(60, .9);
                newState(state.STATE_TIMER_TWO);
                break;

            case STATE_TIMER_TWO:
                if (stateTime.time() >= 3.5){
                    robot.leftMotor.setPower(0);
                    robot.rightMotor.setPower(0);
                    fanctions.setDegrees(-45);
                    newState(state.STATE_TURN_AGAIN);
                }
                break;

            case STATE_TURN_AGAIN:
                if (fanctions.PIDWithinTolerance()){
                    fanctions.endmove();
                    newState(state.STATE_FIND_WHITE_LINE);
                }else{
                    fanctions.turn(fanctions.PIDTurn());
                }
                break;

            case STATE_FIND_WHITE_LINE:
                if (robot.lineSensor.getLightDetected() >= .35){
                    fanctions.setDegrees(90);
                    newState(state.STATE_TURN_ON_WHITE_LINE);
                }
                break;

            case STATE_TURN_ON_WHITE_LINE:
                if (fanctions.PIDWithinTolerance()){
                    fanctions.endmove();
                    newState(state.STATE_CHOOSE_COLOR);
                }else{
                    fanctions.turn(fanctions.PIDTurn());
                }
                break;

            case STATE_CHOOSE_COLOR:
                if (sameCola()) {
                    robot.ramServo.setPosition(1);
                    newState(state.STATE_RAM);
                } else {
                    robot.ramServo.setPosition(0);
                    newState(state.STATE_RAM);
                }
                break;

            case STATE_RAM:
                fanctions.setPos(6, 1);
                newState(state.STATE_LAST_TIMER_LEL_HEHE_XD);
                break;

            case STATE_LAST_TIMER_LEL_HEHE_XD:
                if (stateTime.time() >= 1){
                    robot.leftMotor.setPower(0);
                    robot.rightMotor.setPower(0);
                    newState(state.STATE_BACK_UP_BOI);
                }
                break;

            case STATE_BACK_UP_BOI:
                fanctions.setPos(-10, .9);
                newState(state.STATE_TIMER_OPOP);
                break;

            case STATE_TIMER_OPOP:
                if (stateTime.time() >= .75){
                    robot.leftMotor.setPower(0);
                    robot.rightMotor.setPower(0);
                    fanctions.setDegrees(-90);
                    newState(state.STATE_TURN_LEL);
                }
                break;

            case STATE_TURN_LEL:
                if (fanctions.PIDWithinTolerance()){
                    fanctions.endmove();
                    newState(state.STATE_FIND_WHITE_LINE_TWO);
                }else{
                    fanctions.turn(fanctions.PIDTurn());
                }
                break;

            case STATE_FIND_WHITE_LINE_TWO:
                if (robot.lineSensor.getLightDetected() >= .35){
                    fanctions.setDegrees(90);
                    newState(state.STATE_TURN_ON_WHITE_LINE_TWO);
                }
                break;

            case STATE_TURN_ON_WHITE_LINE_TWO:
                if (fanctions.PIDWithinTolerance()){
                    fanctions.endmove();
                    newState(state.STATE_CHOOSE_COLOR_TWO);
                }else{
                    fanctions.turn(fanctions.PIDTurn());
                }
                break;

            case STATE_CHOOSE_COLOR_TWO:
                if (sameCola()) {
                    robot.ramServo.setPosition(1);
                    newState(state.STATE_RAM_TWO);
                } else {
                    robot.ramServo.setPosition(0);
                    newState(state.STATE_RAM_TWO);
                }
                break;

            case STATE_RAM_TWO:
                fanctions.setPos(6, 1);
                newState(state.STATE_I_PROMISE_THIS_IS_THE_LAST_TIMER_INSERT_LENNY_HEHE_XD);
                break;

            case STATE_I_PROMISE_THIS_IS_THE_LAST_TIMER_INSERT_LENNY_HEHE_XD:
                if (stateTime.time() >= 1){
                    robot.leftMotor.setPower(0);
                    robot.rightMotor.setPower(0);
                }
        }
    }

    @Override
    public void stop() {
    }

    private void newState(state newState) {
        currentState = newState;
        stateTime.reset();
    }

    public boolean sameCola(){
        if(beaconIsRed && isRed) {
            return true;
        } else if (!beaconIsRed && !isRed) {
            return true;
        } else {
            return false;
        }
    }
}
