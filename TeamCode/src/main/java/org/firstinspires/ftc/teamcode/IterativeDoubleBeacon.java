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

@Autonomous(name="DoubleBeacon", group="WTR")  // @Autonomous(...) is the other common choice
//@Disabled
public class IterativeDoubleBeacon extends OpMode
{
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime stateTime = new ElapsedTime();
    RobotHardware robot = new RobotHardware();
    IterativeFunctions fanctions = new IterativeFunctions();

    byte[] colorCcache;

    private enum state {
        STATE_MOVE_TO_POSITION_TO_FIND_WHITE_LINE,
        //Move from starting position by turning, then moving toward wall where beacons are, then turning.
        STATE_FIND_FIRST_WHITE_LINE,
        //start going straight and use opticalsensor to find white line, when finding line, turn.
        STATE_SENSE_COLOR,
        //after turning find the color of one side
        STATE_WAIT_TO_PRESS_RIGHT_COLOR,
        //waiting
        STATE_PRESS_RIGHT_COLOR,
        //if the team code is the same as detected side of beacon, press it, if not, press the other side.
        STATE_FIND_SECOND_WHITE_LINE,
        //go straight and find the second white line, then turn.
        STATE_SENSE_COLOR_TWO,
        //after turning find the color of one side.
        STATE_WAIT_TO_PRESS_RIGHT_COLOR_TWO,
        //waiting again
        STATE_PRESS_RIGHT_COLOR_TWO
        //if the team code is the same as detected side of beacon, press it, if not, press the other side.
    }

    state currentstate;

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
    }

    @Override
    public void loop() {
        colorCcache = robot.beaconSensorReader.read(0x04, 1);

        switch (currentstate){

            case STATE_MOVE_TO_POSITION_TO_FIND_WHITE_LINE:
                fanctions.setDegrees(45);
                fanctions.doneTurning();
                fanctions.setPos(72, .6);
                //newstate(state.STATE_FIND_FIRST_WHITE_LINE)
                fanctions.setDegrees(90);
                fanctions.endmove();
                break;
            case STATE_FIND_FIRST_WHITE_LINE:
                fanctions.doneTurning();
                if(robot.lineSensor.getLightDetected() > 0.4) {
                    fanctions.setDegrees(-90);
                    fanctions.doneTurning();
                    //newstate(state.STATE_SENSE_COLOR)
                    fanctions.endmove();
                }
                break;
            case STATE_SENSE_COLOR:
                if(fanctions.sameCola(colorCcache[0] & 0xFF)) {
                    robot.setRamServoRight();
                } else {
                    robot.setRamServoLeft();
                }
                //newstate(state.STATE_WAIT_TO_PRESS_RIGHT_COLOR)
                break;
            case STATE_WAIT_TO_PRESS_RIGHT_COLOR:
                if(stateTime.time() >= 1) {
                    //newState(state.STATE_PRESS_RIGHT_COLOR);
                    fanctions.setPos(6,1);
                }
                break;
            case STATE_PRESS_RIGHT_COLOR:
                if(!fanctions.driveMotorsAreBusy()){
                    fanctions.endmove();
                    //newstate(state.STATE_FIND_SECOND_WHITE_LINE)
                }
                break;
            case STATE_FIND_SECOND_WHITE_LINE:
                if(robot.lineSensor.getLightDetected() > 0.4) {
                    fanctions.setDegrees(-90);
                    fanctions.doneTurning();
                    //newstate(state.STATE_SENSE_COLOR_TWO)
                    fanctions.endmove();
                }
                break;
            case STATE_SENSE_COLOR_TWO:
                if(fanctions.sameCola(colorCcache[0] & 0xFF)) {
                    robot.setRamServoRight();
                } else {
                    robot.setRamServoLeft();
                }
                //newstate(state.STATE_WAIT_TO_PRESS_RIGHT_COLOR_TWO)
                break;
            case STATE_WAIT_TO_PRESS_RIGHT_COLOR_TWO:
                if(stateTime.time() >= 1) {
                    //newState(state.STATE_PRESS_RIGHT_COLOR_TWO);
                    fanctions.setPos(6,1);
                }
                break;
            case STATE_PRESS_RIGHT_COLOR_TWO:
                if(!fanctions.driveMotorsAreBusy()){
                    fanctions.endmove();
                }
                break;



        }

    }

    @Override
    public void stop() {
    }

}
