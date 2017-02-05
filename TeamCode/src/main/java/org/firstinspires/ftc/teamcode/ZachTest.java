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

@Autonomous(name="Zach", group="WTR")  // @Autonomous(...) is the other common choice
//@Disabled
public class ZachTest extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime statetime = new ElapsedTime();
    RobotHardware robot = new RobotHardware();
    IterativeFunctions fanctions = new IterativeFunctions(robot, telemetry);

    private enum state{
        STATE_LEFT,
        STATE_RIGHT,
        STATE_END
    }

    public state currentState = null;

    @Override
    public void init() {
        robot.initRobotHardware(hardwareMap);
        telemetry.addData("CALIBRATION", "GYRO CALIBRATION IN PROGRESS...");
        robot.gyro.calibrate();
    }

    @Override
    public void init_loop() {
        fanctions.anushalizeRobotHardware();
        if(!robot.gyro.isCalibrating()) {
            telemetry.addData("CALIBRATION", "CALIBRATION COMPLETE!");
        }
        telemetry.addData("gyro", robot.gyro.getIntegratedZValue());
    }

    @Override
    public void start() {
        telemetry.clear();
        runtime.reset();
        newState(state.STATE_LEFT);
        fanctions.setDegrees(45);
    }

    @Override
    public void loop() {

        telemetry.addData("State", currentState);

        switch(currentState) {

            case STATE_LEFT:

                if(fanctions.PIDWithinTolerance()) {
                    fanctions.endTurn();
                    fanctions.setDegrees(-90);
                    newState(state.STATE_RIGHT);
                } else {
                    fanctions.turn(fanctions.PIDTurn());
                }

                break;

            case STATE_END:
                break;
        }

    }

    @Override
    public void stop() {
    }

    private void newState(state newState) {
        currentState = newState;
        statetime.reset();
    }

}
