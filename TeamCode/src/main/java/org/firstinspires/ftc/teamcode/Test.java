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

@Autonomous(name="test", group="WTR")  // @Autonomous(...) is the other common choice
//@Disabled
public class Test extends OpMode
{
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime statetime = new ElapsedTime();
    RobotHardware robot = new RobotHardware();
    IterativeFunctions fanctions = new IterativeFunctions(robot);

    private enum state {
        STATE_MOVE,
        STATE_END
    }

    state currrentState;

    @Override
    public void init() {
        robot.initRobotHardware(hardwareMap);
        //TODO add telemetry that tells the driver the gyro is not yet calibrated, something like telemetry.addData("GYRO", "CALIBRATING: DO NOT START!");
        robot.gyro.calibrate();
        //TODO add telemetry here that will update the last telemetry to let the driver know the gyro is done calibrating, something like telemetry.addData("GYRO", "CALIBRATION COMPLETE!");
    }

    @Override
    public void init_loop() {
        fanctions.setTransportDown();

    }

    @Override
    public void start() {
        runtime.reset();
        newState(state.STATE_MOVE);
        fanctions.setPos(24,.6);
    }

    @Override
    public void loop() {

        telemetry.addData("state", currrentState);
        telemetry.addData("left encoder", robot.leftMotor.getCurrentPosition());
        telemetry.addData("right encoder", robot.rightMotor.getCurrentPosition());
        telemetry.addData("degrees", (Math.abs(robot.gyro.getHeading()-fanctions.initheading)));
        //TODO add a shit ton of telemetry for when you test, anything you can think of that might be usefull, degrees, ticks, inches etc. it helps a lot

        switch (currrentState) {
            case STATE_MOVE:
                if (fanctions.doneDriving()) {
                    fanctions.endmove();
                    newState(state.STATE_END);
                }
                break;
            case STATE_END:
                break;
            //TODO test encoders, if that works you are set, start working on a real auto
        }
    }

    @Override
    public void stop() {
    }

    private void newState(state newState) {
        currrentState = newState;
        statetime.reset();
    }

}
