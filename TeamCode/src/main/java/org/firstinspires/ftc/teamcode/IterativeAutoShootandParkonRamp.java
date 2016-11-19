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

@Autonomous(name="ShootandParkonRamp", group="WTR")  // @Autonomous(...) is the other common choice
//@Disabled
public class IterativeAutoShootandParkonRamp extends OpMode
{
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime statetime = new ElapsedTime();
    RobotHardware robot = new RobotHardware();
    IterativeFunctions Fanctions = new IterativeFunctions();

    private enum state {
        STATE_MOVE_TO_SHOOTING_POSITION,//moving to the shooting position and waiting for drive motors not to be busy
        STATE_WARM_UP_SHOOTER_MOTOR,//waiting for the shooter motors to warm up, and setting the position
        STATE_SHOOT,//shooting
        STATE_TURN_TOWARDS_RAMP,// moving in position to park
        STATE_PARK,//parking
        STATE_UP,
        STATE_WAIT
    }
    private state currentState;
    private void newState(state newState) {
        // Reset the state time, and then change to next state.
        statetime.reset();
        currentState = newState;
    }

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
        Fanctions.setPos(48,.6);
    }

    @Override
    public void loop() {
        switch (currentState) {
            case STATE_WARM_UP_SHOOTER_MOTOR:
                //waiting for the shooter motors to warm up, and setting position for the drive motors.
                if (robot.leftShooterMotor.getPower() >= .8 && robot.rightShooterMotor.getPower() >= .8) {
                    newState(state.STATE_MOVE_TO_SHOOTING_POSITION);
                    Fanctions.setPos(48,.6);
                }
                break;
            case STATE_MOVE_TO_SHOOTING_POSITION:
                //moving to position, while waiting for drive motors not to be busy
                if (!Fanctions.driveMotorsAreBusy()) {
                    newState(state.STATE_UP);
                    robot.setTransportsUp();
                }
                break;
            case STATE_UP:
                if (robot.transportsAreUp()) {
                    newState(state.STATE_WAIT);

                }
                break;
            //Moving transport ramp up to shoot ball.
            case STATE_WAIT:
                if (statetime.time() >= .5) {
                    robot.setTransportsDown();
                    newState(state.STATE_TURN_TOWARDS_RAMP);
                }
                break;
            case STATE_TURN_TOWARDS_RAMP:

            }


    }

    @Override
    public void stop() {
    }



}
