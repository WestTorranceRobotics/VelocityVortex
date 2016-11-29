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
@Disabled
public class ShootandParkinCenterFromCenter extends OpMode
{
    private ElapsedTime runTime = new ElapsedTime();
    private ElapsedTime stateTime = new ElapsedTime();
    RobotHardware robot = new RobotHardware();
    IterativeFunctions fanctions = new IterativeFunctions(robot);

    private enum state{
        STATE_WARM_UP_SHOOTER_MOTOR,
        STATE_TURN_TO_SHOOTING_POSITION,
        STATE_MOVE_TO_SHOOTING_POSITION,
        STATE_UP,
        STATE_SHOOT,
        STATE_WAIT,
        STATE_TURN_OFF_SHOOTERS,
        STATE_MOVE_FORWARDS,
        STATE_TURN_PARALLEL_TO_CENTER_DIVIDE,
        STATE_PARK_ON_CENTER,
        STATE_END,
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
        runTime.reset();
    }

    @Override
    public void loop() {

        telemetry.addData("state", currentState);

        switch (currentState) {

            case STATE_WARM_UP_SHOOTER_MOTOR:
                //Waits for the shooter motors to warm up and turns towards the shooting position.
                if (robot.leftShooterMotor.getPower() >= .8 && robot.rightShooterMotor.getPower() >= .8) {
                    fanctions.setDegrees(-22);
                    newState(state.STATE_TURN_TO_SHOOTING_POSITION);
                }
                break;

            case STATE_TURN_TO_SHOOTING_POSITION:
                //Waits for robot to turn towards the shooting position and then moves towards the shooting position.
                if (fanctions.doneTurning()) {
                    fanctions.endmove();
                    fanctions.setPos(50, .6);
                    newState(state.STATE_MOVE_TO_SHOOTING_POSITION);
                } else {
                    fanctions.turn();
                }
                break;

            case STATE_MOVE_TO_SHOOTING_POSITION:
                //Waits for the robot to move into the shooting position and sets transport ramp up to shooot balls.
                if (!fanctions.driveMotorsAreBusy()){
                    robot.setTransportsUp();
                    newState(state.STATE_UP);
                }
                break;

            case STATE_UP:
                //Waits for transport ramp to be set up and waits for balls to pass by.
                if (robot.transportsAreUp()) {
                newState(state.STATE_WAIT);
                }
                break;


            case STATE_WAIT:
                //Waits for .5 seconds before setting the transport ramp back down.
                if (stateTime.time() >= .5) {
                    robot.setTransportsDown();
                    newState(state.STATE_TURN_OFF_SHOOTERS);
                }
                break;

            case STATE_TURN_OFF_SHOOTERS:
                //Turns off shooter motors and moves forwards.
                robot.setShooterSpeed(0);
                fanctions.setPos(32,.6);
                newState(state.STATE_MOVE_FORWARDS);
                break;

            case STATE_MOVE_FORWARDS:
                //Waits until the robot is in position and turns -157 degrees to be parallel with the center divide.
                if (!fanctions.driveMotorsAreBusy()) {
                    fanctions.setDegrees(-157);
                    newState(state.STATE_TURN_PARALLEL_TO_CENTER_DIVIDE);
                }
                break;

            case STATE_TURN_PARALLEL_TO_CENTER_DIVIDE:
                //Waits until the robot is in position before moving backwards to park on the center.
                if (fanctions.doneTurning()){
                    fanctions.endmove();
                    fanctions.setPos(-26,.6);
                    newState(state.STATE_PARK_ON_CENTER);
                } else {
                    fanctions.turn();
                }
                break;

            case STATE_PARK_ON_CENTER:
                //Checks to make sure that the robot is parked.
                if (!fanctions.driveMotorsAreBusy()) {
                    newState(state.STATE_END);
                }
                break;

            case STATE_END:
                //Directs the robot to do nothing.
                fanctions.endmove();
                break;
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
