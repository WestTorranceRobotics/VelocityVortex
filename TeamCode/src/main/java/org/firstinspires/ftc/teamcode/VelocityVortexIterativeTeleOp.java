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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="Iterative TeleOp", group="WTR")  // @Autonomous(...) is the other common choice
//@Disabled
public class VelocityVortexIterativeTeleOp extends OpMode
{

    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime transportStateTime = new ElapsedTime();

    RobotHardware robot = new RobotHardware();

    private enum transportState {
        STATE_SPOOL_UP,
        STATE_UP,
        STATE_WAIT,
        STATE_STANDBY,
        STATE_STAY
    }



    private transportState currentTransportState;

    @Override
    public void init() {
        robot.initRobotHardware(hardwareMap);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {

    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
        newTransportState(transportState.STATE_STANDBY);
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        robot.tankDrive(-gamepad1.left_stick_y, -gamepad1.right_stick_y);

        //sets the intake motor to 100% TO 0% based on button pressed
        if(gamepad1.right_bumper){
            robot.intakeMotor.setPower(1);
        }else{
            robot.intakeMotor.setPower(0);
        }

        if(gamepad2.right_bumper){
            robot.leftShooterMotor.setPower(1);
            robot.rightShooterMotor.setPower(1);
        }else {
            robot.leftShooterMotor.setPower(0);
            robot.rightShooterMotor.setPower(0);
        }

        transportSwitch();
    }

    @Override
    public void stop() {
    }

    private void newTransportState(transportState newState) {
        // Reset the state time, and then change to next state.
        transportStateTime.reset();
        currentTransportState = newState;
    }

    private void transportSwitch() {
        telemetry.addData("state",currentTransportState);
        switch (currentTransportState){
            case STATE_STANDBY:
                if (gamepad2.a || gamepad2.b) {
                    robot.setShooterSpeed(0.8);
                    newTransportState(transportState.STATE_SPOOL_UP);
                }
                break;

            case STATE_SPOOL_UP:
                if(!gamepad2.a && !gamepad2.b) {
                    robot.setShooterSpeed(0);
                    newTransportState(transportState.STATE_STANDBY);
                } else if (robot.leftShooterMotor.getPower() >= .8 && robot.rightShooterMotor.getPower() >= .8) {
                    robot.setTransportsUp();
                    if(gamepad2.a) {
                        newTransportState(transportState.STATE_UP);
                    } else if(gamepad2.b) {
                        newTransportState(transportState.STATE_STAY);
                    }
                }

            case STATE_UP:
                if (robot.transportsAreUp()) {
                    newTransportState(transportState.STATE_WAIT);
                }
                break;

            case STATE_WAIT:
                if(transportStateTime.time() >= .5) {
                    robot.setTransportsDown();
                    robot.setShooterSpeed(0);
                    newTransportState(transportState.STATE_STANDBY);
                }
                break;

            case STATE_STAY:
                if(!(robot.leftShooterMotor.getPower() >= .8 && robot.rightShooterMotor.getPower() >= .8)) {
                    robot.setTransportsDown();
                    newTransportState(transportState.STATE_SPOOL_UP);
                } else if(!gamepad2.b){
                    robot.setTransportsDown();
                    robot.setShooterSpeed(0);
                    newTransportState(transportState.STATE_STANDBY);
                }
                break;
        }
    }
}



