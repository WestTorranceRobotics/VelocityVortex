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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


@Autonomous(name="AutoTemplate", group="WTR")  // @Autonomous(...) is the other common choice
// @Disabled
public class ShootandSingleBeacon extends LinearOpMode {

    RobotHardware robot = new RobotHardware();


    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);

        waitForStart();
        runtime.reset();

        robot.leftStick.setPosition(0);
        robot.rightStick.setPosition(0);

        robot.setShooterSpeed(.8);
        runtoposition(48, .6);

        while (robot.leftShooterMotor.getPower() < .8 && robot.rightShooterMotor.getPower() < .8) {

        }

        transportBall();

        wait(1);

        robot.setShooterSpeed(0);

        turn(-45);

        runtoposition(42, .6);

        turn(45);

        while(notWhite()) {
            robot.leftMotor.setPower(.4);
            robot.rightMotor.setPower(.4);
        }

        turn(-90);

        if(!sameColor()) {
            robot.rightStick.setPosition(-1);
        } else {
            robot.leftStick.setPosition(-1);
            }
        //make function to go forward to press button.
    }

    public void setPos(double inches, double goes) {

        robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        int ticks = (int)(inches*(1/4*3.14159265359)*(16/24)*(1120));
        int currentleft = robot.leftMotor.getCurrentPosition();
        int currentright = robot.rightMotor.getCurrentPosition();

        robot.leftMotor.setTargetPosition(ticks+ currentleft);
        robot.rightMotor.setTargetPosition(ticks+ currentright);
        robot.leftMotor.setPower(goes);
        robot.rightMotor.setPower(goes);
    }
    public void endmove(){
        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);
    }
    public void runtoposition(double inches, double speed) {
        setPos(inches, speed);
        while (robot.leftMotor.isBusy() && robot.rightMotor.isBusy() && opModeIsActive()) {

        }
        endmove();
    }

    public void turn(int degrees) {
        int intheading = robot.gyro.getHeading();

        int multiplier = (degrees/Math.abs(degrees));

        robot.leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while (Math.abs(robot.gyro.getHeading()-intheading) <= Math.abs(degrees)) {

            robot.leftMotor.setPower(multiplier * .6);
            robot.rightMotor.setPower(-.6 * multiplier);

        }
        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);
    }

    public void transportBall(){
        //make this function later
    }

    public boolean notWhite() {
        //make this function
        return robot.lineSensor.getRawLightDetected() < 100;
        //less is floor, more is the line

        }



    public void wait(double time){
        double initialTime = runtime.time();
        while (runtime.time()- initialTime <time){

        }

    }


    public boolean isRed() {
        return robot.beaconSenser.red()>robot.beaconSenser.blue();
    }

    public boolean teamColorIsRed() {
        return robot.teamSwitch.getState();
    }

    public boolean sameColor() {
        return isRed() == teamColorIsRed();
    }
    //Todo: make a function to move the robot and push the button
    }
