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
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class IterativeFunctions {

    RobotHardware robot;

    public IterativeFunctions(RobotHardware hardware){
        robot = hardware;
    }

    public int degrees = 0;

    public void setPos(double inches, double goes) {
        robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        int ticks = (int) (inches * (robot.inchToTickConversion));
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

    public boolean driveMotorsAreBusy() {
        return (robot.leftMotor.isBusy() && robot.rightMotor.isBusy());
    }

    public void setDegrees(int degrees){
        this.degrees = degrees * colorNumber();
        robot.leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public boolean doneTurning (){
        //returmimg if the robot are done turning
        int intheading = robot.gyro.getHeading();
        return (Math.abs(robot.gyro.getHeading()-intheading) >= Math.abs(degrees));
    }

    public void turn(){
        //setting the motors to turning power
        int multiplier = (degrees/Math.abs(degrees));
        robot.leftMotor.setPower(.6 * multiplier);
        robot.rightMotor.setPower(-.6 * multiplier);
    }

    public void turn(double power){
        //overloading thing
        int multiplier = (degrees/Math.abs(degrees));
        robot.leftMotor.setPower(power * multiplier);
        robot.rightMotor.setPower(-power * multiplier);
    }

    public boolean sameCola(int colorNumber){
        if(colorNumber == 10 && !robot.teamSwitch.getState()) {
            return true;
        } else if (colorNumber == 3 && robot.teamSwitch.getState()) {
            return true;
        } else {
            return false;
        }
    }

    public int colorNumber() {
        if(robot.teamSwitch.getState()){
            return -1;
        } else {
            return 1;
        }
    }

    public void tankDrive(double left, double right) {
        if (Math.abs(left) < .15) {
            robot.leftMotor.setPower(0);
        } else {
            robot.leftMotor.setPower(left);
        }

        if (Math.abs(right) < .15) {
            robot.rightMotor.setPower(0);
        } else {
            robot.rightMotor.setPower(right);
        }
    }

    public void anushalizeRobotHardware() {
        //robot.ramServo.setPosition(robot.ramServoMiddle);
        robot.transportServo.setPosition(robot.transport1Down);
        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void changeDriveTrainMode(DcMotor.RunMode mode){
        robot.leftMotor.setMode(mode);
        robot.rightMotor.setMode(mode);
    }

    public void setShooterSpeed(double speed) {
        robot.leftShooterMotor.setPower(speed);
        robot.rightShooterMotor.setPower(speed);
    }

    public void setRamServoRight() { robot.ramServo.setPosition(robot.ramServoRight);}

    public void setRamServoLeft() { robot.ramServo.setPosition(robot.ramServoLeft);}

    public void setTransport1Up(){
        robot.transportServo.setPosition(robot.transport1Up);
    }

    public void setTransport1Down(){
        robot.transportServo.setPosition(robot.transport1Down);
    }


    public boolean transportIsDown (){
        return robot.transportServo.getPosition() == robot.transport1Down;
    }

    public boolean transportIsUp (){
        return robot.transportServo.getPosition() == robot.transport1Up;
    }

}
