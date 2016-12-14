
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

    public int initheading = 0;

    public boolean beaconIsRed = false;

    public int turningError = 0;

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
        this.degrees = degrees/* * teamNumber()*/;
        robot.leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        initheading = robot.gyro.getHeading();
    }

    public boolean doneTurning (){
        //returning if the robot are done turning
        return (Math.abs(robot.gyro.getHeading()-initheading) >= Math.abs(degrees));
    }

    public void turn(){
        //setting the motors to turning power
        int multiplier = (degrees/Math.abs(degrees));
        robot.leftMotor.setPower(.5 * multiplier);
        robot.rightMotor.setPower(-.5 * multiplier);
    }


    public void turn(double power){
        //overloading thing
        robot.leftMotor.setPower(power);
        robot.rightMotor.setPower(-power);
    }

    public double PIDTurn(){
        double error = degrees - (robot.gyro.getHeading() - initheading);
        double motorPower = .027 * error;
        return motorPower;
    }

    public boolean sameCola(){
        if(robot.teamSwitch.getState() == beaconIsRed) {
            return true;
        } else {
            return false;
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

    public void setTransportUp(){
        robot.transportServo.setPosition(robot.transport1Up);
    }

    public void setTransportDown(){
        robot.transportServo.setPosition(robot.transport1Down);
    }


    public boolean transportIsDown (){
        return robot.transportServo.getPosition() == robot.transport1Down;
    }


    public boolean transportIsUp (){
        return robot.transportServo.getPosition() == robot.transport1Up;
    }
    public int teamNumber () {
        if (robot.teamSwitch.getState()) {
            return 1;
        } else {
            return -1;
        }
    }
}
