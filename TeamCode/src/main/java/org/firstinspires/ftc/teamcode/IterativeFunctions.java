package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotHardware;

public class IterativeFunctions {

    RobotHardware robot;
    Telemetry robotTelemetry;

    public IterativeFunctions(RobotHardware hardware){
        robot = hardware;
    }

    public IterativeFunctions(RobotHardware hardware, Telemetry tele) {
        robot = hardware;
        robotTelemetry = tele;
    }

    public int degrees = 0;

    public int leftTarget = 0;

    public int rightTarget = 0;

    public int initheading = 0;

    public int initEncoder = 0;

    public boolean beaconIsRed = false;

    public double lastTime = 0;

    public int lastTick = 0;

    public int turningError = 0;

    public void setPos(double inches, double power) {
        robot.leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        int ticks = (int) (inches * (robot.inchToTickConversion));
        int currentleft = robot.leftMotor.getCurrentPosition();
        int currentright = robot.rightMotor.getCurrentPosition();

        robot.leftMotor.setTargetPosition(ticks + currentleft);
        robot.rightMotor.setTargetPosition(ticks + currentright);
        robot.leftMotor.setPower(power);
        robot.rightMotor.setPower(power);

    }

    public void setPIDPos (double inches) {
        robot.leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        int ticks = (int) (inches * (robot.inchToTickConversion));
        int currentleft = robot.leftMotor.getCurrentPosition();
        int currentright = robot.rightMotor.getCurrentPosition();

        leftTarget = (ticks + currentleft);
        rightTarget = (ticks + currentright);
    }

    public double leftPIDPower (double inch, double timer) {
        int error = (leftTarget - (robot.leftMotor.getCurrentPosition() - initEncoder));
        double traveledTicks = robot.leftMotor.getCurrentPosition() - lastTick;
        double traveledTime = timer - lastTime;
        double deriv = (traveledTicks/traveledTime);
        lastTime = timer;
        lastTick = robot.leftMotor.getCurrentPosition();
        return (.0028571 * error) - (0 * deriv);

    }

    public double rightPIDPower (double inch, double timer) {
        int error = (rightTarget - (robot.rightMotor.getCurrentPosition() - initEncoder));
        double traveledTicks = robot.rightMotor.getCurrentPosition() - lastTick;
        double traveledTime = timer - lastTime;
        double deriv = (traveledTicks/traveledTime);
        lastTime = timer;
        lastTick = robot.rightMotor.getCurrentPosition();
        return (.0028571 * error) - (0 * deriv);

    }

    public void endmove() {
        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);
    }

    public void endTurn() {
        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);
        degrees = 0;
        initheading = 0;
    }

    public boolean doneDriving() {
        return (!robot.leftMotor.isBusy() && !robot.rightMotor.isBusy());
    }

    public void setDegrees(int degrees){
        this.degrees = degrees;
        robot.leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        initheading = robot.gyro.getIntegratedZValue();
    }

    public boolean doneTurning (){
        //returning if the robot are done turning
        return (Math.abs(robot.gyro.getIntegratedZValue() - initheading) >= Math.abs(degrees));
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
        double error = degrees - (robot.gyro.getIntegratedZValue() - initheading);
        double motorPower = -.0077 * error;
        return motorPower;
    }

    //TODO if you use PIDTurn, you need an exit condition. ill make an example of how it could work below
    public boolean PIDWithinTolerance() {
        double error = degrees - (robot.gyro.getIntegratedZValue() - initheading);
        robotTelemetry.addData("error", error);
        robotTelemetry.addData("init heading", initheading);
        robotTelemetry.addData("target", degrees);
        if (error <= 1){
            return true;
        } else {
            return false;
        }
    }

    //TODO use the encoders on the shooter motors to make sure they go at the same rate and so that it wont go too fast at full battery



    /*public boolean sameCola(){
        if(robot.teamSwitch.getState() == beaconIsRed) {
            return true;
        } else {
            return false;

        }
    }*/

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
        robot.leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        setRamServoLeft();
        degrees = 0;
        initheading = 0;
        initEncoder = 0;
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
    /*public int teamNumber () {
        if (robot.teamSwitch.getState()) {
            return 1;
        } else {
            return -1;
        }
    }*/
}