package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.RobotHardware;

public class IterativeFunctions {

    RobotHardware robot;

    public IterativeFunctions(RobotHardware hardware){
        robot = hardware;
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
        robot.rightMotor.setPower(power * .4);

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

    public boolean doneDriving() {
        return (!robot.leftMotor.isBusy() && !robot.rightMotor.isBusy());
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
        //TODO this function still does not yeild a perfect 90 degree turn which should be attainable to a certain degree. use telemetry to see how much the robot is actually turning
        //TODO if the robot doesnt know its isnt turning 90 degrees, good  luck fixing it. my tips below could be the answer in either situation, but dont spend too much time on it if you dont understand it
        double error = degrees - (robot.gyro.getHeading() - initheading);
        double motorPower = .027 * error;
        return motorPower;
        //TODO I would recommend adding a derivative term to this feedback loop, which may entail increasing the proportional gain
        //TODO if you are confused by the derivative term, forget that word, its just a stupid calc term. all derivative means is rate of change
        //TODO which means all you have to do is count the time between loops and see how many degress you go, divide and you have your rate in degrees/second
        //TODO what may be easier is using the raw data from the gyro as your derivative, because the gyro integrates on the chip, which is why you get a heading and dont have to do all that stupid time math
        //TODO this unitegrated rate may be robot.gyro.rawZ();, but i am unsure and a quick google search did not turn up any answer. remmeber to watch that video again if youre lost or havent seen it in the first place
        //TODO heres the link https://www.youtube.com/watch?v=4Y7zG48uHRo
        //TODO i also found this video which demonstrates PID on a real system well    https://www.youtube.com/watch?v=fusr9eTceEo
    }

    //TODO if you use PIDTurn, you need an exit condition. ill make an example of how it could work below
    public boolean PIDWithinTolerance() {
        double error = degrees - (robot.gyro.getHeading() - initheading);
        if (error <= 2){
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
        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //TODO I would try to use this if I were you guys, just another step to take out, but you may have to tweak this function a little, use it for servos and stuff
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