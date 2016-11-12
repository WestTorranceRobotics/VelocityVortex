package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LightSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorAdafruitRGB;

/**
 * This is NOT an opmode.
 */
public class RobotHardware {

    /* Public OpMode members. */
    public DcMotor  leftMotor         = null;
    public DcMotor  rightMotor        = null;
    public DcMotor  intakeMotor       = null;
    public DcMotor  leftShooterMotor  = null;
    public DcMotor  rightShooterMotor = null;
    public DcMotor  capBallMotor      = null;

    public Servo leftStick = null;
    public Servo rightStick = null;
    public Servo transportServo1 = null;
    public Servo transportServo2 = null;




    public LightSensor lineSensor = null;
    public ColorSensor beaconSenser = null;
    public GyroSensor gyro = null;
    public DigitalChannel teamSwitch = null;



    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    public double rightStickUp = 1;
    public double rightStickDown = 0;
    public double leftStickUp = 1;
    public double leftStickDown = 0;
    public double transport1Up = 1;
    public double transport1Down = 0;
    public double transport2Up = 1;
    public double transport2Down = 0;


    /* Constructor */
    public RobotHardware(){

    }

    //@Override
    public void runOpMode() throws InterruptedException {

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftMotor   = hwMap.dcMotor.get("leftmotor");
        rightMotor  = hwMap.dcMotor.get("rightmotor");
        intakeMotor = hwMap.dcMotor.get("inmotor");
        leftShooterMotor = hwMap.dcMotor.get("LShootmotor");
        rightShooterMotor = hwMap.dcMotor.get("RShootmotor");
        capBallMotor = hwMap.dcMotor.get("capmotor");

        beaconSenser = hwMap.colorSensor.get("sensor");
        gyro = hwMap.gyroSensor.get("gyro");
        teamSwitch = hwMap.digitalChannel.get("teamSwitch");
        lineSensor = hwMap.lightSensor.get("lineSensor");

        leftStick = hwMap.servo.get("leftStick");
        rightStick = hwMap.servo.get("rightStick");
        transportServo1 = hwMap.servo.get("transervo1");
        transportServo2 = hwMap.servo.get("transervo2");


        // /leftCapBallServo = hwMap.servo.get("leftcapmotor");  If they ever use a left or right capballmotor, then use this.
        //rightCapBallServo = hwMap.servo.get("rightcapmotor");


        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        leftShooterMotor.setDirection(DcMotor.Direction.FORWARD);
        rightShooterMotor.setDirection(DcMotor.Direction.REVERSE);
        capBallMotor.setDirection(DcMotor.Direction.FORWARD);

        leftShooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightShooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Set all motors to zero power
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        intakeMotor.setPower(0);
        leftShooterMotor.setPower(0);
        rightShooterMotor.setPower(0);
        capBallMotor.setPower(0);



        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftShooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightShooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        capBallMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     * @throws InterruptedException
     */
    public void waitForTick(long periodMs) throws InterruptedException {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            Thread.sleep(remaining);

        // Reset the cycle clock for the next pass.
        period.reset();
    }

    public void tankDrive(double left, double right) {

        if (Math.abs(left) < .15) {
            leftMotor.setPower(0);
        } else {
            leftMotor.setPower(left);
        }

        if (Math.abs(right) < .15) {
            rightMotor.setPower(0);
        } else {
            rightMotor.setPower(right);
        }
    }

    public void setShooterSpeed(double speed) {
        leftShooterMotor.setPower(speed);
        rightShooterMotor.setPower(speed);
    }

    public void rightServoUp(){
        rightStick.setPosition(rightStickUp);
    }

    public void rightServoDown() {
        rightStick.setPosition(rightStickDown);
    }

    public void leftServoUp() {
        leftStick.setPosition(leftStickUp);
    }

    public void leftServoDown() {
        leftStick.setPosition(leftStickDown);
    }

    public void setTransport1Up(){
        transportServo1.setPosition(transport1Up);
    }

    public void setTransport1Down(){
        transportServo1.setPosition(transport1Down);
    }

    public void setTransport2Up(){
        transportServo2.setPosition(transport2Up);
    }

    public void setTransport2Down(){
        transportServo2.setPosition(transport2Down);
    }

    public void setSticksUp() {
        rightServoUp();
        leftServoUp();
    }

    public void setSticksDown(){
        rightServoDown();
        leftServoDown();
    }

    public void setTransportsUp(){
        setTransport1Up();
        setTransport2Up();
    }

    public void setTransportsDown(){
        setTransport1Down();
        setTransport2Down();
    }
}

