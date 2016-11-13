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

    //Motors
    public DcMotor  leftMotor         = null;
    public DcMotor  rightMotor        = null;
    public DcMotor  intakeMotor       = null;
    public DcMotor  leftShooterMotor  = null;
    public DcMotor  rightShooterMotor = null;
    public DcMotor  capBallMotor      = null;

    //Servos
    public Servo leftStick       = null;
    public Servo rightStick      = null;
    public Servo transportServo1 = null;
    public Servo transportServo2 = null;

    //Sensors
    public LightSensor lineSensor    = null;
    public ColorSensor beaconSenser  = null;
    public GyroSensor gyro           = null;
    public DigitalChannel teamSwitch = null;
    //TODO add the switches for team color and other possible preferences

    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    //Constants for servos
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

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        //Motors
        leftMotor = hwMap.dcMotor.get("leftmotor");
        rightMotor = hwMap.dcMotor.get("rightmotor");
        intakeMotor = hwMap.dcMotor.get("inmotor");
        leftShooterMotor = hwMap.dcMotor.get("LShootmotor");
        rightShooterMotor = hwMap.dcMotor.get("RShootmotor");
        capBallMotor = hwMap.dcMotor.get("capmotor");

        //Servos
        leftStick = hwMap.servo.get("leftStick");
        rightStick = hwMap.servo.get("rightStick");
        transportServo1 = hwMap.servo.get("transervo1");
        transportServo2 = hwMap.servo.get("transervo2");

        //Sensors
        beaconSenser = hwMap.colorSensor.get("sensor");
        gyro = hwMap.gyroSensor.get("gyro");
        teamSwitch = hwMap.digitalChannel.get("teamSwitch");
        lineSensor = hwMap.lightSensor.get("lineSensor");

        //Set the directions for each motor
        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        leftShooterMotor.setDirection(DcMotor.Direction.FORWARD);
        rightShooterMotor.setDirection(DcMotor.Direction.REVERSE);
        capBallMotor.setDirection(DcMotor.Direction.FORWARD);

        //allow shooter motors to coast
        leftShooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightShooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        // Set all motors to zero power
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        intakeMotor.setPower(0);
        leftShooterMotor.setPower(0);
        rightShooterMotor.setPower(0);
        capBallMotor.setPower(0);

        //Set motors to their appropriate modes
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftShooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightShooterMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        capBallMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void initLooporwhateveryouwanttocallit() {
        //TODO make an init for servos and other hardware for use during init_loop
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