package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Iterative TeleOp", group="WTR")  // @Autonomous(...) is the other common choice
//@Disabled
public class VelocityVortexIterativeTeleOp extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();

    //RobotHardware robot = new RobotHardware();
   // IterativeFunctions fanctions = new IterativeFunctions(robot);

    public DcMotor  leftMotor              = null;
    public DcMotor  rightMotor             = null;
    public DcMotor  intakeMotor            = null;
    public DcMotor  leftShooterMotor       = null;
    public DcMotor  rightShooterMotor      = null;
    public ServoController servoController = null;
    public Servo transportServo1           = null;

    @Override
    public void init() {
        //robot.initRobotHardware(hardwareMap);
        leftMotor = hardwareMap.dcMotor.get("leftmotor");
        rightMotor = hardwareMap.dcMotor.get("rightmotor");
        intakeMotor = hardwareMap.dcMotor.get("inmotor");
        leftShooterMotor = hardwareMap.dcMotor.get("LShootmotor");
        rightShooterMotor = hardwareMap.dcMotor.get("RShootmotor");

        transportServo1 = hardwareMap.servo.get("transervo1");

        leftMotor.setDirection(DcMotor.Direction.REVERSE);
        rightMotor.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
        rightShooterMotor.setDirection(DcMotor.Direction.REVERSE);
        leftShooterMotor.setDirection(DcMotor.Direction.FORWARD);
        rightShooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftShooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        servoController = hardwareMap.servoController.get("sv1");
    }

    @Override
    public void init_loop() {
        SetServo(.5);
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {

        telemetry.addData("1", transportServo1.getPortNumber());
        //telemetry.addData("2", transportServo2.getPortNumber());

        //Driver2, a -> transport up, if not then down
        if(gamepad2.a){
            SetServo(.275);
        } if(!gamepad2.a) {
            SetServo(0.4);
        }

        /*slow moving code
        if(!transportsAreUp() && gamepad2.a){
            SetServo(currentPos - .001);
        } if(!gamepad2.a) {
            SetServo(0.5);
        }
        */

        //driver1, left stick left motor
        if (Math.abs(gamepad1.left_stick_y) < .15) {
            leftMotor.setPower(0);
        } else {
            leftMotor.setPower(-gamepad1.left_stick_y);
        }

        //driver1, right stick right motor
        if (Math.abs(gamepad1.right_stick_y) < .15) {
            rightMotor.setPower(0);
        } else {
            rightMotor.setPower(-gamepad1.right_stick_y * .5);
        }

        //driver1, right bumper -> intake in, left bumper -> intake out
        if(gamepad1.right_bumper) {
            intakeMotor.setPower(.6);
        } else if(gamepad1.left_bumper) {
            intakeMotor.setPower(-.6);
        } else {
            intakeMotor.setPower(0);
        }

        //if right bumper on driver2 is pressed then set shooters to 70%
        if(gamepad2.right_bumper) {
            rightShooterMotor.setPower(.7);
            leftShooterMotor.setPower(.7);
        } else {
            rightShooterMotor.setPower(0);
            leftShooterMotor.setPower(0);
        }
    }

    @Override
    public void stop() {
    }

    public double transport1Up = 0.25;
    public double transport1Down = 0;

    public void setTransport1Down(){
        transportServo1.setPosition(transport1Down);
    }

    public boolean transportsAreDown (){
        return transportServo1.getPosition() == transport1Down;
    }

    public boolean transportsAreUp (){
        return currentPos <= transport1Up;
    }

    public void SetServo(double pos){
        currentPos = pos;
        transportServo1.setPosition(pos);
    }
    public boolean servoIsUp(){
        return currentPos <= .25;
    }

    public double currentPos = 0;
}



