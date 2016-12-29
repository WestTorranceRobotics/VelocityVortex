package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="TeleOp", group="WTR")
//@Disabled
public class VelocityVortexIterativeTeleOp extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime stateTime = new ElapsedTime();

    //RobotHardware robot = new RobotHardware();
   // IterativeFunctions fanctions = new IterativeFunctions(robot);

    public DcMotor  leftMotor              = null;
    public DcMotor  rightMotor             = null;
    public DcMotor  intakeMotor            = null;
    public DcMotor  leftShooterMotor       = null;
    public DcMotor  rightShooterMotor      = null;
    public ServoController servoController = null;
    public Servo transportServo1           = null;
    public Servo ramServo                  = null;

    private boolean ramIsLeft = true;
    private boolean leftBumperPressed = false;

    private int tunaCounter = 0;

    private enum state {
        STATE_SPOOL_UP_SHOOTERS,
        STATE_LOAD_BALL,
        STATE_WAIT_FOR_SHOOTERS,
        STATE_STANDBY,
        STATE_WAIT_FOR_BIG_SHOOT,
        STATE_WAIT_AGAIN,
    }

    private boolean autoIsRunning = false;

    private state currentState = state.STATE_STANDBY;

    @Override
    public void init() {
        //robot.initRobotHardware(hardwareMap);
        leftMotor = hardwareMap.dcMotor.get("leftmotor");
        rightMotor = hardwareMap.dcMotor.get("rightmotor");
        intakeMotor = hardwareMap.dcMotor.get("inmotor");
        leftShooterMotor = hardwareMap.dcMotor.get("LShootmotor");
        rightShooterMotor = hardwareMap.dcMotor.get("RShootmotor");

        transportServo1 = hardwareMap.servo.get("transervo1");
        ramServo = hardwareMap.servo.get("ramservo");

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
        SetServo(.4);
        ramServo.setPosition(0.15);
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
/*
        //Driver2, a -> transport up, if not then down
        if(gamepad1.a){
            SetServo(.275);
        } if(!gamepad1.a) {
            SetServo(0.4);
        }
*/
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
            rightMotor.setPower(-gamepad1.right_stick_y);
        }

        //driver1, right bumper -> intake in, left bumper -> intake out
        if(gamepad1.right_bumper) {
            intakeMotor.setPower(1);
        } else {
            intakeMotor.setPower(0);
        }
<<<<<<< Updated upstream
        if (autoIsRunning){
            if (gamepad1.left_bumper){
                newState(state.STATE_STANDBY);
=======


        if (ifAutoIsRunning && !leftBumperPressed){
            if (gamepad1.left_bumper){
                newShooterState(shooterState.STATE_STANDBY);
                leftBumperPressed = true;
>>>>>>> Stashed changes
            }
        } else if (!gamepad1.left_bumper) {
            leftBumperPressed = false;
        }


/*
        //if right bumper on driver2 is pressed then set shooters to 70%
        if(gamepad1.left_bumper) {
            rightShooterMotor.setPower(.7);
            leftShooterMotor.setPower(.7);
        } else {
            rightShooterMotor.setPower(0);
            leftShooterMotor.setPower(0);
        }
*/
        if(gamepad1.left_trigger > .2) {
            ramIsLeft = true;
        } else if (gamepad1.right_trigger > .2) {
            ramIsLeft = false;
        }

        if(ramIsLeft) {
            ramServo.setPosition(.15);
        } else {
            ramServo.setPosition(1);
        }

        switch (currentState) {

            case STATE_SPOOL_UP_SHOOTERS:
                leftShooterMotor.setPower(.8);
                rightShooterMotor.setPower(.8);
                transportServo1.setPosition(.4);
                autoIsRunning = true;
                newState(state.STATE_WAIT_FOR_SHOOTERS);
                break;

            case STATE_WAIT_FOR_SHOOTERS:
                autoIsRunning = true;
                if (stateTime.time() >= 1.5) {
                    newState(state.STATE_LOAD_BALL);
                }
                break;

            case STATE_LOAD_BALL:
                autoIsRunning = true;
                transportServo1.setPosition(.275);
                tunaCounter++;
                newState(state.STATE_WAIT_FOR_BIG_SHOOT);
                break;

            case STATE_WAIT_FOR_BIG_SHOOT:
                autoIsRunning = true;
                if(tunaCounter > 3) {
                    newState(state.STATE_STANDBY);
                } else if (stateTime.time() >= 1.5) {
                    transportServo1.setPosition(0.4);
                    newState(state.STATE_WAIT_AGAIN);
                }
                break;

            case STATE_WAIT_AGAIN:
                if (stateTime.time() >= .25){
                    newState(state.STATE_LOAD_BALL);
                }
                break;

            case STATE_STANDBY:
                if (gamepad1.left_bumper) {
                    newState(state.STATE_SPOOL_UP_SHOOTERS);
                }
                leftShooterMotor.setPower(0);
                rightShooterMotor.setPower(0);
                transportServo1.setPosition(.4);
                tunaCounter = 0;
                autoIsRunning = false;
                break;
        }
    }

    @Override
    public void stop() {
    }

    public void newState(state newState) {
        currentState = newState;
        stateTime.reset();
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