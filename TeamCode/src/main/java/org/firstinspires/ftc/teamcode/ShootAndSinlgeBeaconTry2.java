package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="ShootAndSinlgeBeaconTry2", group="WTR")  // @Autonomous(...) is the other common choice
//@Disabled 
public class ShootAndSinlgeBeaconTry2 extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime stateTime = new ElapsedTime();
    //RobotHardware robot = new RobotHardware();
    //IterativeFunctions fanctions = new IterativeFunctions(robot);

    private enum state {
        STATE_SPECIAL_SNOWFLAKE,
        STATE_SPOOL_UP_SHOOTERS,
        //Rev these shooters.
        STATE_DRIVE_TO_VORTEX,
        //Driving to vortex.
        STATE_WAIT_FOR_SHOOTERS,
        //Waiting for shooters to get power.
        STATE_UP,
        //Moving transport ramp up to shoot ball.
        STATE_WAIT,
        //Waiting half a second to move transport ramp back down.
        STATE_WAIT_MORE,
        STATE_UP2,
        //Moving transport ramp up to shoot ball.
        STATE_WAIT2,
        //Waiting half a second to move transport ramp back down.
        STATE_SHUT_OFF_SHOOTERS,
        //Turning off shooter motors.
        STATE_PARK_IN_CENTER,
        STATE_STOP_MOVING,
        STATE_WAIT_THRID,
        STATE_WAIT_FOR_SHOOT,
        STATE_WAIT_FOURTH,
        STATE_TURN,
        STATE_MOVE_A_LOT,
        STATE_TIMER_TWO,
        STATE_TURN_AGAIN,
        STATE_FIND_WHITE_LINE,
        STATE_TURN_ON_WHITE_LINE,
        STATE_CHOOSE_COLOR,
        STATE_RAM,
        STATE_LAST_TIMER_LEL_HEHE_XD,
        STATE_NOOOOO,

    }
    private state currentState = null;

    public DcMotor  leftMotor         = null;
    public DcMotor  rightMotor        = null;
    public DcMotor  intakeMotor       = null;
    public DcMotor  leftShooterMotor  = null;
    public DcMotor  rightShooterMotor = null;
    public ServoController servoController = null;
    public Servo transportServo1 = null;
    public GyroSensor gyro             = null;
    public Servo ramServo        = null;
    public OpticalDistanceSensor lineSensor    = null;
    public ColorSensor beaconSensor = null;
    boolean isPressed = false;
    boolean isRed = true;
    boolean beaconIsRed = false;


    @Override
    public void init() {
        leftMotor = hardwareMap.dcMotor.get("leftmotor");
        rightMotor = hardwareMap.dcMotor.get("rightmotor");
        intakeMotor = hardwareMap.dcMotor.get("inmotor");
        leftShooterMotor = hardwareMap.dcMotor.get("LShootmotor");
        rightShooterMotor = hardwareMap.dcMotor.get("RShootmotor");
        ramServo = hardwareMap.servo.get("ramservo");
        gyro = hardwareMap.gyroSensor.get("gyro");
        lineSensor = hardwareMap.opticalDistanceSensor.get("lineSensor");
        beaconSensor = hardwareMap.colorSensor.get("colorsensor");

        transportServo1 = hardwareMap.servo.get("transervo1");
        //transportServo2 = hardwareMap.servo.get("transervo2");

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
        SetServo(0.40);
        ramServo.setPosition(0.5);

        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void start() {
        runtime.reset();
        newState(state.STATE_PARK_IN_CENTER);
    }

    @Override
    public void loop() {

        telemetry.addData("state", currentState);
        telemetry.addData("light", lineSensor.getLightDetected());

        if(gamepad1.a && !isPressed){
            isRed = !isRed; isPressed = true;
        } else if(!gamepad1.a){
            isPressed = false;
        }

        if(beaconSensor.blue()>beaconSensor.red()) {
            beaconIsRed = false;
        } else {
            beaconIsRed = true;
        }



        switch (currentState) {



           /*case STATE_SPOOL_UP_SHOOTERS:
                if (leftShooterMotor.getPower() >= .9 && rightShooterMotor.getPower() >= .9) {
                    SetServo(.40);
                    newState(state.STATE_DRIVE_TO_VORTEX);
                } else {
                    leftShooterMotor.setPower(.9);
                    rightShooterMotor.setPower(.9);
                }
                break;
            //Rev these shooters.

            case STATE_DRIVE_TO_VORTEX:
                newState(state.STATE_WAIT_FOR_SHOOTERS);
                break;
            //Driving to vortex.

            case STATE_WAIT_FOR_SHOOTERS:
                if (stateTime.time() >= 1) {
                    newState(state.STATE_UP);
                }
                    break;
                    //Waiting for shooters to get power.

            case STATE_UP:
                SetServo(.275);
                newState(state.STATE_WAIT);

                break;
                    //Moving transport ramp up to shoot ball.

            case STATE_WAIT:
                if (stateTime.time() >= 2) {
                    leftShooterMotor.setPower(0);
                    rightShooterMotor.setPower(0);
                    newState(state.STATE_WAIT_FOR_SHOOT);
                }
                break;

            case STATE_WAIT_FOR_SHOOT:
                if (stateTime.time() >= 2) {
                    SetServo(.4);
                    newState(state.STATE_WAIT_MORE);
                }
                break;

            case STATE_WAIT_MORE:
                if (stateTime.time() >= 1.5) {
                    leftShooterMotor.setPower(.8);
                    rightShooterMotor.setPower(.8);
                    newState(state.STATE_WAIT_FOURTH);
                }
                break;
                    //Rev these shooters.

            case STATE_WAIT_FOURTH:
                if (stateTime.time() >= .5) {
                    newState(state.STATE_SPECIAL_SNOWFLAKE);
                }
                break;

            case STATE_SPECIAL_SNOWFLAKE:
                if (leftShooterMotor.getPower() >= .8 && rightShooterMotor.getPower() >= .8) {
                    SetServo(.40);
                    newState(state.STATE_WAIT_THRID);
                } else {
                    leftShooterMotor.setPower(.8);
                    rightShooterMotor.setPower(.8);
                }
                break;

            case STATE_WAIT_THRID:
                if (stateTime.time() >= 1) {
                    newState(state.STATE_UP2);
                }
                break;

            case STATE_UP2:
                SetServo(.275);
                newState(state.STATE_WAIT2);

                break;
                    //Moving transport ramp up to shoot ball.

            case STATE_WAIT2:
                if (stateTime.time() >= 1) {
                    SetServo(0.4);
                    newState(state.STATE_SHUT_OFF_SHOOTERS);
                }
                break;
                    //Waiting half a second to move transport ramp back down.

            case STATE_SHUT_OFF_SHOOTERS:
                leftShooterMotor.setPower(0);
                rightShooterMotor.setPower(0);
                newState(state.STATE_PARK_IN_CENTER);
                break;*/
                    //Turning off shooter motors.

            case STATE_PARK_IN_CENTER:
                setPos(-10, .9);
                newState(state.STATE_STOP_MOVING);
                break;

            case STATE_STOP_MOVING:
                if (stateTime.time() >= .75) {
                    setDegrees(270);
                    newState(state.STATE_TURN);
                }
                break;

            case STATE_TURN:
                if (stateTime.time() >= 1.5) {
                    endmove();
                    newState(state.STATE_MOVE_A_LOT);
                }
                break;

            case STATE_MOVE_A_LOT:
                setPos(60, .9);
                newState(state.STATE_TIMER_TWO);
                break;

            case STATE_TIMER_TWO:
                if (stateTime.time() >= 3){
                    leftMotor.setPower(0);
                    rightMotor.setPower(0);
                    newState(state.STATE_NOOOOO);
                }
                break;

            case STATE_NOOOOO:
                if (stateTime.time() >= .5){
                    setDegrees(270);
                    newState(state.STATE_TURN_AGAIN);
                }
                break;

            case STATE_TURN_AGAIN:
               if (stateTime.time() >=1.5){
                   endmove();
                   newState(state.STATE_FIND_WHITE_LINE);
               }
                break;

            case STATE_FIND_WHITE_LINE:
                if (lineSensor.getLightDetected() >= .35){
                    setDegrees(270);
                    newState(state.STATE_TURN_ON_WHITE_LINE);
                }else {
                    setPos(40, .6);
                }
                break;

            case STATE_TURN_ON_WHITE_LINE:
                if (stateTime.time() >= 4.25){
                    endmove();
                    newState(state.STATE_CHOOSE_COLOR);
                }
                break;

            case STATE_CHOOSE_COLOR:
                if (sameCola()) {
                    ramServo.setPosition(1);
                    newState(state.STATE_RAM);
                } else {
                    ramServo.setPosition(0);
                    newState(state.STATE_RAM);
                }
                break;

            case STATE_RAM:
                if (stateTime.time() >= 1) {
                setPos(6, 1);
                newState(state.STATE_LAST_TIMER_LEL_HEHE_XD);}
                break;

            case STATE_LAST_TIMER_LEL_HEHE_XD:
                if (stateTime.time() >= 1){
                    leftMotor.setPower(0);
                    rightMotor.setPower(0);
                }
                break;
        }

    }


    public void stop() {
    }

    private void newState(state newState) {
        // Reset the state time, and then change to next state. 
        stateTime.reset();
        currentState = newState;
    }
    public void setPos(double inches, double goes) {

        leftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        int ticks = (int) (inches * (118.8356));
        int currentleft = leftMotor.getCurrentPosition();
        int currentright = rightMotor.getCurrentPosition();

        leftMotor.setTargetPosition(ticks + currentleft);
        rightMotor.setTargetPosition(ticks + currentright);
        leftMotor.setPower(goes);
        rightMotor.setPower(goes*.55);

    }

    public void endmove() {
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

    public boolean driveMotorsAreBusy() {

        return (leftMotor.isBusy() || rightMotor.isBusy());
    }

    public boolean transportsAreUp (){
        return currentPos <= transport1Up;
    }

    public void SetServo(double pos){
        currentPos = pos;
        transportServo1.setPosition(pos);
    }

    public double currentPos = 0;
    public double transport1Up = 0.37;

    public void setDegrees(int degrees){
        this.degrees = degrees/* * teamNumber()*/;
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        initheading = gyro.getHeading();
    }
    public void turn(){
        //setting the motors to turning power
        int multiplier = (degrees/Math.abs(degrees));
        leftMotor.setPower(.5 * multiplier);
        rightMotor.setPower(-.5 * multiplier);
    }

    public boolean doneTurning (){
        //returning if the robot are done turning
        return (Math.abs(gyro.getHeading()-initheading) >= Math.abs(degrees));
    }

    public double PIDTurn() {
        double error = degrees - (gyro.getHeading() - initheading);
        double motorPower = .027 * error;
        return motorPower;
    }

    public void turn(double power){
        //overloading thing
        leftMotor.setPower(power);
        rightMotor.setPower(-power);
    }

    public boolean PIDWithinTolerance() {
        double error = degrees - (gyro.getHeading() - initheading);
        if (error <= 2){
            telemetry.addData("degressturned", true);
            return true;
        } else {
            telemetry.addData("degressturned", false);

            return false;
        }
    }

    public boolean sameCola(){
        if(beaconIsRed && isRed) {
            return true;
        } else if (!beaconIsRed && !isRed) {
            return true;
        } else {
            return false;
        }
    }

    public int initheading = 0;

    public int degrees = 0;
} 