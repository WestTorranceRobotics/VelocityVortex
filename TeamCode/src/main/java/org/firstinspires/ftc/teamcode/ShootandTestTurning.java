package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Turn test", group="WTR")  // @Autonomous(...) is the other common choice
//@Disabled 
public class ShootandTestTurning extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime stateTime = new ElapsedTime();
   // RobotHardware robot = new RobotHardware();
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
        STATE_TEST_TURNING,
    }
    private state currentState = null;

    public DcMotor  leftMotor         = null;
    public DcMotor  rightMotor        = null;
    public DcMotor  intakeMotor       = null;
    public DcMotor  leftShooterMotor  = null;
    public DcMotor  rightShooterMotor = null;
    public ServoController servoController = null;
    public Servo transportServo1 = null;
    public GyroSensor gyro           = null;


    @Override
    public void init() {
        leftMotor = hardwareMap.dcMotor.get("leftmotor");
        rightMotor = hardwareMap.dcMotor.get("rightmotor");
        intakeMotor = hardwareMap.dcMotor.get("inmotor");
        leftShooterMotor = hardwareMap.dcMotor.get("LShootmotor");
        rightShooterMotor = hardwareMap.dcMotor.get("RShootmotor");

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
    }

    @Override
    public void start() {
        runtime.reset();
        newState(state.STATE_SPOOL_UP_SHOOTERS);
    }

    @Override
    public void loop() {

        telemetry.addData("state", currentState);

        switch (currentState) {

            case STATE_SPOOL_UP_SHOOTERS:
                if (leftShooterMotor.getPower() >= 1 && rightShooterMotor.getPower() >= 1){
                    SetServo(.4);
                    newState(state.STATE_DRIVE_TO_VORTEX);
                }else{
                    leftShooterMotor.setPower(1);
                    rightShooterMotor.setPower(1);
                }
                break;
            //Rev these shooters.

            case STATE_DRIVE_TO_VORTEX:
                newState(state.STATE_WAIT_FOR_SHOOTERS);
                break;
            //Driving to vortex.

            case STATE_WAIT_FOR_SHOOTERS:
                if (stateTime.time() >= 0) {
                    newState(state.STATE_UP);
                }
                break;
            //Waiting for shooters to get power.

            case STATE_UP:

                if(!transportsAreUp()){
                    SetServo(currentPos - 1.75);
                } else {
                    newState(state.STATE_WAIT);
                }

                break;
            //Moving transport ramp up to shoot ball.

            case STATE_WAIT:
                if (stateTime.time() >= .5) {
                    SetServo(0.4);
                    newState(state.STATE_WAIT_MORE);
                }
                break;
            case STATE_WAIT_MORE:
                if (stateTime.time() >= 3){
                    newState(state.STATE_UP2);
                }
                break;
            //Rev these shooters.
            case STATE_UP2:

                if(!transportsAreUp()){
                    SetServo(currentPos - 1.75);
                } else {
                    newState(state.STATE_WAIT2);
                }

                break;
            //Moving transport ramp up to shoot ball.

            case STATE_WAIT2:
                if (stateTime.time() >= .5) {
                    SetServo(0.4);
                    newState(state.STATE_SHUT_OFF_SHOOTERS);
                }
                break;
            //Waiting half a second to move transport ramp back down.

            case STATE_SHUT_OFF_SHOOTERS:
                leftShooterMotor.setPower(0);
                rightShooterMotor.setPower(0);
                setPos(20, .6);
                newState(state.STATE_TEST_TURNING);
                break;

            //Turning off shooter motors.

            case STATE_TEST_TURNING:
                if (!driveMotorsAreBusy()){
                    setDegrees(90);
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
        rightMotor.setPower(goes);
    }

    public void endmove() {
        leftMotor.setPower(0);
        rightMotor.setPower(0);
    }

    public boolean driveMotorsAreBusy() {
        return (leftMotor.isBusy() || rightMotor.isBusy());
    }

    public int degrees = 0;

    public void setDegrees(int degrees){
        this.degrees = degrees;
        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public boolean doneTurning (){
        //returmimg if the motors are done turning
        int intheading = gyro.getHeading();
        return (Math.abs(gyro.getHeading()-intheading) >= Math.abs(degrees));
    }

    public void turn() {
        //setting the motors to turning power
        int multiplier = (degrees / Math.abs(degrees));
        leftMotor.setPower(.6 * multiplier);
        rightMotor.setPower(-.6 * multiplier);
    }

    public boolean transportsAreUp (){
        return currentPos <= transport1Up;
    }

    public void SetServo(double pos){
        currentPos = pos;
        transportServo1.setPosition(pos);
    }
    //public boolean servoIsUp(){
     //   return currentPos <= .25;
    //}

    public double currentPos = 0;
    public double transport1Up = 0.275;
    //public double transport1Down = 0;

} 