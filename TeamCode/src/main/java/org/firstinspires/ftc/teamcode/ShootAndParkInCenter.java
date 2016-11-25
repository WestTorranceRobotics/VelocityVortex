package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Iterative Auto", group="WTR")  // @Autonomous(...) is the other common choice 
//@Disabled 
public class ShootAndParkInCenter extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime stateTime = new ElapsedTime();
    RobotHardware robot = new RobotHardware();
    IterativeFunctions fanctions = new IterativeFunctions();

    private enum state {
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
        STATE_SHUT_OFF_SHOOTERS,
        //Turning off shooter motors. 
        STATE_TURN_RIGHT,
        //Turning right 90 degrees. 
        STATE_MOVE_FORWARD,
        //Move forward two feet. 
        STATE_TURN_RIGHT_MORE,
        //Turning right 45 degrees. 
        STATE_PARK_IN_CENTER_BACKWARDS,
        //Move two feet backwards to park on center. 
    }
    private state currentState;


    @Override
    public void init() {
        robot.initRobotHardware(hardwareMap);
    }

    @Override
    public void init_loop() {
        robot.anushalizeRobotHardware();
    }

    @Override
    public void start() {
        runtime.reset();
        newState(state.STATE_SPOOL_UP_SHOOTERS);
    }

    @Override
    public void loop() {
        switch (currentState) {

            case STATE_SPOOL_UP_SHOOTERS:
                robot.setShooterSpeed(.8);
                newState(state.STATE_DRIVE_TO_VORTEX);
                break;
            //Rev these shooters.

            case STATE_DRIVE_TO_VORTEX:
                fanctions.setPos(48, .6);
                newState(state.STATE_WAIT_FOR_SHOOTERS);
                break;
            //Driving to vortex.

            case STATE_WAIT_FOR_SHOOTERS:
                if (robot.leftShooterMotor.getPower() >= .8 && robot.rightShooterMotor.getPower() >= .8) {
                    robot.setTransportsUp();
                    newState(state.STATE_UP);
                }
                break;
            //Waiting for shooters to get power.

            case STATE_UP:
                if (robot.transportsAreUp()) {
                    newState(state.STATE_WAIT);
                }
                break;
            //Moving transport ramp up to shoot ball.

            case STATE_WAIT:
                if (stateTime.time() >= .5) {
                    robot.setTransportsDown();
                    newState(state.STATE_SHUT_OFF_SHOOTERS);
                }
                break;
            //Waiting half a second to move transport ramp back down.

            case STATE_SHUT_OFF_SHOOTERS:
                robot.setShooterSpeed(0);
                newState(state.STATE_TURN_RIGHT);
                fanctions.setDegrees(90);
                break;
            //Turning off shooter motors.

            case STATE_TURN_RIGHT:
                if(fanctions.doneTurning()) {
                    fanctions.endmove();
                    newState(state.STATE_MOVE_FORWARD);
                } else {
                    fanctions.turn();
                }
                break;
            //Turning right 90 degrees.

            case STATE_MOVE_FORWARD:
                fanctions.setPos(24, .6);
                fanctions.setDegrees(45);
                break;
            //Move forward two feet.

            case STATE_TURN_RIGHT_MORE:
                if(fanctions.doneTurning()){
                    fanctions.endmove();
                    newState(state.STATE_PARK_IN_CENTER_BACKWARDS);
                } else {
                    fanctions.turn();
                }
                break;
            //Turning right 45 degrees.

            case STATE_PARK_IN_CENTER_BACKWARDS:
                fanctions.setPos(-24, .6);
                break;
            //Move two feet backwards to park on center.
        }
    }

    public void stop() {
    }
    private void newState(state newState) {
        // Reset the state time, and then change to next state. 
        stateTime.reset();
        currentState = newState;
    }
} 