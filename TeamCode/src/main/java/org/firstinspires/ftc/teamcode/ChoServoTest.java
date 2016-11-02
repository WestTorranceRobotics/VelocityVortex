package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * This OpMode scans a single servo back and forwards until Stop is pressed.
 * The code is structured as a LinearOpMode
 * INCREMENT sets how much to increase/decrease the servo position each cycle
 * CYCLE_MS sets the update period.
 *
 * This code assumes a Servo configured with the name "left claw" as is found on a pushbot.
 *
 * NOTE: When any servo position is set, ALL attached servos are activated, so ensure that any other
 * connected servos are able to move freely before running this test.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@TeleOp(name = "Cho Servo Test", group = "Zach")

public class ChoServoTest extends LinearOpMode {

    static final double INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final int    CYCLE_MS    =   50;     // period of each cycle
    static final double MAX_POS     =  1.0;     // Maximum rotational position
    static final double MIN_POS     =  0.0;     // Minimum rotational position

    // Define class members
    Servo   servoLeft;
    Servo   servoRight;

    boolean leftBumperPressed = false;
    boolean rightBumperPressed = false;
    boolean isOpen = false;

    double leftClosedPosition = 1.0;
    double rightClosedPosition = 0;
    double leftOpenPosition = 0;
    double rightOpenPosition = 1.0;




    @Override
    public void runOpMode() throws InterruptedException {

        servoRight = hardwareMap.servo.get("right");
        servoLeft = hardwareMap.servo.get("left");


        telemetry.addData("Status", " Press Start to begin test." );
        telemetry.update();
        waitForStart();
        telemetry.clear();


        while(opModeIsActive()){

            if(gamepad1.left_bumper && isOpen) {
                leftBumperPressed = true;
                isOpen = false;
            } else if (!gamepad1.left_bumper) {
                leftBumperPressed = false;
            }

            if(gamepad1.right_bumper && !isOpen) {
                rightBumperPressed = true;
                isOpen = true;
            } else if (!gamepad1.right_bumper) {
                rightBumperPressed = false;
            }


            if(isOpen) {
                servoLeft.setPosition(leftOpenPosition);
                servoRight.setPosition(rightOpenPosition);
            } else {
                servoLeft.setPosition(leftClosedPosition);
                servoRight.setPosition(rightClosedPosition);
            }


            idle();
        }
    }
}
