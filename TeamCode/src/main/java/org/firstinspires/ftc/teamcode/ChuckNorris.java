package org.firstinspires.ftc.teamcode;

import

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * Created by Barry on 10/25/2016.
 */

public class ChuckNorris {
}
@TeleOp(name="Cool Beans", group="WTR")

public class Test extends LinearOpMode {

    /* Declare OpMode members. */
    public ElapsedTime runtime = new ElapsedTime();
    DcMotor leftMotor = null;
     DcMotor rightMotor = null;
    double power;
    double turn;
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

