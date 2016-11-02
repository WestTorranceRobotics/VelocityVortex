/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.util.ElapsedTime;




@Autonomous(name="code1", group="code1")  // @Autonomous(...) is the other common choice
// @Disabled
public class Code1 extends LinearOpMode {

    DcMotor leftmotor;
    DcMotor rightmotor;
    GyroSensor gyro;

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime turnTimer = new ElapsedTime();
    // DcMotor leftMotor = null;
    // DcMotor rightMotor = null;

    @Override
    public void runOpMode() throws InterruptedException {


        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        // leftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE
        while (opModeIsActive()) {
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();

            // eg: Run wheels in tank mode (note: The joystick goes negative when pushed forwards)
            // leftMotor.setPower(-gamepad1.left_stick_y);
            // rightMotor.setPower(-gamepad1.right_stick_y);

            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
    }


    public void setPos(double inches, double goes) {

        leftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                int ticks = (int)(inches*(1/4*3.14159265359)*(16/24)*(1120));
                int currentleft = leftmotor.getCurrentPosition();
                int currentright = rightmotor.getCurrentPosition();

                leftmotor.setTargetPosition(ticks+ currentleft);
                rightmotor.setTargetPosition(ticks+ currentright);
                leftmotor.setPower(goes);
                rightmotor.setPower(goes);
            }
            public void endmove(){
                leftmotor.setPower(0);
                rightmotor.setPower(0);
            }
            public void runtoposition(double inches, double speed) {
                setPos(inches, speed);
                while (leftmotor.isBusy() && rightmotor.isBusy() && opModeIsActive()) {

                }
                endmove();
            }

    public void gyrogyro(int degrees) {
        int intheading = gyro.getHeading();

        int multiplier = (degrees/Math.abs(degrees));

        leftmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while (Math.abs(gyro.getHeading()-intheading) <= Math.abs(degrees)) {

            leftmotor.setPower(multiplier * .6);
            rightmotor.setPower(-.6 * multiplier);

        }
        leftmotor.setPower(0);
        rightmotor.setPower(0);
    }

}