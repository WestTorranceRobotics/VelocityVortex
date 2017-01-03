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
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Iterative Auto", group="WTR")  // @Autonomous(...) is the other common choice
//@Disabled
public class ShooterSpeedControl extends OpMode
{
    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime statetime = new ElapsedTime();

    RobotHardware robot = new RobotHardware();
    IterativeFunctions fanctions = new IterativeFunctions(robot);
    double lInitTime = 0;
    double lInitTick = 0;
    double rInitTime = 0;
    double rInitTick = 0;

    private enum state {
        STATE_SPOOL_UP,
        STATE_MEASURE,
        STATE_STOP,
        STATE_CALCULATE,
        STATE_WAIT,
        STATE_IMITATE
    }

    state currentState = null;

    double lencoderRate = 0;
    double rencoderRate = 0;

    double lerrorsum = 0;
    double rerrorsum = 0;

    int linitial = 0;
    int rinitial = 0;

    int lencoderTravel = 0;
    int rencoderTravel = 0;

    @Override
    public void init() {
        robot.initRobotHardware(hardwareMap);
        robot.leftShooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightShooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void init_loop() {
        fanctions.anushalizeRobotHardware();
        robot.leftShooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightShooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void start() {
        runtime.reset();
        newState(state.STATE_SPOOL_UP);
    }

    @Override
    public void loop() {

        switch (currentState) {

            case STATE_SPOOL_UP:
                if(statetime.time() >= 5) {
                    linitial = robot.leftShooterMotor.getCurrentPosition();
                    rinitial = robot.rightShooterMotor.getCurrentPosition();
                    newState(state.STATE_MEASURE);
                } else {
                    fanctions.setShooterSpeed(0.8);
                }
                break;

            case STATE_MEASURE:
                if(statetime.time() >= 10) {
                    lencoderTravel = robot.leftShooterMotor.getCurrentPosition() - linitial;
                    rencoderTravel = robot.rightShooterMotor.getCurrentPosition() - rinitial;
                    newState(state.STATE_STOP);
                }
                break;

            case STATE_STOP:
                fanctions.setShooterSpeed(0);
                newState(state.STATE_IMITATE);
                break;

            case STATE_CALCULATE:
                lencoderRate = lencoderTravel/10;
                rencoderRate = rencoderTravel/10;
                break;

            case STATE_WAIT:
                if(statetime.time() >= 5) {
                    newState(state.STATE_IMITATE);
                }
                break;

            case STATE_IMITATE:


                break;
        }

    }

    @Override
    public void stop() {
    }

    public void newState(state newState) {
        currentState = newState;
        statetime.reset();
    }

    public double lCurrentRate(){
        double timeChange = runtime.time() - lInitTime;
        double distanceChange = robot.leftMotor.getCurrentPosition() - lInitTick;
        double rate = distanceChange / timeChange;
        lInitTime = runtime.time();
        lInitTick = robot.leftMotor.getCurrentPosition();
        return rate;

    }

    public double rCurrentRate(){
        double timeChange = runtime.time() - rInitTime;
        double distanceChange = robot.rightMotor.getCurrentPosition() - rInitTick;
        double rate = distanceChange / timeChange;
        rInitTime = runtime.time();
        rInitTick = robot.rightMotor.getCurrentPosition();
        return rate;

    }

    public double lShooterPID(){
        double error = lencoderRate - lCurrentRate();
        lerrorsum += error;

        return(0 * error) + (0 * lerrorsum);
    }

    public double rShooterPID(){
        double error = rencoderRate - rCurrentRate();
        rerrorsum += error;

        return(0 * error) + (0 * rerrorsum);
    }
}
