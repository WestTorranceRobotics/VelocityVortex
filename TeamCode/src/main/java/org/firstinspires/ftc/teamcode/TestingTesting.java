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
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.ElapsedTime;
@Autonomous(name="TestingTesting", group="WTR")
 @Autonomous(...) is the other common choice
 //@Disabled
 public class TestingTesting extends OpMode { //RobotHardware robot = new RobotHardware(); ColorSensor beaconSensor = null;
 I2cDeviceSynch beaconSensorReader = null; public ServoController servoController = null;
 public Servo ramServo = null; public Servo transportservo1 = null; boolean isPressed = false;
 boolean isRed = true;
 boolean beaconIsRed = false; byte[] colorCcache;
 @Override public void init() { beaconSensor = hardwareMap.colorSensor.get("colorsensor");
 beaconSensorReader = new I2cDeviceSynchImpl(beaconSensor, I2cAddr.create8bit(0x3c), false);
 beaconSensorReader.engage(); transportservo1 = hardwareMap.servo.get("transervo1");
 ramServo = hardwareMap.servo.get("ramservo"); servoController = hardwareMap.servoController.get("sv1");
beaconSensorReader.engage();
 beaconSensorReader.write8(3, 0);
 beaconSensor.enableLed(false); } @Override public void init_loop() { ramServo.setPosition(.5);
 transportservo1.setPosition(.4);
 telemetry.addData("red", beaconSensor.red());
telemetry.addData("blue", beaconSensor.blue());
     } @Override public void start() { } @Override public void loop()
     { if(beaconSensor.blue()>beaconSensor.red()) { beaconIsRed = false;
     } else { beaconIsRed = true; } telemetry.addData("team is red", isRed);
         telemetry.addData("beaconisred", beaconIsRed); if(gamepad1.a && !isPressed){ isRed = !isRed; isPressed = true;
     } else if(!gamepad1.a){ isPressed = false; } if (sameCola()) { ramServo.setPosition(1);
     } else { ramServo.setPosition(0);
     } } @Override public void stop() { } public boolean sameCola(){ if(beaconIsRed && isRed) { return true;
     } else if (!beaconIsRed && !isRed) { return true; } else { return false; } } }