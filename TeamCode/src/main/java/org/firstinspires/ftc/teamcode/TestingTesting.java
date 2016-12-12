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

@Autonomous(name="TestingTesting", group="WTR")  // @Autonomous(...) is the other common choice
//@Disabled
public class TestingTesting extends OpMode
{
    //RobotHardware robot = new RobotHardware();

    ColorSensor beaconSensor  = null;
    I2cDeviceSynch beaconSensorReader = null;
    public ServoController servoController = null;
    public Servo ramServo = null;
    public Servo transportservo1 = null;

    boolean isPressed = false;
    boolean isRed = true;
    boolean beaconIsRed = false;

    byte[] colorCcache;


    @Override
    public void init() {
        beaconSensor = hardwareMap.colorSensor.get("colorsensor");
        //beaconSensorReader = new I2cDeviceSynchImpl(beaconSensor, I2cAddr.create8bit(0x3c), false);
        //beaconSensorReader.engage();
        transportservo1 = hardwareMap.servo.get("transervo1");
        ramServo = hardwareMap.servo.get("ramservo");
        servoController = hardwareMap.servoController.get("sv1");
        ////beaconSensorReader.engage();
      //  beaconSensorReader.write8(3, 0);
        beaconSensor.enableLed(false);
    }

    @Override
    public void init_loop() {
        ramServo.setPosition(.5);
        transportservo1.setPosition(.4);

        telemetry.addData("red", beaconSensor.red());
        telemetry.addData("blue", beaconSensor.blue());
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {

        if(beaconSensor.blue()>beaconSensor.red()) {
            beaconIsRed = false;
        } else {
            beaconIsRed = true;
        }

        telemetry.addData("team is red", isRed);
        telemetry.addData("beaconisred", beaconIsRed);


        if(gamepad1.a && !isPressed){
            isRed = !isRed;
            isPressed = true;
        } else if(!gamepad1.a){
            isPressed = false;
        }

        if (sameCola()) {
            ramServo.setPosition(1);
        } else {
            ramServo.setPosition(0);
        }


    }

    @Override
    public void stop() {
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
}