package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
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
    private ElapsedTime runtime = new ElapsedTime();
    //RobotHardware robot = new RobotHardware();

    I2cDevice beaconSensor  = null;
    I2cDeviceSynch beaconSensorReader = null;
    public ServoController servoController = null;
    public Servo ramServo = null;

    boolean isPressed = false;
    boolean isRed = true;

    byte[] colorCcache;


    @Override
    public void init() {
        beaconSensor = hardwareMap.i2cDevice.get("cc");
        beaconSensorReader = new I2cDeviceSynchImpl(beaconSensor, I2cAddr.create8bit(0x3c), false);
        beaconSensorReader.engage();
        ramServo = hardwareMap.servo.get("ramservo");
        servoController = hardwareMap.servoController.get("sv1");
        beaconSensorReader.engage();
        beaconSensorReader.write8(3, 0);
    }

    @Override
    public void init_loop() {
        ramServo.setPosition(0);
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {

        telemetry.addData("team is red", isRed);
        telemetry.addData("color", colorCcache);

        if(gamepad1.a && !isPressed){
            isRed = !isRed;
            isPressed = true;
        } else if(!gamepad1.a){
            isPressed = false;
        }

        if (sameCola(colorCcache[0] & 0xFF)) {
            ramServo.setPosition(1);
        } else {
            ramServo.setPosition(-1);
        }

        colorCcache = beaconSensorReader.read(0x04, 1);
    }

    @Override
    public void stop() {
    }

    public boolean sameCola(int colorNumber){
        if(colorNumber == 10 && isRed) {
            return true;
        } else if (colorNumber == 3 && !isRed) {
            return true;
        } else {
            return false;
        }
    }
}