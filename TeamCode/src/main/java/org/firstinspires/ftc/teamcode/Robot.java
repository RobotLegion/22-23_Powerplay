package org.firstinspires.ftc.teamcode;

import android.os.Environment;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.io.FileWriter;
import java.io.IOException;
import java.sql.Timestamp;
import java.text.SimpleDateFormat;

// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class Robot {

    // CONFIGURATION
    double clawOpenPosition = 0.2;
    double clawClosePosition = 0.55;
    double feetToTicks = (19.2 * 28.0 * 304.8) / (Math.PI * 96.0);
    double ticksToFeet = 1.0 / feetToTicks;
    int colorSensorSamples = 5;

    double coneLiftlevel = 0.08; // feet
    double terminalLiftlevel = 0.11; // feet
    double smallLiftlevel = 0.83; // feet
    double mediumLiftlevel = 1.31; // feet
    // double coneStackLevel = //feet
    int currentLiftLevel = 0;    // index
    double[] liftLevels = {coneLiftlevel, terminalLiftlevel, smallLiftlevel, mediumLiftlevel};
    String[] liftLevelNames = {"coneLiftlevel", "terminalLiftlevel", "smallLiftlevel", "mediumLiftlevel"};

    // DRIVETRAIN
    DcMotor topRight; //port 0 control hub
    DcMotor bottomRight; //port 1 control hub
    DcMotor topLeft; //port 0 expansion hub
    DcMotor bottomLeft; //port 1 expansion hub

    // CLAW
    Servo clawServo; //port 0 control hub

    // LIFT
    DcMotor liftMotor; //port 2 control hub

    // SENSORS
    BNO055IMU imu;
    ColorSensor colorSensorBack; //port I2C 2
    ColorSensor colorSensorLeft; //port  I2C 1
    ColorSensor colorSensorRight; //port I2C 0
    double ALPHA_THRESHOLD = 200.0;

    // File
    FileWriter f;


    // initalize the robot hardware
    public void init(HardwareMap hardwareMap) {

        // TODO: verify comments are correct ports!

        // initalize drive train
        topRight = hardwareMap.dcMotor.get("topRight");                  // control hub port 0
        bottomRight = hardwareMap.dcMotor.get("bottomRight");               // control hub port 1
        topLeft = hardwareMap.dcMotor.get("topLeft");                   // control hub port 2
        bottomLeft = hardwareMap.dcMotor.get("bottomLeft");                // control hub port 3

        // initalize claw
        clawServo = hardwareMap.servo.get("clawServo");                        // expansion hub port 0
        clawServo.scaleRange(clawOpenPosition, clawClosePosition);

        // initalize lift
        liftMotor = hardwareMap.dcMotor.get("liftMotor");                 // expansion hub port 0

        // initalize sensors
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        colorSensorBack = hardwareMap.get(ColorSensor.class, "colorSensorBack");// port ??
        colorSensorLeft = hardwareMap.get(ColorSensor.class, "colorSensorLeft");// port ??
        colorSensorRight = hardwareMap.get(ColorSensor.class, "colorSensorRight");// port ??


        // setup IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        imu.initialize(parameters);

        // .\adb.exe shell ls /storage/self/primary/FIRST/data
        Timestamp timestamp = new Timestamp(System.currentTimeMillis());
        SimpleDateFormat sdf1 = new SimpleDateFormat("MM.dd.HH.mm.ss");
        String logFilePath = String.format("%s/FIRST/data/%s.txt",
                Environment.getExternalStorageDirectory().getAbsolutePath(),
                sdf1.format(timestamp));

        try {
            f = new FileWriter(logFilePath);
        } catch (IOException e) {

        }

        // while (!isStopRequested() && !imu.isGyroCalibrated()) {
        //     sleep(50);
        //     idle();
        // }
    }

    public void destroy() {
        try {
            f.close();
        } catch (IOException e) {

        }
    }

    public void log(String s) {

        try {
            Timestamp timestamp = new Timestamp(System.currentTimeMillis());
            SimpleDateFormat sdf1 = new SimpleDateFormat("MM.dd.HH.mm.ss.SSS");

            String logEntry = String.format("%s: %s\n",
                    sdf1.format(timestamp),
                    s);

            f.write(logEntry);
            f.flush();
        } catch (IOException e) {

        }
    }

    public void log(String s, double d) {
        String tmp = String.format("%s %f", s, d);
        log(tmp);
    }

    public void log(String s, String s2) {
        String tmp = String.format("%s %s", s, s2);
        log(tmp);
    }

    // DRIVETRAIN FUNCTIONS
    public void stopDriveMotors() {
        setDrivePower(0);
    }

    // NOTE: direction is set by signs given to "setTargetPosition"
    public void setDrivePower(double speed) {
        topRight.setPower(speed);
        bottomRight.setPower(speed);
        topLeft.setPower(speed);
        bottomLeft.setPower(speed);
    }

    public void driveWithoutEncoder() {
        topLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        topRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bottomLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bottomRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void driveWithEncoder() {
        topLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        topRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void driveStopAndReset() {
        topLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void driveRunToPosition() {
        topLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        topRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bottomLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bottomRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    // CLAW FUNCTIONS
    public void clawOpen() {
        clawServo.setPosition(0.0);
    }

    public void clawClose() {
        clawServo.setPosition(1.0);
    }

    public boolean isClawOpen() {
        return clawServo.getPosition() < 0.5;
    }

    // COLOR SENSOR FUNCTIONS
    // return an average red value
    public double redAverage(ColorSensor colorSensor) {

        // initalize redSum to 0
        int redSum = 0;

        // run for colorSensorSamples iterations
        for (int i = 0; i < colorSensorSamples; i++) {
            // increment redSum by our new red color sensor reading
            redSum += colorSensor.red();
        }

        // return the average red value (redSum divided by the number of samples we take)
        return (double) redSum / (double) colorSensorSamples;
    }

    // return an average blue value
    public double blueAverage(ColorSensor colorSensor) {

        // initalize blueSum to 0
        int blueSum = 0;

        // run for colorSensorSamples iterations
        for (int i = 0; i < colorSensorSamples; i++) {
            // increment blueSum by our new blue color sensor reading
            blueSum += colorSensor.blue();
        }

        // return the average blue value (blueSum divided by the number of samples we take)
        return (double) blueSum / (double) colorSensorSamples;
    }

    // return an average green value
    public double greenAverage(ColorSensor colorSensor) {

        // initalize greenSum to 0
        int greenSum = 0;

        // run for colorSensorSamples iterations
        for (int i = 0; i < colorSensorSamples; i++) {
            // increment greenSum by our new green color sensor reading
            greenSum += colorSensor.green();
        }

        // return the average green value (greenSum divided by the number of samples we take)
        return (double) greenSum / (double) colorSensorSamples;
    }

    // returns an average alpha value
    public double alphaAverage(ColorSensor colorSensor) {

        // initalize alphaSum to 0
        int alphaSum = 0;

        // run for NUM_SAMPLES iterations
        for (int i = 0; i < colorSensorSamples; i++) {
            // increment alphaSum by our new alpha color sensor reading
            alphaSum += colorSensor.alpha();
        }

        // return the average alpha value (alphaSum divided by the number of samples we take)
        return (double) alphaSum / (double) colorSensorSamples;
    }

    public boolean isColorValid(ColorSensor colorSensor) {

        double alphaAvg = alphaAverage(colorSensor);

        // color sensor is valid
        return alphaAvg >= ALPHA_THRESHOLD;
    }

    // COLOR SENSOR FUNCTIONS
    // check if the input color sensor r,g,b values represent parking zone 1
    public boolean isParking1(double r, double g, double b) {

        // perfect reading:   r=1.00, g=0.50, b=0.70
        // imperfect reading: r=0.70, g=1.00, b=0.900

        // initalize all check variables to false
        boolean redCheck = false;
        boolean greenCheck = false;
        boolean blueCheck = false;

        // check if r is in the range
        if (r >= 0.6 && r <= 1.0) {
            redCheck = true;
        }

        // check if g is in the range
        if (g >= 0.4 && g <= 1.0) {
            greenCheck = true;
        }

        // check if b is in the range
        if (b >= 0.6 && b <= 1.0) {
            blueCheck = true;
        }

        // if all color checks are true, return true
        // otherwise, return false
        return redCheck && greenCheck && blueCheck;
    }

    // check if the input color sensor r,g,b values represent parking zone 3
    public boolean isParking3(double r, double g, double b) {

        // perfect reading:   r=0.56, g=1.00, b=0.27
        // imperfect reading: r=0.58, g=1.00, b=0.64

        // initalize all check variables to false
        boolean redCheck = false;
        boolean greenCheck = false;
        boolean blueCheck = false;

        // check if r is in the range
        if (r >= 0.4 && r <= 0.6) {
            redCheck = true;
        }

        // check if g is in the range
        if (g >= 0.7 && g <= 1.0) {
            greenCheck = true;
        }

        // check if b is in the range
        if (b >= 0.1 && b <= 0.7) {
            blueCheck = true;
        }

        // if all color checks are true, return true
        // otherwise, return false
        return redCheck && greenCheck && blueCheck;
    }

}