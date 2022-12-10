package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.hardware.bosch.BNO055IMU;

import com.qualcomm.robotcore.hardware.HardwareMap;

// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class Robot {

    // CONFIGURATION
    double      clawOpenPosition    = 0.2;
    double      clawClosePosition   = 0.55;
    double      feetToTicks         = (19.2*28.0*304.8) / (Math.PI*96.0);
    double      ticksToFeet         = 1.0/feetToTicks;
    int         colorSensorSamples  = 5;
    
    int         coneLiftlevel       = 0.06; // feet
    int         terminalLiftlevel   = 0.14; // feet
    int         smallLiftlevel      = 0.86; // feet
    int         mediumLiftlevel     = 1.34; // feet
    int         currentLiftlevel    = 0;    // index
    double[]    liftLevels          = {coneLiftlevel, terminalLiftlevel, smallLiftlevel, mediumLiftlevel};

    // DRIVETRAIN
    DcMotor     topRight;
    DcMotor     bottomRight;
    DcMotor     topLeft;
    DcMotor     bottomLeft;

    // CLAW
    Servo       clawServo;

    // LIFT
    DcMotor     liftMotor;

    // SENSORS
    BNO055IMU   imu;
    ColorSensor colorSensorBack;

    // initalize the robot hardware
    public void init(HardwareMap hardwareMap) {

        // TODO: verify comments are correct ports!
        // TODO: create new robot configuration with new names!

        // initalize drive train
        topRight        = hardwareMap.dcMotor.get("topRight");                  // control hub port 0
        bottomRight     = hardwareMap.dcMotor.get("bottomRight");               // control hub port 1
        topLeft         = hardwareMap.dcMotor.get("topLeft");                   // control hub port 2
        bottomLeft      = hardwareMap.dcMotor.get("bottomLeft");                // control hub port 3
        
        // initalize claw
        clawServo       = hardwareMap.servo.get("claw");                        // expansion hub port 0
        clawServo.scaleRange(clawOpenPosition, clawClosePosition);

        // initalize lift
        liftMotor       = hardwareMap.dcMotor.get("liftMotor");                 // expansion hub port 0

        // initalize sensors
        imu             = hardwareMap.get(BNO055IMU.class, "imu");
        colorSensorBack = hardwareMap.get(ColorSensor.class, "colorSensorBack");// port ??
        

        // setup IMU
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode                 = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit            = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled       = false;
        imu.initialize(parameters);

        // while (!isStopRequested() && !imu.isGyroCalibrated()) {
        //     sleep(50);
        //     idle();
        // }
    }

    // DRIVETRAIN FUNCTIONS
    public void stopDriveMotors(){
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
        return (double)redSum / (double)colorSensorSamples;
    }

    // return an average blue value
    public double blueAverage(ColorSensor colorSensor) {

        // initalize blueSum to 0
        int blueSum = 0;

        // run for colorSensorSamples iterations
        for (int i = 0; i < 5; i++) {
            // increment blueSum by our new blue color sensor reading
            blueSum += colorSensor.blue();
        }

        // return the average blue value (blueSum divided by the number of samples we take)
        return (double)blueSum / (double)colorSensorSamples;
    }

    // return an average green value
    public double greenAverage(ColorSensor colorSensor) {

        // initalize greenSum to 0
        int greenSum = 0;

        // run for colorSensorSamples iterations
        for (int i = 0; i < 5; i++) {
            // increment greenSum by our new green color sensor reading
            greenSum += colorSensor.green();
        }

        // return the average green value (greenSum divided by the number of samples we take)
        return (double)greenSum / (double)colorSensorSamples;
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
        return (double)alphaSum / (double)colorSensorSamples;
    }
}