import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.hardware.bosch.BNO055IMU;

import com.qualcomm.robotcore.hardware.HardwareMap;

// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class Robot {

    // DRIVETRAIN
    DcMotor     topRight;
    DcMotor     bottomRight;
    DcMotor     topLeft;
    DcMotor     bottomLeft;
    double      feetToTicks = (19.2*28.0*304.8) / (Math.PI*96.0);

    // CLAW
    Servo       clawServo;

    // LIFT
    DcMotor     liftMotor;

    // SENSORS
    BNO055IMU   imu;
    ColorSensor colorSensorBack;

    // initalize the robot hardware
    public void init() {

        // TODO: verify comments are correct ports!
        // TODO: create new robot configuration with new names!

        // initalize drive train
        topRight        = hardwareMap.dcMotor.get("topRight");                  // control hub port 0
        bottomRight     = hardwareMap.dcMotor.get("bottomRight");               // control hub port 1
        topLeft         = hardwareMap.dcMotor.get("topLeft");                   // control hub port 2
        bottomLeft      = hardwareMap.dcMotor.get("bottomLeft");                // control hub port 3
        
        // initalize claw
        claw            = hardwareMap.servo.get("claw");                        // expansion hub port 0
        
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
}