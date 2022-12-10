//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.ColorSensor;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
//import com.qualcomm.hardware.bosch.BNO055IMU;
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
//import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
//import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
//import org.firstinspires.ftc.robotcore.external.navigation.Position;
//import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
//
//@Autonomous(name="autonomous rotation", group="Robot")
//
//
//public class AutoRotate extends LinearOpMode {
//    //Internet Password: Robotics 0145
//    //Blue: 00,00,255 Parking 1
//    //Magenta: 255 00 255 Parking 2
//    //yellow: 255 255 00 Parking 3
//    //Alpha threshold is about 300
//    //Flat colors
//    //Drive towards the signal "How do we know when we are 2cm from the cone?" yes
//    //Read the color
//    //use if statement to decide what to do when each color is read
//
//    //if color=turquoise:
//    //then back strafe left then forwards
//    //else if color=magenta:
//    //drive forwards
//    //else if color=green:
//    //back, strafe right, forward
//    //Clockwise = -90
//    //Counter-clockwise = 90
//    //Behind = -180
//
//    // define variables for motors
//    DcMotor topRight;
//    DcMotor bottomRight;
//    DcMotor topLeft;
//    DcMotor bottomLeft;
//
//    BNO055IMU imu;
//    Orientation angles = new Orientation();
//
//    // define variable for color sensor
//    ColorSensor color;
//
//    // conversion factor for GoBila 19.2:1 gear motors
//    // from feet to ticks of motor encoder
//    double feetToTicks = (19.2*28.0*304.8) / (Math.PI*96.0);
//
//    int NUM_SAMPLES = 5;
//
//    // target position as measured on playing field
//    // T=position from starting point to where we need to strafe for parking 1/3 (in feet)
//    double T=26.5/12.0;
//    // S=position from T to left or right for parking 1/3 (in feet)
//    double S=25.0/12.0;
//    // P=position from starting point to parking position for 1/2/3 (in feet)
//    double P=38.0/12.0;
//
//
//    ElapsedTime runtime = new ElapsedTime();
//
//
//    public void runOpMode() {
//        // Hardware Maps
//        topRight = hardwareMap.dcMotor.get("TR");//control hub port 0
//        bottomRight = hardwareMap.dcMotor.get("BR");//control hub port 1
//        topLeft = hardwareMap.dcMotor.get("TL"); //control hub port 2
//        bottomLeft = hardwareMap.dcMotor.get("BL"); //control hub port 3
//        color = hardwareMap.get(ColorSensor.class, "Color");
//
//        // Set direction of all motors so that when we command
//        // the direction "forward", the values of speed are positive
//        topLeft.setDirection(DcMotor.Direction.FORWARD);
//        topRight.setDirection(DcMotor.Direction.REVERSE);
//        bottomLeft.setDirection(DcMotor.Direction.FORWARD);
//        bottomRight.setDirection(DcMotor.Direction.REVERSE);
//
//        // Stop motor and reset encoders to 0
//        topLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        topRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        bottomLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        bottomRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        // Enables motor encoders to track how much the motors have rotated
//        topLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        topRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        bottomLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        bottomRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        parameters.mode                = BNO055IMU.SensorMode.IMU;
//        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
//        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        parameters.loggingEnabled      = false;
//        imu = hardwareMap.get(BNO055IMU.class, "imu");
//        imu.initialize(parameters);
//
//        telemetry.addData("Mode", "calibrating...");
//        telemetry.update();
//
//        // make sure the imu gyro is calibrated before continuing.
//        while (!isStopRequested() && !imu.isGyroCalibrated()) {
//            sleep(50);
//            idle();
//        }
//
//        telemetry.addData("Mode", "waiting for start");
//        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
//        telemetry.update();
//
//        // Wait for start button press on Driver Station
//        waitForStart();
//
//        // While the mode is active (has not been stopped / time has not expired)
//        if (opModeIsActive()) {
//
//
//
//                rotate(90.0, 0.2);
//
//
//        }
//    }
//
//    // stop all the motors
//    public void stopMotors(){
//        topRight.setPower(0);
//        bottomRight.setPower(0);
//        topLeft.setPower(0);
//        bottomLeft.setPower(0);
//    }
//
//    // Direction=forward/backward/left/right
//    // Speed=0.0-1.0
//    // Target=Feet
//    public void drive(String direction, float speed, double target){
//        // stop motors and reset encoder to 0
//        topLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        topRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        bottomLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        bottomRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        // convert our target (in feet) to ticks that the motor can understand
//        int tickTarget = (int)(target * feetToTicks);
//
//        // set motor target positions based on direction
//        if (direction == "forward"){
//            // forward = all positive
//            topLeft.setTargetPosition(tickTarget);
//            topRight.setTargetPosition(tickTarget);
//            bottomLeft.setTargetPosition(tickTarget);
//            bottomRight.setTargetPosition(tickTarget);
//        }
//        else if(direction == "backward"){
//            // backward = all negative
//            topLeft.setTargetPosition(-tickTarget);
//            topRight.setTargetPosition(-tickTarget);
//            bottomLeft.setTargetPosition(-tickTarget);
//            bottomRight.setTargetPosition(-tickTarget);
//        }
//        else if(direction == "left"){
//            // left = topLeft and bottomRight negative,
//            // topRight and bottomLeft positive
//            topLeft.setTargetPosition(-tickTarget);
//            topRight.setTargetPosition(tickTarget);
//            bottomLeft.setTargetPosition(tickTarget);
//            bottomRight.setTargetPosition(-tickTarget);
//        }
//        else if(direction == "right"){
//            // right = opposite of left
//            topLeft.setTargetPosition(tickTarget);
//            topRight.setTargetPosition(-tickTarget);
//            bottomLeft.setTargetPosition(-tickTarget);
//            bottomRight.setTargetPosition(tickTarget);
//        }
//
//        // tell motors to run to target position
//        topLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        topRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        bottomLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        bottomRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//        // set speed based on input
//        // NOTE: all positive speeds because direction is specified by signs
//        // given to setTargetPosition
//        topRight.setPower(speed);
//        bottomRight.setPower(speed);
//        topLeft.setPower(speed);
//        bottomLeft.setPower(speed);
//
//        // wait in this loop as long as at least 1 motor is still moving
//        // motors report busy until they reach the target position
//        while (opModeIsActive() && (bottomRight.isBusy() || topRight.isBusy() || bottomLeft.isBusy() || topLeft.isBusy() )) {
//            telemetry.addData("top left", topLeft.getCurrentPosition()*(1.0/feetToTicks));
//            telemetry.addData("top right", topRight.getCurrentPosition()*(1.0/feetToTicks));
//            telemetry.addData("bottom left", bottomLeft.getCurrentPosition()*(1.0/feetToTicks));
//            telemetry.addData("bottom right", bottomRight.getCurrentPosition()*(1.0/feetToTicks));
//            telemetry.update();
//        }
//
//        // stop motors
//        stopMotors();
//    }
//    //Clockwise = -90
//    //Counter-clockwise = 90
//    //Behind = -180
//    public void rotate(double degree, double speed) {
//
////        if (degree > 0.0) {
////            telemetry.addData("rotate counter clockwise to", degree);
////        }
////        else {
////            telemetry.addData("rotate clockwise to", degree);
////        }
//        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//
//        double delta =Math.abs( degree-angles.firstAngle);
//
//        while (delta > 0.0 ){
//            telemetry.addData("rotate", delta);
//            telemetry.update();
//            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//
//           delta =Math.abs( degree-angles.firstAngle);
//        }
//        telemetry.addLine("stop rotating");
//        telemetry.update();
//
//
//    }
//    // returns an average alpha value
//    public double alphaAverage() {
//
//        // initalize alphaSum to 0
//        int alphaSum = 0;
//
//        // run for NUM_SAMPLES iterations
//        for (int i = 0; i < NUM_SAMPLES; i++) {
//            // increment alphaSum by our new alpha color sensor reading
//            alphaSum += color.alpha();
//        }
//
//        // return the average alpha value (alphaSum divided by the number of samples we take)
//        return (double)alphaSum / (double)NUM_SAMPLES;
//    }
//
//    // return an average red value
//    public double redAverage() {
//
//        // initalize redSum to 0
//        int redSum = 0;
//
//        // run for NUM_SAMPLES iterations
//        for (int i = 0; i < NUM_SAMPLES; i++) {
//            // increment redSum by our new red color sensor reading
//            redSum += color.red();
//        }
//
//        // return the average red value (redSum divided by the number of samples we take)
//        return (double)redSum / (double)NUM_SAMPLES;
//    }
//
//    // return an average blue value
//    public double blueAverage() {
//
//        // initalize blueSum to 0
//        int blueSum = 0;
//
//        // run for NUM_SAMPLES iterations
//        for (int i = 0; i < 5; i++) {
//            // increment blueSum by our new blue color sensor reading
//            blueSum += color.blue();
//        }
//
//        // return the average blue value (blueSum divided by the number of samples we take)
//        return (double)blueSum / (double)NUM_SAMPLES;
//    }
//
//    // return an average green value
//    public double greenAverage() {
//
//        // initalize greenSum to 0
//        int greenSum = 0;
//
//        // run for NUM_SAMPLES iterations
//        for (int i = 0; i < 5; i++) {
//            // increment greenSum by our new green color sensor reading
//            greenSum += color.green();
//        }
//
//        // return the average green value (greenSum divided by the number of samples we take)
//        return (double)greenSum / (double)NUM_SAMPLES;
//    }
//
//    // check if the input color sensor r,g,b values represent parking zone 1
//    public boolean isParking1(double r, double g, double b) {
//
//        // perfect reading:   r=1.00, g=0.50, b=0.70
//        // imperfect reading: r=0.70, g=1.00, b=0.900
//
//        // initalize all check variables to false
//        boolean redCheck = false;
//        boolean greenCheck = false;
//        boolean blueCheck = false;
//
//        // check if r is in the range
//        if (r >= 0.6 && r <= 1.0) {
//            redCheck = true;
//        }
//
//        // check if g is in the range
//        if (g >= 0.3 && g <= 1.0) {
//            greenCheck = true;
//        }
//
//        // check if b is in the range
//        if (b >= 0.5 && b <= 1.0) {
//            blueCheck = true;
//        }
//
//        // if all color checks are true, return true
//        // otherwise, return false
//        if (redCheck && greenCheck && blueCheck) {
//            return true;
//        } else return false;
//    }
//
//    // check if the input color sensor r,g,b values represent parking zone 3
//    public boolean isParking3(double r, double g, double b) {
//
//        // perfect reading:   r=0.56, g=1.00, b=0.27
//        // imperfect reading: r=0.58, g=1.00, b=0.64
//
//        // initalize all check variables to false
//        boolean redCheck = false;
//        boolean greenCheck = false;
//        boolean blueCheck = false;
//
//        // check if r is in the range
//        if (r >= 0.3 && r <= 0.7) {
//            redCheck = true;
//        }
//
//        // check if g is in the range
//        if (g >= 0.8 && g <= 1.0 ) {
//            greenCheck = true;
//        }
//
//        // check if b is in the range
//        if (b >= 0.1 && b <= 0.8) {
//            blueCheck = true;
//        }
//
//        // if all color checks are true, return true
//        // otherwise, return false
//        if (redCheck && greenCheck && blueCheck) {
//            return true;
//        } else return false;
//    }
//}
//
