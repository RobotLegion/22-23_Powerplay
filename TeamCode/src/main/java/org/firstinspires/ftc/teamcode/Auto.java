package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@Autonomous(name="autonomous", group="Robot")


public class Auto extends LinearOpMode {

    Robot robot = Robot();


    //Blue: 00,00,255 Parking 1
    //Magenta: 255 00 255 Parking 2
    //yellow: 255 255 00 Parking 3
    //Alpha threshold is about 300
    //Flat colors
    //Drive towards the signal "How do we know when we are 2cm from the cone?" yes
    //Read the color
    //use if statement to decide what to do when each color is read

    //if color=turquoise:
    //then back strafe left then forwards
    //else if color=magenta:
    //drive forwards
    //else if color=green:
    //back, strafe right, forward
    //Clockwise = -90
    //Counter-clockwise = 90
    //Behind = -180


    // BNO055IMU imu;
    Orientation angles = new Orientation();
    Orientation lastAngles = new Orientation();
    double globalAngle, power = .30, correction;
    float LiftPower = 0.3f;

    // define variable for color sensor
    // ColorSensor color;

    double distanceToJunction = 0.1;            //feet
    double distanceToRotate = 0.7;

    int NUM_SAMPLES = 5;
    double Smallliftlevel = 1.0;
    double Groundliftlevel =0.0;

    // target position as measured on playing field
    // T=position from starting point to where we need to strafe for parking 1/3 (in feet)
    double T=26.5/12.0;
    // S=position from T to left or right for parking 1/3 (in feet)
    double S=25.0/12.0;
    // P=position from starting point to parking position for 1/2/3 (in feet)
    double P=38.0/12.0;
    //R=position where the robot can rotate at the beginning of the match to score.

    //liftlevel in feet. Speed in 0-1
    public void Lift(double liftlevel, float speed){

        int tickTarget = (int)(liftlevel * robot.feetToTicks);
        robot.liftMotor.setTargetPosition(tickTarget);

        // tell motors to run to target position
        robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // set speed based on lift power
        robot.liftMotor.setPower(speed);

        // wait in this loop as long as at least 1 motor is still moving
        // motors report busy until they reach the target position
        while (opModeIsActive() && (robot.liftMotor.isBusy())) {
            telemetry.addData("lift motor", robot.liftMotor.getCurrentPosition()*(1.0/robot.feetToTicks));
            telemetry.update();
        }
        robot.liftMotor.setPower(0);

    }



    ElapsedTime runtime = new ElapsedTime();

    public void resetAngle()
    {
        lastAngles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    public double getAngle() {

        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    public void rotate(int degrees, double power)
    {
        double  leftPower, rightPower;

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0)
        {   // turn right.
            leftPower = power;
            rightPower = -power;

        }
        else if (degrees > 0)
        {   // turn left.
            leftPower = -power;
            rightPower = power;
        }
        else return;

        // set power to rotate.
        robot.topRight.setPower(leftPower);
        robot.bottomLeft.setPower(rightPower);
        robot.topLeft.setPower(leftPower);
        robot.bottomRight.setPower(rightPower);




        // rotate until turn is completed.
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {}

            while (opModeIsActive() && getAngle() > degrees) {}
        }
        else    // left turn.
            while (opModeIsActive() && getAngle() < degrees) {}

        // turn the motors off.
        robot.topRight.setPower(0);
        robot.bottomLeft.setPower(0);
        robot.topLeft.setPower(0);
        robot.bottomRight.setPower(0);

        // wait for rotation to stop.
        //sleep(1000);

        // reset angle tracking on new heading.
        resetAngle();
    }

    public void withoutEncoder() {
        robot.topLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.topRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.bottomLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.bottomRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void withEncoder(){
        robot.topLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.topRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.bottomLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.bottomRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void runOpMode() {

        // INITALIZE ROBOT
        robot.init(hardwareMap);



        // Set direction of all motors so that when we command
        // the direction "forward", the values of speed are positive
        robot.topLeft.setDirection(DcMotor.Direction.FORWARD);
        robot.topRight.setDirection(DcMotor.Direction.REVERSE);
        robot.bottomLeft.setDirection(DcMotor.Direction.FORWARD);
        robot.bottomRight.setDirection(DcMotor.Direction.REVERSE);

        // Stop motor and reset encoders to 0
        robot.topLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.topRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bottomLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bottomRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Enables motor encoders to track how much the motors have rotated
        robot.topLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.topRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.bottomLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.bottomRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !robot.imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", robot.imu.getCalibrationStatus().toString());
        telemetry.update();

        // Wait for start button press on Driver Station
        waitForStart();


        // While the mode is active (has not been stopped / time has not expired)
        if (opModeIsActive()) {

            /* WE ARE AT STARTING POSITION */
            //Step 1= close claw
            //Step 2= rotate counter clockwise -135 deg
            //Step 3 = score
                //Step 3a= drive to junction
                //Step 3b= lift up to small position
                //Step 3c= open claw
                //Step 3d = backup
                //Step 3e= close claw
                //Step 3f= lower lift
            //Step 4= rotate -135 so the back of the robot faces the signal cone.
            //Step 5= Basic Auto


            //Step 1
            robot.clawClose();

            //Test before rotating so the robot doesn't hit the wall.
            drive("right", 0.6f, distanceToRotate);

            //Step 2
            withoutEncoder();
            rotate(130, 0.8);
            withEncoder();


            //Step3a
            drive("backward", 0.3f, distanceToJunction);

            //Step3b
            Lift(Smallliftlevel, LiftPower);

            //Step3c
            robot.clawOpen();

            //Step3d
            drive("forward", 0.1f, distanceToJunction);

            //Step3e
            robot.clawClose();

            //Step3f
            Lift(Groundliftlevel,LiftPower);

            //Step4
            withoutEncoder();
            rotate(135, 0.5);
           withEncoder();

//
//            rotate(90, 0.5);
//
//            // Drive forward at speed 0.1 while alpha is < 200
//            double speed = 0.3;
//            while (alphaAverage() < 200) {
//                bottomRight.setPower(speed);
//                topRight.setPower(speed);
//                bottomLeft.setPower(speed);
//                topLeft.setPower(speed);
//            }
//
//            // Once alpha >= 200, stop robot
//            stopMotors();
//
//            /* WE ARE AT "c", READ CONE */
//
//            // read cone color
//            double red = redAverage();
//            double green = greenAverage();
//            double blue = blueAverage();
//
//            // determine the max color value out of r,g,b
//            double colorMax = Math.max(Math.max(red,green),blue);
//
//            // divide each color by max value to normalize
//            double redNorm = red / colorMax;
//            double greenNorm = green / colorMax;
//            double blueNorm = blue / colorMax;
//
//            // calculate c which represents the distance from starting point
//            // to where we detected the cone
//            double c=topLeft.getCurrentPosition()*(1.0/robot.feetToTicks);
//
//            // check which parking zone the cone represents
//            if (isParking1(redNorm, greenNorm, blueNorm)){
//                telemetry.addLine("Parking 1");
//                telemetry.update();
//
//                // drive forward at 0.2 speed to position T (relative)
//                drive("forward",0.4f,T-c);
//
//                //At position T turn cw 135 deg
//                rotate(135, 0.8);
//
//                // drive left at 0.4 speed to position S (relative)
//                drive("left",0.4f,S);
//
//                // drive forward at 0.2 speed to position P (relative)
//                drive("forward",0.3f,P-T);
//            }
//            else if (isParking3(redNorm, greenNorm, blueNorm)){
//                telemetry.addLine("Parking 3");
//                telemetry.update();
//
//                // drive forward at 0.2 speed to position T (relative)
//                drive("forward",0.4f,T-c);
//
//                // drive right at 0.4 speed to position S (relative)
//                drive("right",0.4f,S);
//
//                // drive forward at 0.2 speed to position P (relative)
//                drive("forward",0.4f,P-T);
//            }
//            else {
//                telemetry.addLine("Parking 2");
//                telemetry.update();
//
//                // drive forward at 0.2 speed to position P (relative)
//                drive("forward",0.4f,P-c);
//            }
//
//            /* WE ARE AT PARKING POSITION */
//            telemetry.addLine("WE DID IT!");
//            telemetry.update();
        }
    }

    // stop all the motors
    public void stopMotors(){
        robot.topRight.setPower(0);
        robot.bottomRight.setPower(0);
        robot.topLeft.setPower(0);
        robot.bottomLeft.setPower(0);
    }

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
//            delta =Math.abs( degree-angles.firstAngle);
//        }
//        telemetry.addLine("stop rotating");
//        telemetry.update();
//
//
//    }
    // Direction=forward/backward/left/right
    // Speed=0.0-1.0
    // Target=Feet
    public void drive(String direction, float speed, double target){
        // stop motors and reset encoder to 0
        robot.topLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.topRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bottomLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.bottomRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // convert our target (in feet) to ticks that the motor can understand
        int tickTarget = (int)(target * robot.feetToTicks);

        // set motor target positions based on direction
        if (direction == "forward"){
            // forward = all positive
            robot.topLeft.setTargetPosition(tickTarget);
            robot.topRight.setTargetPosition(tickTarget);
            robot.bottomLeft.setTargetPosition(tickTarget);
            robot.bottomRight.setTargetPosition(tickTarget);
        }
        else if(direction == "backward"){
            // backward = all negative
            robot.topLeft.setTargetPosition(-tickTarget);
            robot.topRight.setTargetPosition(-tickTarget);
            robot.bottomLeft.setTargetPosition(-tickTarget);
            robot.bottomRight.setTargetPosition(-tickTarget);
        }
        else if(direction == "left"){
            // left = topLeft and bottomRight negative,
            // topRight and bottomLeft positive
            robot.topLeft.setTargetPosition(-tickTarget);
            robot.topRight.setTargetPosition(tickTarget);
            robot.bottomLeft.setTargetPosition(tickTarget);
            robot.bottomRight.setTargetPosition(-tickTarget);
        }
        else if(direction == "right"){
            // right = opposite of left
            robot.topLeft.setTargetPosition(tickTarget);
            robot.topRight.setTargetPosition(-tickTarget);
            robot.bottomLeft.setTargetPosition(-tickTarget);
            robot.bottomRight.setTargetPosition(tickTarget);
        }

        // tell motors to run to target position
        robot.topLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.topRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.bottomLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.bottomRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // set speed based on input
        // NOTE: all positive speeds because direction is specified by signs
        // given to setTargetPosition
        robot.topRight.setPower(speed);
        robot.bottomRight.setPower(speed);
        robot.topLeft.setPower(speed);
        robot.bottomLeft.setPower(speed);

        // wait in this loop as long as at least 1 motor is still moving
        // motors report busy until they reach the target position
        while (opModeIsActive() && (robot.bottomRight.isBusy() || robot.topRight.isBusy() || robot.bottomLeft.isBusy() || robot.topLeft.isBusy() )) {
            telemetry.addData("top left", robot.topLeft.getCurrentPosition()*(1.0/robot.feetToTicks));
            telemetry.addData("top right", robot.topRight.getCurrentPosition()*(1.0/robot.feetToTicks));
            telemetry.addData("bottom left", robot.bottomLeft.getCurrentPosition()*(1.0/robot.feetToTicks));
            telemetry.addData("bottom right", robot.bottomRight.getCurrentPosition()*(1.0/robot.feetToTicks));
            telemetry.update();
        }

        // stop motors
        stopMotors();
    }
    //Clockwise = -90
    //Counter-clockwise = 90
    //Behind = -180
    public void rotate(double degree, float speed) {

    }
    // returns an average alpha value
    public double alphaAverage() {

        // initalize alphaSum to 0
        int alphaSum = 0;

        // run for NUM_SAMPLES iterations
        for (int i = 0; i < NUM_SAMPLES; i++) {
            // increment alphaSum by our new alpha color sensor reading
            alphaSum += robot.colorSensorBack.alpha();
        }

        // return the average alpha value (alphaSum divided by the number of samples we take)
        return (double)alphaSum / (double)NUM_SAMPLES;
    }

    // return an average red value
    public double redAverage() {

        // initalize redSum to 0
        int redSum = 0;

        // run for NUM_SAMPLES iterations
        for (int i = 0; i < NUM_SAMPLES; i++) {
            // increment redSum by our new red color sensor reading
            redSum += robot.colorSensorBack.red();
        }

        // return the average red value (redSum divided by the number of samples we take)
        return (double)redSum / (double)NUM_SAMPLES;
    }

    // return an average blue value
    public double blueAverage() {

        // initalize blueSum to 0
        int blueSum = 0;

        // run for NUM_SAMPLES iterations
        for (int i = 0; i < 5; i++) {
            // increment blueSum by our new blue color sensor reading
            blueSum += robot.colorSensorBack.blue();
        }

        // return the average blue value (blueSum divided by the number of samples we take)
        return (double)blueSum / (double)NUM_SAMPLES;
    }

    // return an average green value
    public double greenAverage() {

        // initalize greenSum to 0
        int greenSum = 0;

        // run for NUM_SAMPLES iterations
        for (int i = 0; i < 5; i++) {
            // increment greenSum by our new green color sensor reading
            greenSum += robot.colorSensorBack.green();
        }

        // return the average green value (greenSum divided by the number of samples we take)
        return (double)greenSum / (double)NUM_SAMPLES;
    }

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
        if (g >= 0.3 && g <= 1.0) {
            greenCheck = true;
        }

        // check if b is in the range
        if (b >= 0.5 && b <= 1.0) {
            blueCheck = true;
        }

        // if all color checks are true, return true
        // otherwise, return false
        if (redCheck && greenCheck && blueCheck) {
            return true;
        } else return false;
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
        if (r >= 0.3 && r <= 0.7) {
            redCheck = true;
        }

        // check if g is in the range
        if (g >= 0.8 && g <= 1.0 ) {
            greenCheck = true;
        }

        // check if b is in the range
        if (b >= 0.1 && b <= 0.8) {
            blueCheck = true;
        }

        // if all color checks are true, return true
        // otherwise, return false
        if (redCheck && greenCheck && blueCheck) {
            return true;
        } else return false;
    }
}

