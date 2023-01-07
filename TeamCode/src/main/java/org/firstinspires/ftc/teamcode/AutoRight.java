package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@Autonomous(name="AutoRight", group="Robot")


public class AutoRight extends LinearOpMode {

    // CONFIGURATION

    boolean DEBUG = true;
    int DEBUG_MS = 0;
    // target positions on playing field
    double distanceToJunction = 4.0/12.0;          // feet
    double distanceToRotate = 0.6;          // feet
    // T=position from starting point to where we need to strafe for parking 1/3 (in feet)
    double distanceToStrafe = (32.0 / 12.0) - distanceToRotate;    // feet
    // S=position from T to left or right for parking 1/3 (in feet)
    double distanceSidewaysToParking = 27.0 / 12.0;    // feet
    // P=position from starting point to parking position for 1/2/3 (in feet)
    double distanceToParkingZone = (39.0 / 12.0) - distanceToRotate;    // feet
    //R=position where the robot can rotate at the beginning of the match to score.

    double correctionForConeReading = (1.0/12.0); //feet

    //Same variables from Robot, but they are negative. Using the Robot variables for lift levels, the lift tried to go down??? This fixed it.
    double coneLiftlevel = 0.08; // feet
    double smallLiftlevel = 0.83; // feet
    double ground = 0.01; //feet

    float rotateSpeed = 0.3f;

    // lift
    float liftPower = 0.8f;         // 0-1


    // instantiate a robot class
    Robot robot = new Robot();

    // setup imu angles
//    Orientation angles      = new Orientation();
    Orientation lastAngles = new Orientation();
    double globalAngle = 0.0;


    public void runOpMode() {

        // INITALIZE ROBOT
        robot.init(hardwareMap);

        // Set direction of all motors so that when we command
        // the direction "forward", the values of speed are positive
        robot.topLeft.setDirection(DcMotor.Direction.REVERSE);
        robot.topRight.setDirection(DcMotor.Direction.FORWARD);
        robot.bottomLeft.setDirection(DcMotor.Direction.REVERSE);
        robot.bottomRight.setDirection(DcMotor.Direction.FORWARD);

        robot.liftMotor.setDirection(DcMotor.Direction.REVERSE);
        // tell lift motor to run with encoder
        robot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Stop motor and reset encoders to 0
        robot.driveStopAndReset();

        // Enables motor encoders to track how much the motors have rotated
        robot.driveWithEncoder();

        telemetry.addLine("Calibrating gyro...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !robot.imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        // Wait for start button press on Driver Station
        telemetry.addLine("Waiting for start...");
        telemetry.update();
        waitForStart();

        // While the mode is active (has not been stopped / time has not expired)
        if (opModeIsActive()) {

            /* WE ARE AT STARTING POSITION */
            //Step 1= close claw
            //Step 2= rotate counter clockwise 135 deg
            //Step 3 = score
            //Step 3a= drive to junction
            //Step 3b= lift up to small position
            //Step 3c= open claw
            //Step 3d = backup
            //Step 3e= close claw
            //Step 3f= lower lift
            //Step 4= rotate 135 so the back of the robot faces the signal cone.
            //Step 5= Basic Auto


            //Step 1
            telemetry.addData("Lift Level", robot.liftLevelNames[robot.currentLiftLevel]);
            telemetry.update();
            moveLiftBlocking(coneLiftlevel, liftPower);
            robot.clawClose();
            driveToPosition("right", 0.8f, distanceToRotate);
            telemetry.addLine("Step1");
            telemetry.update();


            //Step 2
            robot.driveWithoutEncoder();
            rotateToAngle(-130, rotateSpeed);
            robot.driveWithEncoder();
            telemetry.addLine("Step2");
            telemetry.update();


            //Step3a
            moveLiftBlocking(smallLiftlevel, liftPower);
            telemetry.addData("Lift Level", robot.liftLevelNames[robot.currentLiftLevel]);
            telemetry.addLine("Step3");
            telemetry.update();


            //Step3b
            driveToPosition("forward", 0.3f, distanceToJunction);
            telemetry.addLine("Step3b");
            telemetry.update();

            //Step3c
            robot.clawOpen();
            telemetry.addLine("Step3c");
            telemetry.update();

            //Step3d
            driveToPosition("backward", 0.3f, distanceToJunction);
            telemetry.addLine("Step3d");
            telemetry.update();

            //Step3f
            moveLiftBlocking(ground, liftPower);
            telemetry.addData("Lift Level", robot.liftLevelNames[robot.currentLiftLevel]);
            telemetry.addLine("Step3f");
            telemetry.update();

            //Step 3g
            robot.clawOpen();

            //Step4
            robot.driveWithoutEncoder();
            rotateToAngle(-123, rotateSpeed);
            robot.driveStopAndReset();
            robot.driveWithEncoder();
            driveToPosition("right", 0.3f, correctionForConeReading);
            telemetry.addLine("Step4");
            telemetry.addData("red", robot.redAverage(robot.colorSensorBack));
            telemetry.addData("green", robot.greenAverage(robot.colorSensorBack));
            telemetry.addData("blue", robot.blueAverage(robot.colorSensorBack));
            telemetry.addData("alpha", robot.alphaAverage(robot.colorSensorBack));
            telemetry.update();

            //Step 5



            // calculate c which represents the distance from starting point to where we detected the cone
           // double c = robot.topLeft.getCurrentPosition() * (1.0 / robot.feetToTicks);

//            driveToPosition("backward", 0.3f, c);
//            robot.stopDriveMotors();

            robot.driveStopAndReset();
            robot.driveWithEncoder();
            double speed = -0.1;

            double distanceDriven = 0.0;
            while (robot.alphaAverage(robot.colorSensorBack) < 200 && distanceDriven <= distanceToParkingZone) {

                distanceDriven = Math.abs(robot.topLeft.getCurrentPosition()) * robot.ticksToFeet;

                if (DEBUG) {
                    telemetry.addData("alpha", robot.alphaAverage(robot.colorSensorBack));
                    telemetry.addData("distance driven", robot.topLeft.getCurrentPosition() * (1.0 / robot.feetToTicks));
                    telemetry.update();
                }
                robot.topLeft.setPower(speed);
                robot.topRight.setPower(speed);
                robot.bottomLeft.setPower(speed);
                robot.bottomRight.setPower(speed);
            }
            robot.stopDriveMotors();

            double c= Math.abs(robot.topLeft.getCurrentPosition()*(1.0/robot.feetToTicks));

            if (DEBUG) {
                telemetry.addData("c", c);
                telemetry.update();
            }


//            double speed = -0.3;
//            double distanceDriven = Math.abs(robot.topLeft.getCurrentPosition())    * robot.ticksToFeet;
//            telemetry.addData("distance", distanceDriven);
//
////            while ((robot.alphaAverage(robot.colorSensorBack) < 200) || (distanceDriven < distanceToParkingZone)) {
////                distanceDriven = Math.abs(robot.topLeft.getCurrentPosition())  * robot.ticksToFeet;
////                telemetry.addData("distance", distanceDriven);
////                telemetry.update();
////                robot.setDrivePower(speed);
////            }
//            if ((robot.alphaAverage(robot.colorSensorBack) < 200) || (distanceDriven < distanceToParkingZone)) {
//                telemetry.addData("distance", distanceDriven);
//                telemetry.update();
//                robot.setDrivePower(speed);
//            } else {
//                robot.stopDriveMotors();
//            }
//            // Once alpha >= 200, stop robot
//            robot.stopDriveMotors();

            /* WE ARE AT "c", READ CONE */

            // read cone color
            double red = robot.redAverage(robot.colorSensorBack);
            double green = robot.greenAverage(robot.colorSensorBack);
            double blue = robot.blueAverage(robot.colorSensorBack);

            // determine the max color value out of r,g,b
            double colorMax = Math.max(Math.max(red, green), blue);

            // divide each color by max value to normalize
            double redNorm = red / colorMax;
            double greenNorm = green / colorMax;
            double blueNorm = blue / colorMax;


            // check which parking zone the cone represents
            if (isParking1(redNorm, greenNorm, blueNorm)) {
                telemetry.addLine("Parking 1");

                if (DEBUG) {
                    telemetry.addData("red", redNorm);
                    telemetry.addData("green", greenNorm);
                    telemetry.addData("blue", blueNorm);
                    telemetry.addData("alpha", robot.alphaAverage(robot.colorSensorBack));
                    telemetry.update();
                    sleep(DEBUG_MS);
                }


                // drive forward at 0.2 speed to position T (relative)
                if (DEBUG) {
                    telemetry.addLine("distanceToStrafe");
                    telemetry.update();
                    sleep(DEBUG_MS);
                }
                driveToPosition("backward", 0.2f, distanceToStrafe - c);

                if (DEBUG) {
                    telemetry.addLine("distanceSidewaysToParking");
                    telemetry.update();
                    sleep(DEBUG_MS);
                }
                // drive left at 0.4 speed to position S (relative)
                driveToPosition("right", 0.6f, distanceSidewaysToParking);

                if (DEBUG) {
                    telemetry.addLine("distanceToParkingZone");
                    telemetry.update();
                    sleep(DEBUG_MS);
                }
                // drive forward at 0.2 speed to position P (relative)
                driveToPosition("backward", 0.2f, distanceToParkingZone - distanceToStrafe);

            } else if (isParking3(redNorm, greenNorm, blueNorm)) {
                telemetry.addLine("Parking 3");
                telemetry.update();

                // drive forward at 0.2 speed to position T (relative)
                if (DEBUG) {
                    telemetry.addLine("distanceToStrafe");
                    telemetry.update();
                    sleep(DEBUG_MS);
                }

                driveToPosition("backward", 0.2f, distanceToStrafe - c);

                // drive right at 0.4 speed to position S (relative)

                if (DEBUG) {
                    telemetry.addLine("distanceSidewaysToParking");
                    telemetry.update();
                    sleep(DEBUG_MS);
                }

                driveToPosition("left", 0.6f, distanceSidewaysToParking);

                // drive forward at 0.2 speed to position P (relative)
                if (DEBUG) {
                    telemetry.addLine("distanceToParkingZone");
                    telemetry.update();
                    sleep(DEBUG_MS);
                }
                telemetry.addLine("distanceToParkingZone");
                telemetry.update();
                driveToPosition("backward", 0.2f, distanceToParkingZone - distanceToStrafe);

            } else {
                telemetry.addLine("Parking 2");
                telemetry.update();

                // drive forward at 0.2 speed to position P (relative)
                if (DEBUG) {
                    telemetry.addLine("distanceToParkingZone");
                    telemetry.update();
                    sleep(DEBUG_MS);
                }
                driveToPosition("backward", 0.2f, distanceToParkingZone - c);

            }

            /* WE ARE AT PARKING POSITION */
            telemetry.addLine("WE DID IT!");
            telemetry.update();
        }
    }

    // stop all the motors
    public void stopMotors() {
        robot.topRight.setPower(0);
        robot.bottomRight.setPower(0);
        robot.topLeft.setPower(0);
        robot.bottomLeft.setPower(0);
    }

    public void resetAngle(){
        lastAngles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0.0;
    }

    // ROTATION FUNCTIONS
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
    public void rotateToAngle(int degrees, double power) {
        double  leftPower, rightPower;

        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0) {          // turn right.
            leftPower = power;
            rightPower = -power;

        } else if (degrees > 0) {   // turn left.
            leftPower = -power;
            rightPower = power;
        } else {                    // do not rotate at all
            return;
        }

        // set power to rotate.
        robot.topRight.setPower(rightPower);
        robot.bottomLeft.setPower(leftPower);
        robot.topLeft.setPower(leftPower);
        robot.bottomRight.setPower(rightPower);

        // rotate until turn is completed.
        if (degrees < 0) {  // CW
            while (opModeIsActive() && getAngle() > degrees) {
                telemetry.addData("angle", globalAngle);
                telemetry.update();
            }
        } else {    // left turn. CCW
            while (opModeIsActive() && getAngle() < degrees) {
                telemetry.addData("angle", globalAngle);
                telemetry.addData("degrees", degrees);
                telemetry.update();

            }
        }

        // turn the motors off.
        robot.stopDriveMotors();

    }

    // LIFT FUNCTIONS
    // move lift to liftLevel (double) at speed
    public double moveLiftBlocking(double liftlevel, float speed) {

        int tickTarget = (int) (liftlevel * robot.feetToTicks);

        // set speed based on lift power
        if (tickTarget > robot.liftMotor.getCurrentPosition()) {
            robot.liftMotor.setPower(speed);
        } else {
            robot.liftMotor.setPower(-speed);
        }

        // wait in this loop as long as at least 1 motor is still moving
        // motors report busy until they reach the target position
        while (opModeIsActive() && Math.abs(robot.liftMotor.getCurrentPosition()-tickTarget) > 10) {
            telemetry.addData("lift motor", Math.abs(robot.liftMotor.getCurrentPosition()-tickTarget));
            telemetry.update();
        }

        robot.liftMotor.setPower(0.0f);

        return liftlevel;
    }

    // DRIVE FUNCTIONS
    // Direction=forward/backward/left/right
    // Speed=0.0-1.0
    // Target=Feet
    public void driveToPosition(String direction, float speed, double target){
        // stop motors and reset encoder to 0
        robot.driveStopAndReset();

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

        // tell motors to be in run to target position mode
        robot.driveRunToPosition();

        // set drive speed
        robot.setDrivePower(speed);

        // wait in this loop as long as at least 1 motor is still moving
        // motors report busy until they reach the target position
        while (opModeIsActive() && (robot.bottomRight.isBusy() || robot.topRight.isBusy() || robot.bottomLeft.isBusy() || robot.topLeft.isBusy() )) {
//            telemetry.addData("top left", robot.topLeft.getCurrentPosition()*(1.0/robot.feetToTicks));
//            telemetry.addData("top right", robot.topRight.getCurrentPosition()*(1.0/robot.feetToTicks));
//            telemetry.addData("bottom left", robot.bottomLeft.getCurrentPosition()*(1.0/robot.feetToTicks));
//            telemetry.addData("bottom right", robot.bottomRight.getCurrentPosition()*(1.0/robot.feetToTicks));
//            telemetry.addData("alpha", robot.alphaAverage(robot.colorSensorBack));
//            telemetry.addData("red", robot.redAverage(robot.colorSensorBack));
//            telemetry.addData("green", robot.greenAverage(robot.colorSensorBack));
//            telemetry.addData("blue", robot.blueAverage(robot.colorSensorBack));
//            telemetry.update();
        }

        // stop motors
        robot.stopDriveMotors();
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

