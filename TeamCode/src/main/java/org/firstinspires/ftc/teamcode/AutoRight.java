package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@Autonomous(name = "AutoRight", group = "Robot")


public class AutoRight extends LinearOpMode {

    // CONFIGURATION

    boolean DEBUG = true;
    int DEBUG_MS = 0;
    // target positions on playing field
    double distanceToJunction = 4.0 / 12.0;          // feet
    double distanceToRotate = 0.6;          // feet
    // T=position from starting point to where we need to strafe for parking 1/3 (in feet)
    double distanceToStrafe = (30.0 / 12.0) - distanceToRotate;    // feet
    // S=position from T to left or right for parking 1/3 (in feet)
    double distanceSidewaysToParking = 27.0 / 12.0;    // feet
    // P=position from starting point to parking position for 1/2/3 (in feet)
    double distanceToParkingZone = (39.0 / 12.0) - distanceToRotate;    // feet
    //R=position where the robot can rotate at the beginning of the match to score.

   // double correctionForConeReading = (1.25 / 12.0); //feet

    //Same variables from Robot, but they are negative. Using the Robot variables for lift levels, the lift tried to go down??? This fixed it.
    double coneLiftlevel = 0.09; // feet
    double smallLiftlevel = 0.86; // feet
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
        robot.log("Begin Auto Right");

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


        robot.log("Calibrating gyro...");


        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !robot.imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        // Wait for start button press on Driver Station
        robot.log("Waiting for start...");

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
            robot.log("Lift Level", robot.liftLevelNames[robot.currentLiftLevel]);
            moveLiftBlocking(coneLiftlevel, liftPower);
            robot.clawClose();
            driveToPosition("right", 0.8f, distanceToRotate);
            robot.log("Step1- Close claw and strafe right distanceToRotate");

            //Step2
            moveLiftBlocking(smallLiftlevel, liftPower);
            robot.log("Lift Level", robot.liftLevelNames[robot.currentLiftLevel]);
            robot.log("Step2-Lift up to smallLiftLevel");

            //Step 3a
            robot.driveWithoutEncoder();
            rotateToAngle(-123, rotateSpeed);
            robot.driveWithEncoder();
            robot.log("Step3a-Rotate clockwise 130 degrees");

            //Step3b
            driveToPosition("forward", 0.3f, distanceToJunction);
            robot.log("Step3b-Drive to junction");

            //Step3c
            robot.clawOpen();
            robot.log("Step3c-Open claw");

            //Step3d
            driveToPosition("backward", 0.3f, distanceToJunction);
            robot.log("Step3d-Drive back from junction");

            //Step 3g
            robot.clawOpen();
            robot.log("Step3g-Open claw");

            //Step4
            robot.driveWithoutEncoder();
            rotateToAngle(-128, rotateSpeed);
            robot.driveStopAndReset();
            robot.driveWithEncoder();
          //  driveToPosition("left", 0.3f, correctionForConeReading);
            robot.log("Step4-Rotate clockwise 123 degrees and then strafe left to correctionForConeReading");

            // calculate c which represents the distance from starting point to where we detected the cone
            // double c = robot.topLeft.getCurrentPosition() * (1.0 / robot.feetToTicks);

//            driveToPosition("backward", 0.3f, c);
//            robot.stopDriveMotors();

            robot.driveStopAndReset();
            robot.driveWithEncoder();
            double speed = -0.3;

            double distanceDriven = 0.0;
            while (!robot.isColorValid(robot.colorSensorBack) && distanceDriven <= distanceToParkingZone) {

                distanceDriven = Math.abs(robot.topLeft.getCurrentPosition()) * robot.ticksToFeet;

                if (DEBUG) {
                    robot.log("distance driven", distanceDriven);
                    robot.log("alpha", robot.alphaAverage(robot.colorSensorBack));
                }
                robot.topLeft.setPower(speed);
                robot.topRight.setPower(speed);
                robot.bottomLeft.setPower(speed);
                robot.bottomRight.setPower(speed);
            }

            robot.stopDriveMotors();

            double c = Math.abs(robot.topLeft.getCurrentPosition() * (1.0 / robot.feetToTicks));

            if (DEBUG) {
                robot.log("final distance driven", c);
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

            robot.log("red", redNorm);
            robot.log("green", greenNorm);
            robot.log("blue", blueNorm);

            // check which parking zone the cone represents
            if (robot.isParking1(redNorm, greenNorm, blueNorm)) {
                robot.log("Parking 1");
                telemetry.addLine("Parking 1");
                telemetry.update();

                // drive forward at 0.2 speed to position T (relative)

                driveToPosition("backward", 0.2f, distanceToStrafe - c);

                // drive left at 0.4 speed to position S (relative)
                driveToPosition("right", 0.6f, distanceSidewaysToParking);

                // drive forward at 0.2 speed to position P (relative)
                driveToPosition("backward", 0.2f, distanceToParkingZone - distanceToStrafe);

                //Step3f
                moveLiftBlocking(ground, liftPower);
                robot.log("Lift Level", robot.liftLevelNames[robot.currentLiftLevel]);
                robot.log("Step3f-Lift to ground (0 ticks)");


            } else if (robot.isParking3(redNorm, greenNorm, blueNorm)) {
                robot.log("Parking 3");
                telemetry.addLine("Parking 3");
                telemetry.update();

                // drive forward at 0.2 speed to position T (relative)
                driveToPosition("backward", 0.2f, distanceToStrafe - c);

                // drive right at 0.4 speed to position S (relative)
                driveToPosition("left", 0.6f, distanceSidewaysToParking);

                // drive forward at 0.2 speed to position P (relative)
                driveToPosition("backward", 0.2f, distanceToParkingZone - distanceToStrafe);

                //Step3f
                moveLiftBlocking(ground, liftPower);
                robot.log("Lift Level", robot.liftLevelNames[robot.currentLiftLevel]);
                robot.log("Step3f-Lift to ground (0 ticks)");

            } else {
                robot.log("Parking 2");
                telemetry.addLine("Parking 2");
                telemetry.update();

                // drive forward at 0.2 speed to position P (relative)
                driveToPosition("backward", 0.2f, distanceToParkingZone - c);

                //Step3f
                moveLiftBlocking(ground, liftPower);
                robot.log("Lift Level", robot.liftLevelNames[robot.currentLiftLevel]);
                robot.log("Step3f-Lift to ground (0 ticks)");

            }

            /* WE ARE AT PARKING POSITION */
            robot.log("WE DID IT!");
        }

        robot.destroy();
    }

    public void resetAngle() {
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
        double leftPower, rightPower;

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
        while (opModeIsActive() && Math.abs(robot.liftMotor.getCurrentPosition() - tickTarget) > 10) {
            telemetry.addData("lift motor", Math.abs(robot.liftMotor.getCurrentPosition() - tickTarget));
            telemetry.update();
        }

        robot.liftMotor.setPower(0.0f);

        return liftlevel;
    }

    // DRIVE FUNCTIONS
    // Direction=forward/backward/left/right
    // Speed=0.0-1.0
    // Target=Feet
    public void driveToPosition(String direction, float speed, double target) {
        // stop motors and reset encoder to 0
        robot.driveStopAndReset();

        // convert our target (in feet) to ticks that the motor can understand
        int tickTarget = (int) (target * robot.feetToTicks);

        // set motor target positions based on direction
        if (direction == "forward") {
            // forward = all positive
            robot.topLeft.setTargetPosition(tickTarget);
            robot.topRight.setTargetPosition(tickTarget);
            robot.bottomLeft.setTargetPosition(tickTarget);
            robot.bottomRight.setTargetPosition(tickTarget);
        } else if (direction == "backward") {
            // backward = all negative
            robot.topLeft.setTargetPosition(-tickTarget);
            robot.topRight.setTargetPosition(-tickTarget);
            robot.bottomLeft.setTargetPosition(-tickTarget);
            robot.bottomRight.setTargetPosition(-tickTarget);
        } else if (direction == "left") {
            // left = topLeft and bottomRight negative,
            // topRight and bottomLeft positive
            robot.topLeft.setTargetPosition(-tickTarget);
            robot.topRight.setTargetPosition(tickTarget);
            robot.bottomLeft.setTargetPosition(tickTarget);
            robot.bottomRight.setTargetPosition(-tickTarget);
        } else if (direction == "right") {
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
        while (opModeIsActive() && (robot.bottomRight.isBusy() || robot.topRight.isBusy() || robot.bottomLeft.isBusy() || robot.topLeft.isBusy())) {
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
}

