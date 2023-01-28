package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@Autonomous(name = "AutoConeStackLeft", group = "Robot")


public class AutoConeStackLeft extends LinearOpMode {

    // CONFIGURATION
    // target positions on playing field
    double distanceToJunction = 4.0 / 12.0;          // feet
    double distanceToRotate = 0.6;          // feet
    //   // T=position from starting point to where we need to strafe for parking 1/3 (in feet)
//   double distanceToStrafe = (32.0 / 12.0) - distanceToRotate;    // feet
    // S=position from T to left or right for parking 1/3 (in feet)
    double distanceSidewaysToParking = 27.0 / 12.0;    // feet
    // P=position from starting point to parking position for 1/2/3 (in feet)
    double distanceToParkingZone = (39.0 / 12.0) - distanceToRotate;    // feet
    //R=position where the robot can rotate at the beginning of the match to score.

    //Distance to line
    double distanceToLine = (32.0 / 12.0); //feet

    //Distance to cone stack
    double distanceToConeStack = (12.0 / 12.0); //feet

    double correctionForConeReading = (1.25 / 12.0); //feet

    //Same variables from Robot, but they are negative. Using the Robot variables for lift levels, the lift tried to go down??? This fixed it.
    double coneLiftlevel = 0.08; // feet
    double smallLiftlevel = 0.86; // feet
    double ground = 0.01; //feet
    double coneStackLevel = 0.35; //feet

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
            //PART 1
            //Step 1= Score on small junction from other auto
            //Step 2= Rotate so the back of the robot is facing the signal cone
            //Step 3= Read color

            //Step 1
            robot.log("Lift Level", robot.liftLevelNames[robot.currentLiftLevel]);
            moveLiftBlocking(coneLiftlevel, liftPower);
            robot.clawClose();
            driveToPosition("left", 0.8f, distanceToRotate);
            robot.log("Step1-Close Claw");

            //Step2
            moveLiftBlocking(smallLiftlevel, liftPower);
            robot.log("Lift Level", robot.liftLevelNames[robot.currentLiftLevel]);
            robot.log("Step2-lift up to low junction position");

            //Step 3a
            robot.driveWithoutEncoder();
            rotateToAngle(141, rotateSpeed);
            robot.driveWithEncoder();
            robot.log("Step3a-Rotate counter-clockwise 130 degrees");


            //Step3b
            driveToPosition("forward", 0.3f, distanceToJunction);
            robot.log("Step3b-Drive to junction");

            //Step3c
            robot.clawOpen();
            robot.log("Step3c-Drop cone");

            //Step3d
            driveToPosition("backward", 0.3f, distanceToJunction);
            robot.log("Step3d-Drive back from junction");

            //Step 3g
            robot.clawOpen();
            robot.log("Step3g-Open claw");

            //Step4
            robot.driveWithoutEncoder();
            rotateToAngle(123, rotateSpeed);
            robot.driveStopAndReset();
            robot.driveWithEncoder();
           // driveToPosition("right", 0.3f, correctionForConeReading);
            robot.log("Step4-Rotate counter clockwise 123 degrees and then strafe right to correctionForConeReading");

            //Step3f
            moveLiftBlocking(ground, liftPower);
            robot.log("Lift Level", robot.liftLevelNames[robot.currentLiftLevel]);
            robot.log("Step3f-Lift down to ground level (0 ticks)");

            robot.driveStopAndReset();
            robot.driveWithEncoder();
            double speed = -0.1;

            double distanceDriven = 0.0;
            while (robot.alphaAverage(robot.colorSensorBack) < 200 && distanceDriven <= distanceToParkingZone) {

                distanceDriven = Math.abs(robot.topLeft.getCurrentPosition()) * robot.ticksToFeet;

                robot.log("alpha", robot.alphaAverage(robot.colorSensorBack));
                robot.log("distance driven", robot.topLeft.getCurrentPosition() * (1.0 / robot.feetToTicks));


                robot.topLeft.setPower(speed);
                robot.topRight.setPower(speed);
                robot.bottomLeft.setPower(speed);
                robot.bottomRight.setPower(speed);
            }

            robot.stopDriveMotors();

            double c = Math.abs(robot.topLeft.getCurrentPosition() * (1.0 / robot.feetToTicks));

            robot.log("c", c);


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

            /* WE ARE AT STARTING POSITION */
            //PART 2
            //Step 4= Drive backward to red line
            //Step 5= Rotate to face cone stack
            //Step 6= Drive forward to cone stack
            //Step 7= Raise lift
            //Step 8= Close claw
            //Step 9= Lift up to medium lift level
            //Step 10= Drive backward to the center of the tile
            //Step 11= Rotate to the medium junction
            //Step 12= Drive forward to touch junction
            //Step 13= Open claw
            //Step 14= Back up same distance driven before
            //Step 15= Lower lift to cone stack height
            //Step 16= Rotate back to cone stack position
            //Step 17= Lower lift to bottom
            //Step 18= Rotate so robot is facing us (backward)
            //Step 19= Strafe left, right, or stop motors depending on parking zone.

            if (robot.isParking1(redNorm, greenNorm, blueNorm)) {
                robot.log("Parking 1");
                telemetry.addLine("Parking 1");
                telemetry.update();

                //Step 4
                driveToPosition("backward", 0.4f, distanceToLine);
                telemetry.addLine("Step4-Drive distanceToLine");
                telemetry.update();
                robot.log("Step4-Drive to distanceToLine");

                //Step 5
                rotateToAngle(90, rotateSpeed);
                robot.log("Step5-Rotate 90 degrees counter clockwise");

                //Step 6
                   moveLiftBlocking(coneStackLevel, liftPower);
                robot.log("Step6-Lift Level", robot.liftLevelNames[robot.currentLiftLevel]);

                //Step 7
                driveToPosition("forward", 0.4f, distanceToConeStack);
                robot.log("Step7-Drive distanceToConeStack");

                //Step 8
                robot.clawClose();
                robot.log("Step8-Close claw");

                //Step 9
                moveLiftBlocking(robot.mediumLiftlevel, liftPower);
                robot.log("Step9-Medium Lift level");

                //Step 10
                driveToPosition("backward", 0.4f, distanceToConeStack);
                robot.log("Step10-Drive distanceToConeStack backwards");

                //Step 11
                rotateToAngle(135, rotateSpeed);
                robot.log("Step11-Rotate 135 degrees counter clockwise.");

                //Step 12
                driveToPosition("forward", 0.3f, distanceToJunction);
                robot.log("Step12-Drive distanceToJunction");

                //Step 13
                robot.clawOpen();
                robot.clawClose();

                //Step 14
                driveToPosition("backward", 0.4f, distanceToJunction);
                robot.log("Step14-Drive distanceToJunction backwards");

                //Step 15
                   moveLiftBlocking(coneStackLevel, liftPower);
                robot.log("Step15-Lift to cone stack level");

                //Step 16
                rotateToAngle(135, rotateSpeed);
                robot.log("Step16-Rotate 135 degrees counter clockwise.");

                //Step 17
                moveLiftBlocking(ground, liftPower);
                robot.log("Step17-Lift to ground (0 ticks)");

                //Step 18
                rotateToAngle(-90, rotateSpeed);
                robot.log("Step18-Rotate 90 degrees clockwise.");

                //Step 19
                driveToPosition("right", 0.4f, distanceSidewaysToParking);

                //Step 20
                stopMotors();

            } else if (robot.isParking3(redNorm, greenNorm, blueNorm)) {
                robot.log("Parking 3");

                //Step 4
                driveToPosition("backward", 0.4f, distanceToLine);
                robot.log("Step4-Drive to distanceToLine");

                //Step 5
                rotateToAngle(90, 0.3f);
                robot.log("Step5-Rotate 90 degrees counter clockwise");

                //Step 6
                   moveLiftBlocking(coneStackLevel, liftPower);
                robot.log("Step6-Lift Level", robot.liftLevelNames[robot.currentLiftLevel]);

                //Step 7
                driveToPosition("forward", 0.4f, distanceToConeStack);
                robot.log("Step7-Drive distanceToConeStack");

                //Step 8
                robot.clawClose();
                robot.log("Step8-Close claw");

                //Step 9
                moveLiftBlocking(robot.mediumLiftlevel, liftPower);
                robot.log("Step9-Medium Lift level");

                //Step 10
                driveToPosition("backward", 0.4f, distanceToConeStack);
                robot.log("Step10-Drive distanceToConeStack backwards");

                //Step 11
                rotateToAngle(135, 0.4);
                robot.log("Step11-Rotate 135 degrees counter clockwise.");

                //Step 12
                driveToPosition("forward", 0.3f, distanceToJunction);
                robot.log("Step12-Drive distanceToJunction");

                //Step 13
                robot.clawOpen();
                robot.clawClose();

                //Step 14
                driveToPosition("backward", 0.4f, distanceToJunction);
                robot.log("Step14-Drive distanceToJunction backwards");

                //Step 15
                   moveLiftBlocking(coneStackLevel, liftPower);
                robot.log("Step15-Lift to cone stack level");

                //Step 16
                rotateToAngle(135, 0.4f);
                robot.log("Step16-Rotate 135 degrees counter clockwise.");

                //Step 17
                moveLiftBlocking(ground, liftPower);
                robot.log("Step17-Lift to ground (0 ticks)");

                //Step 18
                rotateToAngle(-90, 0.4f);
                robot.log("Step18-Rotate 90 degrees clockwise.");

                //Step 19
                driveToPosition("right", 0.4f, distanceSidewaysToParking);

                //Step 20
                stopMotors();

            }
        } else {
            robot.log("Parking 2");

            //Step 4
            driveToPosition("backward", 0.4f, distanceToLine);
            robot.log("Step4-Drive to distanceToLine");

            //Step 5
            rotateToAngle(90, 0.3f);
            robot.log("Step5-Rotate 90 degrees counter clockwise");

            //Step 6
               moveLiftBlocking(coneStackLevel, liftPower);
            robot.log("Step6-Lift Level", robot.liftLevelNames[robot.currentLiftLevel]);

            //Step 7
            driveToPosition("forward", 0.4f, distanceToConeStack);
            robot.log("Step7-Drive distanceToConeStack");

            //Step 8
            robot.clawClose();
            robot.log("Step8-Close claw");

            //Step 9
            moveLiftBlocking(robot.mediumLiftlevel, liftPower);
            robot.log("Step9-Medium Lift level");

            //Step 10
            driveToPosition("backward", 0.4f, distanceToConeStack);
            robot.log("Step10-Drive distanceToConeStack backwards");

            //Step 11
            rotateToAngle(135, 0.4);
            robot.log("Step11-Rotate 135 degrees counter clockwise.");

            //Step 12
            driveToPosition("forward", 0.3f, distanceToJunction);
            robot.log("Step12-Drive distanceToJunction");

            //Step 13
            robot.clawOpen();
            robot.clawClose();

            //Step 14
            driveToPosition("backward", 0.4f, distanceToJunction);
            robot.log("Step14-Drive distanceToJunction backwards");

            //Step 15
               moveLiftBlocking(coneStackLevel, liftPower);
            robot.log("Step15-Lift to cone stack level");

            //Step 16
            rotateToAngle(135, 0.4f);
            robot.log("Step16-Rotate 135 degrees counter clockwise.");

            //Step 17
            moveLiftBlocking(ground, liftPower);
            robot.log("Step17-Lift to ground (0 ticks)");

            //Step 18
            rotateToAngle(-90, 0.4f);
            robot.log("Step18-Rotate 90 degrees clockwise.");
        }

        /* WE ARE AT PARKING POSITION */
        robot.log("WE DID IT! :D");
    }


    // stop all the motors
    public void stopMotors() {
        robot.topRight.setPower(0);
        robot.bottomRight.setPower(0);
        robot.topLeft.setPower(0);
        robot.bottomLeft.setPower(0);
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

        if (deltaAngle < -180) deltaAngle += 360;
        else if (deltaAngle > 180) deltaAngle -= 360;

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

