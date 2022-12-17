package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.Robot;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@Autonomous(name="autonomous", group="Robot")


public class Auto extends LinearOpMode {

    // CONFIGURATION
    // target positions on playing field
    double distanceToJunction           = 0.1;          // feet
    double distanceToRotate             = 0.7;          // feet
    // T=position from starting point to where we need to strafe for parking 1/3 (in feet)
    double distanceToCone               = 26.5/12.0;    // feet
    // S=position from T to left or right for parking 1/3 (in feet)
    double distanceSidewaysToParking    = 25.0/12.0;    // feet
    // P=position from starting point to parking position for 1/2/3 (in feet)
    double distanceToParkingZone        = 38.0/12.0;    // feet
    //R=position where the robot can rotate at the beginning of the match to score.

    // lift
    float liftPower                     = 0.5f;         // 0-1


    // instantiate a robot class
    Robot robot = new Robot();

    // setup imu angles
    Orientation angles      = new Orientation();
    Orientation lastAngles  = new Orientation();
    double      globalAngle = 0.0;


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
            driveToPosition("right", 0.6f, distanceToRotate);

            //Step 2
            robot.driveWithoutEncoder();
            rotateToAngle(130, 0.8);
            robot.driveWithEncoder();


            //Step3a
            driveToPosition("backward", 0.3f, distanceToJunction);

            //Step3b
            moveLiftBlocking(robot.smallLiftlevel, liftPower);

            //Step3c
            robot.clawOpen();

            //Step3d
            driveToPosition("forward", 0.1f, distanceToJunction);

            //Step3e
            robot.clawClose();

            //Step3f
            moveLiftBlocking(robot.groundLiftLevel, liftPower);

            //Step4
            robot.driveWithoutEncoder();
            rotateToAngle(135, 0.5);
            robot.driveWithEncoder();


            rotateToAngle(90, 0.5);

            // Drive forward at speed 0.1 while alpha is < 200
            double speed = 0.3;
            while (robot.alphaAverage(robot.colorSensorBack) < 200) {
                robot.setDrivePower(speed);
            }

            // Once alpha >= 200, stop robot
            robot.stopDriveMotors();

            /* WE ARE AT "c", READ CONE */

            // read cone color
            double red = robot.redAverage(robot.colorSensorBack);
            double green = robot.greenAverage(robot.colorSensorBack);
            double blue = robot.blueAverage(robot.colorSensorBack);

            // determine the max color value out of r,g,b
            double colorMax = Math.max(Math.max(red,green),blue);

            // divide each color by max value to normalize
            double redNorm = red / colorMax;
            double greenNorm = green / colorMax;
            double blueNorm = blue / colorMax;

            // calculate c which represents the distance from starting point
            // to where we detected the cone
            double c=robot.topLeft.getCurrentPosition()*(1.0/robot.feetToTicks);

            // check which parking zone the cone represents
            if (isParking1(redNorm, greenNorm, blueNorm)){
                telemetry.addLine("Parking 1");
                telemetry.update();

                // drive forward at 0.2 speed to position T (relative)
                driveToPosition("forward",0.4f,distanceToParkingZone-c);

                //At position T turn cw 135 deg
                rotateToAngle(135, 0.8);

                // drive left at 0.4 speed to position S (relative)
                driveToPosition("left",0.4f,distanceSidewaysToParking);

                // drive forward at 0.2 speed to position P (relative)
                driveToPosition("forward",0.3f,distanceToParkingZone-c);
            }
            else if (isParking3(redNorm, greenNorm, blueNorm)){
                telemetry.addLine("Parking 3");
                telemetry.update();

                // drive forward at 0.2 speed to position T (relative)
                driveToPosition("forward",0.4f,distanceToCone-c);

                // drive right at 0.4 speed to position S (relative)
                driveToPosition("right",0.4f,distanceSidewaysToParking);

                // drive forward at 0.2 speed to position P (relative)
                driveToPosition("forward",0.4f,distanceToParkingZone-c);
            }
            else {
                telemetry.addLine("Parking 2");
                telemetry.update();

                // drive forward at 0.2 speed to position P (relative)
                driveToPosition("forward",0.4f,distanceToParkingZone-c);
            }

            /* WE ARE AT PARKING POSITION */
            telemetry.addLine("WE DID IT!");
            telemetry.update();
        }
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
    public void resetAngle() {
        lastAngles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
    }
    public void rotateToAngle(int degrees, double power) {
        double  leftPower, rightPower;

        // restart imu movement tracking.
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
        };

        // set power to rotate.
        robot.topRight.setPower(rightPower);
        robot.bottomLeft.setPower(leftPower);
        robot.topLeft.setPower(leftPower);
        robot.bottomRight.setPower(rightPower);

        // rotate until turn is completed.
        if (degrees < 0) {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0) {}

            while (opModeIsActive() && getAngle() > degrees) {}
        } else {    // left turn.
            while (opModeIsActive() && getAngle() < degrees) {}
        }

        // turn the motors off.
        robot.stopDriveMotors();

        // reset angle tracking on new heading.
        resetAngle();
    }

    // LIFT FUNCTIONS
    // move lift to liftLevel (double) at speed
    public void moveLiftBlocking(double liftlevel, float speed){

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
            telemetry.addData("top left", robot.topLeft.getCurrentPosition()*(1.0/robot.feetToTicks));
            telemetry.addData("top right", robot.topRight.getCurrentPosition()*(1.0/robot.feetToTicks));
            telemetry.addData("bottom left", robot.bottomLeft.getCurrentPosition()*(1.0/robot.feetToTicks));
            telemetry.addData("bottom right", robot.bottomRight.getCurrentPosition()*(1.0/robot.feetToTicks));
            telemetry.update();
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

