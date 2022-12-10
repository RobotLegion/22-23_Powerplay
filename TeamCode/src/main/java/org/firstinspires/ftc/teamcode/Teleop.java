package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.Robot;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.ColorSensor;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.ButtonReader;

// started with 475!

@TeleOp(name = "Fy22TeleOp", group = "TeleOp" )
public class Teleop extends LinearOpMode {

    // CONFIGURATION
    double speed            = 1.0;
    double speedfactor      = 0.1;
    float  liftPower        = 0.5f;


    // STATE
    boolean liftIsMoving    = false;
    float   gamepad1LeftY   = 0.0f;
    float   gamepad1LeftX   = 0.0f;
    float   gamepad1RightX  = 0.0f;
    float   gamepad2RightY  = 0.0f;

    // setup robot class
    Robot robot = new Robot();


    @Override
    public void runOpMode() throws InterruptedException {

        // initalize robot
        robot.init(hardwareMap);

       
        // setup lift motor
        robot.liftMotor.setDirection(DcMotor.Direction.REVERSE);
        robot.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // setup gamepad extension class
        GamepadEx myGamepad2 = new GamepadEx(gamepad2);
        GamepadEx myGamepad1 = new GamepadEx(gamepad1);

        // Wait for start button press on Driver Station
        telemetry.addLine("Waiting for start...");
        telemetry.update();
        waitForStart();

        // open claw
        robot.clawOpen();

       

        while (opModeIsActive()) {

            // read current gamepad values
            float   gamepad1LeftY   = gamepad1.left_stick_y;
            float   gamepad1LeftX   = -gamepad1.left_stick_x;
            float   gamepad1RightX  = -gamepad1.right_stick_x;
            float   gamepad2RightY  = -gamepad2.right_stick_y;

            // update gamepad extension state
            myGamepad1.readButtons();
            myGamepad2.readButtons();


            // check if gamepad2 LB was pressed, move lift up!
            if (myGamepad2.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                //decrement currentliftlevel
                if (Currentliftlevel > 0) {
                    Currentliftlevel--;
                }
                moveLiftNonBlocking(liftLevels[Currentliftlevel], 0.7f);
            }

            // check if gamepad2 RB was pressed, move lift down!
           if (myGamepad2.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                //incrementing currentliftlevel
                if (Currentliftlevel < liftLevels.length - 1) {
                    Currentliftlevel++;
                }
                moveLiftNonBlocking(liftLevels[Currentliftlevel], 0.5f);
           }

            //TODO: Use liftIsMoving variable
            if (LiftMotor.isBusy()) {
                telemetry.addData("lift", LiftMotor.getCurrentPosition() * (1.0 / feetToTicks));

                telemetry.update();
            } else {
                LiftMotor.setPower(0);
            }

            // TODO: disable this if liftIsMoving is true
            if (Math.abs(gamepad2RightY) > 0.05) {
                liftPower = (gamepad2RightY * 0.2) + Math.copySign(0.5, gamepad2RightY);
            } else {
                liftPower = 0.0;
            }

            // TODO: this needs to be cleaned up
            double currentFeet = LiftMotor.getCurrentPosition() * ticksToFeet;
            if (myGamepad2.isDown(GamepadKeys.Button.B)) {   // PRESS B TO OVERRIDE MIN AND MAX SAFTEY
                // if ( (currentFeet <= 0.0 && LiftPower < 0.0) || (currentFeet >= Mediumliftlevel && LiftPower > 0.0) )
                // {
                //     LiftPower = 0.0;
                // }
                LiftMotor.setPower(LiftPower);
            }


            // check if gamepad2 LB
            // TODO: update this to trigger!
            if (myGamepad2.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                if (claw.getPosition() > 0.5) {
                    robot.clawOpen();
                } else {
                    robot.clawClose();
                }
            }

            //Mechanum formulas
            double TopRightSpeed = gamepad1LeftY + gamepad1LeftX + gamepad1RightX; //Combines the inputs of the sticks to clip their output to a value between 1 and -1
            double TopLeftSpeed = -gamepad1LeftY + gamepad1LeftX + gamepad1RightX; //Combines the inputs of the sticks to clip their output to a value between 1 and -1
            double BottomRightSpeed = gamepad1LeftY - gamepad1LeftX + gamepad1RightX; //Combines the inputs of the sticks to clip their output to a value between 1 and -1
            double BottomLeftSpeed = -gamepad1LeftY - gamepad1LeftX + gamepad1RightX; //Combines the inputs of the sticks to clip their output to a value between 1 and -1

            // sets speed
            //I changed 3 to 2 in an attempt to make the robot drive slower
            double topLeftCorrectedSpeed = Range.clip(Math.pow(TopRightSpeed, 3), -speed, speed); //Slows down the motor and sets its max/min speed to the double "speed"
            double topRightCorrectedSpeed = Range.clip(Math.pow(TopLeftSpeed, 3), -speed, speed); //Slows down the motor and sets its max/min speed to the double "speed"
            double bottomLeftCorrectedSpeed = Range.clip(Math.pow(BottomRightSpeed, 3), -speed, speed); //Slows down the motor and sets its max/min speed to the double "speed"
            double bottomRightCorrectedSpeed = Range.clip(Math.pow(BottomLeftSpeed, 3), -speed, speed); //Slows down the motor and sets its max/min speed to the double "speed"

            if (myGamepad1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.05) {
                speedfactor = 0.8;
            } else {
                speedfactor = 0.5;
            }
            topRight.setPower(topRightCorrectedSpeed * speedfactor);
            bottomRight.setPower(bottomRightCorrectedSpeed * speedfactor);
            topLeft.setPower(topLeftCorrectedSpeed * speedfactor);
            bottomLeft.setPower(bottomLeftCorrectedSpeed * speedfactor);



            // TODO: is this required
            robot.liftMotor.setPower(liftPower);

            double redAvg       = robot.redAverage(robot.colorSensorBack)
            double greenAvg     = (double) greenSum / 5.0;
            double blueAvg      = (double) blueSum / 5.0;
            double alphaAvg     = (double) alphaSum / 5.0;
       }

       


       

       


       // It intialize the sum to 0
       int redSum = 0;
       int greenSum = 0;
       int blueSum = 0;
       int alphaSum = 0;

       // It adds the color sensor readings of Red, Green, Blue, and Alpha
       for (int i = 0; i < 5; i++) {
           redSum += color.red();
           greenSum += color.green();
           blueSum += color.blue();
           alphaSum += color.alpha();
       }

       //It div all the number to find the average
       double redAvg = (double) redSum / 5.0;
       double greenAvg = (double) greenSum / 5.0;
       double blueAvg = (double) blueSum / 5.0;
       double alphaAvg = (double) alphaSum / 5.0;

       // Findng the max of r,g and b
       double colorMax = Math.max(Math.max(redAvg, greenAvg), blueAvg);

       // dividing the colors by the max to get the norm
       double redNorm = redAvg / colorMax;
       double greenNorm = greenAvg / colorMax;
       double blueNorm = blueAvg / colorMax;

       // Printing out the color values
       telemetry.addData("Red norm", redNorm);
       telemetry.addData("Green norm", greenNorm);
       telemetry.addData("Blue norm", blueNorm);
       telemetry.addData("Red", redAvg);
       telemetry.addData("Green", greenAvg);
       telemetry.addData("Blue", blueAvg);
       telemetry.addData("Alpha", alphaAvg);
       telemetry.addData("encoder-top-right", topRight.getCurrentPosition());
       telemetry.addData("LiftMotor", LiftMotor.getCurrentPosition());


       if (alphaAvg >= 300.0) {
           // color sensor is valid

           // Takes the norm and detects the is statements
           boolean poleCheck = isPole(redNorm, greenNorm, blueNorm);
           boolean redConeCheck = isRedCone(redNorm, greenNorm, blueNorm);
           boolean blueConeCheck = isBlueCone(redNorm, greenNorm, blueNorm);

           //Prints out the results
           telemetry.addData("Pole", poleCheck);
           telemetry.addData("RedCone", redConeCheck);
           telemetry.addData("BlueCone", blueConeCheck);
       } else {
           // color sensor is not valid
           telemetry.addLine("color sensor invaild");
       }

       // push telemetry update
       telemetry.update();


//minimum color value
       //green, magenta, turquoise
       //blue & red 2


   }














    // LIFT FUNCTIONS

    // set lift to target position and then return, do not block in function
    public void moveLiftNonBlocking(double liftlevel, float speed) {

        int tickTarget = (int)(liftlevel * feetToTicks);
        robot.liftMotor.setTargetPosition(tickTarget);

        // tell motors to run to target position
        robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // set speed based on lift power
        robot.liftMotor.setPower(speed);

        // set global variable liftIsMoving to true to indicate a lift command is being executed
        liftIsMoving = true;
    }


    // COLOR SENSOR FUNCTIONS
    //color sensor for seeing red cone, using a range of RGB values focusing on red and green
    public boolean isRedCone(double r, double g, double b) {

        // initalize all check variables to false
        boolean redCheck = false;
        boolean greenCheck = false;
        boolean blueCheck = false;

        // if r is in the range
        if (r >= 0.9 && r <= 1.0) {
            redCheck=true;
        }

        // if g is in the range
        if (g >= 0.4 && g <= 0.7) {
            greenCheck=true;
        }

        // if b is in the range
        if (b >= 0.0 && b <= 0.4) {
            blueCheck=true;
        }

        // if all color checks are true, return true
        // otherwise, return false
        if (redCheck && greenCheck && blueCheck) {
            return true;
        } else {
            return false;
        }
    }

    //color sensor for seeing blue cone, using a range of RGB values focusing on red and green
    public boolean isBlueCone(double r, double g, double b) {

        // initalize all check variables to false
        boolean redCheck = false;
        boolean greenCheck = false;
        boolean blueCheck = false;

        // if r is in the range
        if (r >= 0.0 && r <= 0.8) {
            redCheck=true;
        }

        // if g is in the range
        if (g >= 0.5 && g <= 1.0) {
            greenCheck=true;
        }

        // if b is in the range
        if (b >= 0.7 && b <= 1.0) {
            blueCheck=true;
        }

        // if all color checks are true, return true
        // otherwise, return false
        if (redCheck && greenCheck && blueCheck){
            return true;
        } else {
            return false;
        }
    }

    //color sensor for seeing yellow poles, using a range of RGB values focusing on red and green
    public boolean isPole(double r, double g, double b) {

        // initalize all check variables to false
        boolean redCheck = false;
        boolean greenCheck = false;
        boolean blueCheck = false;

        // if r is in the range
        if (r >= 0.6 && r <= 0.9) {
            redCheck=true;
        }

        // if g is in the range
        if (g >= 0.8 && g <= 1.0) {
            greenCheck=true;
        }

        // if b is in the range
        if (b >= 0.0 && b <= 0.5) {
            blueCheck=true;
        }

        // if all color checks are true, return true
        // otherwise, return false
        if (redCheck && greenCheck && blueCheck){
            return true;
        } else {
            return false;
        }
    }

}


