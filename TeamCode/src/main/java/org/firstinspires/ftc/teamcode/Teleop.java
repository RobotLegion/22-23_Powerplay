package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.Robot;

//import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
//import java.util.concurrent.TimeUnit;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
//import org.firstinspires.ftc.robotcore.external.ClassFactory;
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.ButtonReader;

@TeleOp(name = "Fy22TeleOp", group = "TeleOp" )
public class Teleop extends LinearOpMode {

    // CONFIGURATION
    boolean DEBUG           = true;
    double speed            = 1.0;
    double speedFactor;
    float  liftPower        = 0.6f;
    float  liftButtonSpeed  = 0.85f;

    // STATE
    boolean liftIsMoving    = false;
    float   gamepad1LeftY   = 0.0f;
    float   gamepad1LeftX   = 0.0f;
    float   gamepad1RightX  = 0.0f;
    float   gamepad2RightY  = 0.0f;


//    private static final String VUFORIA_KEY = "AUz/xEr/////AAABmZjEf3Mc1kGYkOsXVF3u1Gl8gO6qZozGZxty6mcO/xE35elxxgMBwh4/zzwC9Dh4EPKvDexbQAVpjQJzz+Cx+PMYbKiPfvJNsyHDoJkWCPC1skmjKJq/4ctLkD1zGtWPhVUsdGK9ib6ze346j5nHgoFwzoi4SAITZUfQZEj2ccyiWs3zvY2DzbL/QgXrk391epqrpmB6y96vnvCsTUYA6i1y8pg7TZmjUBNWC/3PMr0EHBAFzu+cgtMWVD2sjR9XYcyh9eCRKFNq1aZwikL2P2F4Px5eyujkCVBsnQ0N+dNBo/UCREIF2az5iJY/x+qnrr8aZ2Rj1Gri12gHuKLT7BWS73HKsC9XVURurHz9RmJs";
//
//    // Declare class members
//    private VuforiaLocalizer vuforia    = null;
//    private WebcamName webcamName       = null;
//
//    ExposureControl myExposureControl;  // declare exposure control object
//    long minExp;
//    long maxExp;
//    long curExp;            // exposure is duration, in time units specified
//
//    GainControl myGainControl;      // declare gain control object
//    int minGain;
//    int maxGain;
//    int curGain;
//    boolean wasSetGainSuccessful;   // returned from setGain()
//
//

    // setup robot class
    Robot robot = new Robot();

    @Override
    public void runOpMode() throws InterruptedException {

        // initalize robot
        robot.init(hardwareMap);

        // Stop motor and reset encoders to 0
        robot.driveStopAndReset();

        // Enables motor encoders to track how much the motors have rotated
        robot.driveWithoutEncoder();

        // setup lift motor
        robot.liftMotor.setDirection(DcMotor.Direction.REVERSE);
        robot.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // setup gamepad extension class
        GamepadEx myGamepad2 = new GamepadEx(gamepad2);
        GamepadEx myGamepad1 = new GamepadEx(gamepad1);


//        telemetry.setMsTransmissionInterval(50);
//
//        // Connect to the webcam, using exact name per robot Configuration.
//        webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
//
//        /*
//         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
//         * We pass Vuforia the handle to a camera preview resource (on the RC screen).
//         */
//
//        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
//        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
//        // VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
//
//        parameters.vuforiaLicenseKey = VUFORIA_KEY;
//
//        // We also indicate which camera we wish to use.
//        parameters.cameraName = webcamName;
//
//        // Assign the Vuforia engine object.
//        vuforia = ClassFactory.getInstance().createVuforia(parameters);
//
//        // Assign the exposure and gain control objects, to use their methods.
//        myExposureControl = vuforia.getCamera().getControl(ExposureControl.class);
//        myGainControl = vuforia.getCamera().getControl(GainControl.class);
//
//        // Display exposure features and settings of this webcam.
//    //    checkExposureFeatures();
//
//        // Retrieve from webcam its current exposure and gain values.
//        curExp = myExposureControl.getExposure(TimeUnit.MILLISECONDS);
//        curGain = myGainControl.getGain();

//        // Display mode and starting values to user.
//        telemetry.addLine("\nTouch Start arrow to control webcam Exposure and Gain");
//        telemetry.addData("\nCurrent exposure mode", myExposureControl.getMode());
//        telemetry.addData("Current exposure value", curExp);
//        telemetry.addData("Current gain value", curGain);
//        telemetry.update();
//

//        // Get webcam exposure limits.
//        minExp = myExposureControl.getMinExposure(TimeUnit.MILLISECONDS);
//        maxExp = myExposureControl.getMaxExposure(TimeUnit.MILLISECONDS);
//
//        // Get webcam gain limits.
//        minGain = myGainControl.getMinGain();
//        maxGain = myGainControl.getMaxGain();
//
//        // Change mode to Manual, in order to control directly.
//        // A non-default setting may persist in the camera, until changed again.
//        myExposureControl.setMode(ExposureControl.Mode.Auto);
//
//        // Set initial exposure and gain, same as current.
////        myExposureControl.setExposure(curExp, TimeUnit.MILLISECONDS);
////        myGainControl.setGain(curGain);
//
//        // This loop allows manual adjustment of exposure and gain,
//        // while observing the effect on the preview image.

        // Wait for start button press on Driver Station
        telemetry.addLine("Waiting for start...");
        telemetry.update();
        waitForStart();

        // open claw
        robot.clawOpen();

        //Cone lift level
        //UNTESTED!!!!!!!
        moveLiftNonBlocking(robot.coneLiftlevel, liftPower);

        while (opModeIsActive()) {

            // read current gamepad values
            float   gamepad1LeftY   = gamepad1.left_stick_y;
            float   gamepad1LeftX   = -gamepad1.left_stick_x;
            float   gamepad1RightX  = -gamepad1.right_stick_x;
            float   gamepad2RightY  = -gamepad2.right_stick_y;

            // update gamepad extension state
            myGamepad1.readButtons();
            myGamepad2.readButtons();


            // CLAW
            if (myGamepad2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.05) {
                robot.clawClose();
            } else if (myGamepad2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.05) {
                robot.clawOpen();
            }

            // DRIVETRAIN
            // Mechanum formulas
            double topRightSpeed                = gamepad1LeftY + gamepad1LeftX + gamepad1RightX;           //Combines the inputs of the sticks to clip their output to a value between 1 and -1
            double topLeftSpeed                 = -gamepad1LeftY + gamepad1LeftX + gamepad1RightX;          //Combines the inputs of the sticks to clip their output to a value between 1 and -1
            double bottomRightSpeed             = gamepad1LeftY - gamepad1LeftX + gamepad1RightX;           //Combines the inputs of the sticks to clip their output to a value between 1 and -1
            double bottomLeftSpeed              = -gamepad1LeftY - gamepad1LeftX + gamepad1RightX;          //Combines the inputs of the sticks to clip their output to a value between 1 and -1
            double topLeftCorrectedSpeed        = Range.clip(Math.pow(topRightSpeed, 3), -speed, speed);    //Slows down the motor and sets its max/min speed to the double "speed"
            double topRightCorrectedSpeed       = Range.clip(Math.pow(topLeftSpeed, 3), -speed, speed);     //Slows down the motor and sets its max/min speed to the double "speed"
            double bottomLeftCorrectedSpeed     = Range.clip(Math.pow(bottomRightSpeed, 3), -speed, speed); //Slows down the motor and sets its max/min speed to the double "speed"
            double bottomRightCorrectedSpeed    = Range.clip(Math.pow(bottomLeftSpeed, 3), -speed, speed);  //Slows down the motor and sets its max/min speed to the double "speed"

            //GO FAST
            // speed override, go faster by pressing the right trigger
            if (myGamepad1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.05) {
                speedFactor = 0.8;
            } else {
                speedFactor = 0.5;
            }
            robot.topRight.setPower(topRightCorrectedSpeed * speedFactor);
            robot.bottomRight.setPower(bottomRightCorrectedSpeed * speedFactor);
            robot.topLeft.setPower(topLeftCorrectedSpeed * speedFactor);
            robot.bottomLeft.setPower(bottomLeftCorrectedSpeed * speedFactor);


            //GO SLOW
            //Newly coded by Micah and not tested yet
            //Was coded to make it easier to line up with a junction, not sure if this is too slow for the robot to strafe if needed.
            // speed override, go slower by pressing the left trigger
            if (myGamepad1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.05) {
                speedFactor = 0.3;
            } else {
                speedFactor = 0.5;
            }
            robot.topRight.setPower(topRightCorrectedSpeed * speedFactor);
            robot.bottomRight.setPower(bottomRightCorrectedSpeed * speedFactor);
            robot.topLeft.setPower(topLeftCorrectedSpeed * speedFactor);
            robot.bottomLeft.setPower(bottomLeftCorrectedSpeed * speedFactor);


            // LIFT
            if (robot.liftMotor.isBusy()) { // executing a bumper press
                // nothing to see here...
                if (DEBUG) {
                    telemetry.addData("lift", robot.liftMotor.getCurrentPosition() * (1.0 / robot.feetToTicks));
                }
            } else { // not currently executing a bumper press

                // put the motor back in to encoder mode that can be controlled by a joystick
                robot.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                
                // TODO: software lift limit!!!!!



                // myGamepad2.isDown(GamepadKeys.Button.B)
                if (Math.abs(gamepad2RightY) > 0.05) {  // joystick control
                    if (robot.liftMotor.getCurrentPosition() < robot.liftLevels[robot.liftLevels.length-1] ||
                            robot.liftMotor.getCurrentPosition() > robot.liftLevels[0]) {
                        liftPower = (gamepad2RightY * 0.2f) + Math.copySign(0.5f, gamepad2RightY);
                    } else {
                        liftPower = 0.0f;
                    }
                } else {
                    liftPower = 0.0f;
                }
                robot.liftMotor.setPower(liftPower);
                
            }
            // check if gamepad2 LB was pressed, move lift up!
            if (myGamepad2.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                //decrement currentLiftLevel
                if (robot.currentLiftLevel > 0) {
                    robot.currentLiftLevel--;
                    moveLiftNonBlocking(robot.liftLevels[robot.currentLiftLevel], liftButtonSpeed);
                }
            }
            // check if gamepad2 RB was pressed, move lift down!
            if (myGamepad2.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                //incrementing currentLiftLevel
                if (robot.currentLiftLevel < robot.liftLevels.length - 1) {
                    robot.currentLiftLevel++;
                    moveLiftNonBlocking(robot.liftLevels[robot.currentLiftLevel], liftButtonSpeed);
                }
//                 else if (robot.currentLiftLevel == robot.liftLevels[robot.currentLiftLevel = 3]) {
//                    robot.liftMotor.setTargetPosition(robot.mediumLiftlevel, liftPower);
//                }
            }


            // COLOR SENSOR
            double redAvg       = robot.redAverage(robot.colorSensorBack);
            double greenAvg     = robot.greenAverage(robot.colorSensorBack);
            double blueAvg      = robot.blueAverage(robot.colorSensorBack);
            double alphaAvg     = robot.alphaAverage(robot.colorSensorBack);

            // find the max of the red, green, and blue colors
            double colorMax     = Math.max(Math.max(redAvg, greenAvg), blueAvg);

            // dividing the colors by the max to get the norm
            double redNorm      = redAvg   / colorMax;
            double greenNorm    = greenAvg / colorMax;
            double blueNorm     = blueAvg  / colorMax;
            

            if (DEBUG) {
                // Printing out the color values
                telemetry.addData("Red norm", redNorm);
                telemetry.addData("Green norm", greenNorm);
                telemetry.addData("Blue norm", blueNorm);
                telemetry.addData("Red", redAvg);
                telemetry.addData("Green", greenAvg);
                telemetry.addData("Blue", blueAvg);
                telemetry.addData("Alpha", alphaAvg);
                telemetry.addData("encoder-top-right", robot.topRight.getCurrentPosition());
                telemetry.addData("LiftMotor position", robot.liftMotor.getCurrentPosition()* robot.ticksToFeet);
                telemetry.addData("Lift Level", robot.liftLevelNames[robot.currentLiftLevel]);
                telemetry.addData("Is busy?", liftIsMoving);


                if (alphaAvg >= 300.0) {
                    // color sensor is valid
        
                    boolean poleCheck       = isPole(redNorm, greenNorm, blueNorm);
                    boolean redConeCheck    = isRedCone(redNorm, greenNorm, blueNorm);
                    boolean blueConeCheck   = isBlueCone(redNorm, greenNorm, blueNorm);
        
                    //Prints out the results
                    telemetry.addData("Pole", poleCheck);
                    telemetry.addData("RedCone", redConeCheck);
                    telemetry.addData("BlueCone", blueConeCheck);
                } else {
                    // color sensor is not valid
                    telemetry.addLine("color sensor invalid");
                }
        
                // push telemetry update
                telemetry.update();

            }
       }
        robot.stopDriveMotors();
   }

    // LIFT FUNCTIONS
    // set lift to target position and then return, do not block in function
    public void moveLiftNonBlocking(double liftlevel, float speed) {

        int tickTarget = (int)(liftlevel * robot.feetToTicks);
        robot.liftMotor.setTargetPosition(tickTarget);

        // tell motors to run to target position
        robot.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // set speed based on lift power
        robot.liftMotor.setPower(speed);
    }
//TODO make cone lift level at the beginning of teleop --> done

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