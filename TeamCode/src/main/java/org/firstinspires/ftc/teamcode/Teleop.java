package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.ColorSensor;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.ButtonReader;

@TeleOp(name = "Fy22TeleOp", group = "TeleOp" )
public class Teleop extends LinearOpMode {


    double feetToTicks = (19.2*28.0*304.8) / (Math.PI*96.0);
    double ticksToFeet = 1.0/feetToTicks;

    boolean liftIsMoving = false;
    
    DcMotor topRight;
    DcMotor bottomRight;
    DcMotor topLeft;
    DcMotor bottomLeft;
    Servo claw;
    DcMotor LiftMotor;
    ColorSensor color;
    double speed = 1;   //change this variable to set speed (1 = 100%, 0.5 = 50%, etc)
    /* one or two motors, put  them on RT  and LT */

    int Coneliftlevel = 100;
    int Groundliftlevel = 150;
    int Smallliftlevel = 200;
    int Mediumliftlevel = 300;
    int Highliftlevel = 400;
    int Currentliftlevel = 0;
    double speedfactor = 0.1;
    //To Do: modify speed factor to the value of 0.5 when RB button is pressed.

    double LiftPower;

    // Functions for Lift

    public void Lift(double liftlevel, float speed){

        int tickTarget = (int)(liftlevel * feetToTicks);
        LiftMotor.setTargetPosition(tickTarget);

        // tell motors to run to target position
        LiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // set speed based on lift power

        LiftMotor.setPower(speed);

        liftIsMoving = true;


    }

        public boolean Conelift(){

            LiftMotor.setTargetPosition(Coneliftlevel);
        return true;
        }

        public boolean Groundlift(){
            LiftMotor.setTargetPosition(Groundliftlevel);
            return true;
        }

        public boolean Smalllift(){
            LiftMotor.setTargetPosition(Smallliftlevel);
            return true;
        }

        public boolean Mediumlift(){
            LiftMotor.setTargetPosition(Mediumliftlevel );
            return true;
        }

        public boolean Highlift(){
            LiftMotor.setTargetPosition(Highliftlevel );
            return true;
        }

    //color sensor for seeing red cone, using a range of RGB values focusing on red and green
    public boolean isRedCone(double r, double g, double b) {

        // initalize all check variables to false
        boolean redCheck = false;
        boolean greenCheck = false;
        boolean blueCheck = false;

        // if r is in the range
        if(r >= 0.9 && r <= 1.0) {
            redCheck=true;
        }

        // if g is in the range
        if(g >= 0.4 && g <= 0.7) {
            greenCheck=true;
        }

        // if b is in the range
        if(b >= 0.0 && b <= 0.4) {
            blueCheck=true;
        }

        // if all color checks are true, return true
        // otherwise, return false
        if(redCheck && greenCheck && blueCheck){
            return true;
        }
        else return false;
    }

    //color sensor for seeing blue cone, using a range of RGB values focusing on red and green
    public boolean isBlueCone(double r, double g, double b) {

        // initalize all check variables to false
        boolean redCheck = false;
        boolean greenCheck = false;
        boolean blueCheck = false;

        // if r is in the range
        if(r >= 0.0 && r <= 0.8) {
            redCheck=true;
        }

        // if g is in the range
        if(g >= 0.5 && g <= 1.0) {
            greenCheck=true;
        }

        // if b is in the range
        if(b >= 0.7 && b <= 1.0) {
            blueCheck=true;
        }

        // if all color checks are true, return true
        // otherwise, return false
        if(redCheck && greenCheck && blueCheck){
            return true;
        }
        else return false;
    }

    //color sensor for seeing yellow poles, using a range of RGB values focusing on red and green
    public boolean isPole(double r, double g, double b) {

        // initalize all check variables to false
        boolean redCheck = false;
        boolean greenCheck = false;
        boolean blueCheck = false;

        // if r is in the range
        if(r >= 0.6 && r <= 0.9) {
            redCheck=true;
        }

        // if g is in the range
        if(g >= 0.8 && g <= 1.0) {
            greenCheck=true;
        }

        // if b is in the range
        if(b >= 0.0 && b <= 0.5) {
            blueCheck=true;
        }

        // if all color checks are true, return true
        // otherwise, return false
        if(redCheck && greenCheck && blueCheck){
            return true;
        }
        else return false;
    }

    public void ClawOpen(){
        claw.setPosition(0.2);
    }

    public void ClawClose(){
        claw.setPosition(1);
    }
//Left bumper (LB) on the gamepad for the claw.

    @Override
    public void runOpMode() throws InterruptedException {
        //hardware maps
        topRight = hardwareMap.dcMotor.get("TR"); // control hub port 0
        bottomRight = hardwareMap.dcMotor.get("BR"); //control hub port 1
        topLeft = hardwareMap.dcMotor.get("TL"); //control hub port 2
        bottomLeft = hardwareMap.dcMotor.get("BL"); //control hub port 3
        claw = hardwareMap.servo.get("claw"); // control hub servo port 0
        LiftMotor = hardwareMap.dcMotor.get("lift"); //port 0

        color = hardwareMap.get(ColorSensor.class, "Color");
        topRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        claw.scaleRange(0, 0.55);

        LiftMotor.setDirection(DcMotor.Direction.FORWARD);
//        LiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        LiftMotor.setTargetPosition(0);
//        Liftright.setTargetPosition(0);
//        LiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        Liftright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        LiftMotor.setPower(0.3);
//        Liftright.setPower(0.3);

        GamepadEx myGamepad2 = new GamepadEx(gamepad2);
        GamepadEx myGamepad1 = new GamepadEx(gamepad1);

        double Coneliftlevel = 0.06;
        double Groundliftlevel = 0.14;
        double Smallliftlevel = 0.86;
        double Mediumliftlevel = 1.34; //MAX
//        int Highliftlevel = 400;
        int Currentliftlevel = 0;
        double speedfactor = 1.0;

        double[] liftLevels = {Coneliftlevel, Groundliftlevel, Smallliftlevel, Mediumliftlevel};
        //To Do: modify speed factor to the value of 0.5 when RB button is pressed.


        double LiftPower = 0.0;


        waitForStart();
        ClawOpen();
        float gamepad1LeftY = 0;
        float gamepad1LeftX = 0;
        float gamepad1RightX = 0;

        while (opModeIsActive()) {
//            //Set gamepad
            gamepad1LeftY = gamepad1.left_stick_y;
            gamepad1LeftX = -gamepad1.left_stick_x;
            gamepad1RightX = -gamepad1.right_stick_x;
            float gamepad1RightY = gamepad1.right_stick_y; // Sets the 1st gamepads right sticks x position to a float;
            float gamepad2LeftY = gamepad2.left_stick_y; //Lift for demo Up
            float gamepadLeftY = gamepad2.left_stick_y; //Lift for demo Down
            float gamepad2LeftX = -gamepad2.left_stick_x; //Lift so you don't have to be perfectly straight on the joystick
            float gamepad2RightY = -gamepad2.right_stick_y; //Lift so you don't have to be perfectly straight on the joystick
//            boolean gamepad2LB = gamepad2.left_bumper; //when the button is pressed toggle claw //We changed right bumper to left bumper because we want right bumper for high junction
//            //lift position buttons
//            boolean gamepad2RB = gamepad2.right_bumper; //when the right bumper is pressed it moves the lift to the high junction position
//            boolean gamepad2Y = gamepad2.y; //when the y button is pressed to the small junction position
//            boolean gamepad2X = gamepad2.x; //when the x button is pressed to the ground junction position
//            boolean gamepad2A = gamepad2.a; //when the a button is pressed to the cone grabbing position
//            boolean gamepad2B = gamepad2.b; //when the b button is pressed to the medium junction position

            //clawCheck

            myGamepad1.readButtons();
            myGamepad2.readButtons();

            LiftMotor.setDirection(DcMotor.Direction.REVERSE);
            LiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            LiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            myGamepad2.readButtons();

            if (myGamepad2.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                //decrement currentliftlevel
                if (Currentliftlevel > 0) {
                    Currentliftlevel--;
                }
                Lift(liftLevels[Currentliftlevel], 0.7f);
            }

            if (myGamepad2.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                //incrementing currentliftlevel
                if (Currentliftlevel < liftLevels.length - 1) {
                    Currentliftlevel++;
                }
                Lift(liftLevels[Currentliftlevel], 0.5f);
            }

            //TODO: Use liftIsMoving variable
            if (LiftMotor.isBusy()) {
                telemetry.addData("lift", LiftMotor.getCurrentPosition() * (1.0 / feetToTicks));

                telemetry.update();
            } else {
                LiftMotor.setPower(0);
            }


            // TODO: this requires non-blocking Lift function
            if (Math.abs(gamepad2RightY) > 0.05) {
                LiftPower = (gamepad2RightY * 0.2) + Math.copySign(0.5, gamepad2RightY);
            } else {
                LiftPower = 0.0;
            }
//
            double currentFeet = LiftMotor.getCurrentPosition() * ticksToFeet;
//
            if (myGamepad2.isDown(GamepadKeys.Button.B)) {   // PRESS B TO OVERRIDE MIN AND MAX SAFTEY
//                    if ( (currentFeet <= 0.0 && LiftPower < 0.0) || (currentFeet >= Mediumliftlevel && LiftPower > 0.0) )
//                    {
//                        LiftPower = 0.0;
//                    }
                LiftMotor.setPower(LiftPower);
            }

//                LiftMotor.setPower(LiftPower);

            telemetry.addData("LiftMotor", currentFeet);

            telemetry.update();
        }

        if (myGamepad2.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {

            if (claw.getPosition() > 0.5) {
                ClawOpen();
            } else {
                ClawClose();
            }
        }


        //Lift
//
//            if (myGamepad2.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
//                Highlift();
//            }
//
//            if (myGamepad2.wasJustPressed(GamepadKeys.Button.Y)) {
//                Smalllift();
//            }
//
//            if (myGamepad2.wasJustPressed(GamepadKeys.Button.X)) {
//                Groundlift();
//            }
//
//            if (myGamepad2.wasJustPressed(GamepadKeys.Button.A)) {
//                Conelift();
//            }
//
//            if (myGamepad2.wasJustPressed(GamepadKeys.Button.B)) {
//                Mediumlift();
//            }

        //Lift with joysticks
         double gamepad2RightY = 0;

        if (gamepad2RightY > 0.05) {
            LiftPower = -0.5;
            telemetry.addLine("Up");
            telemetry.update();
        } else if (gamepad2RightY < -0.05) {
            LiftPower = 0.5;
            telemetry.addLine("Down");
            telemetry.update();
        } else {
            LiftPower = 0.0;
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


        LiftMotor.setPower(LiftPower);


        //Color Sensor
//            double colorMax = Math.max(Math.max(color.red(),color.green()),color.blue());
//            double redNorm = (double)color.red() / colorMax ;
//            double greenNorm = (double)color.green() / colorMax;
//            double blueNorm = (double)color.blue() / colorMax;
//
//            telemetry.addData("Red", redNorm);
//            telemetry.addData("Green", greenNorm);
//            telemetry.addData("Blue",blueNorm);
//            telemetry.update();


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
    }


