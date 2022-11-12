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
    DcMotor topRight;
    DcMotor bottomRight;
    DcMotor topLeft;
    DcMotor bottomLeft;
    Servo claw;
    DcMotor Liftright;
    DcMotor Liftleft;
    ColorSensor color;
    double speed = 1;   //change this variable to set speed (1 = 100%, 0.5 = 50%, etc)
    /* one or two motors, put  them on RT  and LT */

    int Coneliftlevel = 100;
    int Groundliftlevel = 150;
    int Smallliftlevel = 200;
    int Mediumliftlevel = 300;
    int Highliftlevel = 400;
    int Currentliftlevel = 0;


    // Functions for Lift

        public boolean Conelift(){

            Liftleft.setTargetPosition(Coneliftlevel);
            Liftright.setTargetPosition(Coneliftlevel);
        return true;
        }

        public boolean Groundlift(){
            Liftleft.setTargetPosition(Groundliftlevel);
            Liftright.setTargetPosition(Groundliftlevel);
            return true;
        }

        public boolean Smalllift(){
            Liftleft.setTargetPosition(Smallliftlevel);
            Liftright.setTargetPosition(Smallliftlevel);
            return true;
        }

        public boolean Mediumlift(){
            Liftleft.setTargetPosition(Mediumliftlevel );
            Liftright.setTargetPosition(Mediumliftlevel );
            return true;
        }

        public boolean Highlift(){
            Liftleft.setTargetPosition(Highliftlevel );
            Liftright.setTargetPosition(Highliftlevel);
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
        claw.setPosition(0);
    }

    public void ClawClose(){
        claw.setPosition(1);
    }
//Right bumper (RB) on the gamepad for the claw.

    @Override
    public void runOpMode() throws InterruptedException {
        //hardware maps
        topRight = hardwareMap.dcMotor.get("TR"); // control hub port 0
        bottomRight = hardwareMap.dcMotor.get("BR"); //control hub port 1
        topLeft = hardwareMap.dcMotor.get("TL"); //control hub port 2
        bottomLeft = hardwareMap.dcMotor.get("BL"); //control hub port 3
        claw = hardwareMap.servo.get("claw"); //servo port 0
        Liftleft = hardwareMap.dcMotor.get("Liftleft"); // port1
        Liftright = hardwareMap.dcMotor.get("Liftright"); //port0
        color = hardwareMap.get(ColorSensor.class, "Color");
        topRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        claw.scaleRange(0, 0.55);

        Liftleft.setDirection(DcMotor.Direction.FORWARD);
        Liftright.setDirection(DcMotor.Direction.FORWARD);
        Liftleft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Liftright.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Liftleft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Liftright.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Liftleft.setTargetPosition(0);
        Liftright.setTargetPosition(0);
        Liftleft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Liftright.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Liftleft.setPower(0.3);
        Liftright.setPower(0.3);

        GamepadEx myGamepad2 = new GamepadEx(gamepad2);
        GamepadEx myGamepad1 = new GamepadEx(gamepad1);

        waitForStart();
        ClawOpen();
        while(opModeIsActive()) {
//            //Set gamepad
            float gamepad1LeftY = gamepad1.left_stick_y; //Sets the gamepads left sticks y position to a float
            float gamepad1LeftX = -gamepad1.left_stick_x; //Sets the gameepads left sticks x position to a float
            float gamepad1RightX = -gamepad1.right_stick_x; //Sets the gamepads right sticks x position to a float
            float gamepad1RightY = gamepad1.right_stick_y; // Sets the 2nd gamepads right sticks x position to a float;
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

            if (myGamepad2.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {

                if(claw.getPosition()>0.5){
                    ClawOpen();
                }
                else{
                    ClawClose();
                }
            }

            //Lift

            if (myGamepad2.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                Highlift();
            }

            if (myGamepad2.wasJustPressed(GamepadKeys.Button.Y)) {
                Smalllift();
            }

            if (myGamepad2.wasJustPressed(GamepadKeys.Button.X)) {
                Groundlift();
            }

            if (myGamepad2.wasJustPressed(GamepadKeys.Button.A)) {
                Conelift();
            }

            if (myGamepad2.wasJustPressed(GamepadKeys.Button.B)) {
                Mediumlift();
            }


            //Mechanum formulas
            double TopRightSpeed = gamepad1LeftY + gamepad1LeftX + gamepad1RightX; //Combines the inputs of the sticks to clip their output to a value between 1 and -1
            double TopLeftSpeed = -gamepad1LeftY + gamepad1LeftX + gamepad1RightX; //Combines the inputs of the sticks to clip their output to a value between 1 and -1
            double BottomRightSpeed = gamepad1LeftY - gamepad1LeftX + gamepad1RightX; //Combines the inputs of the sticks to clip their output to a value between 1 and -1
            double BottomLeftSpeed = -gamepad1LeftY - gamepad1LeftX + gamepad1RightX; //Combines the inputs of the sticks to clip their output to a value between 1 and -1

            // sets speed
            double topLeftCorrectedSpeed = Range.clip(Math.pow(TopRightSpeed, 3), -speed, speed); //Slows down the motor and sets its max/min speed to the double "speed"
            double topRightCorrectedSpeed = Range.clip(Math.pow(TopLeftSpeed, 3), -speed, speed); //Slows down the motor and sets its max/min speed to the double "speed"
            double bottomLeftCorrectedSpeed = Range.clip(Math.pow(BottomRightSpeed, 3), -speed, speed); //Slows down the motor and sets its max/min speed to the double "speed"
            double bottomRightCorrectedSpeed = Range.clip(Math.pow(BottomLeftSpeed, 3), -speed, speed); //Slows down the motor and sets its max/min speed to the double "speed"

            topRight.setPower(topRightCorrectedSpeed);
            bottomRight.setPower(bottomRightCorrectedSpeed);
            topLeft.setPower(topLeftCorrectedSpeed);
            bottomLeft.setPower(bottomLeftCorrectedSpeed);

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
                redSum+=color.red();
                greenSum+=color.green();
                blueSum+=color.blue();
                alphaSum+=color.alpha();
            }

            //It div all the number to find the average
            double redAvg=(double)redSum/5.0;
            double greenAvg=(double)greenSum/5.0;
            double blueAvg=(double)blueSum/5.0;
            double alphaAvg=(double)alphaSum/5.0;

            // Findng the max of r,g and b
            double colorMax = Math.max(Math.max(redAvg,greenAvg),blueAvg);

            // dividing the colors by the max to get the norm
            double redNorm = redAvg / colorMax;
            double greenNorm = greenAvg / colorMax;
            double blueNorm = blueAvg / colorMax;

            // Printing out the color values
            telemetry.addData("Red norm", redNorm);
            telemetry.addData("Green norm", greenNorm);
            telemetry.addData("Blue norm",blueNorm);
            telemetry.addData("Red", redAvg);
            telemetry.addData("Green", greenAvg);
            telemetry.addData("Blue",blueAvg);
            telemetry.addData("Alpha",alphaAvg);
            telemetry.addData("encoder-top-right", topRight.getCurrentPosition());
            telemetry.addData("Liftleft", Liftleft.getCurrentPosition());
            telemetry.addData("Liftright", Liftright.getCurrentPosition());


            if(alphaAvg>=300.0 ){
                // color sensor is valid

                // Takes the norm and detects the is statements
                boolean poleCheck= isPole(redNorm, greenNorm, blueNorm);
                boolean redConeCheck= isRedCone(redNorm, greenNorm, blueNorm);
                boolean blueConeCheck= isBlueCone(redNorm, greenNorm, blueNorm);

                //Prints out the results
                telemetry.addData("Pole", poleCheck);
                telemetry.addData("RedCone",redConeCheck);
                telemetry.addData("BlueCone",blueConeCheck);
            }
            else {
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
}

