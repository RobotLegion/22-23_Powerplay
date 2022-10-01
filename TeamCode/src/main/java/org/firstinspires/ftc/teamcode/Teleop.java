package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp(name = "Fy22TeleOp", group = "TeleOp" )
public class Teleop extends LinearOpMode {
    DcMotor topRight;
    DcMotor bottomRight;
    DcMotor topLeft;
    DcMotor bottomLeft;
    ColorSensor color;

    double speed = 1;   //change this variable to set speed (1 = 100%, 0.5 = 50%, etc)
/* one or two moters, put  them on RT  and LT
*
* color sensor for seeing yellow poles, using a range of RGB values focusing on red and green*/

    @Override
    public void runOpMode() throws InterruptedException {
        //hardware maps
        topRight = hardwareMap.dcMotor.get("TR"); // control hub port 0
        bottomRight = hardwareMap.dcMotor.get("BR"); //control hub port 1
        topLeft = hardwareMap.dcMotor.get("TL"); //control hub port 2
        bottomLeft = hardwareMap.dcMotor.get("BL"); //control hub port 3
        color = hardwareMap.get(ColorSensor.class, "Color");

        waitForStart();
        while(opModeIsActive()) {
            //Set gamepad
            float gamepad1LeftY = gamepad1.left_stick_y; //Sets the gamepads left sticks y position to a float
            float gamepad1LeftX = -gamepad1.left_stick_x; //Sets the gameepads left sticks x position to a float
            float gamepad1RightX = -gamepad1.right_stick_x; //Sets the gamepads right sticks x position to a float
            float gamepad2RightY = gamepad1.right_stick_y; // Sets the 2nd gamepads right sticks x position to a float;

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
//            double redValue = (double)color.red() / colorMax ;
//            double greenValue = (double)color.green() / colorMax;
//            double blueValue = (double)color.blue() / colorMax;
//
//            telemetry.addData("Red", redValue);
//            telemetry.addData("Green", greenValue);
//            telemetry.addData("Blue",blueValue);
//            telemetry.update();



            int redSum = 0;
            int greenSum = 0;
            int blueSum = 0;
            for (int i = 0; i < 5; i++) {
                redSum+=color.red();
                greenSum+=color.green();
                blueSum+=color.blue();
            }
            double red=(double)redSum/5.0;
            double green=(double)greenSum/5.0;
            double blue=(double)blueSum/5.0;

            double colorMax = Math.max(Math.max(red,green),blue);
            double redValue = red / colorMax ;
            double greenValue = green / colorMax;
            double blueValue = blue / colorMax;

            boolean redCheck= false;
            boolean greenCheck= false;
            boolean blueCheck= false;
            boolean poleCheck=false;

            if(redValue<= 1.0 && redValue>=0.8) {
                redCheck=true;
            }

            if(greenValue<= 1.0 && greenValue>=0.8) {
                greenCheck=true;
            }

            if(blueValue>= 0.0 && blueValue<=0.5) {
                blueCheck=true;
            }
//minimum color value
            //green, magenta, turquoise
            //blue & red 2

            if(redCheck && greenCheck && blueCheck){
                poleCheck=true;
            }

            telemetry.addData("Red", redValue);
            telemetry.addData("Green", greenValue);
            telemetry.addData("Blue",blueValue);
            telemetry.addData("Pole", poleCheck);
            telemetry.update();
        }
    }
}

