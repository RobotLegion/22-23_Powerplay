package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "FY22TeleOp", group = "TeleOp" )
public class Teleop extends LinearOpMode {
    DcMotor topRight;
    DcMotor bottomRight;
    DcMotor topLeft;
    DcMotor bottomLeft;

    double speed = 1;
/* one or two moters, put them on RT  and LT
*
* color sensor for seeing yellow poles, using a range of RGB values focusing on red and green*/

    @Override
    public void runOpMode() throws InterruptedException {
        //hardware maps
        topRight = hardwareMap.dcMotor.get("TR"); // control hub port 0
        bottomRight = hardwareMap.dcMotor.get("BR"); //control hub port 1
        topLeft = hardwareMap.dcMotor.get("TL"); //control hub port 2
        bottomLeft = hardwareMap.dcMotor.get("BL"); //control hub port 3
        // Add a hardware map here for the color sensor ~Nathan

        waitForStart();

        while(opModeIsActive()) {

            //Set gamepad
            float gamepad1LeftY = gamepad1.left_stick_y; //Sets the gamepads left sticks y position to a float
            float gamepad1LeftX = gamepad1.left_stick_x; //Sets the gamepads left sticks x position to a float
            float gamepad1RightX = gamepad1.right_stick_x; //Sets the gamepads right sticks x position to a float
            float gamepad1RightY = gamepad1.right_stick_y; // Sets the 2nd gamepads right sticks x position to a float;
            // setup any additional controls here first ~Nathan

/* DELETE THIS COMMENT BLOCK WHEN THE NEW FORMULAE ARE ADDED - OLD CODE
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
 */


/* A note on mechanum drive and some suggested code to implement:
*
Mechanum drive works by deciding which angular direction you want to drive and applying
* individual power to each wheel in order to achieve motion in that direction.  When thinking about
* angular direction, think about points on a clock (in our case, 3 o'clock corresponds
* to 0 degrees/0 radians).  When talking about angles in this type of math, we usually talk about
* radians, which is a unit for measuring angles.  This is something you will learn later in high
* school but you can ask me or your math teachers if you want to learn more about this now.
*
* The formulae for our overall power level will be a function to find the magnitude of the
* direction we point the left joystick.  The magnitude is like the size of a math object called a
* vector: a larger magnitude is a bigger number and a smaller magnitude is a smaller number.  We use
* vectors to say what direction we want to go and how fast we want to go in that direction.  To find
* the magnitude, we will use a special formula called the Pythagorean Theorem.  This formula lets us
* find the lengths of different sides of triangles.  Here is an example of code to implement this:
*
* double magnitude = Math.sqrt(gamepad1LeftX**2.0 + gamepad1LeftY**2.0);
*
* Remember, floats are decimals so we have to use "2.0" instead of just "2" here.
*
* Now we have the speed we want the robot to move at, we can tell the robot what direction to move.
* There is more tricky math involved in these formulae, but basically we are using triangles and
* circles to figure out the angle we want to go like we talked about above, and then turn this angle
* into power levels for each wheel.  We will use more special functions called "sine and arctangent"
* to find the values we need:
*
* double position = Math.atan(gamepad1LeftY/gamepad1LeftX);
*
* And once we find our position and magnitude, we can set our wheel powers:
*
* double TopRightSpeed = magnitude * Math.sin(position - Math.PI/4) + gamepad1RightX;
* double TopLeftSpeed = magnitude * Math.sin(position + Math.PI/4) + gamepad1RightX;
* double BottomRightSpeed = magnitude * Math.sin(position + Math.PI/4) + gamepad1RightX;
* double BottomLeftSpeed = magnitude * Math.sin(position - Math.PI/4) + gamepad1RightX;
*
* Because of the way the wheels are supposed to work, you may have to reverse some of the directions
* of the motors.  If a wheel needs to be reversed, just add a minus (-) in front of the word
* "magnitude" in the line for the wheel that needs to be reversed.
*
* You may notice that we are adding the value from "gamepad1RightX" to each of the lines above. This
* allows for the robot to turn clockwise/anti-clockwise in addition to foward/backward/left/right.
*
* We still want to protect our motors from having more than 100% speed, so we will correct the speed
* equations to have a value between -1 and 1:
*
* double topLeftCorrectedSpeed = Range.clip(TopLeftSpeed, -speed, speed);
* double topRightCorrectedSpeed = Range.clip(TopRightSpeed, -speed, speed);
* double bottomLeftCorrectedSpeed = Range.clip(BottomLeftSpeed, -speed, speed);
* double bottomRightCorrectedSpeed = Range.clip(BottomRightSpeed, -speed, speed);
*
* Putting it all together, here is one possible way to implement the code in our Teleop:
*
* float magnitude = Math.sqrt(gamepad1LeftX**2.0 + gamepad1LeftY**2.0);
* double position = Math.atan(gamepad1LeftY/gamepad1LeftX);
*
* double TopRightSpeed = magnitude * Math.sin(position - Math.PI/4) + gamepad1RightX;
* double TopLeftSpeed = magnitude * Math.sin(position + Math.PI/4) + gamepad1RightX;
* double BottomRightSpeed = magnitude * Math.sin(position + Math.PI/4) + gamepad1RightX;
* double BottomLeftSpeed = magnitude * Math.sin(position - Math.PI/4) + gamepad1RightX;
*
* double topLeftCorrectedSpeed = Range.clip(TopLeftSpeed, -speed, speed);
* double topRightCorrectedSpeed = Range.clip(TopRightSpeed, -speed, speed);
* double bottomLeftCorrectedSpeed = Range.clip(BottomLeftSpeed, -speed, speed);
* double bottomRightCorrectedSpeed = Range.clip(BottomRightSpeed, -speed, speed);
*
* topRight.setPower(topRightCorrectedSpeed);
* bottomRight.setPower(bottomRightCorrectedSpeed);
* topLeft.setPower(topLeftCorrectedSpeed);
* bottomLeft.setPower(bottomLeftCorrectedSpeed);
*
* Don't forget to add a minus sign to the word magnitude if a motor needs it and make sure that the
* motors are in the correct spots on the actual robot (example: confirm that topRight is actually
* the "top right" motor).  We may still need to modify some of the above equations to allow for the
* smooth speed control we talked about, but this should get us going more effectively.
*
* Good luck and message me on discord or ask another mentor if you have any questions!
*
* ~ Nathan Garrett
*
* */

            telemetry.addData("x1", gamepad1.left_stick_x);
            telemetry.addData("y1", gamepad1.left_stick_y);
            telemetry.addData("x2", gamepad1.right_stick_x);
            telemetry.addData("y2", gamepad1.right_stick_y);
            telemetry.update();
        }
    }
}

