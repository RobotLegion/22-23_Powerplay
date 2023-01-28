//package org.firstinspires.ftc.teamcode;
//
//        import com.arcrobotics.ftclib.gamepad.GamepadEx;
//        import com.arcrobotics.ftclib.gamepad.GamepadKeys;
//        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//        import com.qualcomm.robotcore.hardware.ColorSensor;
//        import com.qualcomm.robotcore.hardware.DcMotor;
//        import com.qualcomm.robotcore.hardware.DcMotorSimple;
//        import com.qualcomm.robotcore.hardware.Gamepad;
//        import com.qualcomm.robotcore.hardware.Servo;
//        import com.qualcomm.robotcore.util.Range;
//
//@TeleOp(name = "Exhibition", group = "TeleOp" )
//
//public class Exhibition extends LinearOpMode {
//    DcMotor topRight;
//    DcMotor bottomRight;
//    DcMotor topLeft;
//    DcMotor bottomLeft;
//    double speed = 1;   //change this variable to set speed (1 = 100%, 0.5 = 50%, etc)
//    /* one or two motors, put  them on RT  and LT */
//
//    double speedfactor = 0.1;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        topRight = hardwareMap.dcMotor.get("TR"); // control hub port 0
//        bottomRight = hardwareMap.dcMotor.get("BR"); //control hub port 1
//        topLeft = hardwareMap.dcMotor.get("TL"); //control hub port 2
//        bottomLeft = hardwareMap.dcMotor.get("BL"); //control hub port 3
//
//        GamepadEx myGamepad1 = new GamepadEx(gamepad1);
//
//        waitForStart();
//        while (opModeIsActive()) {
////            //Set gamepad
//            float gamepad1LeftY = gamepad1.left_stick_y; //Sets the gamepads left sticks y position to a float
//            float gamepad1LeftX = -gamepad1.left_stick_x; //Sets the gameepads left sticks x position to a float
//            float gamepad1RightX = -gamepad1.right_stick_x; //Sets the gamepads right sticks x position to a float
//            float gamepad1RightY = gamepad1.right_stick_y; // Sets the 1st gamepads right sticks x position to a float;
//
//            myGamepad1.readButtons();
//
//
//            //Mechanum formulas
//            double TopRightSpeed = gamepad1LeftY + gamepad1LeftX + gamepad1RightX; //Combines the inputs of the sticks to clip their output to a value between 1 and -1
//            double TopLeftSpeed = -gamepad1LeftY + gamepad1LeftX + gamepad1RightX; //Combines the inputs of the sticks to clip their output to a value between 1 and -1
//            double BottomRightSpeed = gamepad1LeftY - gamepad1LeftX + gamepad1RightX; //Combines the inputs of the sticks to clip their output to a value between 1 and -1
//            double BottomLeftSpeed = -gamepad1LeftY - gamepad1LeftX + gamepad1RightX; //Combines the inputs of the sticks to clip their output to a value between 1 and -1
//
//            // sets speed
//            //I changed 3 to 2 in an attempt to make the robot drive slower
//            double topLeftCorrectedSpeed = Range.clip(Math.pow(TopRightSpeed, 3), -speed, speed); //Slows down the motor and sets its max/min speed to the double "speed"
//            double topRightCorrectedSpeed = Range.clip(Math.pow(TopLeftSpeed, 3), -speed, speed); //Slows down the motor and sets its max/min speed to the double "speed"
//            double bottomLeftCorrectedSpeed = Range.clip(Math.pow(BottomRightSpeed, 3), -speed, speed); //Slows down the motor and sets its max/min speed to the double "speed"
//            double bottomRightCorrectedSpeed = Range.clip(Math.pow(BottomLeftSpeed, 3), -speed, speed); //Slows down the motor and sets its max/min speed to the double "speed"
//
//            if (myGamepad1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.05) {
//                speedfactor = 0.3;
//            } else {
//                speedfactor = 0.2;
//            }
//            topRight.setPower(topRightCorrectedSpeed*speedfactor);
//            bottomRight.setPower(bottomRightCorrectedSpeed*speedfactor);
//            topLeft.setPower(topLeftCorrectedSpeed*speedfactor);
//            bottomLeft.setPower(bottomLeftCorrectedSpeed*speedfactor);
//
//
//
//        }
//
//
//    }
//
//}
//
//
//
