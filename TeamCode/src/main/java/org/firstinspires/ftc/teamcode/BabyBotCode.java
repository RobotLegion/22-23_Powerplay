package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

//WIFI- Robotics Password- 0145

@TeleOp(name = "BabyBot", group = "TeleOp")
public class BabyBotCode extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {

        // initalize robot
        init(hardwareMap);

        //Setup Controller
        GamepadEx myGamepad1 = new GamepadEx(gamepad1);

        telemetry.addLine("Waiting for start...");
        telemetry.update();
        waitForStart();

        float gamepad1LeftY = 0;
        float gamepad1LeftX = 0;
        float gamepad1RightX = 0;
        while (opModeIsActive()) {
            // read current gamepad values
            gamepad1LeftY = gamepad1.left_stick_y;
            gamepad1LeftX = -gamepad1.left_stick_x;

            telemetry.addLine("Running");
            telemetry.update();

            // update gamepad extension state
            myGamepad1.readButtons();

            // CLAW
            if (myGamepad1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.05) {
                clawClose();
            } else if (myGamepad1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.05) {
                clawOpen();
            }
        }

        double speed = 0.5;

        //Drivetrain Formulas
        double RightSpeed = gamepad1LeftY + gamepad1LeftX + gamepad1RightX;           //Combines the inputs of the sticks to clip their output to a value between 1 and -1
        double LeftSpeed = -gamepad1LeftY + gamepad1LeftX + gamepad1RightX;          //Combines the inputs of the sticks to clip their output to a value between 1 and -1
//        double LeftCorrectedSpeed = Range.clip(Math.pow(RightSpeed, 3), -speed, speed);    //Slows down the motor and sets its max/min speed to the double "speed"
//        double RightCorrectedSpeed = Range.clip(Math.pow(LeftSpeed, 3), -speed, speed);     //Slows down the motor and sets its max/min speed to the double "speed"

    }

    public void init(HardwareMap hardwareMap){

        double clawOpenPosition = 0.0;
        double clawClosePosition = 1.0;

        // initalize drive train
        DcMotor right = hardwareMap.dcMotor.get("Right");
        DcMotor left = hardwareMap.dcMotor.get("Left");

        // initalize claw
        Servo clawServo = hardwareMap.servo.get("clawServo");
        clawServo.scaleRange(clawOpenPosition, clawClosePosition);

        // initalize lift
        DcMotor lift = hardwareMap.dcMotor.get("lift");
    }

    //Claw Functions
    public void clawOpen(){

    }

    public void clawClose() {

    }
//    public boolean isClawOpen() {
//       // return clawServo.getPosition() < 0.5;
//    }

    public void lift(){

    }

}