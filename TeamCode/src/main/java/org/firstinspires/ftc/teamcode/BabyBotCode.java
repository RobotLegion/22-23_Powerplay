package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

//WIFI- Robotics Password- 0145

@TeleOp(name = "BabyBot", group = "TeleOp")
public class BabyBotCode extends LinearOpMode {


    double driveSpeed = 1.0;
    double speedFactor;
    float armSpeed = 0.2f;
    float armDownSpeed = 0.1f; //Make sure armDownSpeed always is less than armSpeed
    boolean armReady = true;

    // setup robot class
    Robot robot = new Robot();

    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor left = hardwareMap.dcMotor.get("Left");
        DcMotor right = hardwareMap.dcMotor.get("Right");
        DcMotor Arm = hardwareMap.dcMotor.get("Arm");
        Servo clawServo = hardwareMap.servo.get("clawServo");
        DigitalChannel limitSwitch = hardwareMap.get(DigitalChannel.class, "switch");

        limitSwitch.setMode(DigitalChannel.Mode.INPUT);


        //Setup Controller
        GamepadEx myGamepad1 = new GamepadEx(gamepad1);

        float gamepad1LeftY = 0;
        float gamepad1LeftX = 0;
        float gamepad1RightX = 0;
        float gamepad1RightY = 0;

        double clawOpenPosition = 0.0;
        double clawClosePosition = 1.0;

        telemetry.addLine("Waiting for start...");
        telemetry.update();
        waitForStart();


        while (opModeIsActive()) {

            // read current gamepad values
            gamepad1LeftY = gamepad1.left_stick_y;
            gamepad1LeftX = -gamepad1.left_stick_x;
            gamepad1RightY = -gamepad1.right_stick_y;
            gamepad1RightX = -gamepad1.right_stick_x;

            // update gamepad extension state
            myGamepad1.readButtons();

            // CLAW
            if (myGamepad1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
                clawServo.setPosition(clawClosePosition);
            } else if (myGamepad1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
                clawServo.setPosition(clawOpenPosition);
            }

            //Arm
            //UNTESTED!

            if (myGamepad1.getButton(GamepadKeys.Button.Y) && armReady) {
                Arm.setPower(armSpeed);
                sleep(1000);
                Arm.setPower(0.0);
                armReady = false;
            } else if (myGamepad1.getButton(GamepadKeys.Button.Y) && !armReady) {
                Arm.setPower(-armDownSpeed);
                sleep(2000);
                Arm.setPower(0.0);
                armReady = true;
            } else {
                Arm.setPower(0.0);
            }

            //Objective for Camera:
            // Make new targets for the camera to view. Allow arm to swing if a target is in view and from the right distance.
            //If a target is not in the right distance or in camera view, do not allow the arm to swing.
            //Do not allow the arm to swing because otherwise it will be off target.
            //Make a button to press to make the robot go left until it meets the requirements to shoot.
            // Auto shoot when the right distance and target is in view.

            double correctDistance = 0.0; //Correct distance
//            if (targetInView && correctDistance) {
//                boolean allowedSwing = true;
//            } else {
//                boolean allowedSwing = false;
//            }

//            boolean allowedSwing = true;
//
//
            //When limitSwitch is default true, so when limitSwitch is false, it is being pressed on.
            //Hi! should display when limitSwitch is pressed on
           if (!limitSwitch.getState()) {
               telemetry.addLine("Hi!");
           }

            //Drivetrain Formulas
            double RightSpeed = gamepad1LeftY + gamepad1LeftX + gamepad1RightX;           //Combines the inputs of the sticks to clip their output to a value between 1 and -1
            double LeftSpeed = -gamepad1LeftY + gamepad1LeftX + gamepad1RightX;          //Combines the inputs of the sticks to clip their output to a value between 1 and -1

            //Speedup trigger
            if (myGamepad1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.05) {
                speedFactor = 0.8;
            } else {
                speedFactor = 0.5;
            }
            right.setPower(gamepad1RightY * speedFactor);
            left.setPower(gamepad1LeftY * speedFactor);
        }

        telemetry.addData("switch", limitSwitch.getState());
        telemetry.update();

    }
}