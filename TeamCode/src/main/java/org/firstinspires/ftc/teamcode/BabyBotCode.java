package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;


@TeleOp(name = "CompetitionTeleop", group = "TeleOp")
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

        while (opModeIsActive()) {
            // read current gamepad values
            float gamepad1LeftY = gamepad1.left_stick_y;
            float gamepad1LeftX = -gamepad1.left_stick_x;

            // update gamepad extension state
            myGamepad1.readButtons();

            // CLAW
            if (myGamepad1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.05) {
                clawClose();
            } else if (myGamepad1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.05) {
                clawOpen();
            }
        }

    }

    public void init(HardwareMap hardwareMap){

        // initalize drive train
        right = hardwareMap.dcMotor.get("Right");
        left = hardwareMap.dcMotor.get("Left");

        // initalize claw
        clawServo = hardwareMap.servo.get("clawServo");
        clawServo.scaleRange(clawOpenPosition, clawClosePosition);

        // initalize lift
        lift = hardwareMap.dcMotor.get("lift");
    }

    public void clawOpen(){
        clawServo.set
    }

    public void clawClose() {

    }

    public void lift(){

    }

}