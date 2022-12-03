package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Motor", group = "TeleOp" )

public class Motor extends LinearOpMode {

    double feetToTicks = (19.2*28.0*304.8) / (Math.PI*96.0);
    double ticksToFeet = 1.0/feetToTicks;

    DcMotor LiftMotor;
    
    @Override
    public void runOpMode() throws InterruptedException {


        GamepadEx myGamepad2 = new GamepadEx(gamepad2);

        LiftMotor = hardwareMap.dcMotor.get("LiftMotor"); //port0
        double Coneliftlevel = 0.06;
        double Groundliftlevel = 0.14;
        double Smallliftlevel = 0.86;
        double Mediumliftlevel = 1.34; //MAX
//        int Highliftlevel = 400;
        int Currentliftlevel = 0;
        double speedfactor = 1.0;

        double[] liftLevels = {Coneliftlevel, Groundliftlevel, Smallliftlevel, Mediumliftlevel};
        //To Do: modify speed factor to the value of 0.5 when RB button is pressed.


        double LiftPower= 0.0;

        // Functions for Lift

        
        waitForStart();
        while(opModeIsActive()) {

            LiftMotor.setDirection(DcMotor.Direction.REVERSE);
            LiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            
            
            LiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            while(opModeIsActive()) {
                //Set gamepad
                float gamepad2RightY = -gamepad2.right_stick_y; //Lift so you don't have to be perfectly straight on the joystick

                //Lift

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
                    if (Currentliftlevel < liftLevels.length-1) {
                        Currentliftlevel++;
                    }
                    Lift(liftLevels[Currentliftlevel], 0.5f);
                }
//    
//                if (myGamepad2.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
//                    Highlift();
//                }
//    
//                if (myGamepad2.wasJustPressed(GamepadKeys.Button.Y)) {
//                    Smalllift();
//                }
//    
//                if (myGamepad2.wasJustPressed(GamepadKeys.Button.X)) {
//                    Groundlift();
//                }
//    
//                if (myGamepad2.wasJustPressed(GamepadKeys.Button.A)) {
//                    Conelift();
//                }
//    
//                if (myGamepad2.wasJustPressed(GamepadKeys.Button.B)) {
//                    Mediumlift();
//                }

                //Lift with joysticks

                // TODO: this requires non-blocking Lift function
                if (Math.abs(gamepad2RightY) > 0.05) {
                    LiftPower = (gamepad2RightY * 0.2) + Math.copySign(0.5, gamepad2RightY);
                } else {
                    LiftPower = 0.0;
                }
//
                double currentFeet= LiftMotor.getCurrentPosition() * ticksToFeet;
//
                if ( myGamepad2.isDown(GamepadKeys.Button.B) ) {   // PRESS B TO OVERRIDE MIN AND MAX SAFTEY
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
        }
    }
    //liftlevel in feet. Speed in 0-1
    public void Lift(double liftlevel, float speed){

        int tickTarget = (int)(liftlevel * feetToTicks);
        LiftMotor.setTargetPosition(tickTarget);

        // tell motors to run to target position
        LiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // set speed based on lift power

        LiftMotor.setPower(speed);

        // wait in this loop as long as at least 1 motor is still moving
        // motors report busy until they reach the target position
        while (opModeIsActive() && (LiftMotor.isBusy())) {
            telemetry.addData("lift", LiftMotor.getCurrentPosition()*(1.0/feetToTicks));

            telemetry.update();
        }

        LiftMotor.setPower(0);

    }
}


