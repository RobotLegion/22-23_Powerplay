package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "ColorSensorCalibration", group = "TeleOp")

public class ColorSensorCalibration extends LinearOpMode {

    // number of samples used for averaging
    int NUM_SAMPLES = 5;


    Robot robot = new Robot();

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        waitForStart();
        while (opModeIsActive()) {

            double redAvg = robot.redAverage(robot.colorSensorBack);
            double greenAvg = robot.greenAverage(robot.colorSensorBack);
            double blueAvg = robot.blueAverage(robot.colorSensorBack);
            double alphaAvg = robot.alphaAverage(robot.colorSensorBack);

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
            telemetry.addLine("");
            telemetry.addData("Red", redAvg);
            telemetry.addData("Green", greenAvg);
            telemetry.addData("Blue", blueAvg);
            telemetry.addData("Alpha", alphaAvg);
            telemetry.addLine("");


            if (robot.isColorValid(robot.colorSensorBack)) {
                // color sensor is valid

                // Takes the norm and detects the is statements
//                boolean poleCheck = isPole(redNorm, greenNorm, blueNorm);
//                boolean redConeCheck = isRedCone(redNorm, greenNorm, blueNorm);
//                boolean blueConeCheck = isBlueCone(redNorm, greenNorm, blueNorm);
                boolean parking1Check = robot.isParking1(redNorm, greenNorm, blueNorm);
                boolean parking3Check = robot.isParking3(redNorm, greenNorm, blueNorm);
                boolean parking2Check = !parking1Check && !parking3Check;

                //Prints out the results
//                telemetry.addData("TELEOP Pole", poleCheck);
//                telemetry.addData("TELEOP RedCone", redConeCheck);
//                telemetry.addData("TELEOP BlueCone", blueConeCheck);

                telemetry.addData("AUTO Parking1", parking1Check);
                telemetry.addData("AUTO Parking2", parking2Check);
                telemetry.addData("AUTO Parking3", parking3Check);
            } else {
                // color sensor is not valid
                telemetry.addLine("Color sensor invalid");
            }

            // push telemetry update
            telemetry.update();
        }
    }

    //color sensor for seeing red cone, using a range of RGB values focusing on red and green
    public boolean isRedCone(double r, double g, double b) {

        // initalize all check variables to false
        boolean redCheck = false;
        boolean greenCheck = false;
        boolean blueCheck = false;

        // if r is in the range
        if (r >= 0.9 && r <= 1.0) {
            redCheck = true;
        }

        // if g is in the range
        if (g >= 0.4 && g <= 0.7) {
            greenCheck = true;
        }

        // if b is in the range
        if (b >= 0.0 && b <= 0.4) {
            blueCheck = true;
        }

        // if all color checks are true, return true
        // otherwise, return false
        return redCheck && greenCheck && blueCheck;
    }

    //color sensor for seeing blue cone, using a range of RGB values focusing on red and green
    public boolean isBlueCone(double r, double g, double b) {

        // initalize all check variables to false
        boolean redCheck = false;
        boolean greenCheck = false;
        boolean blueCheck = false;

        // if r is in the range
        if (r >= 0.0 && r <= 0.8) {
            redCheck = true;
        }

        // if g is in the range
        if (g >= 0.5 && g <= 1.0) {
            greenCheck = true;
        }

        // if b is in the range
        if (b >= 0.7 && b <= 1.0) {
            blueCheck = true;
        }

        // if all color checks are true, return true
        // otherwise, return false
        return redCheck && greenCheck && blueCheck;
    }

    //color sensor for seeing yellow poles, using a range of RGB values focusing on red and green
    public boolean isPole(double r, double g, double b) {

        // initalize all check variables to false
        boolean redCheck = false;
        boolean greenCheck = false;
        boolean blueCheck = false;

        // if r is in the range
        if (r >= 0.6 && r <= 0.9) {
            redCheck = true;
        }

        // if g is in the range
        if (g >= 0.8 && g <= 1.0) {
            greenCheck = true;
        }

        // if b is in the range
        if (b >= 0.0 && b <= 0.5) {
            blueCheck = true;
        }

        // if all color checks are true, return true
        // otherwise, return false
        return redCheck && greenCheck && blueCheck;
    }
}


