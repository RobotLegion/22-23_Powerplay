package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "ColorSensorCalibration", group = "TeleOp" )

public class ColorSensorCalibration extends LinearOpMode {

    // define color sensor
    ColorSensor colorSensorBack;

    // number of samples used for averaging
    int NUM_SAMPLES = 5;
    double ALPHA_THRESHOLD = 200.0;

    @Override
    public void runOpMode() throws InterruptedException {

        // initalize color sensor
        colorSensorBack = hardwareMap.get(ColorSensor.class, "Color");


        waitForStart();
        while(opModeIsActive()) {

            double redAvg = redAverage();
            double greenAvg = greenAverage();
            double blueAvg = blueAverage();
            double alphaAvg = alphaAverage();

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
            telemetry.addLine("");


            if(alphaAvg >= ALPHA_THRESHOLD){
                // color sensor is valid

                // Takes the norm and detects the is statements
                boolean poleCheck = isPole(redNorm, greenNorm, blueNorm);
                boolean redConeCheck = isRedCone(redNorm, greenNorm, blueNorm);
                boolean blueConeCheck = isBlueCone(redNorm, greenNorm, blueNorm);
                boolean parking1Check = isParking1(redNorm, greenNorm, blueNorm);
                boolean parking3Check = isParking3(redNorm, greenNorm, blueNorm);
                boolean parking2Check = !parking1Check && !parking3Check;

                //Prints out the results
                telemetry.addData("TELEOP Pole", poleCheck);
                telemetry.addData("TELEOP RedCone",redConeCheck);
                telemetry.addData("TELEOP BlueCone",blueConeCheck);

                telemetry.addLine("");

                telemetry.addData("AUTO Parking1", parking1Check);
                telemetry.addData("AUTO Parking2", parking2Check);
                telemetry.addData("AUTO Parking3", parking3Check);
            }
            else {
                // color sensor is not valid
                telemetry.addLine("Color sensor invaild");
            }

            // push telemetry update
            telemetry.update();
        }
    }

    // returns an average alpha value
    public double alphaAverage() {

        // initalize alphaSum to 0
        int alphaSum = 0;

        // run for NUM_SAMPLES iterations
        for (int i = 0; i < NUM_SAMPLES; i++) {
            // increment alphaSum by our new alpha color sensor reading
            alphaSum += colorSensorBack.alpha();
        }

        // return the average alpha value (alphaSum divided by the number of samples we take)
        return (double)alphaSum / (double)NUM_SAMPLES;
    }

    // return an average red value
    public double redAverage() {

        // initalize redSum to 0
        int redSum = 0;

        // run for NUM_SAMPLES iterations
        for (int i = 0; i < NUM_SAMPLES; i++) {
            // increment redSum by our new red color sensor reading
            redSum += colorSensorBack.red();
        }

        // return the average red value (redSum divided by the number of samples we take)
        return (double)redSum / (double)NUM_SAMPLES;
    }

    // return an average blue value
    public double blueAverage() {

        // initalize blueSum to 0
        int blueSum = 0;

        // run for NUM_SAMPLES iterations
        for (int i = 0; i < 5; i++) {
            // increment blueSum by our new blue color sensor reading
            blueSum += colorSensorBack.blue();
        }

        // return the average blue value (blueSum divided by the number of samples we take)
        return (double)blueSum / (double)NUM_SAMPLES;
    }

    // return an average green value
    public double greenAverage() {

        // initalize greenSum to 0
        int greenSum = 0;

        // run for NUM_SAMPLES iterations
        for (int i = 0; i < 5; i++) {
            // increment greenSum by our new green color sensor reading
            greenSum += colorSensorBack.green();
        }

        // return the average green value (greenSum divided by the number of samples we take)
        return (double)greenSum / (double)NUM_SAMPLES;
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

    // check if the input color sensor r,g,b values represent parking zone 1
    public boolean isParking1(double r, double g, double b) {

        // perfect reading:   r=1.00, g=0.50, b=0.70
        // imperfect reading: r=0.70, g=1.00, b=0.900

        // initalize all check variables to false
        boolean redCheck = false;
        boolean greenCheck = false;
        boolean blueCheck = false;

        // check if r is in the range
        if (r >= 0.6 && r <= 1.0) {
            redCheck = true;
        }

        // check if g is in the range
        if (g >= 0.3 && g <= 1.0) {
            greenCheck = true;
        }

        // check if b is in the range
        if (b >= 0.5 && b <= 1.0) {
            blueCheck = true;
        }

        // if all color checks are true, return true
        // otherwise, return false
        if (redCheck && greenCheck && blueCheck) {
            return true;
        } else return false;
    }

    // check if the input color sensor r,g,b values represent parking zone 3
    public boolean isParking3(double r, double g, double b) {

        // perfect reading:   r=0.56, g=1.00, b=0.27
        // imperfect reading: r=0.58, g=1.00, b=0.64

        // initalize all check variables to false
        boolean redCheck = false;
        boolean greenCheck = false;
        boolean blueCheck = false;

        // check if r is in the range
        if (r >= 0.3 && r <= 0.7) {
            redCheck = true;
        }

        // check if g is in the range
        if (g >= 0.8 && g <= 1.0 ) {
            greenCheck = true;
        }

        // check if b is in the range
        if (b >= 0.1 && b <= 0.8) {
            blueCheck = true;
        }

        // if all color checks are true, return true
        // otherwise, return false
        if (redCheck && greenCheck && blueCheck) {
            return true;
        } else return false;
    }
}


