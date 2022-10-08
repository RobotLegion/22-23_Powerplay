package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;

@Autonomous(name="autonomous", group="Robot")

public class Auto extends LinearOpMode {
    //Blue: 00,00,255 Parking 1
    //Magenta: 255 00 255 Parking 2
    //yellow: 255 255 00 Parking 3
    //Alpha threshold is about 300
    //Flat colors
    //Drive towards the signal "How do we know when we are 2cm from the cone?" yes
    //Read the color
    //use if statement to decide what to do when each color is read

    //if color=turquoise:
    //then back strafe left then forwards
    //else if color=majenta:
    //drive forwards
    //else if color=green:
    //back, strafe right, forward

    DcMotor topRight;
    DcMotor bottomRight;
    DcMotor topLeft;
    DcMotor bottomLeft;
    ColorSensor color;

    ElapsedTime runtime = new ElapsedTime();

    double speed = 1;
    double scale = 1;

    double scale2 = 2;


    @Override
    public void runOpMode() {
        //Hardware Maps
        topRight = hardwareMap.dcMotor.get("TR");//control hub port 0
        bottomRight = hardwareMap.dcMotor.get("BR");//control hub port 1
        topLeft = hardwareMap.dcMotor.get("TL"); //control hub port 2
        bottomLeft = hardwareMap.dcMotor.get("BL"); //control hub port 3
        color = hardwareMap.get(ColorSensor.class, "Color");



        waitForStart();
        while(opModeIsActive()) {
            double alphaAvg = alphaAverage();
            /*while(alphaAvg<200) {
                topRight.setPower(0.7071 / scale);
                bottomRight.setPower(0.7071 / scale);
                topLeft.setPower(0.7071 / scale);
                bottomLeft.setPower(0.7071 / scale);

                telemetry.addData("Alpha",alphaAvg);
                telemetry.update();

                alphaAvg = alphaAverage();
            }*/



            if(alphaAvg>=200) {
                topRight.setPower(0);
                bottomRight.setPower(0);
                topLeft.setPower(0);
                bottomLeft.setPower(0);

                telemetry.addLine("Something");
                telemetry.update();

                double redAvg = redAverage();
                double greenAvg = greenAverage();
                double blueAvg = blueAverage();
                double colorMax = Math.max(Math.max(redAvg,greenAvg),blueAvg);

                // dividing the colors by the max to get the norm
                double redNorm = redAvg / colorMax;
                double greenNorm = greenAvg / colorMax;
                double blueNorm = blueAvg / colorMax;

                if(isParking1(redNorm, greenNorm, blueNorm)){

                    topRight.setPower(0.5);
                    bottomRight.setPower(0.5);
                    topLeft.setPower(-0.5);
                    bottomLeft.setPower(-0.5);
                    runtime.reset();
                    while(runtime.seconds() < 1.5){
                        telemetry.addData("Path", "Leg1: %4.1f S Elapsed", runtime.seconds());
                        telemetry.update();
                    }

                    topRight.setPower(0);
                    bottomRight.setPower(0);
                    topLeft.setPower(0);
                    bottomLeft.setPower(0);
                    runtime.reset();


                }

                if(isParking3(redNorm, greenNorm, blueNorm)){

                    topRight.setPower(0.5);
                    bottomRight.setPower(0.5);
                    topLeft.setPower(-0.5);
                    bottomLeft.setPower(-0.5);

                    runtime.reset();
                    while(runtime.seconds() < 1.5){
                        telemetry.addData("Path", "Leg1: %4.1f S Elapsed", runtime.seconds());
                        telemetry.update();
                    }

                    topRight.setPower(0);
                    bottomRight.setPower(0);
                    topLeft.setPower(0);
                    bottomLeft.setPower(0);
                    runtime.reset();

                }

                else{
                    topRight.setPower(0.5);
                    bottomRight.setPower(0.5);
                    topLeft.setPower(-0.5);
                    bottomLeft.setPower(-0.5);

                    runtime.reset();
                    while(runtime.seconds() < 1.5){
                        telemetry.addData("Path", "Leg1: %4.1f S Elapsed", runtime.seconds());
                        telemetry.update();
                    }

                    topRight.setPower(0);
                    bottomRight.setPower(0);
                    topLeft.setPower(0);
                    bottomLeft.setPower(0);
                    runtime.reset();


                }
            }
            else {
                topRight.setPower(-0.5);
                bottomRight.setPower(-0.5);
                topLeft.setPower(0.5);
                bottomLeft.setPower(0.5);

                telemetry.addData("Alpha", alphaAvg);
                telemetry.update();

                alphaAvg = alphaAverage();
            }
        }
    }

    public double alphaAverage(){

        int alphaSum = 0;

        for (int i = 0; i < 5; i++) {
            //redSum+=color.red();
            // greenSum+=color.green();
            //blueSum+=color.blue();
            alphaSum+=color.alpha();
        }

        double alphaAvg=(double)alphaSum/5.0;

        return alphaAvg;
    }
    public double redAverage(){

        int redSum = 0;

        for (int i = 0; i < 5; i++) {
            //redSum+=color.red();
            // greenSum+=color.green();
            //blueSum+=color.blue();
            redSum+=color.red();
        }

        double redAvg=(double)redSum/5.0;

        return redAvg;
    }
    public double blueAverage(){

        int blueSum = 0;

        for (int i = 0; i < 5; i++) {
            //redSum+=color.red();
            // greenSum+=color.green();
            //blueSum+=color.blue();
            blueSum+=color.blue();
        }

        double blueAvg=(double)blueSum/5.0;

        return blueAvg;
    }
    public double greenAverage(){

        int greenSum = 0;

        for (int i = 0; i < 5; i++) {
            //redSum+=color.red();
            // greenSum+=color.green();
            //blueSum+=color.blue();
            greenSum+=color.green();
        }

        double greenAvg=(double)greenSum/5.0;

        return greenAvg;
    }

    public boolean isParking1(double r, double g, double b) {
        boolean redCheck= false;
        boolean greenCheck= false;
        boolean blueCheck= false;
        if(r<= 1 && r>=0.8) {
            redCheck=true;
        }

        if(g>=0.2) {
            greenCheck=true;
        }

        if(b>= 1.0 && b<=0.8) {
            blueCheck=true;
        }
        if(redCheck && greenCheck && blueCheck){
            return true;
        }
        else return false;
    }

    public boolean isParking3(double r, double g, double b) {
        //TODO - sometimes triggers true when a Red Cone is present
        boolean redCheck= false;
        boolean greenCheck= false;
        boolean blueCheck= false;
        if(r<= 1.0 && r>=0.8) {
            redCheck=true;
        }

        if(g<= 1.0 && g>=0.8) {
            greenCheck=true;
        }

        if(b<=0.2) {
            blueCheck=true;
        }
        if(redCheck && greenCheck && blueCheck){
            return true;
        }
        else return false;
    }
}
