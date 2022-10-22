package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;

@Autonomous(name="autonomous", group="Robot")

public class Auto extends LinearOpMode {
    //Internet Password: Robotics 0145
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
    //else if color=magenta:
    //drive forwards
    //else if color=green:
    //back, strafe right, forward

    DcMotor topRight;
    DcMotor bottomRight;
    DcMotor topLeft;
    DcMotor bottomLeft;
    ColorSensor color;

    double feetToTicks = (19.2*28.0*304.8) / (Math.PI*96.0);




    ElapsedTime runtime = new ElapsedTime();

    double speed = 1;
    double scale = 1;

    double scale2 = 2;


    public void runOpMode() {
        //Hardware Maps
        topRight = hardwareMap.dcMotor.get("TR");//control hub port 0
        bottomRight = hardwareMap.dcMotor.get("BR");//control hub port 1
        topLeft = hardwareMap.dcMotor.get("TL"); //control hub port 2
        bottomLeft = hardwareMap.dcMotor.get("BL"); //control hub port 3
        color = hardwareMap.get(ColorSensor.class, "Color");

        topLeft.setDirection(DcMotor.Direction.FORWARD);
        topRight.setDirection(DcMotor.Direction.REVERSE);
        bottomLeft.setDirection(DcMotor.Direction.FORWARD);
        bottomRight.setDirection(DcMotor.Direction.REVERSE);

        topLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        topLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        topRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bottomRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//        topLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        topRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        bottomLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        bottomRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
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

            //0.5 speed, 2.0 feet.
            drive("forward",0.5f,2.0);

            //0.5 speed, 1.0 foot.
            drive("backward",0.5f,1.0);


            //Strafe Left bottomRight (-0.5), topLeft (-0.5), topRight (0.5), bottomLeft (0.5)
            //Stright all positve.

            telemetry.addData("top left", topLeft.getCurrentPosition());
            telemetry.addData("top right", topRight.getCurrentPosition());
            telemetry.addData("bottom left", bottomLeft.getCurrentPosition());
            telemetry.addData("bottom right", bottomRight.getCurrentPosition());
            telemetry.update();

//            if (alphaAvg >= 200) {
//                topRight.setPower(0);
//                bottomRight.setPower(0);
//                topLeft.setPower(0);
//                bottomLeft.setPower(0);
//
//                double redAvg = redAverage();
//                double greenAvg = greenAverage();
//                double blueAvg = blueAverage();
//                double colorMax = Math.max(Math.max(redAvg, greenAvg), blueAvg);
//
//                // dividing the colors by the max to get the norm
//                double redNorm = redAvg / colorMax;
//                double greenNorm = greenAvg / colorMax;
//                double blueNorm = blueAvg / colorMax;
//
//                if (isParking1(redNorm, greenNorm, blueNorm)) {
//
//                    topRight.setPower(-0.5);
//                    bottomRight.setPower(-0.5);
//                    topLeft.setPower(0.5);
//                    bottomLeft.setPower(0.5);
//                    runtime.reset();
//                    while (runtime.seconds() < 1.5) {
//                        telemetry.addData("Path", "Leg1: %4.1f S Elapsed", runtime.seconds());
//                        telemetry.update();
//                    }
//
//                    topRight.setPower(0.5);
//                    bottomRight.setPower(0.5);
//                    topLeft.setPower(-0.5);
//                    bottomLeft.setPower(-0.5);
//                    runtime.reset();
//                    while (runtime.seconds() < 1.5) {
//                        telemetry.addData("Path", "Leg1: %4.1f S Elapsed", runtime.seconds());
//                        telemetry.update();
//                    }
//
//
//                    topRight.setPower(0);
//                    bottomRight.setPower(0);
//                    topLeft.setPower(0);
//                    bottomLeft.setPower(0);
//                    runtime.reset();
//
//
//                }
//
//                if (isParking3(redNorm, greenNorm, blueNorm)) {
//
//                    topRight.setPower(0);
//                    bottomRight.setPower(0);
//                    topLeft.setPower(0);
//                    bottomLeft.setPower(0);
//
//                    runtime.reset();
//                    while (runtime.seconds() < 1.5) {
//                        telemetry.addData("Path", "Leg1: %4.1f S Elapsed", runtime.seconds());
//                        telemetry.update();
//                    }
//
//                    topRight.setPower(-0.5);
//                    bottomRight.setPower(-0.5);
//                    topLeft.setPower(-0.5);
//                    bottomLeft.setPower(-0.2);
//                    runtime.reset();
//                    while (runtime.seconds() < 0.2) {
//                        telemetry.addData("Path", "Leg1: %4.1f S Elapsed", runtime.seconds());
//                        telemetry.update();
//                    }
//
//                    topRight.setPower(0);
//                    bottomRight.setPower(0);
//                    topLeft.setPower(0);
//                    bottomLeft.setPower(0);
//                    runtime.reset();
//
//                } else {
//                    topRight.setPower(0);
//                    bottomRight.setPower(0);
//                    topLeft.setPower(0);
//                    bottomLeft.setPower(0);
//
//                    runtime.reset();
//                    while (runtime.seconds() < 10) {
//                        telemetry.addData("Path", "Leg1: %4.1f S Elapsed", runtime.seconds());
//                        telemetry.update();
//                    }
//
//                    topRight.setPower(0);
//                    bottomRight.setPower(0);
//                    topLeft.setPower(0);
//                    bottomLeft.setPower(0);
//                    runtime.reset();
//
//
//                }
//            } else {
//                topRight.setPower(-0.5);
//                bottomRight.setPower(-0.5);
//                topLeft.setPower(0.5);
//                bottomLeft.setPower(0.5);
//
//                telemetry.addData("Alpha", alphaAvg);
//                telemetry.update();
//
//                alphaAvg = alphaAverage();
//            }
        }
    }

    public void drive(String direction, float speed, double target){
        topLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        topRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        topLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        topRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bottomLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        bottomRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        int tickTarget = (int)(target * feetToTicks);
        if (direction == "forward"){
            topLeft.setTargetPosition(tickTarget);
            topRight.setTargetPosition(tickTarget);
            bottomLeft.setTargetPosition(tickTarget);
            bottomRight.setTargetPosition(tickTarget);
        }
        else if(direction == "backward"){
            topLeft.setTargetPosition(-tickTarget);
            topRight.setTargetPosition(-tickTarget);
            bottomLeft.setTargetPosition(-tickTarget);
            bottomRight.setTargetPosition(-tickTarget);
        }
        else if(direction == "left"){
            topLeft.setTargetPosition(-tickTarget);
            topRight.setTargetPosition(tickTarget);
            bottomLeft.setTargetPosition(tickTarget);
            bottomRight.setTargetPosition(-tickTarget);
        }
        else if(direction == "right"){
            topLeft.setTargetPosition(tickTarget);
            topRight.setTargetPosition(-tickTarget);
            bottomLeft.setTargetPosition(-tickTarget);
            bottomRight.setTargetPosition(tickTarget);
        }

        topRight.setPower(speed);
        bottomRight.setPower(speed);
        topLeft.setPower(speed);
        bottomLeft.setPower(speed);
    }

    public double alphaAverage() {

        int alphaSum = 0;

        for (int i = 0; i < 5; i++) {
            //redSum+=color.red();
            // greenSum+=color.green();
            //blueSum+=color.blue();
            alphaSum += color.alpha();
        }

        double alphaAvg = (double) alphaSum / 5.0;

        return alphaAvg;
    }

    public double redAverage() {

        int redSum = 0;

        for (int i = 0; i < 5; i++) {
            //redSum+=color.red();
            // greenSum+=color.green();
            //blueSum+=color.blue();
            redSum += color.red();
        }

        double redAvg = (double) redSum / 5.0;

        return redAvg;
    }

    public double blueAverage() {

        int blueSum = 0;

        for (int i = 0; i < 5; i++) {
            //redSum+=color.red();
            // greenSum+=color.green();
            //blueSum+=color.blue();
            blueSum += color.blue();
        }

        double blueAvg = (double) blueSum / 5.0;

        return blueAvg;
    }

    public double greenAverage() {

        int greenSum = 0;

        for (int i = 0; i < 5; i++) {
            //redSum+=color.red();
            // greenSum+=color.green();
            //blueSum+=color.blue();
            greenSum += color.green();
        }

        double greenAvg = (double) greenSum / 5.0;

        return greenAvg;
    }

    public boolean isParking1(double r, double g, double b) {
        boolean redCheck = false;
        boolean greenCheck = false;
        boolean blueCheck = false;
        if (r <= 1 && r >= 0.8) {
            redCheck = true;
        }

        if (g >= 0.2) {
            greenCheck = true;
        }

        if (b >= 1.0 && b <= 0.8) {
            blueCheck = true;
        }
        if (redCheck && greenCheck && blueCheck) {
            return true;
        } else return false;
    }

    public boolean isParking3(double r, double g, double b) {
        //TODO - sometimes triggers true when a Red Cone is present
        boolean redCheck = false;
        boolean greenCheck = false;
        boolean blueCheck = false;
        if (r <= 1.0 && r >= 0.8) {
            redCheck = true;
        }

        if (g <= 1.0 && g >= 0.8) {
            greenCheck = true;
        }

        if (b <= 0.2) {
            blueCheck = true;
        }
        if (redCheck && greenCheck && blueCheck) {
            return true;
        } else return false;
    }


    public void Mecanum_drive(String Dir, double Spd, long Slp) {
      /*
      topRight = hardwareMap.dcMotor.get("TR"); //Control Hub Port 0
      bottomRight = hardwareMap.dcMotor.get("BR"); //Control Hub Port 1
      topLeft = hardwareMap.dcMotor.get("TL"); //Control Hub Port 2
      bottomLeft = hardwareMap.dcMotor.get("BL"); //Control Hub Port 3
      */

        switch (Dir) {
            case "Forward":
                topLeft.setDirection(DcMotorSimple.Direction.REVERSE);
                topRight.setDirection(DcMotorSimple.Direction.REVERSE);
                bottomLeft.setDirection(DcMotorSimple.Direction.FORWARD);
                bottomRight.setDirection(DcMotorSimple.Direction.FORWARD);
                break;
            case "Backward":
                topLeft.setDirection(DcMotorSimple.Direction.FORWARD);
                topRight.setDirection(DcMotorSimple.Direction.FORWARD);
                bottomLeft.setDirection(DcMotorSimple.Direction.REVERSE);
                bottomRight.setDirection(DcMotorSimple.Direction.REVERSE);
                break;
            case "Left":
                topLeft.setDirection(DcMotorSimple.Direction.FORWARD);
                topRight.setDirection(DcMotorSimple.Direction.REVERSE);
                bottomLeft.setDirection(DcMotorSimple.Direction.FORWARD);
                bottomRight.setDirection(DcMotorSimple.Direction.REVERSE);
                break;
            case "Right":
                topLeft.setDirection(DcMotorSimple.Direction.REVERSE);
                topRight.setDirection(DcMotorSimple.Direction.FORWARD);
                bottomLeft.setDirection(DcMotorSimple.Direction.REVERSE);
                bottomRight.setDirection(DcMotorSimple.Direction.FORWARD);
                break;
        }
    }
}

//Old Robot Auto Code

////Starts 1 square away from the carousel
//package org.firstinspires.ftc.teamcode;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.ColorSensor;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.util.Range;
//
////1 centimeter - 0.393701 inches
//
//@Autonomous(name = "FY21Blue_1_Carousel", group = "team")
//
//public class FY21BlueStorageStart extends LinearOpMode {
//   //define motors and stuff
//   DcMotor topRight;
//   DcMotor bottomRight;
//   DcMotor topLeft;
//   DcMotor bottomLeft;
//   DcMotor carouselSpinner;
//   ColorSensor duckScannerLeft; //left
//   ColorSensor duckScannerRight; //right
//   DcMotor linearSlide;
//   //ColorSensor ColorSensor;
//   //define variables
//   int currentstep = 0;
//   String barcode = "none";
//
//   public void runOpMode() {
//      //define hardware map
//      duckScannerLeft = hardwareMap.colorSensor.get("DSL"); //Extension Hub I2C bus 3
//      duckScannerRight = hardwareMap.colorSensor.get("DSR"); //Control Hub I2C bus 3
//      topRight = hardwareMap.dcMotor.get("TR"); //Control Hub Port 0
//      bottomRight = hardwareMap.dcMotor.get("BR"); //Control Hub Port 1
//      topLeft = hardwareMap.dcMotor.get("TL"); //Control Hub Port 2
//      bottomLeft = hardwareMap.dcMotor.get("BL"); //Control Hub Port 3
//      carouselSpinner = hardwareMap.dcMotor.get("CS"); //Expansion Hub Port 2
//
//
//      topLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//      topRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//      bottomLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//      bottomRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//      topLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//      topRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//      bottomLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//      bottomRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//      /*
//      topLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//       topRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//      bottomLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//      bottomRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//*/
//
//      waitForStart();
//      while (opModeIsActive()) {
//
//         if (currentstep == 0) {
//            currentstep++;
//         }
//
//         if (currentstep == 1) {
//
//            Mecanum_drive("Backward", 0.5, 625);
//                carouselSpinner.setPower(-0.5);
//                //sleep(3500), this is if the shield does not get put on, this is instead of the sleep statement bellow (sleep (3800))
//                sleep(3800);
//                carouselSpinner.setPower(0);
//                Mecanum_drive("Forward", 0.5, 625);
//                Mecanum_Turn("Right", 1, 408);
//                Mecanum_drive("Forward", 0.5, 665);
//                Mecanum_Turn("Right", 1, 408);
//                Mecanum_drive("Forward", 0.5, 810);
//            currentstep++;
//         }
//      }
//   }
//
//
//   public void Mecanum_drive(String Dir, double Spd, long Slp) {
//      /*
//      topRight = hardwareMap.dcMotor.get("TR"); //Control Hub Port 0
//      bottomRight = hardwareMap.dcMotor.get("BR"); //Control Hub Port 1
//      topLeft = hardwareMap.dcMotor.get("TL"); //Control Hub Port 2
//      bottomLeft = hardwareMap.dcMotor.get("BL"); //Control Hub Port 3
//      */
//
//      switch (Dir) {
//         case "Forward":
//            topLeft.setDirection(DcMotorSimple.Direction.REVERSE);
//            topRight.setDirection(DcMotorSimple.Direction.REVERSE);
//            bottomLeft.setDirection(DcMotorSimple.Direction.FORWARD);
//            bottomRight.setDirection(DcMotorSimple.Direction.FORWARD);
//            break;
//         case "Backward":
//            topLeft.setDirection(DcMotorSimple.Direction.FORWARD);
//            topRight.setDirection(DcMotorSimple.Direction.FORWARD);
//            bottomLeft.setDirection(DcMotorSimple.Direction.REVERSE);
//            bottomRight.setDirection(DcMotorSimple.Direction.REVERSE);
//            break;
//         case "Left":
//            topLeft.setDirection(DcMotorSimple.Direction.FORWARD);
//            topRight.setDirection(DcMotorSimple.Direction.REVERSE);
//            bottomLeft.setDirection(DcMotorSimple.Direction.FORWARD);
//            bottomRight.setDirection(DcMotorSimple.Direction.REVERSE);
//            break;
//         case "Right":
//            topLeft.setDirection(DcMotorSimple.Direction.REVERSE);
//            topRight.setDirection(DcMotorSimple.Direction.FORWARD);
//            bottomLeft.setDirection(DcMotorSimple.Direction.REVERSE);
//            bottomRight.setDirection(DcMotorSimple.Direction.FORWARD);
//            break;
//      }
//      /*Dist = Math.abs(Dist);
//      topLeft.setTargetPosition(Dist+topLeft.getCurrentPosition());
//      topRight.setTargetPosition(Dist+topRight.getCurrentPosition());
//      bottomLeft.setTargetPosition(Dist+bottomLeft.getCurrentPosition());
//      bottomRight.setTargetPosition(Dist+bottomRight.getCurrentPosition());
//      topLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//      topRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//      bottomLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//      bottomRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//*/
//      Spd = Range.clip(Spd, 0, 1);
//      topLeft.setPower(Spd);
//      topRight.setPower(Spd);
//      bottomLeft.setPower(Spd);
//      bottomRight.setPower(Spd);
//
//
//      // while (opModeIsActive() && topLeft.isBusy())
//      //leftMotor.getCurrentPosition() < leftMotor.getTargetPosition())
//      // {
//      telemetry.addData("encoder-fwd-left", topLeft.getCurrentPosition() + "busy=" + topLeft.isBusy());
//      telemetry.addData("encoder-fwd-right", topRight.getCurrentPosition() + "busy=" + topRight.isBusy());
//      telemetry.addData("encoder-fwd-BL", bottomLeft.getCurrentPosition() + "busy=" + bottomLeft.isBusy());
//      telemetry.addData("encoder-fwd-BR", bottomRight.getCurrentPosition() + "busy=" + bottomRight.isBusy());
//      telemetry.addData("Dir:", Dir + "Spd:" + Spd + "Slp:" + Slp);
//      telemetry.update();
//      //    idle();
//      // }
//      //wait (run motors) for Slp number of milliseconds
//      sleep(Slp);
//
//      //stop motors
//      topLeft.setPower(0);
//      topRight.setPower(0);
//      bottomLeft.setPower(0);
//      bottomRight.setPower(0);
//   }
//
//   public void Mecanum_Turn(String DirT, double SpdT, long SlpT) {
//    /*  double RobotDiameter = 20; //Max robot size is 18x18 with max diagonal width of 25.46 in)
//      //Robot spins in a circle, rough diameter of robot's circle can be no more than 25.42 (diagonal)
//      double RobotCircumference = RobotDiameter * 3.14;//Max circumference of Robot (d * pi) = 80 in
//      double WheelSize = 4;  //diameter in inches of wheels (the engineers like 4in)
//      double WheelCircumference = WheelSize*3.14; //Circumference (d * pi) of wheel (distance wheel travels for 1 rotation)
//      double RotationsPerCircle = RobotCircumference/WheelCircumference;// wheel rotations to turns in complete circle
//      int DriveTicks = 1440;  //1 wheel rotation = DriveTicks - based on motor and gear ratio  => 1 Tetrix DC motor 60:1 revolution = 1440 encoder ticks (20:1 = 480 ticks (divide by 60/20) or 400 ticks = 1 foot)
//      //DriveTicks * RotationsPerCircle = 360 degrees
//      //Rotations per degree
//      int TicksPerDegree = (int) Math.round((DriveTicks * RotationsPerCircle)/360);
//      int Rotate = (int) Math.round(Deg * TicksPerDegree);
//      */
//
//      /*telemetry.addData("Rotating", Rotate + "ticks or " + Deg + " degrees");
//      telemetry.update();*/
//
//      /*topLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//      topRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//      bottomLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//      bottomRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);*/
//
//      if (DirT.equals("Left")) {
//         topLeft.setDirection(DcMotorSimple.Direction.REVERSE);
//         topRight.setDirection(DcMotorSimple.Direction.REVERSE);
//         bottomLeft.setDirection(DcMotorSimple.Direction.REVERSE);
//         bottomRight.setDirection(DcMotorSimple.Direction.REVERSE);
//      }
//      if (DirT.equals("Right")) {
//         topLeft.setDirection(DcMotorSimple.Direction.FORWARD);
//         topRight.setDirection(DcMotorSimple.Direction.FORWARD);
//         bottomLeft.setDirection(DcMotorSimple.Direction.FORWARD);
//         bottomRight.setDirection(DcMotorSimple.Direction.FORWARD);
//      }
//     /* Rotate = Math.abs(Rotate);
//      topLeft.setTargetPosition(Rotate);
//      topRight.setTargetPosition(Rotate);
//      bottomLeft.setTargetPosition(Rotate);
//      bottomRight.setTargetPosition(Rotate);
//      topLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//      topRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//      bottomLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//      bottomRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//*/
//      SpdT = Range.clip(SpdT, 0, 1);
//      topLeft.setPower(SpdT);
//      topRight.setPower(SpdT);
//      bottomLeft.setPower(SpdT);
//      bottomRight.setPower(SpdT);
//
//      sleep(SlpT);
//
//      /*while (opModeIsActive() && topLeft.isBusy())
//      //leftMotor.getCurrentPosition() < leftMotor.getTargetPosition())
//      {
//         telemetry.addData("encoder-fwd-left", topLeft.getCurrentPosition() + "busy=" + topLeft.isBusy());
//         telemetry.addData("encoder-fwd-right", topRight.getCurrentPosition() + "busy=" + topRight.isBusy());
//         telemetry.update();
//         idle();
//      }
//       */
//      //stop
//      topLeft.setPower(0);
//      topRight.setPower(0);
//      bottomLeft.setPower(0);
//      bottomRight.setPower(0);
//   }
//
//}