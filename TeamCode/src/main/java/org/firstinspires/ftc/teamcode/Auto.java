package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;

@Autonomous(name="autonomous", group="Robot")
@Disabled

public class Auto extends LinearOpMode {
    //Turquoise: 64,224,208 Parking 1
    //Magenta: 255 00 191 Parking 2
    //Green: 00 255 00 Parking 3
    //Alpha threshold is about 300
    //Flat colors
    //Drive towards the signal "How do we know when we are 2cm from the cone?"
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

    double speed = 1;
    double scale = 1;


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
            while(color.alpha()<300) {
                topRight.setPower(0.7071 / scale);
                bottomRight.setPower(0.7071 / scale);
                topLeft.setPower(0.7071 / scale);
                bottomLeft.setPower(0.7071 / scale);
            }
        }
    }
}
