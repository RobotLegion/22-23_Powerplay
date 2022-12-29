## TeamCode Module

###Teleop Explanation
Teleop mode is the section of gameplay that the driver controls the robot.
In our teleop mode we have basic driving, intake controls, and lift controls.

###Autonomous
In autonomous mode the robot scores 1 cone on a medium junction and then parks in the correct parking zone with the custom signal.
We are using an IMU to detect how much we rotate. The robot also uses functions we made that tell it to open or close the claw. The robot

###Drive Teleop
Our drive is with Mecanum wheels which means it can strafe as well as move forward, backward, and rotate.
To move the robot we use the joysticks on gamepad1 and we used some formulas so that when we use the joystick,
to move forward even if it reads something like 10000 the value will be clipped to just 1. Same with backward.
We also have a variable, "speed" that sets the speed of the robot to a slow speed so it doesn't go crazy.

###Lift
Our lift is coded to go up one level using a bumper press and down one level using the other bumper press.
The lift is also coded to use the right joystick for fine-grain movements, but you should not use it to move to different lift levels, because it will not update the encoder.
This means that if the lift is at smallLiftLevel, and then you use the joystick to move to mediumLiftLevel, but then needed to go back to smallLiftLevel, if you tried to go down one level with the bumper press, you would go to the groundLiftLevel instead of smallLiftLevel because the robot still thought you were at smallLiftLevel, not mediumLiftLevel.
The lift is controlled by two motors we call Liftleft and Liftright and to make sure that the code worked, we added telemetry to say up and down.

###Intake
Our intake is a basic claw that moves open and closed using servos.
To make the claw open, we make the servo move the claw to position 0 and to close the claw it moves to position 1.

###Color Sensors
For the color sensor we have a few different objects to detect.
In autonomous mode we have the blue, pink, and yellow signal colors and in teleop we have coded for the yellow poles,
the blue cones, and the red cones. Basically for the detection we had the sensor sense the color value for r g and b.
R g and b are red green and blue. Then in the code, we added the values for r g and b into different comparison statements
 so if r g and b equalled certain values, then the robot would know what object/color it was looking at.

###Drive Autonomous
Driving in autonomous is based on variables we created for different spots on the field.
The different variables we had for autonomous was; p, which is the distance the robot has to travel from its start position to the parking zone,
c is the distance the robot needs to travel from the start position to the signal cone to read it,
d is the distance the robot has to go from the signal cone to the distance it can safely strafe left and right,
and the last variable is s which is the distance for how far the robot has to strafe after it reaches position d.
To get the robot to these different variables, we have to set speed to go to the variable which we measured in feet.
This is the case for each variable except for c, which we used alpha values to make the robot drive until it reached an alpha value of 200
or more. An alpha value is basically just the color intensity reading the color so it can tell how far it is from an object.

###Spin Autonomous
Spinning in autonomous uses an IMU (explanation of IMU abbreviation) which measures the angle the robot is turning. We use the IMU to measure the angle the robot has already turned and we can use that information to tell the robot how much farther it needs to turn before it has reached its target angle. In our autonomous mode the robot needs to turn to get to a position it can drop the cone into a medium junction and then turn back into a position it can strafe to be in line to park in a parking zone.

###Lift Autonomous
(Explain how the robot uses the lift to score on the medium junction.)

###Release Autonomous
(Explain how the robot releases the cone from the claw during autonomous mode.)

###