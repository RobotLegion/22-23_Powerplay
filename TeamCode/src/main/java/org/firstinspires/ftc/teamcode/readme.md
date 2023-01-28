## TeamCode Module

###Teleop Explanation
Teleop mode is the section of gameplay that the driver controls the robot.
In our teleop mode we have basic driving, intake controls, and lift controls.

###Autonomous
In autonomous mode the robot scores 1 cone on a medium junction, a low junction, and then parks in the correct parking zone with the custom signal.
We are using an IMU to detect how much we rotate. The robot also uses functions we made that tell it to open or close the claw.
The robot scores on the low junction using our lift level command that takes the lift to a preset height.
Next the robot scores on the medium junction using the cone stack. Before the robot goes to the cone stack, it reads the color sensor first.
This is so when the robot has scored on the medium junction, it already knows whether to go to the parking zone on the left, on the right, or not to go anywhere.
We also added a new logging system to track what the robot does during an autonomous period, so that if something goes wrong, we can see the file it wrote in.
This helps us debug if something goes wrong during autonomous. We have a different logging function for a string, a string and a double, and a string and a string.
We have two autonomous modes as well, one for only the low junction and the parking zones, and the other for the robot parking in the zones,
and scoring on both the low and medium junctions.

###Drive Teleop
Our drive is with Mecanum wheels which means it can strafe as well as move forward, backward, and rotate.
To move the robot we use the joysticks on gamepad1 and we used some formulas so that when we use the joystick,
to move forward even if it reads something like 10000 the value will be clipped to just 1. Same with backward.
We also have a variable, "speed" that sets the speed of the robot to a slow speed so it doesn't go crazy.

###Lift
Our lift is coded to go up one level using a bumper press and down one level using the other bumper press.
The lift is also coded to use the right joystick for fine-grain movements, but you should not use it to move to different lift levels, because it will not update the encoder.
This means that if the lift is at smallLiftLevel, and then you use the joystick to move to mediumLiftLevel, but then needed to go back to smallLiftLevel, if you tried to go down one level with the bumper press, you would go to the groundLiftLevel instead of smallLiftLevel because the robot still thought you were at smallLiftLevel, not mediumLiftLevel.
The lift is controlled by a motor called LiftMotor and to make sure that the code worked, we added telemetry to say up and down.
The robot also uses a variable called LiftPower to keep thelift moving at a consistent speed.
The different lift levels are ground, cone level, ground junction level, low junction level, cone stack level, and medium level.

###Intake
Our intake is a basic claw that moves open and closed using servos.
Our claw functions for open and close are in the robot file so we can use them in autonomous and Teleop without making changes in different files.
To make the claw open, we make the servo move the claw to position 0 and to close the claw it moves to position 1.

###Color Sensors
For the color sensor we have a few different objects to detect.
In autonomous mode we have the blue, pink, and yellow signal colors and in Teleop we have coded for the yellow poles,
the blue cones, and the red cones. Basically for the detection we had the sensor sense the color value for r g and b.
R g and b are red green and blue. Then in the code, we added the values for r g and b into different comparison statements
so if r g and b equalled certain values, then the robot would know what object/color it was looking at.
We moved all of our color sensor code into the robot file we have so it is easier to keep track of changes.

###Drive Autonomous
Driving in autonomous is based on variables we created for different spots on the field.
The different variables we had for autonomous without the medium junction were; distanceToParking, which is the distance the robot has to travel from its start position to the parking zone,
c is the distance the robot needs to travel from the start position to the signal cone to read it,
distanceToStrafe is the distance the robot has to go from the signal cone to the distance it can safely strafe left and right,
and the last variable is distanceSidewaysToParking which is the distance for how far the robot has to strafe after it reaches position distanceToStrafe.
To get the robot to these different variables, we have to set speed to go to the variable which we measured in feet and then converted to ticks.
This is the case for each variable except for c, which we used alpha values to make the robot drive until it reached an alpha value of 200
or more. An alpha value is basically just the color intensity reading the color so it can tell how far it is from an object.
In our autonomous mode with the medium junction included, the robot needs to turn to get to a position it can drop the cone into a medium junction
and then turn back into a position it can strafe to be in line to park in a parking zone using the distanceSidewaysToParking variable.

###Spin Autonomous
Spinning in autonomous uses an IMU which measures the angle the robot is turning.
We use the IMU to measure the angle the robot has already turned and we can use that information to tell the robot how much farther it needs to turn before it has reached
its target angle.

###Lift Autonomous
The lift in autonomous is controlled by a lift function that has measurements in feet for how high the robot's lift has to go,
but then is converted to ticks using feetToTicks, so the robot can understand it.
The lift uses our new logging system to write out which lift level it is at during autonomous. It uses an array of Strings and uses current lift level as the index.

###Release Autonomous
The robot releases cones in autonomous by using our clawOpen() command we keep in the robot file so we can also access it easily in Teleop.
###