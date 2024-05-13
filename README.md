# Swerve Drive From Built Chassis To Path Following Steps

## Pre-requisites
0. Make sure all motors have valid CTRE pro license and are visible in phoenix tool
1. Update all motor controllers to latest PRO version
2. Make sure all motors have correct can id according to code (change the canid on the controller, dont change the code)
3. Make sure the wheelbase width and length, motor choice, as well as gearing are set correctly in swerveconstants.h

## Make sure the drive and steering motors have the correct inversion
0. The robot should be on blocks so it can't drive on the ground!!!
1. Using pheonix tuner, drive the azimuth motor "forward" (blinking green), the module azimuth should rotate counter-clockwise when viewed from the top
2. Manually move the azimuth of the module to 0 degrees (0 being where the drive wheel will drive forward with the bevel gear facing the center of the robot) and drive the drive motor "forward" (blinking green). The module wheel should be spinning so that the robot would move forward if on the ground.
3. Do this for all 4 modules and record them. Change the code to fix this if any of them are wrong.

## Calibrate the encoder offset for each module
0. While the robot is still on blocks, shove a straight metal bar on the left side of the wheel forks so they are parallel to eachother. MAKE SURE THE BEVEL GEAR OF THE MODULE IS FACING TOWARDS THE CENTER OF THE ROBOT
1. In phoenix tuner, record the RAW encoder value (without offset) for both the front and back left encoders. Have someone hold the bar in place without moving. 
2. Repeat for the right side
3. Change the encoder offsets in the code to match. 

## Calibrate gyro
0. Make sure the pigeon 2 imu is securely mounted on the robot
1. Follow the direction from CTRE to calibrate the IMU.
2. Record the offset values and change the code to match

## Calibrate FF + PID For Steering Motors
0. Put the robot on the ground and manually move the azimuth motor to its 0 position
1. Run the sysid steer torque commands using a controller. Make sure to run all 4 tests (quasistatic and dynamic forward and backwards)
2. Using tuner, generate a log file getting the motor position, motor velocity, and motor torquecurrent and sysid state.
3. Open sysid and find gains (use sysid guide for this)
4. If you are using mk4n's in the back, repeat for back module using a different sysid command
5. Record the ff and PID constants and update the code to match

## Test swerve movement
0. Set the robot on the ground
1. Point the front of the robot away from you and make sure the driverstation is behind the robot.
2. Open advantage scope. All of the swerve movement should match what you see in the "Swerve" Tab in advantage scope.
3. Change the driverstation alliance to blue
4. Verify the robot moves with the joystick in the correct way. Push the left stick forward slowly. the robot should move forward. Push the left stick back and the robot should move backwards.  
5. Push the left stick to the right. The robot should move right. Push the left stick to the left. The robot should move left.
6. Push the right stick to the right. The robot should rotate clockwise looking at it from the top. Push the right stick left. The robot should rotate counter clockwise
7. Rotate the robot with the stick to 90 degrees and make it go forward. The robot should move away from you. (not to the right)
8. In the advantage scope odometry tab, the robot should be in the correct spot on the field as if the robot was started at 0,0 (the right side wall of the blue alliance corner.)

## Determining slip current values
0. Put the robot against a wall with frc carpet on the floor.
1. Open tuner and graph the torquecurrent and velocity of one of the drive wheels
2. Slowly drive the robot into the wall while slowly increasing the speed using the joystick until the velocity of the wheel is not zero
3. Record the torque current of the wheel at the point where the velocity isnt zero
4. Update the slip current value in the code to match

## Calibrate FF + PID for Drive motors
0. Put the robot on the ground and manually move the all of the azimuth motors to their 0 positions
1. Run the sysid drive torque commands using a controller. Make sure to run all 4 tests (quasistatic and dynamic forward and backwards)
2. Using tuner, generate a log file getting the motor position, motor velocity, and motor torquecurrent and sysid state.
3. Open sysid and find gains (use sysid guide for this)
5. Record the ff and PID constants and update the code to match

## Calibrate Wheel Radius
0. Put the robot on the ground facing forward and turn it on
1. Run the wheel radius command from a controller and make sure it rotates fully around 2 times. Do the same in reverse mode
2. The wheel radius will be printed in the console connected to the DS. Record this and change the code to match
3. Put a tape measure on the ground and drive the robot forward 3 feet. Make sure your odometry measured distance matches

## Test closed loop driving
0. Change the drive command to use closed loop driving instead of open loop
1. Move the robot around in teleop to make sure it responds the same as open loop driving

## Basic path following
0. Create a path in choreo that starts at 0,0 and moves forward 3 ft give it a slow max velocity 
1. Run the path and see if it completes the path within tolerance
2. Adjust path pid tuning to make it follow correctly.
3. Try driving a path where you drive in a square
4. Try a path where you drive straight and rotate
5. Try a more complicated path
6. Up the choreo max torque until you can't track accuratley. 
7. Add vision (EZ)
8. Keep upping max torque until you cant even track with vision 