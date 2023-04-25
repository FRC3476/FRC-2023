# The Brain of Baja

Code Orange's robot code for the 2023 FRC season, Rapid React

# High Level Overview

- We're using [AdvantageKit](https://github.com/Mechanical-Advantage/AdvantageKit)
  and [AdvantageScope](https://github.com/Mechanical-Advantage/AdvantageScope)
  to log *everything* in our robot code. We're utilizing
  an [IO Layer](https://github.com/Mechanical-Advantage/AdvantageKit/blob/main/docs/CODE-STRUCTURE.md#code-structure--io-layers)
  to abstract the control logic and the actual and record all our inputs to our subsystems.
    - We have over 450 fields being logged!
- The [AutoBuilder](https://github.com/FRC3476/AutoBuilder) is used for creating all out autonomouses
- Controls
    - Normal Usage
        - The Operator selects what scoring location we want in a grid using a button panel arranged in a 3x3 grid.
        - The driver uses an Xbox Controller
            - Movement and Strafing are on the sticks
            - Left Paddle triggers [auto lineup](#Auto-Lineup)
            - Right Paddle [toggles the mechanism up and down](#mechanism-toggling)
            - Left Bumper opens/closes the grabber
            - Left Trigger toggles ground intake mode
        - Automations
            - A beam break sensor is used to automatically close the grabber in certain mechanism states
            - The mechanism is automatically closed in certain states once the grabber is closed
            - The grabber is always automatically closed when retracting
    - Manual Controls
        - Our stick's Y axis can be used to rotate the pivot of our grabber mechanism up and down
        - A joystick on our button panel can be used to move the grabber up and down
        - Extra buttons on the joystick can be used to explicitly set the mechanism to different states
            - Would be required if our pose estimator completely failed (which has never happened at comp)
            - Mainly used for systems testing in the pit

# Vision

Our vision system utilizes 3 cameras (2 limelight and
an [Intel Realsense D455 connected to a Beelink MiniPC](https://github.com/FRC3476/AprilTags)).

## Intel Realsense D455

The Camera is mounted on top of our elevator facing forward on our robot.

The pose each for tag is sent over NT to the roborio. Each tag pose is then transformed into field space on the rio and
sent to the pose estimator individually.

When calculating the translation of the robot from each tag we don't use the tags' orientation. We instead use the
expected orientation using information from our gyro which helps us get useful poses from anywhere on the field.

## Limelights

After running into occlusion issues with only having our main camera at our first comp we added two limelights that
point to the left & right at an ~15-degree angle. They ensure that we always have a tag visible while we're scoring and
picking up game pieces.

We're simply using the botpose the limelights output and applying them directly to our pose estimator. We found that the
pose estimation degraded quickly once we moved too far away and throw out any poses when we're > 4 meters away from a
tag. It's possible that the new camera calibration features would help significantly for this.

# Auto Lineup

- While the button is held down control of the robot's drivetrain is swapped to the robot code.
- When the button is initially pressed the robot requests the robot commands the robot to continue moving at the current
  velocity. A path is then generated asynchronously taking the robot from the current position to the target goal.

## Path Generation

### Choosing where to go

- The same button is used to line up to scoring nodes is also used to line up to pickup positions
    - We first decide weather we want to go to a pickup position or scoring node based on which side of the field we're
      on + some velocity based lookahead.
        - Scoring
            - The grid that we want to go is predicted based on the robot's position + a velocity lookahead.
            - Once the grid is determined the actual node we want to go to is based on what is selected on the
              operator's button panel
        - Pickup
            - The normal left-paddle will auto lineup to the inner double-substation
            - The right-bumper is used to line up to the outer double-substation
                - We initially tried position and velocity based prediction for this, but it was too unpredictable for
                  our driver

### Creating the path

- We start creating a spline utilizing the robot's current velocity as the heading and length (times a scaling factor)
  and the position as the starting point.
- The spline ends at the target position with a spline that points into the wall a short amount. (This helps the robot
  drive a smoother S-curve)
- Other points are dynamically added in between the start and end position if needed to avoid the charging station.
- We then pass this to WPILib's trajectory generator to return a path the robot can drive.

### Driving the path

- We use the same path following code that we have for driving during the autonomous period.
- Once the path is complete we have an additional PID controller that continues running.
- Vision is continually correcting the position of the robot as it drives the path.

# Mechanism Toggling

- The same button is used to bring the mechanism up for scoring and double-substation pickup.
    - The robot's position on the field determines the which setpoint to go to
- In scoring the selected position on the button panel is also determines the setpoint to use.
- The selected position also determines which piece mode to use which changes the strength of the grabber.