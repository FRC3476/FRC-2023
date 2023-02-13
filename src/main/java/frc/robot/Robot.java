// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.dacubeking.AutoBuilder.robot.robotinterface.AutonomousContainer;
import com.dacubeking.AutoBuilder.robot.robotinterface.CommandTranslator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.ScoringPositionManager.PositionType;
import frc.subsytem.AbstractSubsystem;
import frc.subsytem.Elevator.Elevator;
import frc.subsytem.Elevator.ElevatorIO;
import frc.subsytem.Elevator.ElevatorIOSparkMax;
import frc.subsytem.MechanismStateManager;
import frc.subsytem.MechanismStateManager.MechanismStates;
import frc.subsytem.drive.Drive;
import frc.subsytem.drive.DriveIO;
import frc.subsytem.drive.DriveIOSparkMax;
import frc.subsytem.grabber.Grabber;
import frc.subsytem.grabber.Grabber.GrabState;
import frc.subsytem.grabber.GrabberIO;
import frc.subsytem.grabber.GrabberIOSparkMax;
import frc.subsytem.robottracker.RobotTracker;
import frc.subsytem.telescopingarm.TelescopingArm;
import frc.subsytem.telescopingarm.TelescopingArmIO;
import frc.subsytem.telescopingarm.TelescopingArmIOSparkMax;
import frc.subsytem.vision.VisionHandler;
import frc.utility.Controller;
import frc.utility.Controller.XboxButtons;
import frc.utility.ControllerDriveInputs;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import static frc.robot.Constants.*;
import static java.lang.Math.abs;


/**
 * The VM is configured to automatically run this class, and to call the methods corresponding to each mode, as described in the
 * TimedRobot documentation. If you change the name of this class or the package after creating this project, you must also update
 * the build.gradle file in the project.
 */
public class Robot extends LoggedRobot {

    private double disabledTime = 0;

    private @NotNull static Drive drive;
    private @NotNull static RobotTracker robotTracker;
    private @NotNull static VisionHandler visionHandler;

    private @NotNull static Elevator elevator;
    private @NotNull static TelescopingArm telescopingArm;
    private @NotNull static Grabber grabber;
    private @NotNull static MechanismStateManager mechanismStateManager;


    private @NotNull Controller xbox;
    private @NotNull Controller stick;

    private @NotNull Controller buttonPanel;

    // Autonomous
    private final LoggedDashboardChooser<String> autoChooser = new LoggedDashboardChooser<>("AutoChooser");

    public static final LoggedDashboardChooser<String> sideChooser = new LoggedDashboardChooser<>("SideChooser");

    /**
     * This method is run when the robot is first started up and should be used for any initialization code.
     */
    @Override
    public void robotInit() {
        Logger.getInstance().recordMetadata("ProjectName", "FRC2023"); // Set a metadata value

        if (isReal()) {
            Logger.getInstance().addDataReceiver(new WPILOGWriter("/home/lvuser/logs"));
            Logger.getInstance().addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
            new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging

            drive = new Drive(new DriveIOSparkMax());
            elevator = new Elevator(new ElevatorIOSparkMax());
            telescopingArm = new TelescopingArm(new TelescopingArmIOSparkMax());
            grabber = new Grabber(new GrabberIOSparkMax());
        } else {
            setUseTiming(false); // Run as fast as possible
            String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
            Logger.getInstance().setReplaySource(new WPILOGReader(logPath)); // Read replay log
            Logger.getInstance().addDataReceiver(
                    new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log

            drive = new Drive(new DriveIO() {});
            elevator = new Elevator(new ElevatorIO() {});
            telescopingArm = new TelescopingArm(new TelescopingArmIO() {});
            grabber = new Grabber(new GrabberIO() {});
        }

        Logger.getInstance().start(); // Start logging! No more data receivers, replay sources, or metadata values may be added

        robotTracker = new RobotTracker();
        visionHandler = new VisionHandler();
        mechanismStateManager = new MechanismStateManager();


        visionHandler.start();
        robotTracker.start();
        mechanismStateManager.start();
        elevator.start();
        telescopingArm.start();
        grabber.start();
        drive.start();
        ScoringPositionManager.getInstance();

        xbox = new Controller(0);
        stick = new Controller(1);
        buttonPanel = new Controller(2);

        AutonomousContainer.getInstance().setDebugPrints(true);
        AutonomousContainer.getInstance().initialize(
                true,
                new CommandTranslator(
                        drive::setAutoPath,
                        drive::stopMovement,
                        drive::setAutoRotation,
                        drive::isFinished,
                        drive::getAutoElapsedTime,
                        robotTracker::resetPose,
                        false

                ),
                false,
                null
        );
        AutonomousContainer.getInstance().getAutonomousNames().forEach(name -> autoChooser.addOption(name, name));

        sideChooser.addDefaultOption("Blue", "blue");
        sideChooser.addOption("Red", "red");

        if (IS_PRACTICE) {
            for (int i = 0; i < 10; i++) {
                System.out.println("USING PRACTICE BOT CONFIG");
            }
        }
    }


    /**
     * This method is called every robot packet, no matter the mode. Use this for items like diagnostics that you want ran during
     * disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic methods, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        AbstractSubsystem.tick();
    }


    @Override
    public void autonomousInit() {
        drive.setBrakeMode(true);
        String autoName = autoChooser.get();
        if (autoName != null) {
            AutonomousContainer.getInstance().runAutonomous(autoName, sideChooser.get(), true);
        }
    }


    /**
     * This method is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {

    }


    /**
     * This method is called once when teleop is enabled.
     */
    @Override
    public void teleopInit() {
        drive.setBrakeMode(true);
    }


    private @Nullable Pose2d teleopDrivingAutoAlignPosition = new Pose2d();

    {
        Logger.getInstance().recordOutput("Auto Align Y", teleopDrivingAutoAlignPosition.getY());
        Logger.getInstance().recordOutput("Auto Align X", teleopDrivingAutoAlignPosition.getX());
        Logger.getInstance().recordOutput("Auto Align Angle", teleopDrivingAutoAlignPosition.getRotation().getDegrees());
    }


    enum WantedMechanismState {
        STOWED, SCORING, FLOOR_PICKUP, STATION_PICKUP
    }

    private WantedMechanismState wantedMechanismState = WantedMechanismState.STOWED;

    private boolean isGrabberOpen = false;

    private double wantedX = -0.489;
    private double wantedY = 0.249;
    private double wantedAngle = MAX_WRIST_ANGLE - 2;

    private final boolean arcadeMode = true;

    /**
     * This method is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
        xbox.update();
        stick.update();
        buttonPanel.update();
        double wantedRumble = 0;


        var scoringPositionManager = ScoringPositionManager.getInstance();
        if (scoringPositionManager.updateSelectedPosition(buttonPanel)) {
            teleopDrivingAutoAlignPosition = null;
        }

//        if (xbox.getRawButton(XboxButtons.START)) { //Should be remapped to one of the back buttons
//            if (xbox.getRisingEdge(XboxButtons.START) || teleopDrivingAutoAlignPosition == null) {
//                updateTeleopDrivingTarget(scoringPositionManager);
//                assert teleopDrivingAutoAlignPosition != null;
//            }
//
//            if (!drive.driveToPosition(
//                    teleopDrivingAutoAlignPosition.getTranslation(),
//                    teleopDrivingAutoAlignPosition.getRotation(),
//                    getControllerDriveInputs()
//            )) {
//                // We failed to generate a trajectory
//                wantedRumble = 1;
//            }
//        } else {
//            drive.swerveDriveFieldRelative(getControllerDriveInputs());
//        }

        if (xbox.getRisingEdge(XboxButtons.A)) {
            robotTracker.resetPose(new Pose2d(robotTracker.getLatestPose().getTranslation(), new Rotation2d()));
        }

        if (stick.getRisingEdge(7)) {
            if (wantedMechanismState == WantedMechanismState.STOWED) {
                wantedMechanismState = WantedMechanismState.SCORING;
            } else {
                wantedMechanismState = WantedMechanismState.STOWED;
            }
        }

        if (stick.getRisingEdge(9)) {
            if (wantedMechanismState == WantedMechanismState.STOWED) {
                wantedMechanismState = WantedMechanismState.FLOOR_PICKUP;
            } else {
                wantedMechanismState = WantedMechanismState.STOWED;
            }
        }

        if (stick.getRisingEdge(11)) {
            if (wantedMechanismState == WantedMechanismState.STOWED) {
                wantedMechanismState = WantedMechanismState.STATION_PICKUP;
            } else {
                wantedMechanismState = WantedMechanismState.STOWED;
            }
        }

        if (!arcadeMode) {
            switch (wantedMechanismState) {
                case STOWED -> mechanismStateManager.setState(MechanismStates.STOWED);
                case SCORING -> {
                    int level = scoringPositionManager.getSelectedPosition().getLevel();
                    if (level == 0) {
                        mechanismStateManager.setState(MechanismStates.LOW_SCORING);
                    } else if (level == 1) {
                        mechanismStateManager.setState(MechanismStates.MIDDLE_SCORING);
                    } else if (level == 2) {
                        mechanismStateManager.setState(MechanismStates.HIGH_SCORING);
                    }
                }
                case FLOOR_PICKUP -> mechanismStateManager.setState(MechanismStates.FLOOR_PICKUP);
                case STATION_PICKUP -> mechanismStateManager.setState(MechanismStates.STATION_PICKUP);
            }
        } else {
            var inputs = new ControllerDriveInputs(stick.getRawAxis(0), stick.getRawAxis(1), stick.getRawAxis(3));
            inputs.applyDeadZone(0.2, 0.2, 0.25, 0.2);
            inputs.squareInputs();
            wantedAngle += inputs.getY();
            var dx = -buttonPanel.getRawAxis(0);
            var dy = buttonPanel.getRawAxis(1);

            if (abs(dx) < 0.1) dx = 0;
            if (abs(dy) < 0.1) dy = 0;
            wantedX += dx / 50.0;
            wantedY += dy / 50.0;

            var wantedMechState = new MechanismStateManager.MechanismStateCoordinates(wantedX, wantedY, wantedAngle);
            var limitedMechState = MechanismStateManager.limitCoordinates(wantedMechState);
            wantedX = limitedMechState.xMeters();
            wantedY = limitedMechState.yMeters();
            wantedAngle = limitedMechState.grabberAngleDegrees();
            mechanismStateManager.setState(new MechanismStateManager.MechanismStateCoordinates(wantedX, wantedY, wantedAngle));
            Logger.getInstance().recordOutput("Robot/Wanted X", wantedX);
            Logger.getInstance().recordOutput("Robot/Wanted Y", wantedY);
            Logger.getInstance().recordOutput("Robot/Wanted Angle", wantedAngle);
        }
        Logger.getInstance().recordOutput("Robot/Wanted Mechanism State", wantedMechanismState.name());

        if (xbox.getRisingEdge(XboxButtons.B)) {
            isGrabberOpen = !isGrabberOpen;

            if (isGrabberOpen) {
                grabber.setGrabState(GrabState.OPEN);
            } else {
                if (scoringPositionManager.getWantedPositionType() == PositionType.CONE) {
                    grabber.setGrabState(GrabState.GRAB_CONE);
                } else {
                    grabber.setGrabState(GrabState.GRAB_CUBE);
                }
            }
        }

        Logger.getInstance().recordOutput("Robot/Is Grabber Open", isGrabberOpen);

        if ((wantedMechanismState == WantedMechanismState.FLOOR_PICKUP || wantedMechanismState == WantedMechanismState.STATION_PICKUP)
                && isGrabberOpen) {
            grabber.setRollerVoltage(GRABBER_ROLLER_VOLTAGE);
        } else if (!isGrabberOpen) {
            grabber.setRollerVoltage(GRABBER_ROLLER_IDLE);
        } else {
            grabber.setRollerVoltage(0);
        }

        xbox.setRumble(RumbleType.kBothRumble, wantedRumble);
    }

    private void updateTeleopDrivingTarget(ScoringPositionManager scoringPositionManager) {
        double x, y;
        Rotation2d rotation;
        if (robotTracker.isOnRedSide() == isRed()) {
            // We're on the same side as our alliance
            // Try to go to the scoring position
            y = ScoringPositionManager.getBestFieldY(
                    scoringPositionManager.getSelectedPosition(),
                    isRed(),
                    robotTracker.getLatestPose().getTranslation(),
                    robotTracker.getVelocity()
            );

            if (isRed()) {
                x = Constants.GRIDS_RED_X + HALF_ROBOT_WIDTH;
                rotation = Constants.SCORING_ANGLE_RED;
            } else {
                x = Constants.GRIDS_BLUE_X - HALF_ROBOT_WIDTH;
                rotation = Constants.SCORING_ANGLE_BLUE;
            }
        } else {
            // We're on the opposite side as our alliance
            // Try to go to the pickup position
            y = PICKUP_POSITION_Y;
            if (isRed()) {
                x = PICKUP_POSITION_X_OFFSET_FROM_WALL;
                rotation = PICKUP_ANGLE_RED;
            } else {
                x = FIELD_WIDTH_METERS - PICKUP_POSITION_X_OFFSET_FROM_WALL;
                rotation = PICKUP_ANGLE_BLUE;
            }
        }

        Logger.getInstance().recordOutput("Auto Align Y", y);
        Logger.getInstance().recordOutput("Auto Align X", x);
        Logger.getInstance().recordOutput("Auto Align Angle", rotation.getDegrees());

        teleopDrivingAutoAlignPosition = new Pose2d(x, y, rotation);
    }


    /**
     * This method is called once when the robot is disabled.
     */
    @Override
    public void disabledInit() {
        AutonomousContainer.getInstance().killAuto();
        disabledTime = Timer.getFPGATimestamp();
    }


    /**
     * This method is called periodically when disabled.
     */
    @Override
    public void disabledPeriodic() {
        if (Timer.getFPGATimestamp() - disabledTime > Constants.COAST_AFTER_DISABLE_TIME) {
            drive.setBrakeMode(false);
        }
    }


    /**
     * This method is called once when test mode is enabled.
     */
    @Override
    public void testInit() {
    }


    /**
     * This method is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
        xbox.update();
        if (xbox.getRawButton(XboxButtons.X) && xbox.getRawButton(XboxButtons.B)
                && xbox.getRisingEdge(XboxButtons.X) && xbox.getRisingEdge(XboxButtons.B)) {
            drive.resetAbsoluteZeros();
        }
    }

    private ControllerDriveInputs getControllerDriveInputs() {
        if (xbox.getRawButton(Controller.XboxButtons.X)) {
            return new ControllerDriveInputs(xbox.getRawAxis(1), xbox.getRawAxis(0), xbox.getRawAxis(4))
                    .applyDeadZone(0.2, 0.2, 0.2, 0.2).squareInputs();
        } else {
            return new ControllerDriveInputs(xbox.getRawAxis(1), xbox.getRawAxis(0), xbox.getRawAxis(4))
                    .applyDeadZone(0.05, 0.05, 0.2, 0.2).squareInputs();
        }
    }

    public boolean isRed() {
        return sideChooser.get().equals("red");
    }

    public static @NotNull Drive getDrive() {
        return drive;
    }

    public static @NotNull RobotTracker getRobotTracker() {
        return robotTracker;
    }

    public static @NotNull VisionHandler getVisionHandler() {
        return visionHandler;
    }

    public static @NotNull Grabber getGrabber() {
        return grabber;
    }

    public static @NotNull TelescopingArm getTelescopingArm() {
        return telescopingArm;
    }

    public static @NotNull Elevator getElevator() {
        return elevator;
    }
}
