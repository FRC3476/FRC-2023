// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.dacubeking.AutoBuilder.robot.robotinterface.AutonomousContainer;
import com.dacubeking.AutoBuilder.robot.robotinterface.CommandTranslator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.subsytem.Drive;
import frc.subsytem.Drive.DriveState;
import frc.subsytem.AbstractSubsystem;
import frc.subsytem.RobotTracker;
import frc.subsytem.VisionHandler;
import frc.utility.Controller;
import frc.utility.Controller.XboxButtons;
import frc.utility.ControllerDriveInputs;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import static frc.robot.Constants.*;


/**
 * The VM is configured to automatically run this class, and to call the methods corresponding to each mode, as described in the
 * TimedRobot documentation. If you change the name of this class or the package after creating this project, you must also update
 * the build.gradle file in the project.
 */
public class Robot extends LoggedRobot {

    private double disabledTime = 0;

    private @NotNull Drive drive;
    private @NotNull RobotTracker robotTracker;
    private @NotNull VisionHandler visionHandler;

    private @NotNull Controller xbox;
    private @NotNull Controller stick;

    private @NotNull Controller buttonPanel;

    // Autonomous
    private final SendableChooser<String> autoChooser = new SendableChooser<>();

    public static final SendableChooser<String> sideChooser = new SendableChooser<>();

    /**
     * This method is run when the robot is first started up and should be used for any initialization code.
     */
    @Override
    public void robotInit() {
        Logger.getInstance().recordMetadata("ProjectName", "MyProject"); // Set a metadata value

        if (isReal()) {
            Logger.getInstance().addDataReceiver(new WPILOGWriter("/home/lvuser/logs"));
            Logger.getInstance().addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
            new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
        } else {
            setUseTiming(false); // Run as fast as possible
            String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
            Logger.getInstance().setReplaySource(new WPILOGReader(logPath)); // Read replay log
            Logger.getInstance().addDataReceiver(
                    new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
        }

        Logger.getInstance().start(); // Start logging! No more data receivers, replay sources, or metadata values may be added


        drive = Drive.getInstance();
        robotTracker = RobotTracker.getInstance();
        visionHandler = VisionHandler.getInstance();
        xbox = new Controller(0);
        stick = new Controller(1);
        buttonPanel = new Controller(2);

        startSubsystems();
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

        sideChooser.setDefaultOption("Blue", "blue");
        sideChooser.addOption("Red", "red");

        SmartDashboard.putData("Auto choices", autoChooser);
        SmartDashboard.putData("Red or Blue", sideChooser);

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
        drive.configBrake();
        String autoName = autoChooser.getSelected();
        if (autoName != null) {
            AutonomousContainer.getInstance().runAutonomous(autoName, sideChooser.getSelected(), true);
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
        drive.configBrake();
    }


    private @Nullable Pose2d teleopDrivingAutoAlignPosition = new Pose2d();

    {
        logData("Auto Align Y", teleopDrivingAutoAlignPosition.getY());
        logData("Auto Align X", teleopDrivingAutoAlignPosition.getX());
        logData("Auto Align Angle", teleopDrivingAutoAlignPosition.getRotation().getDegrees());
    }

    /**
     * This method is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
        xbox.update();
        buttonPanel.update();
        double wantedRumble = 0;


        var scoringPositionManager = ScoringPositionManager.getInstance();
        if (scoringPositionManager.updateSelectedPosition(buttonPanel)) {
            teleopDrivingAutoAlignPosition = null;
        }

        if (xbox.getRawButton(XboxButtons.START)) { //Should be remapped to one of the back buttons
            if (xbox.getRisingEdge(XboxButtons.START) || teleopDrivingAutoAlignPosition == null) {
                updateTeleopDrivingTarget(scoringPositionManager);
                assert teleopDrivingAutoAlignPosition != null;
            }

            if (!drive.driveToPosition(
                    teleopDrivingAutoAlignPosition.getTranslation(),
                    teleopDrivingAutoAlignPosition.getRotation(),
                    getControllerDriveInputs()
            )) {
                // We failed to generate a trajectory
                wantedRumble = 1;
            }
        } else {
            drive.swerveDriveFieldRelative(getControllerDriveInputs());
        }

        if (xbox.getRisingEdge(XboxButtons.A)) {
            robotTracker.resetPose(new Pose2d(robotTracker.getLatestPose().getTranslation(), new Rotation2d()));
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

        logData("Auto Align Y", y);
        logData("Auto Align X", x);
        logData("Auto Align Angle", rotation.getDegrees());

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
            drive.configCoast();
        }
    }


    /**
     * This method is called once when test mode is enabled.
     */
    @Override
    public void testInit() {
        drive.setDriveState(DriveState.TELEOP);
    }


    /**
     * This method is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
        xbox.update();
        if (xbox.getRawButton(XboxButtons.X) && xbox.getRawButton(XboxButtons.B)
                && xbox.getRisingEdge(XboxButtons.X) && xbox.getRisingEdge(XboxButtons.B)) {
            drive.setAbsoluteZeros();
        }
    }

    public void startSubsystems() {
        drive.start();
        robotTracker.start();
        visionHandler.start();
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


    NetworkTable loggingTable = NetworkTableInstance.getDefault().getTable("Robot");

    public void logData(@NotNull String key, @NotNull Object value) {
        loggingTable.getEntry(key).setValue(value);
    }

    public boolean isRed() {
        return sideChooser.getSelected().equals("red");
    }
}
