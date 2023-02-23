// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.dacubeking.AutoBuilder.robot.reflection.ClassInformationSender;
import com.dacubeking.AutoBuilder.robot.robotinterface.AutonomousContainer;
import com.dacubeking.AutoBuilder.robot.robotinterface.CommandTranslator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.IterativeRobotBase;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Watchdog;
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
import frc.utility.PathGenerator;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.rlog.RLOGServer;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import java.io.File;
import java.io.IOException;
import java.lang.reflect.Field;
import java.nio.file.Files;
import java.util.Arrays;
import java.util.NoSuchElementException;

import static frc.robot.Constants.*;
import static java.lang.Math.abs;


/**
 * The VM is configured to automatically run this class, and to call the methods corresponding to each mode, as described in the
 * TimedRobot documentation. If you change the name of this class or the package after creating this project, you must also update
 * the build.gradle file in the project.
 */
public class Robot extends LoggedRobot {

    public static final int XBOX_START_AUTO_DRIVE = XboxButtons.RIGHT_CLICK;
    public static final int XBOX_AUTO_BALENCE = XboxButtons.Y;
    public static final int XBOX_AUTO_ROTATE = XboxButtons.RIGHT_BUMPER;
    public static final int XBOX_RESET_HEADING = XboxButtons.A;
    public static final int STICK_TOGGLE_SCORING = 7;
    public static final int STICK_TOGGLE_FLOOR_PICKUP = 9;
    public static final int STICK_TOGGLE_PICKUP = 11;
    public static final int XBOX_TOGGLE_GRABBER = XboxButtons.B;
    public static final LoggedDashboardChooser<String> sideChooser = new LoggedDashboardChooser<>("SideChooser");
    private @NotNull
    static Drive drive;
    private @NotNull
    static RobotTracker robotTracker;
    private @NotNull
    static VisionHandler visionHandler;

    private @NotNull
    static Elevator elevator;
    private @NotNull
    static TelescopingArm telescopingArm;
    private @NotNull
    static Grabber grabber;
    private @NotNull
    static MechanismStateManager mechanismStateManager;
    // Autonomous
    private final LoggedDashboardChooser<String> autoChooser = new LoggedDashboardChooser<>("AutoChooser");
    public boolean isTurnToTargetMode = false;
    private double disabledTime = 0;
    private @NotNull Controller xbox;
    private @NotNull Controller stick;
    private @NotNull Controller buttonPanel;
    private @Nullable Pose2d teleopDrivingAutoAlignPosition = new Pose2d();
    private WantedMechanismState wantedMechanismState = WantedMechanismState.STOWED;
    private @Nullable WantedMechanismState lastWantedMechanismState = null;
    private boolean isGrabberOpen = true;
    private double wantedX = -0.489;
    private double wantedY = 0.249;
    private double wantedAngle = MAX_WRIST_ANGLE - 2;

    {
        Logger.getInstance().recordOutput("Auto Align Y", teleopDrivingAutoAlignPosition.getY());
        Logger.getInstance().recordOutput("Auto Align X", teleopDrivingAutoAlignPosition.getX());
        Logger.getInstance().recordOutput("Auto Align Angle", teleopDrivingAutoAlignPosition.getRotation().getDegrees());
    }

    public static boolean isRed() {
        return sideChooser.get().equals("red");
    }

    public static boolean isOnAllianceSide() {
        return isRed() == robotTracker.isOnRedSide();
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

    public static @NotNull MechanismStateManager getMechanismStateManager() {
        return mechanismStateManager;
    }

    /**
     * This method is run when the robot is first started up and should be used for any initialization code.
     */
    @Override
    public void robotInit() {
        Logger.getInstance().recordMetadata("ProjectName", "FRC2023"); // Set a metadata value


        String logPath = null;

        if (!isReal()) {
            try {
                logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
            } catch (NoSuchElementException e) {
                System.out.println("Failed to Find a log file, not loading one");
            }
        }

        Logger.getInstance().disableDeterministicTimestamps(); // Disable deterministic timestamps (they cause issues wit the
        // autoBuilder)

        if (isReal() || logPath == null) {
            var directory = new File(LOG_DIRECTORY);
            if (!directory.exists()) {
                directory.mkdir();
            }

            // ensure that there is enough space on the roboRIO to log data
            if (directory.getFreeSpace() < MIN_FREE_SPACE) {
                var files = directory.listFiles();
                if (files != null) {
                    // Sorting the files by name will ensure that the oldest files are deleted first
                    files = Arrays.stream(files).sorted().toArray(File[]::new);

                    long bytesToDelete = MIN_FREE_SPACE - directory.getFreeSpace();

                    for (File file : files) {
                        if (file.getName().endsWith(".wpilog")) {
                            try {
                                bytesToDelete -= Files.size(file.toPath());
                            } catch (IOException e) {
                                System.out.println("Failed to get size of file " + file.getName());
                                continue;
                            }
                            if (file.delete()) {
                                System.out.println("Deleted " + file.getName() + " to free up space");
                            } else {
                                System.out.println("Failed to delete " + file.getName());
                            }
                            if (bytesToDelete <= 0) {
                                break;
                            }
                        }
                    }
                }
            }

            Logger.getInstance().addDataReceiver(new WPILOGWriter(LOG_DIRECTORY));
            Logger.getInstance().addDataReceiver(new RLOGServer(5800)); // Publish data to NetworkTables
            new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging

            drive = new Drive(new DriveIOSparkMax());
            elevator = new Elevator(new ElevatorIOSparkMax());
            telescopingArm = new TelescopingArm(new TelescopingArmIOSparkMax());
            grabber = new Grabber(new GrabberIOSparkMax());
        } else {
            setUseTiming(false); // Run as fast as possible
            Logger.getInstance().setReplaySource(new WPILOGReader(logPath)); // Read replay log
            Logger.getInstance().addDataReceiver(
                    new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log

            drive = new Drive(new DriveIO() {
            });
            elevator = new Elevator(new ElevatorIO() {
            });
            telescopingArm = new TelescopingArm(new TelescopingArmIO() {
            });
            grabber = new Grabber(new GrabberIO() {
            });
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

        if (!isReal()) {
            ClassInformationSender.updateReflectionInformation("frc");
        }

        sideChooser.addDefaultOption("Blue", "blue");
        sideChooser.addOption("Red", "red");

        if (IS_PRACTICE) {
            for (int i = 0; i < 10; i++) {
                System.out.println("USING PRACTICE BOT CONFIG");
            }
        }

        // Set the initial states of the mechanisms
        // TODO: Change this to CLOSED when we're confident the grabber won't try to tear itself apart when it's trying to close
        grabber.setGrabState(GrabState.IDLE);
        mechanismStateManager.setState(MechanismStates.STOWED);

        Class<PathGenerator> loadPathGenerator = PathGenerator.class; // Load the PathGenerator class to ensure that it is
        // initialized (and creates the thread to generate paths) before the robot starts

        // TODO: Remove this when we're done optimizing our code
        try {
            Field watchDog = IterativeRobotBase.class.getDeclaredField("m_watchdog");
            watchDog.setAccessible(true);
            Watchdog actualWatchDog = (Watchdog) watchDog.get(this);
            actualWatchDog.setTimeout(1);
        } catch (NoSuchFieldException | IllegalAccessException e) {
            throw new RuntimeException(e);
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
        if (autoName == null) {
            autoName = "";
        }

        AutonomousContainer.getInstance().runAutonomous(autoName, sideChooser.get(), true);
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

        if (xbox.getRisingEdge(XBOX_AUTO_ROTATE)) {
            isTurnToTargetMode = true;
            updateTeleopDrivingTarget(scoringPositionManager);
        }

        if (xbox.getRawButton(XBOX_START_AUTO_DRIVE)) { //Should be remapped to one of the back buttons
            if (xbox.getRisingEdge(XBOX_START_AUTO_DRIVE) || teleopDrivingAutoAlignPosition == null) {
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
        } else if (xbox.getRawButton(XBOX_AUTO_BALENCE)) {
            drive.autoBalance(getControllerDriveInputs());
        } else {
            if (isTurnToTargetMode) {
                assert teleopDrivingAutoAlignPosition != null;
                if (teleopDrivingAutoAlignPosition != null) {
                    var controllerDriveInputs = getControllerDriveInputs();
                    drive.setTurn(controllerDriveInputs,
                            new State(teleopDrivingAutoAlignPosition.getRotation().getRadians(), 0),
                            0);

                    if (Math.abs(controllerDriveInputs.getRotation()) > 0) {
                        isTurnToTargetMode = false;
                    }
                }
            } else {
                drive.swerveDriveFieldRelative(getControllerDriveInputs());
            }
        }

        if (xbox.getRisingEdge(XBOX_RESET_HEADING)) {
            robotTracker.resetPose(new Pose2d(robotTracker.getLatestPose().getTranslation(), new Rotation2d()));
        }

        if (stick.getRisingEdge(STICK_TOGGLE_SCORING)) {
            if (wantedMechanismState == WantedMechanismState.STOWED) {
                wantedMechanismState = WantedMechanismState.SCORING;
            } else {
                setStowed();
            }
        }

        if (stick.getRisingEdge(STICK_TOGGLE_FLOOR_PICKUP)) {
            if (wantedMechanismState == WantedMechanismState.STOWED) {
                wantedMechanismState = WantedMechanismState.FLOOR_PICKUP;
                isGrabberOpen = true;
            } else {
                setStowed();
            }
        }

        if (stick.getRisingEdge(STICK_TOGGLE_PICKUP)) {
            if (wantedMechanismState == WantedMechanismState.STOWED) {
                wantedMechanismState = WantedMechanismState.STATION_PICKUP;
                isGrabberOpen = true;
            } else {
                setStowed();
            }
        }


        if (wantedMechanismState != lastWantedMechanismState) {
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
        }

        lastWantedMechanismState = wantedMechanismState;


        var limitedMechCoords = MechanismStateManager.limitCoordinates(mechanismStateManager.getCurrentWantedState());

        Logger.getInstance().recordOutput("Robot/Wanted X", wantedX);
        Logger.getInstance().recordOutput("Robot/Wanted Y", wantedY);
        Logger.getInstance().recordOutput("Robot/Wanted Angle", wantedAngle);

        var mechInputs = new ControllerDriveInputs(stick.getRawAxis(0), stick.getRawAxis(1), stick.getRawAxis(3));
        mechInputs.applyDeadZone(0.1, 0.1, 0.25, 0.2);
        mechInputs.squareInputs();

        var mechDx = -buttonPanel.getRawAxis(0);
        var mechDy = buttonPanel.getRawAxis(1);

        // Basic deadzone
        if (abs(mechDx) < 0.1) mechDx = 0;
        if (abs(mechDy) < 0.1) mechDy = 0;

        if (mechInputs.getX() != 0 || mechDx != 0 || mechDy != 0) {
            wantedX = limitedMechCoords.xMeters();
            wantedY = limitedMechCoords.yMeters();
            wantedAngle = limitedMechCoords.grabberAngleDegrees();

            wantedAngle += mechInputs.getY() * ARCADE_WRIST_ANGLE_SPEED * NOMINAL_DT;

            wantedX += mechDx * ARCADE_MODE_TRANSLATION_SPEED * NOMINAL_DT;
            wantedY += mechDy * ARCADE_MODE_TRANSLATION_SPEED * NOMINAL_DT;

            mechanismStateManager.setState(new MechanismStateManager.MechanismStateCoordinates(wantedX, wantedY, wantedAngle));
        }

        Logger.getInstance().recordOutput("Robot/Wanted Mechanism State", wantedMechanismState.name());

        if (xbox.getRisingEdge(XBOX_TOGGLE_GRABBER)) {
            isGrabberOpen = !isGrabberOpen;
        }

        if (isGrabberOpen) {
            if (wantedMechanismState == WantedMechanismState.STOWED) {
                grabber.setGrabState(GrabState.IDLE);
            } else {
                grabber.setGrabState(GrabState.OPEN);
            }
        } else {
            if (scoringPositionManager.getWantedPositionType() == PositionType.CONE) {
                grabber.setGrabState(GrabState.GRAB_CONE);
            } else {
                grabber.setGrabState(GrabState.GRAB_CUBE);
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

    /**
     * Sets the mechanism state to stowed and closes the grabber
     */
    private void setStowed() {
        wantedMechanismState = WantedMechanismState.STOWED;
        isGrabberOpen = false;
    }

    private void updateTeleopDrivingTarget(ScoringPositionManager scoringPositionManager) {
        double x, y;
        Rotation2d rotation;

        var predictedPose = robotTracker.getLatestPose().getTranslation().plus(robotTracker.getVelocity().times(0.5));
        var isOnRedSide = predictedPose.getX() < FIELD_WIDTH_METERS / 2;
        if (isOnRedSide == isRed()) {
            // We're on the same side as our alliance
            // Try to go to the scoring position
            y = ScoringPositionManager.getBestFieldY(
                    scoringPositionManager.getSelectedPosition(),
                    isRed(),
                    robotTracker.getLatestPose().getTranslation(),
                    robotTracker.getVelocity()
            );

            Logger.getInstance().recordOutput("Robot/Wanted Y Auto Drive", y);

            if (isRed()) {
                x = Constants.GRIDS_RED_X + HALF_ROBOT_WIDTH + SCORING_POSITION_OFFSET_FROM_WALL;
                rotation = Constants.SCORING_ANGLE_RED;
            } else {
                x = Constants.GRIDS_BLUE_X - HALF_ROBOT_WIDTH - SCORING_POSITION_OFFSET_FROM_WALL;
                rotation = Constants.SCORING_ANGLE_BLUE;
            }
        } else {
            // We're on the opposite side as our alliance
            // Try to go to the pickup position
            y = PICKUP_POSITION_Y;
            if (isRed()) {
                x = FIELD_WIDTH_METERS - PICKUP_POSITION_X_OFFSET_FROM_WALL;
                rotation = PICKUP_ANGLE_RED;
            } else {
                x = PICKUP_POSITION_X_OFFSET_FROM_WALL;
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
        System.out.println("Trying to kill auto");
        AutonomousContainer.getInstance().killAuto();
        System.out.println("Finished Killing Auto");
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
        if (xbox.getRawButton(XboxButtons.X) && xbox.getRawButton(XBOX_TOGGLE_GRABBER)
                && xbox.getRisingEdge(XboxButtons.X) && xbox.getRisingEdge(XBOX_TOGGLE_GRABBER)) {
            drive.resetAbsoluteZeros();
        }
    }

    private ControllerDriveInputs getControllerDriveInputs() {
        ControllerDriveInputs inputs;
        if (isRed()) {
            inputs = new ControllerDriveInputs(-xbox.getRawAxis(1), -xbox.getRawAxis(0), -xbox.getRawAxis(4));
        } else {
            inputs = new ControllerDriveInputs(xbox.getRawAxis(1), xbox.getRawAxis(0), -xbox.getRawAxis(4));
        }
        if (xbox.getRawButton(Controller.XboxButtons.X)) {
            inputs.applyDeadZone(0.2, 0.2, 0.2, 0.2);
        } else {
            inputs.applyDeadZone(0.05, 0.05, 0.2, 0.2);
        }

        inputs.squareInputs();
        Logger.getInstance().recordOutput("Robot/Controller X", inputs.getX());
        Logger.getInstance().recordOutput("Robot/Controller Y", inputs.getY());
        Logger.getInstance().recordOutput("Robot/Controller Rotation", inputs.getRotation());

        return inputs;
    }

    enum WantedMechanismState {
        STOWED, SCORING, FLOOR_PICKUP, STATION_PICKUP
    }
}
