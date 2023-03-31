// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.dacubeking.AutoBuilder.robot.GuiAuto;
import com.dacubeking.AutoBuilder.robot.annotations.AutoBuilderAccessible;
import com.dacubeking.AutoBuilder.robot.reflection.ClassInformationSender;
import com.dacubeking.AutoBuilder.robot.robotinterface.AutonomousContainer;
import com.dacubeking.AutoBuilder.robot.robotinterface.CommandTranslator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.IterativeRobotBase;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Watchdog;
import frc.robot.ScoringPositionManager.PositionType;
import frc.robot.ScoringPositionManager.SelectedPosition;
import frc.subsytem.AbstractSubsystem;
import frc.subsytem.Elevator.Elevator;
import frc.subsytem.Elevator.ElevatorIO;
import frc.subsytem.Elevator.ElevatorIOSparkMax;
import frc.subsytem.MechanismStateManager;
import frc.subsytem.MechanismStateManager.MechanismStateCoordinates;
import frc.subsytem.MechanismStateManager.MechanismStateSubsystemPositions;
import frc.subsytem.MechanismStateManager.MechanismStates;
import frc.subsytem.drive.Drive;
import frc.subsytem.drive.DriveIO;
import frc.subsytem.drive.DriveIOFalcon;
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
import frc.utility.Controller.XboxAxes;
import frc.utility.Controller.XboxButtons;
import frc.utility.ControllerDriveInputs;
import frc.utility.PathGenerator;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;
import org.littletonrobotics.junction.rlog.RLOGServer;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import java.io.File;
import java.io.IOException;
import java.lang.reflect.Field;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.NoSuchElementException;
import java.util.Objects;
import java.util.concurrent.ConcurrentLinkedDeque;
import java.util.concurrent.ExecutionException;

import static frc.robot.Constants.*;
import static java.lang.Math.abs;


/**
 * The VM is configured to automatically run this class, and to call the methods corresponding to each mode, as described in the
 * TimedRobot documentation. If you change the name of this class or the package after creating this project, you must also update
 * the build.gradle file in the project.
 */
public class Robot extends LoggedRobot {

    public static final int XBOX_START_AUTO_DRIVE = XboxButtons.RIGHT_CLICK;
    public static final int XBOX_TOGGLE_MECH = XboxButtons.LEFT_CLICK;

    public static final int XBOX_AUTO_BALANCE = XboxButtons.Y;
    public static final int XBOX_AUTO_DRIVE_LOWER_SUBSTATION = XboxButtons.RIGHT_BUMPER;
    public static final int XBOX_RESET_HEADING = XboxButtons.A;
    public static final int CONTROLLER_TOGGLE_FLOOR_PICKUP = XboxAxes.LEFT_TRIGGER;
    public static final int CONTROLLER_TOGGLE_TIPPED_FLOOR_PICKUP = XboxAxes.RIGHT_TRIGGER;
    public static final int XBOX_TOGGLE_GRABBER = XboxButtons.LEFT_BUMPER;
    public static final int XBOX_BABY_BIRD = XboxButtons.X;

    public static final int STICK_TOGGLE_SCORING = 7;
    public static final int STICK_TOGGLE_FLOOR_PICKUP = 9;
    public static final int STICK_TOGGLE_PICKUP_DOUBLE = 11;
    public static final int STICK_TOGGLE_PICKUP_SINGLE = 12;
    public static final int STICK_TOGGLE_AUTO_GRAB = 8;
    public static final int STICK_HOME_ELEVATOR = 3;
    public static final int STICK_HOME_TELESCOPING_ARM = 4;

    private double disabledTime = 0;

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
    private @NotNull
    static PowerDistribution powerDistribution;

    private @NotNull Controller xbox;
    private @NotNull Controller stick;

    private @NotNull Controller buttonPanel;


    // Autonomous
    private final LoggedDashboardChooser<String> autoChooser = new LoggedDashboardChooser<>("AutoChooser");
    public static final LoggedDashboardChooser<String> sideChooser = new LoggedDashboardChooser<>("SideChooser");

    {
        Logger.getInstance().registerDashboardInput(autoChooser);
        Logger.getInstance().registerDashboardInput(sideChooser);
    }

    private static Thread mainThread;

    /**
     * This method is run when the robot is first started up and should be used for any initialization code.
     */
    @Override
    public void robotInit() {
        mainThread = Thread.currentThread();
        Logger.getInstance().recordMetadata("ProjectName", "FRC2023"); // Set a metadata value


        String logPath = null;

        if (!isReal()) {
            try {
                logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
            } catch (NoSuchElementException e) {
                System.out.println("Failed to Find a log file, not loading one");
            }
        }

        Logger.getInstance().disableDeterministicTimestamps(); // Disable deterministic timestamps (they cause issues with the
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
            powerDistribution = new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging

            drive = new Drive(new DriveIOFalcon());
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
                null,
                this
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

        System.out.println("Using GIT SHA: " + BuildConstants.GIT_SHA + " on branch " + BuildConstants.GIT_BRANCH
                + " built on " + BuildConstants.BUILD_DATE);

        drive.resetPeriodicFrames();
        System.out.println("Running Warm Up Auto");
        AutonomousContainer.getInstance().runAutonomous("warmup", "red", false);
        for (int i = 0; i < 10; i++) {
            double pathGenStartTime = Timer.getFPGATimestamp();
            var path = PathGenerator.generateTrajectory(new Translation2d(2, 3), new Translation2d(0, 0),
                    new Translation2d(10, 3),
                    0, false, 3);
            path.thenCompose((trajectory -> {
                System.out.println("Finished Generating Path. Path len: " + trajectory.get().getTotalTimeSeconds()
                        + "s. Took: " + (Timer.getFPGATimestamp() - pathGenStartTime));
                return null;
            }));
            try {
                path.get();
            } catch (InterruptedException | ExecutionException e) {
                throw new RuntimeException(e);
            }
        }
    }

    private @Nullable String lastSelectedAuto = null;
    private @Nullable String lastSelectedSide = null;
    private @Nullable GuiAuto guiAuto = null;

    private final LoggedDashboardNumber autoPositionErrorX = new LoggedDashboardNumber("Auto Position Error X", 0);
    private final LoggedDashboardNumber autoPositionErrorY = new LoggedDashboardNumber("Auto Position Error Y", 0);
    private final LoggedDashboardNumber autoPositionErrorTheta = new LoggedDashboardNumber("Auto Position Error Theta", 0);

    {
        Logger.getInstance().registerDashboardInput(autoPositionErrorX);
        Logger.getInstance().registerDashboardInput(autoPositionErrorY);
        Logger.getInstance().registerDashboardInput(autoPositionErrorTheta);
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
        runAsyncScheduledTasks();
        AbstractSubsystem.tick();

        // Record video is FMS is attached
        visionHandler.forceRecord(true);

        if (!(Objects.equals(lastSelectedAuto, autoChooser.get()) || Objects.equals(lastSelectedSide, sideChooser.get()))) {
            lastSelectedAuto = autoChooser.get();
            lastSelectedSide = sideChooser.get();
            System.out.println("Auto: " + autoChooser.get() + " Side: " + sideChooser.get());
            guiAuto = AutonomousContainer.getInstance().getAuto(autoChooser.get(), sideChooser.get(), true);
        }

        if (guiAuto != null && guiAuto.getInitialPose() != null) {
            var poseDiffFromWantedAutoPlacement = guiAuto.getInitialPose().minus(robotTracker.getLatestPose());
            autoPositionErrorX.set(poseDiffFromWantedAutoPlacement.getTranslation().getX());
            autoPositionErrorY.set(poseDiffFromWantedAutoPlacement.getTranslation().getY());
            autoPositionErrorTheta.set(poseDiffFromWantedAutoPlacement.getRotation().getDegrees());
        } else {
            autoPositionErrorX.set(0);
            autoPositionErrorY.set(0);
            autoPositionErrorTheta.set(0);
        }
    }

    private static double autoStartTime = 0;

    private boolean wasAuto = false;
    @Override
    public void autonomousInit() {
        autoStartTime = Timer.getFPGATimestamp();
        drive.setBrakeMode(true);
        drive.setDriveVoltageCompLevel(SWERVE_DRIVE_VOLTAGE_LIMIT_AUTO);
        String autoName = autoChooser.get();
        if (autoName == null) {
            autoName = "";
        }

        wasAuto = true;
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
        drive.setDriveVoltageCompLevel(SWERVE_DRIVE_VOLTAGE_LIMIT_TELEOP);
        mechanismStateManager.setKeepoutsEnabled(true);
        if (wasAuto) {
            mechanismStateManager.setState(MechanismStates.STOWED);
            setCurrentWantedState(WantedMechanismState.STOWED);
            setFutureGrabberClose = true;
            wasAuto = false;
        }
    }


    private @Nullable Pose2d teleopDrivingAutoAlignPosition = new Pose2d();
    private boolean hasReachedAutoAlignPosition = false;

    {
        Logger.getInstance().recordOutput("Auto Align Y", teleopDrivingAutoAlignPosition.getY());
        Logger.getInstance().recordOutput("Auto Align X", teleopDrivingAutoAlignPosition.getX());
        Logger.getInstance().recordOutput("Auto Align Angle", teleopDrivingAutoAlignPosition.getRotation().getDegrees());
    }


    enum WantedMechanismState {
        STOWED(false),
        SCORING(false),
        FLOOR_PICKUP(true),
        TIPPED_FLOOR_PICKUP(true),
        STATION_PICKUP_DOUBLE(true),
        STATION_PICKUP_SINGLE(true);

        public final boolean shouldAutoGrab;

        WantedMechanismState(boolean shouldAutoGrab) {
            this.shouldAutoGrab = shouldAutoGrab;
        }
    }

    private static WantedMechanismState wantedMechanismState = WantedMechanismState.STOWED;
    private @Nullable WantedMechanismState lastWantedMechanismState = null;


    private boolean isGrabberOpen = true;
    private boolean setFutureGrabberClose = false;


    private double mechWantedX = -0.489;
    private double mechWantedY = 0.249;
    private double mechWantedAngle = MAX_WRIST_ANGLE - 2;
    public boolean isTurnToTargetMode = false;

    private double grabberOpenTime = 0;
    private boolean wantToClose = false;

    private boolean useAutoGrab = true;
    LoggedDashboardBoolean autoGrabDashboard = new LoggedDashboardBoolean("Auto Grab", useAutoGrab);

    {
        Logger.getInstance().registerDashboardInput(autoGrabDashboard);
    }

    private final ArrayList<MechanismStates> halfSpeedMechanismStates = new ArrayList<>() {{
        add(MechanismStates.CONE_MIDDLE_SCORING);
        add(MechanismStates.CUBE_MIDDLE_SCORING);
        add(MechanismStates.CONE_HIGH_SCORING);
        add(MechanismStates.CUBE_HIGH_SCORING);
        add(MechanismStates.FINAL_CONE_MIDDLE_SCORING);
    }};

    /**
     * This method is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
        xbox.update();
        stick.update();
        buttonPanel.update();
        double wantedRumble = 0;

        Logger.getInstance().recordOutput("Controls/Is Left Toggle Grabber Pressed", xbox.getRawButton(XBOX_TOGGLE_GRABBER));
        Logger.getInstance().recordOutput("Controls/Is Left Toggle Mech Pressed", xbox.getRawButton(XBOX_TOGGLE_MECH));

        var scoringPositionManager = ScoringPositionManager.getInstance();
        if (scoringPositionManager.updateSelectedPosition(buttonPanel)) {
            teleopDrivingAutoAlignPosition = null;
        }

        if (xbox.getRawButton(XBOX_START_AUTO_DRIVE) || xbox.getRawButton(
                XBOX_AUTO_DRIVE_LOWER_SUBSTATION)) { //Should be remapped to one of the back buttons
            if (xbox.getRisingEdge(XBOX_START_AUTO_DRIVE) || xbox.getRisingEdge(XBOX_AUTO_DRIVE_LOWER_SUBSTATION)) {
                updateTeleopDrivingTarget(true);
                hasReachedAutoAlignPosition = false;
                assert teleopDrivingAutoAlignPosition != null;
            } else if (teleopDrivingAutoAlignPosition == null) {
                // We're not recalculating the grid position b/c we only need to recalculate the path because the operator
                // changed the grid position
                updateTeleopDrivingTarget(false);
                hasReachedAutoAlignPosition = false;
                assert teleopDrivingAutoAlignPosition != null;
            }


            Translation2d autoDriveAlignError =
                    teleopDrivingAutoAlignPosition.minus(robotTracker.getLatestPose()).getTranslation();
            if (false &&
                    Math.abs(autoDriveAlignError.getX()) < 0.2 && Math.abs(
                    autoDriveAlignError.getY()) < 0.05 && !isOnAllianceSide()
                    && mechanismStateManager.isMechAtFinalPos()
                    && wantedMechanismState == WantedMechanismState.STATION_PICKUP_DOUBLE) {
                hasReachedAutoAlignPosition = true;
            }

            double maxAutoDriveAccel = autoDrivePosition == AutoDrivePosition.PICKUP_DOUBLE_SUBSTATION ?
                    MAX_ACCEL_SINGLE_SUBSTATION_PICKUP : MAX_ACCEL_AUTO_DRIVE;

            if (hasReachedAutoAlignPosition) {
                drive.alignToYAndYaw(teleopDrivingAutoAlignPosition.getRotation().getRadians(),
                        teleopDrivingAutoAlignPosition.getTranslation().getY(),
                        getControllerDriveInputs());
            } else if (!drive.driveToPosition(
                    teleopDrivingAutoAlignPosition.getTranslation(),
                    teleopDrivingAutoAlignPosition.getRotation(),
                    getControllerDriveInputs(),
                    autoDrivePosition == AutoDrivePosition.PICKUP_SINGLE_SUBSTATION,
                    maxAutoDriveAccel
            )) {
                // We failed to generate a trajectory
                wantedRumble = 1;
            }
        } else if (xbox.getRawButton(XBOX_AUTO_BALANCE)) {
            drive.autoBalance(getControllerDriveInputs());
        } else {
            if (isTurnToTargetMode) {
                if (teleopDrivingAutoAlignPosition == null) {
                    updateTeleopDrivingTarget(true);
                }
                var controllerDriveInputs = getControllerDriveInputs();
                if (teleopDrivingAutoAlignPosition != null) {
                    drive.setTurn(controllerDriveInputs,
                            new State(teleopDrivingAutoAlignPosition.getRotation().getRadians(), 0),
                            0);
                }

                if (Math.abs(controllerDriveInputs.getRotation()) > 0) {
                    isTurnToTargetMode = false;
                }
            } else {
                ControllerDriveInputs controllerDriveInputs = getControllerDriveInputs();
                if (halfSpeedMechanismStates.contains(mechanismStateManager.getLastState())) {
                    controllerDriveInputs.scaleInputs(0.25);
                }
                drive.swerveDriveFieldRelative(controllerDriveInputs);
            }
        }

        if (xbox.getRisingEdge(XBOX_RESET_HEADING)) {
            if (isRed()) {
                robotTracker.resetPose(new Pose2d(robotTracker.getLatestPose().getTranslation(), Rotation2d.fromDegrees(0)));
            } else {
                robotTracker.resetPose(new Pose2d(robotTracker.getLatestPose().getTranslation(), Rotation2d.fromDegrees(180)));
            }
        }

        if (grabber.isGrabbed() && wantedMechanismState.shouldAutoGrab) {
            setStowed();
        }

        if (isGrabberOpen
                && (wantedMechanismState == WantedMechanismState.SCORING)
                && xbox.getFallingEdge(XBOX_TOGGLE_GRABBER)) {
            wantToClose = true;
        }

        if (wantToClose) {
            if (scoringPositionManager.getWantedPositionType() == PositionType.CUBE &&
                    Timer.getFPGATimestamp() - grabberOpenTime > 0.5) {
                setStowed();
            }
            if (scoringPositionManager.getWantedPositionType() == PositionType.CONE &&
                    Timer.getFPGATimestamp() - grabberOpenTime > 0.1) {
                setStowed();
            }
        }

        if (stick.getRisingEdge(STICK_TOGGLE_SCORING)) {
            if (wantedMechanismState == WantedMechanismState.STOWED) {
                wantedMechanismState = WantedMechanismState.SCORING;
            } else {
                setStowed();
            }
        }

        if (stick.getRisingEdge(STICK_TOGGLE_FLOOR_PICKUP) ||
                xbox.getRisingEdge(CONTROLLER_TOGGLE_FLOOR_PICKUP, 0.1)) {
            if (wantedMechanismState == WantedMechanismState.STOWED) {
                wantedMechanismState = WantedMechanismState.FLOOR_PICKUP;
                isGrabberOpen = true;
            } else {
                setStowed();
            }
        }

        if (xbox.getRisingEdge(CONTROLLER_TOGGLE_TIPPED_FLOOR_PICKUP, 0.1) && false) {
            if (wantedMechanismState == WantedMechanismState.STOWED) {
                wantedMechanismState = WantedMechanismState.TIPPED_FLOOR_PICKUP;
                isGrabberOpen = true;
            } else {
                setStowed();
            }
        }

        if (stick.getRisingEdge(STICK_TOGGLE_PICKUP_DOUBLE)) {
            if (wantedMechanismState == WantedMechanismState.STOWED) {
                wantedMechanismState = WantedMechanismState.STATION_PICKUP_DOUBLE;
                isGrabberOpen = true;
            } else {
                setStowed();
            }
        }

        if (stick.getRisingEdge(STICK_TOGGLE_PICKUP_SINGLE)) {
            if (wantedMechanismState == WantedMechanismState.STOWED) {
                wantedMechanismState = WantedMechanismState.STATION_PICKUP_SINGLE;
                isGrabberOpen = true;
            } else {
                setStowed();
            }
        }

        if (xbox.getRisingEdge(XBOX_BABY_BIRD)) {
            wantedMechanismState = WantedMechanismState.STATION_PICKUP_SINGLE;
            teleopDrivingAutoAlignPosition = new Pose2d(new Translation2d(), SINGLE_STATION_ANGLE);
            hasReachedAutoAlignPosition = false;
            isTurnToTargetMode = true;
            isGrabberOpen = true;
        }

        if (xbox.getRisingEdge(XBOX_TOGGLE_MECH)) {
            if (wantedMechanismState == WantedMechanismState.STOWED) {
                if (isOnAllianceSide()) {
                    wantedMechanismState = WantedMechanismState.SCORING;
                } else {
                    if ((isRed() && robotTracker.getLatestPose().getRotation()
                            .getDegrees() < SINGLE_SUBSTATION_PICKUP_ANGLE_CUTOFF_DEGREES)
                            || (!isRed()
                            && robotTracker.getLatestPose().getRotation().getDegrees() > -135
                            && robotTracker.getLatestPose().getRotation().getDegrees() < 0)

                    ) {
                        wantedMechanismState = WantedMechanismState.STATION_PICKUP_SINGLE;
                    } else {
                        wantedMechanismState = WantedMechanismState.STATION_PICKUP_DOUBLE;
                    }
                    isGrabberOpen = true;
                }
            } else {
                setStowed();
            }
        }

        if (stick.getRisingEdge(STICK_TOGGLE_AUTO_GRAB)) {
            useAutoGrab = !useAutoGrab;
            autoGrabDashboard.set(useAutoGrab);
        }


        if (wantedMechanismState != lastWantedMechanismState) {
            switch (wantedMechanismState) {
                case STOWED -> mechanismStateManager.setState(MechanismStates.STOWED);
                case SCORING -> {
                    int level = scoringPositionManager.getSelectedPosition().getLevel();
                    if (level == 0) {
                        mechanismStateManager.setState(MechanismStates.LOW_SCORING);
                    } else if (level == 1) {
                        if (scoringPositionManager.getWantedPositionType() == PositionType.CONE) {
                            mechanismStateManager.setState(MechanismStates.CONE_MIDDLE_SCORING);
                        } else {
                            mechanismStateManager.setState(MechanismStates.CUBE_MIDDLE_SCORING);
                        }
                    } else if (level == 2) {
                        if (scoringPositionManager.getWantedPositionType() == PositionType.CONE) {
                            mechanismStateManager.setState(MechanismStates.CONE_HIGH_SCORING);
                        } else {
                            mechanismStateManager.setState(MechanismStates.CUBE_HIGH_SCORING);
                        }
                    }
                }
                case FLOOR_PICKUP -> mechanismStateManager.setState(MechanismStates.FLOOR_PICKUP);
                case TIPPED_FLOOR_PICKUP -> mechanismStateManager.setState(MechanismStates.TIPPED_FLOOR_PICKUP);
                case STATION_PICKUP_DOUBLE -> mechanismStateManager.setState(MechanismStates.DOUBLE_STATION_PICKUP);
                case STATION_PICKUP_SINGLE -> {
                    if (scoringPositionManager.getWantedPositionType() == PositionType.CONE) {
                        mechanismStateManager.setState(MechanismStates.SINGLE_SUBSTATION_PICKUP_CONE);
                    } else {
                        mechanismStateManager.setState(MechanismStates.SINGLE_SUBSTATION_PICKUP_CUBE);
                    }
                }
            }

            if (wantedMechanismState != lastWantedMechanismState && useAutoGrab) {
                Robot.getGrabber().setAutoGrab(wantedMechanismState.shouldAutoGrab);
            }
        }
        lastWantedMechanismState = wantedMechanismState;

        var limitedMechCoords = MechanismStateManager.limitCoordinates(mechanismStateManager.getCurrentWantedState());

        Logger.getInstance().recordOutput("Robot/Mechanism Wanted X", mechWantedX);
        Logger.getInstance().recordOutput("Robot/Mechanism Wanted Y", mechWantedY);
        Logger.getInstance().recordOutput("Robot/Mechanism Wanted Angle", mechWantedAngle);

        var mechInputs = new ControllerDriveInputs(stick.getRawAxis(0), stick.getRawAxis(1), stick.getRawAxis(3));
        mechInputs.applyDeadZone(0.1, 0.1, 0.25, 0.1);
        mechInputs.squareInputs();

        var mechDx = -buttonPanel.getRawAxis(0);
        var mechDy = buttonPanel.getRawAxis(1);

        // Basic deadzone
        if (abs(mechDx) < 0.1) mechDx = 0;
        if (abs(mechDy) < 0.1) mechDy = 0;

        if (mechInputs.getX() != 0 || mechDx != 0 || mechDy != 0) {
            mechWantedX = limitedMechCoords.xMeters();
            mechWantedY = limitedMechCoords.yMeters();
            mechWantedAngle = limitedMechCoords.grabberAngleDegrees();

            // Calculate the current wanted subsystem positions
            var currentWantedSubsystemPositions =
                    MechanismStateManager.coordinatesToSubsystemPositions(
                            new MechanismStateCoordinates(mechWantedX, mechWantedY, mechWantedAngle)
                    );

            // Adjust the grabber angle in the current wanted subsystem positions (so that adjusting the grabber angle doesn't
            // move the mechanism)
            currentWantedSubsystemPositions = new MechanismStateSubsystemPositions(
                    currentWantedSubsystemPositions.elevatorPositionMeters(),
                    currentWantedSubsystemPositions.telescopingArmPositionMeters(),
                    currentWantedSubsystemPositions.grabberAngleDegrees() - mechInputs.getY() * ARCADE_WRIST_ANGLE_SPEED * NOMINAL_DT
            );


            // Calculate the new wanted coordinates from the current wanted subsystem positions
            var newWantedCoordinates = MechanismStateManager.subsystemPositionsToCoordinates(currentWantedSubsystemPositions);

            mechWantedX = newWantedCoordinates.xMeters();
            mechWantedY = newWantedCoordinates.yMeters();
            mechWantedAngle = newWantedCoordinates.grabberAngleDegrees();


            mechWantedX += mechDx * ARCADE_MODE_TRANSLATION_SPEED * NOMINAL_DT;
            mechWantedY += mechDy * ARCADE_MODE_TRANSLATION_SPEED * NOMINAL_DT;

            mechanismStateManager.setState(
                    new MechanismStateManager.MechanismStateCoordinates(mechWantedX, mechWantedY, mechWantedAngle));
        }

        Logger.getInstance().recordOutput("Robot/Wanted Mechanism State", wantedMechanismState.name());

        if (xbox.getRisingEdge(XBOX_TOGGLE_GRABBER)) {
            if (grabber.isAutoGrabEnabled() && !isGrabberOpen) {
                // Auto Grab isn't letting us close so disable it
                grabber.setAutoGrab(false);
            } else {
                if (wantedMechanismState == WantedMechanismState.SCORING
                        && scoringPositionManager.getSelectedPosition().getLevel() == 1
                        && scoringPositionManager.getWantedPositionType() == PositionType.CONE) {
                    // We're in the scoring in the middle level with a cone. Instead of opening the grabber, we want to
                    // shove the cone down onto the pole

                    // Update the last state so we can open the grabber later, and disable the keepouts
                    mechanismStateManager.setState(MechanismStates.FINAL_CONE_MIDDLE_SCORING);
                    mechanismStateManager.setState(mechanismStateManager.getCurrentWantedState()
                            .adjust(MechanismStateManager.CONE_DUNK_LOWER_METERS,
                                    -MechanismStateManager.CONE_DUNK_LOWER_METERS, 0));
                } else {
                    isGrabberOpen = !isGrabberOpen;
                    if (isGrabberOpen) {
                        grabberOpenTime = Timer.getFPGATimestamp();
                    }
                }
            }
        }

        if (mechanismStateManager.isMechAtFinalPos()
                && mechanismStateManager.getLastState() == MechanismStates.FINAL_CONE_MIDDLE_SCORING) {
            // We've finished shoving the cone down onto the pole. Now we want to open the grabber
            isGrabberOpen = true;
        }

        if (isGrabberOpen) {
            if (wantedMechanismState == WantedMechanismState.STOWED) {
                grabber.setGrabState(GrabState.IDLE);
                if (setFutureGrabberClose && telescopingArm.getPosition() < 0.2) {
                    isGrabberOpen = false;
                }
            } else {
                if (wantedMechanismState.shouldAutoGrab
                        && grabber.isOpen() && IS_AUTO_GRAB_ENABLED && mechanismStateManager.isMechAtFinalPos() && grabber.isAutoGrabEnabled()) {
                    isGrabberOpen = false;
                } else {
                    grabber.setGrabState(GrabState.OPEN);
                }
            }
        } else {
            if (scoringPositionManager.getWantedPositionType() == PositionType.CONE) {
                grabber.setGrabState(GrabState.GRAB_CONE);
            } else {
                grabber.setGrabState(GrabState.GRAB_CUBE);
            }
        }

        Logger.getInstance().recordOutput("Robot/Is Grabber Open", isGrabberOpen);

        if ((wantedMechanismState == WantedMechanismState.FLOOR_PICKUP
                || wantedMechanismState == WantedMechanismState.STATION_PICKUP_DOUBLE
                || wantedMechanismState == WantedMechanismState.STATION_PICKUP_SINGLE)
                && isGrabberOpen) {
            grabber.setRollerVoltage(GRABBER_ROLLER_VOLTAGE);
        } else if (!isGrabberOpen) {
            grabber.setRollerVoltage(GRABBER_ROLLER_IDLE);
        } else {
            grabber.setRollerVoltage(0);
        }

        if (stick.getRisingEdge(STICK_HOME_ELEVATOR)) {
            elevator.home();
        }

        if (stick.getFallingEdge(STICK_HOME_ELEVATOR)) {
            elevator.cancelHome();
        }

        if (stick.getRisingEdge(STICK_HOME_TELESCOPING_ARM)) {
            telescopingArm.home();
        }

        if (stick.getFallingEdge(STICK_HOME_TELESCOPING_ARM)) {
            telescopingArm.cancelHome();
        }

        xbox.setRumble(RumbleType.kBothRumble, wantedRumble);
    }

    /**
     * Sets the mechanism state to stowed and closes the grabber
     */
    private void setStowed() {
        wantedMechanismState = WantedMechanismState.STOWED;
        setFutureGrabberClose = true;
        wantToClose = false;
    }


    private int lastGridIndex = 0;

    private enum AutoDrivePosition {
        PICKUP_DOUBLE_SUBSTATION,
        PICKUP_SINGLE_SUBSTATION,
        SCORING
    }

    private AutoDrivePosition autoDrivePosition = AutoDrivePosition.SCORING;

    private void updateTeleopDrivingTarget(boolean recalculateGridPosition) {
        ScoringPositionManager scoringPositionManager = ScoringPositionManager.getInstance();

        double x, y;
        Rotation2d rotation;

        var predictedPose = robotTracker.getLatestPose().getTranslation().plus(robotTracker.getVelocity().times(0.5));
        var isOnRedSide = predictedPose.getX() < FIELD_WIDTH_METERS / 2;
        if (isOnRedSide == isRed()) {
            // We're on the same side as our alliance
            // Try to go to the scoring position
            if (recalculateGridPosition) {
                var bestY = ScoringPositionManager.getBestFieldY(
                        scoringPositionManager.getSelectedPosition(),
                        isRed(),
                        robotTracker.getLatestPose().getTranslation(),
                        robotTracker.getVelocity(),
                        false,
                        0
                );
                y = bestY.y();
                lastGridIndex = bestY.gridIndex();
            } else {
                var bestY = ScoringPositionManager.getBestFieldY(
                        scoringPositionManager.getSelectedPosition(),
                        isRed(),
                        robotTracker.getLatestPose().getTranslation(),
                        robotTracker.getVelocity(),
                        true,
                        lastGridIndex
                );
                y = bestY.y();
            }

            Logger.getInstance().recordOutput("Robot/Wanted Y Auto Drive", y);

            double scoringPositionOffset;
            if (scoringPositionManager.getSelectedPosition() == SelectedPosition.MIDDLE_LEFT || scoringPositionManager.getSelectedPosition() == SelectedPosition.MIDDLE_RIGHT) {
                scoringPositionOffset = SCORING_POSITION_OFFSET_CUBE_FROM_WALL;
            } else if (scoringPositionManager.getWantedPositionType() == PositionType.CUBE) {
                scoringPositionOffset = SCORING_POSITION_OFFSET_CUBE_FROM_WALL;
            } else {
                scoringPositionOffset = SCORING_POSITION_OFFSET_CONE_FROM_WALL;
            }
            if (isRed()) {
                x = Constants.GRIDS_RED_X + HALF_ROBOT_WIDTH + scoringPositionOffset;
                rotation = Constants.SCORING_ANGLE_RED;
            } else {
                x = Constants.GRIDS_BLUE_X - HALF_ROBOT_WIDTH - scoringPositionOffset;
                rotation = Constants.SCORING_ANGLE_BLUE;
            }

            autoDrivePosition = AutoDrivePosition.SCORING;
        } else {
            // We're on the opposite side as our alliance
            // Try to go to the pickup position
            var predictedPoseForPickup = robotTracker.getLatestPose().getTranslation().plus(
                    robotTracker.getVelocity().times(0));

            if ((isRed() && robotTracker.getLatestPose().getRotation()
                    .getDegrees() < SINGLE_SUBSTATION_PICKUP_ANGLE_CUTOFF_DEGREES)
                    || (!isRed()
                    && robotTracker.getLatestPose().getRotation().getDegrees() > -135
                    && robotTracker.getLatestPose().getRotation().getDegrees() < 0)) {
                y = SINGLE_STATION_Y;

                if (isRed()) {
                    x = SINGLE_STATION_RED_X;
                } else {
                    x = SINGLE_STATION_BLUE_X;
                }
                rotation = SINGLE_STATION_ANGLE;
                autoDrivePosition = AutoDrivePosition.PICKUP_SINGLE_SUBSTATION;
            } else {
                if (xbox.getRawButton(XBOX_AUTO_DRIVE_LOWER_SUBSTATION)) {
                    y = LOWER_PICKUP_POSITION_Y;
                } else {
                    y = UPPER_PICKUP_POSITION_Y;
                }

                if (isRed()) {
                    x = FIELD_WIDTH_METERS - PICKUP_POSITION_X_OFFSET_FROM_WALL;
                    rotation = PICKUP_ANGLE_RED;
                } else {
                    x = PICKUP_POSITION_X_OFFSET_FROM_WALL;
                    rotation = PICKUP_ANGLE_BLUE;
                }
                autoDrivePosition = AutoDrivePosition.PICKUP_DOUBLE_SUBSTATION;
            }
        }

        Logger.getInstance().recordOutput("Robot/Auto Align Y", y);
        Logger.getInstance().recordOutput("Robot/Auto Align X", x);
        Logger.getInstance().recordOutput("Robot/Auto Align Angle", rotation.getDegrees());
        Logger.getInstance().recordOutput("Robot/Auto Drive Position", autoDrivePosition.name());


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
        // Disable the grabber
        grabber.setAutoGrab(false);
        grabber.setGrabState(GrabState.IDLE);
        wasAuto = false;
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
        Logger.getInstance().recordOutput("Robot/Xbox Controller X", inputs.getX());
        Logger.getInstance().recordOutput("Robot/Xbox Controller Y", inputs.getY());
        Logger.getInstance().recordOutput("Robot/Xbox Controller Rotation", inputs.getRotation());

        return inputs;
    }

    public static boolean isRed() {
        return Objects.equals(sideChooser.get(), "red");
    }

    public static boolean isOnAllianceSide() {
        return isRed() == robotTracker.isOnRedSide();
    }

    @AutoBuilderAccessible
    public static @NotNull Drive getDrive() {
        return drive;
    }

    @AutoBuilderAccessible
    public static @NotNull RobotTracker getRobotTracker() {
        return robotTracker;
    }

    @AutoBuilderAccessible
    public static @NotNull VisionHandler getVisionHandler() {
        return visionHandler;
    }

    @AutoBuilderAccessible
    public static @NotNull Grabber getGrabber() {
        return grabber;
    }

    @AutoBuilderAccessible
    public static @NotNull TelescopingArm getTelescopingArm() {
        return telescopingArm;
    }

    @AutoBuilderAccessible
    public static @NotNull Elevator getElevator() {
        return elevator;
    }

    @AutoBuilderAccessible
    public static @NotNull MechanismStateManager getMechanismStateManager() {
        return mechanismStateManager;
    }

    public static @NotNull PowerDistribution getPowerDistribution() {
        return powerDistribution;
    }


    private static ConcurrentLinkedDeque<Runnable> toRunOnMainThread = new ConcurrentLinkedDeque<>();

    public static void runOnMainThread(Runnable runnable) {
        toRunOnMainThread.add(runnable);
    }

    private void runAsyncScheduledTasks() {
        while (!toRunOnMainThread.isEmpty()) {
            toRunOnMainThread.poll().run();
        }
    }

    public static boolean isOnMainThread() {
        return mainThread == Thread.currentThread();
    }

    public static void setCurrentWantedState(WantedMechanismState state) {
        runOnMainThread(() -> wantedMechanismState = state);
    }

    public static double getAutoStartTime() {
        return autoStartTime;
    }
}
