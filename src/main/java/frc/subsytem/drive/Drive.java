// Copyright 2019 FRC Team 3476 Code Orange

package frc.subsytem.drive;

import com.dacubeking.AutoBuilder.robot.sender.pathpreview.RobotPositionSender;
import com.dacubeking.AutoBuilder.robot.sender.pathpreview.RobotState;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.subsytem.AbstractSubsystem;
import frc.utility.ControllerDriveInputs;
import frc.utility.OrangeUtility;
import frc.utility.PathGenerator;
import frc.utility.net.editing.LiveEditableValue;
import frc.utility.swerve.SwerveSetpointGenerator;
import frc.utility.swerve.SwerveSetpointGenerator.KinematicLimit;
import frc.utility.wpimodified.HolonomicDriveController;
import frc.utility.wpimodified.PIDController;
import org.jetbrains.annotations.Contract;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import java.util.Optional;
import java.util.concurrent.CompletableFuture;

import static frc.robot.Constants.*;

public final class Drive extends AbstractSubsystem {
    private static final Pose2d IDENTITY_POSE = new Pose2d();
    private final SwerveSetpointGenerator setpointGenerator = new SwerveSetpointGenerator(SWERVE_DRIVE_KINEMATICS);
    private final @NotNull LiveEditableValue<Double> turnP = new LiveEditableValue<>(DEFAULT_TURN_P, SmartDashboard.getEntry(
            "TurnPIDP"));
    private final @NotNull LiveEditableValue<Double> turnI = new LiveEditableValue<>(DEFAULT_TURN_I, SmartDashboard.getEntry(
            "TurnPIDI"));
    private final @NotNull LiveEditableValue<Double> turnD = new LiveEditableValue<>(DEFAULT_TURN_D, SmartDashboard.getEntry(
            "TurnPIDD"));
    private final @NotNull PIDController turnPID;
    private final @NotNull PIDController balancePID;

    private final DriveIO io;
    private final DriveInputsAutoLogged inputs = new DriveInputsAutoLogged();
    private final LiveEditableValue<Double> autoP = new LiveEditableValue<>(DEFAULT_AUTO_P, SmartDashboard.getEntry("AutoP"));
    private final LiveEditableValue<Double> autoI = new LiveEditableValue<>(DEFAULT_AUTO_I, SmartDashboard.getEntry("AutoI"));

    private final LiveEditableValue<Double> autoD = new LiveEditableValue<>(DEFAULT_AUTO_D, SmartDashboard.getEntry("AutoD"));

    private final LiveEditableValue<Double> driveKa = new LiveEditableValue<>(DRIVE_FEEDFORWARD[0].ka, SmartDashboard.getEntry(
            "DriveKa"));
    private double autoStartTime;
    private boolean swerveAutoControllerInitialized = false;
    private Trajectory currentAutoTrajectory;
    private Rotation2d autoTargetHeading;
    private double lastTurnUpdate = 0;
    private @NotNull DriveState driveState = DriveState.TELEOP;
    private @Nullable HolonomicDriveController swerveAutoController;
    private double nextAllowedPrintError = 0;
    private @NotNull ChassisSpeeds nextChassisSpeeds = new ChassisSpeeds();
    private boolean isHold = false;
    private KinematicLimit kinematicLimit = KinematicLimits.NORMAL_DRIVING.kinematicLimit;
    private @Nullable CompletableFuture<Optional<Trajectory>> trajectoryToDrive = null;
    private double realtimeTrajectoryStartTime = 0;
    private @Nullable Translation2d realtimeTrajectoryStartVelocity = null;

    {
        turnPID = new PIDController(turnP.get(), turnI.get(), turnD.get());
        turnPID.enableContinuousInput(-Math.PI, Math.PI);
        turnPID.setIntegratorRange(-Math.PI * 2 * 4, Math.PI * 2 * 4);
    }

    {
        // Balance PID takes in degrees of rotation on the axis parallel to the wide edge of the charging station and returns a velocity in meters per second
        balancePID = new PIDController(Constants.BALANCE_P, Constants.BALANCE_I, Constants.BALANCE_D);
        balancePID.setSetpoint(0);
    }

    public Drive(DriveIO driveIO) {
        super();
        this.io = driveIO;
        setDriveState(DriveState.TELEOP);
    }

    private void resetAuto() {
        ProfiledPIDController autoTurnPIDController
                = new ProfiledPIDController(turnP.get(), turnI.get(), turnD.get(),
                new TrapezoidProfile.Constraints(2.2 * Math.PI, Math.PI * 5));
        autoTurnPIDController.enableContinuousInput(-Math.PI, Math.PI);
        autoTurnPIDController.setTolerance(Math.toRadians(1));

        swerveAutoController = new HolonomicDriveController(
                new PIDController(autoP.get(), autoI.get(), autoD.get()),
                new PIDController(autoP.get(), autoI.get(), autoD.get()),
                autoTurnPIDController);
        swerveAutoController.setEnabled(true);
        swerveAutoController.setTolerance(
                new Pose2d(ALLOWED_XY_ERROR_RAMSETE, ALLOWED_XY_ERROR_RAMSETE, Rotation2d.fromDegrees(5)));
    }

    /**
     * @return Returns requested drive wheel velocity in Meters per second
     */
    private double getSwerveDriveVelocity(int motorNum) {
        return inputs.driveMotorVelocities[motorNum];
    }

    public synchronized void setDriveState(@NotNull DriveState driveState) {
        if (driveState == DriveState.STOP && this.driveState == DriveState.HOLD) {
            return;
        }
        this.driveState = driveState;
    }

    public synchronized void doHold() {
        setDriveState(DriveState.HOLD);
    }

    public synchronized void swerveDrive(@NotNull ControllerDriveInputs inputs) {
        setDriveState(DriveState.TELEOP);

        nextChassisSpeeds = new ChassisSpeeds(
                DRIVE_HIGH_SPEED_M * inputs.getX(),
                DRIVE_HIGH_SPEED_M * inputs.getY(),
                inputs.getRotation() * MAX_TELEOP_TURN_SPEED);
        kinematicLimit = KinematicLimits.NORMAL_DRIVING.kinematicLimit;
    }

    public synchronized void swerveDriveFieldRelative(@NotNull ControllerDriveInputs inputs) {
        setDriveState(DriveState.TELEOP);

        nextChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                DRIVE_HIGH_SPEED_M * inputs.getX(),
                DRIVE_HIGH_SPEED_M * inputs.getY(),
                inputs.getRotation() * MAX_TELEOP_TURN_SPEED,
                getPredictedRobotAngleInLoopCenter());
        kinematicLimit = KinematicLimits.NORMAL_DRIVING.kinematicLimit;
    }

    private static final LoggedDashboardNumber rotationPredictionSeconds =
            new LoggedDashboardNumber("Rotation Prediction Seconds", 0.08);

    static {
        Logger.getInstance().registerDashboardInput(rotationPredictionSeconds);
    }

    /**
     * Robot angle to use for swerve drive field relative calculations. Adds a fudge factor to the robot angle to correct for the
     * robot drifting while turning.
     *
     * @return Robot angle to use for swerve drive field relative calculations
     */
    private static Rotation2d getPredictedRobotAngleInLoopCenter() {
        double predictionSeconds = rotationPredictionSeconds.get();
        return Robot.getRobotTracker().getGyroAngleAtTime(Timer.getFPGATimestamp()).toRotation2d()
                .plus(Rotation2d.fromDegrees(
                        Robot.getRobotTracker().getAngularVelocity() * predictionSeconds));
    }

    PIDController drivePositionPidX = new PIDController(4.1, 0, 0.0);
    PIDController drivePositionPidY = new PIDController(4.1, 0, 0.0);

    LoggedDashboardNumber autoDrivePathEarlyEndSeconds =
            new LoggedDashboardNumber("Auto Drive Path Early End Seconds", AUTO_DRIVE_PATH_EARLY_END_SECONDS);

    {
        SmartDashboard.putData(drivePositionPidX);
        SmartDashboard.putData(drivePositionPidY);
        Logger.getInstance().registerDashboardInput(autoDrivePathEarlyEndSeconds);
    }

    /**
     * Drive the robot to a given position using a trajectory
     *
     * @param targetPosition The target position to drive to
     * @param targetAngle    The target angle to drive to
     * @param inputs         Controller inputs to use if the trajectory fails to be generated
     * @return A boolean indicating whether the robot was successfully able to generate a trajectory to the target position.
     */
    public synchronized boolean driveToPosition(Translation2d targetPosition, Rotation2d targetAngle,
                                                ControllerDriveInputs inputs, boolean singleStationPickup,
                                                double maxAcceleration) {

        Translation2d positionError = Robot.getRobotTracker().getLatestPose().getTranslation().minus(targetPosition);
        if (// Are we close enough to the target position?
                Math.abs(positionError.getX()) < PID_CONTROL_RANGE_AUTO_DRIVE_METERS
                        && Math.abs(positionError.getY()) < PID_CONTROL_RANGE_AUTO_DRIVE_METERS

                        // Is the trajectory done? (bypass this check if we have no trajectory)
                        && (trajectoryToDrive == null || !trajectoryToDrive.isDone() || trajectoryToDrive.join().isEmpty()
                        || getAutoElapsedTime() + autoDrivePathEarlyEndSeconds.get() >=
                        trajectoryToDrive.join().get().getTotalTimeSeconds())) {
            if (Math.abs(positionError.getX()) < ALLOWED_AUTO_DRIVE_POSITION_ERROR_METERS
                    && Math.abs(positionError.getY()) < ALLOWED_AUTO_DRIVE_POSITION_ERROR_METERS) {

                // We're within allowed error of the target position, only turn to the target angle
                setTurn(new ControllerDriveInputs(),
                        new State(targetAngle.getRadians(), 0),
                        ALLOWED_AUTO_DRIVE_ANGLE_ERROR_RADIANS);
            } else {
                // We're not within allowed error of the target position, try to pid to the target position
                var currPos = Robot.getRobotTracker().getLatestPose().getTranslation();
                setTurn(
                        new ControllerDriveInputs(
                                drivePositionPidX.calculate(currPos.getX(), targetPosition.getX()) / DRIVE_HIGH_SPEED_M,
                                drivePositionPidY.calculate(currPos.getY(), targetPosition.getY()) / DRIVE_HIGH_SPEED_M,
                                0
                        ),
                        new State(targetAngle.getRadians(), 0),
                        0,
                        true
                );
            }
            return true;
        }
        if (!(driveState == DriveState.WAITING_FOR_PATH || driveState == DriveState.RAMSETE)) {
            // Path hasn't been generated yet, generate it
            var robotTracker = Robot.getRobotTracker();
            realtimeTrajectoryStartVelocity = robotTracker.getVelocity();
            trajectoryToDrive = PathGenerator.generateTrajectory(
                    realtimeTrajectoryStartVelocity,
                    robotTracker.getLatestPose().getTranslation(),
                    targetPosition,
                    START_POS_PREDICT_AHEAD, singleStationPickup, maxAcceleration);
            realtimeTrajectoryStartTime = Timer.getFPGATimestamp() + START_POS_PREDICT_AHEAD;
            setDriveState(DriveState.WAITING_FOR_PATH);
            setTurn(new ControllerDriveInputs(
                            realtimeTrajectoryStartVelocity.getX() / DRIVE_HIGH_SPEED_M,
                            realtimeTrajectoryStartVelocity.getY() / DRIVE_HIGH_SPEED_M,
                            0
                    ),
                    new State(targetAngle.getRadians(), 0),
                    0,
                    false);
            kinematicLimit = KinematicLimits.NORMAL_DRIVING.kinematicLimit;
        }

        if (driveState == DriveState.WAITING_FOR_PATH) { // wow
            // Path is being generated
            assert trajectoryToDrive != null;
            assert realtimeTrajectoryStartVelocity != null;
            if (trajectoryToDrive.isDone()) {
                if (Timer.getFPGATimestamp() > realtimeTrajectoryStartTime) {
                    // Optional of the trajectory. Empty if the trajectory failed to be generated
                    var trajectory = trajectoryToDrive.join();

                    if (trajectory.isPresent()) {
                        // Path was generated successfully, set the auto path
                        if (Timer.getFPGATimestamp() + EXPECTED_TELEOP_DRIVE_DT > realtimeTrajectoryStartTime) {
                            DriverStation.reportError("Trajectory Generation was late by: "
                                    + (Timer.getFPGATimestamp() - realtimeTrajectoryStartTime) + "s", false);


                            realtimeTrajectoryStartTime =
                                    (Timer.getFPGATimestamp() - realtimeTrajectoryStartTime) / 3 + Timer.getFPGATimestamp();
                        }
                        setAutoPath(trajectory.get(), realtimeTrajectoryStartTime); // Sets the DriveState to RAMSETE
                        setAutoRotation(targetAngle);
                    } else {
                        // Path finished generating before it's start time, just continue driving at the current velocity until
                        // we reach the start time
                        setTurn(new ControllerDriveInputs(
                                        realtimeTrajectoryStartVelocity.getX() / DRIVE_HIGH_SPEED_M,
                                        realtimeTrajectoryStartVelocity.getY() / DRIVE_HIGH_SPEED_M,
                                        0
                                ),
                                new State(targetAngle.getRadians(), 0),
                                0,
                                false);
                        kinematicLimit = KinematicLimits.NORMAL_DRIVING.kinematicLimit;
                        return false;
                    }
                }
            } else {
                // Path is still being generated, continue driving at the current velocity until it's done (also start turning
                // to the correct angle)
                assert realtimeTrajectoryStartVelocity != null;
                setTurn(new ControllerDriveInputs(
                                realtimeTrajectoryStartVelocity.getX() / DRIVE_HIGH_SPEED_M,
                                realtimeTrajectoryStartVelocity.getY() / DRIVE_HIGH_SPEED_M,
                                0
                        ),
                        new State(targetAngle.getRadians(), 0),
                        0,
                        false);
                kinematicLimit = KinematicLimits.NORMAL_DRIVING.kinematicLimit;
            }
        }
        return true;
    }

    public synchronized double getRemainingAutoDriveTime() {
        if ((driveState == DriveState.RAMSETE || driveState == DriveState.TURN)
                && trajectoryToDrive != null
                && trajectoryToDrive.isDone()
                && trajectoryToDrive.join().isPresent()) {
            return trajectoryToDrive.join().get().getTotalTimeSeconds() - getAutoElapsedTime();
        }
        return Double.MAX_VALUE;
    }

    public synchronized void alignToYAndYaw(double targetYaw, double targetY, ControllerDriveInputs inputs) {

        var currPos = Robot.getRobotTracker().getLatestPose().getTranslation();
        var wantedYCommand = drivePositionPidY.calculate(currPos.getY(), targetY) / DRIVE_HIGH_SPEED_M;

        if (Math.abs(currPos.getY() - targetY) < ALLOWED_AUTO_DRIVE_POSITION_ERROR_METERS) {
            setTurn(new ControllerDriveInputs(inputs.getX(), 0, 0),
                    new State(targetYaw, 0),
                    ALLOWED_AUTO_DRIVE_ANGLE_ERROR_RADIANS);
        } else {
            setTurn(
                    new ControllerDriveInputs(
                            inputs.getX(),
                            wantedYCommand,
                            0
                    ),
                    new State(targetYaw, 0),
                    0,
                    true
            );
        }
    }


    private synchronized void swerveDrive(@NotNull ChassisSpeeds desiredRobotRelativeSpeeds, KinematicLimit kinematicLimit,
                                          double dt) {

        var moduleStates = SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(desiredRobotRelativeSpeeds);

        boolean rotate = desiredRobotRelativeSpeeds.omegaRadiansPerSecond != 0
                || desiredRobotRelativeSpeeds.vxMetersPerSecond != 0
                || desiredRobotRelativeSpeeds.vyMetersPerSecond != 0;


        SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates,
                DRIVE_FEEDFORWARD[0].maxAchievableVelocity(SWERVE_DRIVE_VOLTAGE_LIMIT_AUTO, 0));

        setSwerveModuleStates(moduleStates, rotate);
    }

    private synchronized void setSwerveModuleStates(SwerveModuleState[] swerveModuleStates, boolean rotate) {
        Logger.getInstance().recordOutput("Drive/Wanted Swerve Module States", swerveModuleStates);

        for (int i = 0; i < 4; i++) {
            var moduleState = swerveModuleStates[i];
            moduleState = SwerveModuleState.optimize(moduleState, Rotation2d.fromDegrees(getWheelRotation(i)));
            double currentAngle = getWheelRotation(i);

            double angleDiff = getAngleDiff(moduleState.angle.getDegrees(), currentAngle);

            if (rotate) {
                if (Math.abs(angleDiff) > ALLOWED_SWERVE_ANGLE_ERROR) {
                    if (USE_CANCODERS) {
                        io.setSwerveMotorPosition(i, inputs.swerveMotorRelativePositions[i] + angleDiff);
                    } else {
                        io.setSwerveMotorPosition(i, moduleState.angle.getDegrees());
                    }
                } else {
                    io.setSwerveMotorPosition(i, inputs.swerveMotorRelativePositions[i]);
                }
            }

            setMotorSpeed(i, moduleState.speedMetersPerSecond, 0);
            //setMotorSpeed(i, 0, 0);

            Logger.getInstance().recordOutput("Drive/SwerveModule " + i + " Wanted Angle", moduleState.angle.getDegrees());
            Logger.getInstance().recordOutput("Drive/SwerveModule " + i + " Wanted Speed", moduleState.speedMetersPerSecond);
            Logger.getInstance().recordOutput("Drive/SwerveModule " + i + " Wanted Acceleration", 0);
            Logger.getInstance().recordOutput("Drive/SwerveModule " + i + " Angle Error", angleDiff);
            Logger.getInstance().recordOutput("Drive/SwerveModule " + i + " Wanted State", moduleState);
            Logger.getInstance().recordOutput("Drive/SwerveModule " + i + " Wanted Relative Angle",
                    inputs.swerveMotorRelativePositions[i] + angleDiff);
        }
    }

    /**
     * @param targetAngle  The angle we want to be at (0-360)
     * @param currentAngle The current angle of the module (0-360)
     * @return the shortest angle between the two angles
     */
    private double getAngleDiff(double targetAngle, double currentAngle) {
        double angleDiff = targetAngle - currentAngle;
        if (angleDiff > 180) {
            angleDiff -= 360;
        }

        if (angleDiff < -180) {
            angleDiff += 360;
        }
        return angleDiff;
    }

    double[] lastModuleVelocities = new double[4];
    double[] lastModuleTimes = new double[4];


    /**
     * Sets the motor voltage
     *
     * @param module   The module to set the voltage on
     * @param velocity The target velocity
     */
    private void setMotorSpeed(int module, double velocity, double acceleration) {
        if (module < 0 || module > DRIVE_FEEDFORWARD.length) {
            throw new IllegalArgumentException("Module must be between 0 and 3");
        }

        double ffv = DRIVE_FEEDFORWARD[module].calculate(velocity, 0);
        io.setDriveMotorVoltage(module, ffv, driveState != DriveState.TELEOP);

        Logger.getInstance().recordOutput("Drive/Out Volts " + module, ffv);
        Logger.getInstance().recordOutput("Drive/Out Volts Ks" + module, DRIVE_FEEDFORWARD[module].ks * Math.signum(velocity));
        Logger.getInstance().recordOutput("Drive/Out Volts Kv" + module, DRIVE_FEEDFORWARD[module].kv * velocity);
        Logger.getInstance().recordOutput("Drive/Out Volts Ka" + module, DRIVE_FEEDFORWARD[module].ka * acceleration);
        Logger.getInstance().recordOutput("Drive/Voltage Contrib to Accel" + module,
                ffv - DRIVE_FEEDFORWARD[module].calculate(getSwerveDriveVelocity(module)));

        double time = inputs.driveIoTimestamp;
        double realAccel = (getSwerveDriveVelocity(module) - lastModuleVelocities[module]) / (time - lastModuleTimes[module]);

        Logger.getInstance().recordOutput("Drive/Acceleration" + module, realAccel);
        Logger.getInstance().recordOutput("Drive/Expected Accel" + module,
                (ffv - DRIVE_FEEDFORWARD[module].calculate(getSwerveDriveVelocity(module)) / DRIVE_FEEDFORWARD[module].ka));

        lastModuleVelocities[module] = getSwerveDriveVelocity(module);
        lastModuleTimes[module] = Logger.getInstance().getRealTimestamp() * SECONDS_PER_MICROSECOND;


        if (PERIODIC_DRIVE_PRINT && (Timer.getFPGATimestamp() > nextPeriodicDrivePrint)) {
            // Used to collect data for characterizing the robot
            nextPeriodicDrivePrint = Timer.getFPGATimestamp() + 0.1;
            for (int i = 0; i < 4; i++) {
                double accelI = (getSwerveDriveVelocity(i) - lastModuleVelocities[i]) / (time - lastModuleTimes[i]);
                if (Math.abs(accelI) < 8.1 && Math.abs(lastModuleVelocities[i]) > 0.01) {
                    System.out.printf("%.5f, %.5f, %.5f%n", ffv, lastModuleVelocities[i], accelI);
                }
            }
        }
    }

    /**
     * Set the robot's rotation when driving a path. This is meant to be used in the auto builder. Code that wants to use this
     * should use {@link Drive#setAutoRotation(Rotation2d)} instead.
     *
     * @param angle The angle to set the robot to
     */
    public synchronized void setAutoRotation(double angle) {
        setAutoRotation(Rotation2d.fromDegrees(angle));
    }

    // May be called from any thread
    public synchronized void setAutoPath(Trajectory trajectory) {
        System.out.println("Setting Auto Path");
        setAutoPath(trajectory, Timer.getFPGATimestamp());
    }

    public synchronized void setAutoPath(Trajectory trajectory, double autoStartTime) {
        swerveAutoControllerInitialized = false;
        this.currentAutoTrajectory = trajectory;
        this.autoStartTime = autoStartTime;
        setDriveState(DriveState.RAMSETE);
        Logger.getInstance().recordOutput("Drive/Trajectory", trajectory);
    }

    @SuppressWarnings("ProhibitedExceptionCaught")
    private synchronized void updateRamsete() {
        if (!swerveAutoControllerInitialized) {
            resetAuto();
            assert currentAutoTrajectory != null;
            swerveAutoControllerInitialized = true;
        }

        double pathTime = Timer.getFPGATimestamp() - autoStartTime;

        Pose2d currentPose = Robot.getRobotTracker().getLatestPose();

        Trajectory.State goal = currentAutoTrajectory.sample(pathTime);

        Rotation2d targetHeading = autoTargetHeading;

        Logger.getInstance().recordOutput("Drive/Auto/Target Heading", targetHeading.getDegrees());
        Logger.getInstance().recordOutput("Drive/Auto/Target X", goal.poseMeters.getTranslation().getX());
        Logger.getInstance().recordOutput("Drive/Auto/Target Y", goal.poseMeters.getTranslation().getY());
        Logger.getInstance().recordOutput("Drive/Auto/Target Velocity", goal.velocityMetersPerSecond);
        Logger.getInstance().recordOutput("Drive/Auto/Target Acceleration", goal.accelerationMetersPerSecondSq);
        Logger.getInstance().recordOutput("Drive/Auto/Target Pose", goal.poseMeters);
        Logger.getInstance().recordOutput("Drive/Auto/Path Time", Timer.getFPGATimestamp() - autoStartTime);
        Logger.getInstance().recordOutput("Drive/Auto/Path Total Time", currentAutoTrajectory.getTotalTimeSeconds());
        Logger.getInstance().recordOutput("Drive/Auto/X Error",
                goal.poseMeters.getTranslation().getX() - currentPose.getTranslation().getX());
        Logger.getInstance().recordOutput("Drive/Auto/Y Error",
                goal.poseMeters.getTranslation().getY() - currentPose.getTranslation().getY());
        Logger.getInstance().recordOutput("Drive/Auto/Heading Error",
                getAngleDiff(targetHeading.getDegrees(), currentPose.getRotation().getDegrees()));

        try {
            if (swerveAutoController == null) {
                DriverStation.reportError("swerveAutoController is null",
                        Thread.getAllStackTraces().get(Thread.currentThread()));
                resetAuto();
            }
            assert swerveAutoController != null;
            Logger.getInstance().recordOutput("Drive/Auto/Profiled Heading Error",
                    getAngleDiff(Math.toDegrees(swerveAutoController.getThetaController().getGoal().position),
                            currentPose.getRotation().getDegrees()));

            Pose2d goalPose = new Pose2d(goal.poseMeters.getTranslation(),
                    new Rotation2d(swerveAutoController.getThetaController().getGoal().position));
            RobotPositionSender.addRobotPosition(new RobotState(goalPose, "Auto Goal Pose"));


            Trajectory.State oldGoal = currentAutoTrajectory.sample(
                    pathTime - EXPECTED_TELEOP_DRIVE_DT
            );

            Trajectory.State nextGoal = currentAutoTrajectory.sample(
                    pathTime + EXPECTED_TELEOP_DRIVE_DT
            );

            var modifiedGoal = new Trajectory.State(
                    goal.timeSeconds,
                    goal.velocityMetersPerSecond,
                    goal.accelerationMetersPerSecondSq,
                    oldGoal.poseMeters,
                    goal.curvatureRadPerMeter
            );

            var autoWantedState = swerveAutoController.calculate(
                    currentPose,
                    modifiedGoal,
                    targetHeading,
                    getPredictedRobotAngleInLoopCenter() // Send the fudged angle to the controller for the swerve drive field
                    // relative calculations
            );

            if (Timer.getFPGATimestamp() - autoStartTime < currentAutoTrajectory.getTotalTimeSeconds()) {
                var currVel = new Translation2d(
                        goal.velocityMetersPerSecond * goal.poseMeters.getRotation().getCos(),
                        goal.velocityMetersPerSecond * goal.poseMeters.getRotation().getSin()
                );

                var nextVel = new Translation2d(
                        nextGoal.velocityMetersPerSecond * nextGoal.poseMeters.getRotation().getCos(),
                        nextGoal.velocityMetersPerSecond * nextGoal.poseMeters.getRotation().getSin()
                );

                // Fudge the acceleration we want into the commanded velocity
                var accel = nextVel.minus(currVel).div(EXPECTED_TELEOP_DRIVE_DT);
                var extraSpeed = accel.times(driveKa.get()).div(DRIVE_FEEDFORWARD[0].kv);
                var extraSpeedRobotRelative = extraSpeed.rotateBy(currentPose.getRotation().unaryMinus());

                autoWantedState.vxMetersPerSecond += extraSpeedRobotRelative.getX();
                autoWantedState.vyMetersPerSecond += extraSpeedRobotRelative.getY();
            }

            nextChassisSpeeds = autoWantedState;
            kinematicLimit = KinematicLimits.NORMAL_DRIVING.kinematicLimit;
            if (swerveAutoController.atReference()
                    && (Timer.getFPGATimestamp() - autoStartTime) >= currentAutoTrajectory.getTotalTimeSeconds()
                    && Robot.getRobotTracker().getVelocity().getNorm() > MAX_VELOCITY_END_PATH) {
                setDriveState(DriveState.DONE);
                stopMovement();
            }
        } catch (NullPointerException exception) {
            if (Timer.getFPGATimestamp() > nextAllowedPrintError) {
                exception.printStackTrace();
                nextAllowedPrintError = Timer.getFPGATimestamp() + 2;
            }
        }
    }

    /**
     * @param rotationGoal The goal to aim at (in radians)
     * @return The speed to turn at (in radians/s)
     */
    private double getTurnPidDeltaSpeed(@NotNull TrapezoidProfile.State rotationGoal, boolean limitSpeed) {
        turnPID.setPID(turnP.get(), turnI.get(), turnD.get());
        turnPID.setSetpoint(rotationGoal.position);

        if (Timer.getFPGATimestamp() - 0.2 > lastTurnUpdate) {
            turnPID.reset();
        } else if (turnPID.getPositionError() > Math.toRadians(MAX_TELEOP_TURN_SPEED)) {
            // This is basically an I-Zone
            turnPID.resetI();
        }
        lastTurnUpdate = Timer.getFPGATimestamp();
        double pidSpeed =
                turnPID.calculate(Robot.getRobotTracker().getGyroAngle().getRadians()) + rotationGoal.velocity;

        if (limitSpeed) {
            pidSpeed = Math.copySign(Math.min(Math.abs(pidSpeed), Constants.TURN_SPEED_LIMIT_WHILE_AIMING), pidSpeed);
        }
        return pidSpeed;
    }

    public synchronized void setAutoRotation(@NotNull Rotation2d rotation) {
        System.out.println("Auto Target Heading: " + rotation);
        autoTargetHeading = rotation;
    }

    public synchronized double getAutoElapsedTime() {
        return Timer.getFPGATimestamp() - autoStartTime;
    }

    private double nextPeriodicDrivePrint = 15;

    @Override
    public synchronized void update() {
        var lastTimeStep = inputs.driveIoTimestamp;
        io.updateInputs(inputs);
        Logger.getInstance().processInputs("Drive", inputs);

        isHold = false;
        switch (driveState) {
            case TURN, WAITING_FOR_PATH -> updateTurn();
            case HOLD -> isHold = true;
            case STOP -> {
                nextChassisSpeeds = new ChassisSpeeds();
                kinematicLimit = KinematicLimits.NORMAL_DRIVING.kinematicLimit;
            }
            case RAMSETE -> updateRamsete();
            case AUTO_BALANCE -> autoBalance(ControllerDriveInputs.ZERO);
        }
        if (isHold) {
            setSwerveModuleStates(Constants.HOLD_MODULE_STATES, true);
        } else if (!DriverStation.isTest() && DriverStation.isEnabled()) {
            var dt = inputs.driveIoTimestamp - lastTimeStep;
            swerveDrive(nextChassisSpeeds, kinematicLimit, dt);
        }
    }

    /**
     * For auto use
     */
    public synchronized void setRotation(double angle, double allowedError) {
        setTurn(new ControllerDriveInputs(), new State(Math.toRadians(angle), 0), Math.toRadians(allowedError));
    }

    /**
     * This method takes in x and y velocity as well as the target heading to calculate how much the robot needs to turn in order
     * to face a target
     * <p>
     * xVelocity and yVelocity are in m/s
     *
     * @param controllerDriveInputs The x and y velocity of the robot (rotation is ignored)
     * @param goal                  The target state at the end of the turn (in radians, radians/s)
     * @param turnErrorRadians      The error in radians that the robot can be off by and still be considered done turning
     */
    public synchronized void setTurn(ControllerDriveInputs controllerDriveInputs, State goal, double turnErrorRadians) {
        setTurn(controllerDriveInputs, goal, turnErrorRadians, true);
    }


    /**
     * This method takes in x and y velocity as well as the target heading to calculate how much the robot needs to turn in order
     * to face a target
     * <p>
     * xVelocity and yVelocity are in m/s
     *
     * @param controllerDriveInputs The x and y velocity of the robot (rotation is ignored)
     * @param goal                  The target state at the end of the turn (in radians, radians/s)
     * @param turnErrorRadians      The error in radians that the robot can be off by and still be considered done turning
     */
    private synchronized void setTurn(ControllerDriveInputs controllerDriveInputs, State goal, double turnErrorRadians,
                                      boolean setTurn) {
        TurnInputs.controllerDriveInputs = controllerDriveInputs;
        TurnInputs.goal = goal;
        TurnInputs.turnErrorRadians = turnErrorRadians;
        if (setTurn) {
            setDriveState(DriveState.TURN);
        }
    }

    private synchronized void updateTurn() {
        if (TurnInputs.controllerDriveInputs == null) {
            TurnInputs.controllerDriveInputs = new ControllerDriveInputs();
        }
        double pidDeltaSpeed = getTurnPidDeltaSpeed(TurnInputs.goal,
                !(TurnInputs.controllerDriveInputs.getX() == 0 && TurnInputs.controllerDriveInputs.getY() == 0));

//        System.out.println(
//                "turn error: " + Math.toDegrees(turnPID.getPositionError()) + " delta speed: " + Math.toDegrees(pidDeltaSpeed));

        if (Math.abs(TurnInputs.goal.position - Robot.getRobotTracker().getGyroAngle().getRadians())
                < TurnInputs.turnErrorRadians) {
            if (this.driveState == DriveState.TURN) {
                this.driveState = DriveState.DONE;
                pidDeltaSpeed = 0;
            }
        }

        nextChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                TurnInputs.controllerDriveInputs.getX() * DRIVE_HIGH_SPEED_M,
                TurnInputs.controllerDriveInputs.getY() * DRIVE_HIGH_SPEED_M,
                pidDeltaSpeed,
                getPredictedRobotAngleInLoopCenter());


        double curSpeed = Robot.getRobotTracker().getAngularVelocity();
        Logger.getInstance().recordOutput("Drive/Turn Position Error", Math.toDegrees(turnPID.getPositionError()));
        Logger.getInstance().recordOutput("Drive/Turn Actual Speed", curSpeed);
        Logger.getInstance().recordOutput("Drive/Turn PID Command", pidDeltaSpeed);
        Logger.getInstance().recordOutput("Drive/Turn PID Setpoint Position", TurnInputs.goal.position);
        Logger.getInstance().recordOutput("Drive/Turn PID Setpoint Velocity", TurnInputs.goal.velocity);
        Logger.getInstance().recordOutput("Drive/Turn PID Measurement", Robot.getRobotTracker().getGyroAngle().getRadians());
    }

    public void stopMovement() {
        setDriveState(DriveState.STOP);
    }

    public synchronized boolean isFinished() {
        return (driveState == DriveState.STOP || driveState == DriveState.DONE || driveState == DriveState.TELEOP);
    }

    @Override
    public synchronized void logData() {
        Logger.getInstance().recordOutput("Drive/Actual Swerve Module States", getModuleStates());
        Logger.getInstance().recordOutput("Drive/Drive State", driveState.toString());
    }

    /**
     * Returns the angle/position of the requested encoder module
     *
     * @param moduleNumber the module to get the angle of
     * @return angle in degrees of the module (0-360)
     */
    public double getWheelRotation(int moduleNumber) {
        if (USE_RELATIVE_ENCODER_POSITION) {
            double relPos = inputs.swerveMotorRelativePositions[moduleNumber] % 360;
            if (relPos < 0) relPos += 360;
            return relPos;
        } else {
            return inputs.swerveMotorAbsolutePositions[moduleNumber];
        }
    }

    /**
     * Returns how far the encoder has measured the wheel has turned since the last reset
     *
     * @param moduleNumber the module to get the position of
     * @return distance in meters
     */
    public double getDrivePosition(int moduleNumber) {
        return inputs.driveMotorPositions[moduleNumber];
    }

    @Contract(pure = true)
    public SwerveModulePosition @NotNull [] getModulePositions() {
        SwerveModulePosition[] swerveModulePositions = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            swerveModulePositions[i] = new SwerveModulePosition(
                    getDrivePosition(i),
                    Rotation2d.fromDegrees(getWheelRotation(i)));
        }
        return swerveModulePositions;
    }

    @Contract(pure = true)
    public SwerveModuleState @NotNull [] getModuleStates() {
        SwerveModuleState[] swerveModuleStates = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            swerveModuleStates[i] = new SwerveModuleState(
                    getSwerveDriveVelocity(i),
                    Rotation2d.fromDegrees(getWheelRotation(i)));
        }
        return swerveModuleStates;
    }

    public void turnToAngle(double degrees, double allowedError) throws InterruptedException {
        System.out.println("Turning to " + degrees);
        if (!Robot.isRed()) {
            // If we are blue, we need to flip the angle, so we don't need to change the scripts in the autobuilder
            double rad = Math.toRadians(degrees);
            double cos = Math.cos(rad);
            double sin = Math.sin(rad);
            degrees = Math.atan2(sin, -cos);
        }
        setRotation(degrees, allowedError);
        while (!isFinished()) {
            //noinspection BusyWait
            Thread.sleep(20);
        }
    }

    LoggedDashboardNumber balanceCompleteThreshold =
            new LoggedDashboardNumber("Auto Balance Complete Threshold", AUTO_BALANCE_COMPLETE_THRESHOLD_DEGREES);
    LoggedDashboardNumber balanceVelocityThreshold =
            new LoggedDashboardNumber("Auto Balance Velocity Threshold", AUTO_BALANCE_VELOCITY_THRESHOLD_DEGREES_PER_SECOND);
    LoggedDashboardNumber balanceReverseVelocity =
            new LoggedDashboardNumber("Auto Balance Reverse Velocity", BALANCE_REVERSE_SPEED);

    {
        Logger.getInstance().registerDashboardInput(balanceCompleteThreshold);
        Logger.getInstance().registerDashboardInput(balanceVelocityThreshold);
        Logger.getInstance().registerDashboardInput(balanceReverseVelocity);
        SmartDashboard.putData(balancePID);
    }

    public synchronized void autoBalance(@NotNull ControllerDriveInputs inputs) {
        var angle = Robot.getRobotTracker().getGyroYAngle(); //TODO: Switch to using raw gyro angles
        var angleMeasure = Math.toDegrees(angle);
        double angularVelocity = Robot.getRobotTracker().getAngularRollVelocity();

        Logger.getInstance().recordOutput("Drive/Auto Balance Angle", angleMeasure);
        Logger.getInstance().recordOutput("Drive/Auto Balance Angle Velocity", angularVelocity);


        double xVelocity;

        if (angleMeasure <= balanceCompleteThreshold.get() && angleMeasure >= -balanceCompleteThreshold.get()) {
            // Stops PID if within this range
            xVelocity = 0;
        } else if (Math.abs(angularVelocity) > balanceVelocityThreshold.get()) {
            // Run backwards a little PID if velocity is too high
            xVelocity = Math.copySign(balanceReverseVelocity.get(), -angleMeasure);
        } else {
            xVelocity = Math.copySign(balancePID.calculate(angleMeasure), angleMeasure);
        }

        Logger.getInstance().recordOutput("Drive/Auto Balance Velocity", xVelocity);

        if (OrangeUtility.doubleEqual(xVelocity, 0, 0.01) && OrangeUtility.doubleEqual(inputs.getY(), 0, 0.01)) {
            nextChassisSpeeds = new ChassisSpeeds();
            isHold = true;
        } else {
            nextChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    xVelocity,
                    DRIVE_HIGH_SPEED_M * inputs.getY(),
                    inputs.getRotation() * MAX_TELEOP_TURN_SPEED,
                    getPredictedRobotAngleInLoopCenter());
        }
    }

    public void autoBalance() throws InterruptedException {
        setDriveState(DriveState.AUTO_BALANCE);
        while (DriverStation.isAutonomous()) {
            Thread.sleep(10);

            if (Timer.getFPGATimestamp() - Robot.getAutoStartTime() > 14.8) {
                setDriveState(DriveState.HOLD);
                return;
            }
        }
    }


    public synchronized void resetAbsoluteZeros() {
        io.resetAbsoluteZeros();
    }

    public synchronized void setBrakeMode(boolean brake) {
        io.setBrakeMode(brake);
    }

    /**
     * @return the timestamp of the last IO update in seconds
     */
    public synchronized double getIoTimestamp() {
        return inputs.driveIoTimestamp;
    }

    public enum DriveState {
        TELEOP, TURN, HOLD, DONE, RAMSETE, STOP, WAITING_FOR_PATH, AUTO_BALANCE
    }

    static final class TurnInputs {
        public static ControllerDriveInputs controllerDriveInputs;
        public static State goal;
        public static double turnErrorRadians;
    }

    /**
     * Sets the voltage compensation level for the drive motors
     *
     * @param voltage the voltage compensation level
     */
    public synchronized void setDriveVoltageCompLevel(double voltage) {
        io.setDriveVoltageCompLevel(voltage);
    }


    public synchronized void resetPeriodicFrames() {
        io.resetPeriodicFrames();
    }
}
