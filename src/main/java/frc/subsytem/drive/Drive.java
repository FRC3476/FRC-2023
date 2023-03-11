// Copyright 2019 FRC Team 3476 Code Orange

package frc.subsytem.drive;

import com.dacubeking.AutoBuilder.robot.sender.pathpreview.RobotPositionSender;
import com.dacubeking.AutoBuilder.robot.sender.pathpreview.RobotState;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
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
import frc.utility.PathGenerator;
import frc.utility.net.editing.LiveEditableValue;
import frc.utility.swerve.SwerveSetpointGenerator;
import frc.utility.swerve.SwerveSetpointGenerator.KinematicLimit;
import frc.utility.swerve.SwerveSetpointGenerator.SwerveSetpoint;
import frc.utility.wpimodified.PIDController;
import org.jetbrains.annotations.Contract;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;
import org.littletonrobotics.junction.Logger;

import java.lang.reflect.Field;
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
    private final Field m_moduleStates;
    private final LiveEditableValue<Double> autoP = new LiveEditableValue<>(DEFAULT_AUTO_P, SmartDashboard.getEntry("AutoP"));
    private final LiveEditableValue<Double> autoI = new LiveEditableValue<>(DEFAULT_AUTO_I, SmartDashboard.getEntry("AutoI"));

    private final LiveEditableValue<Double> autoD = new LiveEditableValue<>(DEFAULT_AUTO_D, SmartDashboard.getEntry("AutoD"));

    private final LiveEditableValue<Double> driveKa = new LiveEditableValue<>(0.3, SmartDashboard.getEntry("DriveKa"));
    private double autoStartTime;
    private boolean swerveAutoControllerInitialized = false;
    private Trajectory currentAutoTrajectory;
    private Rotation2d autoTargetHeading;
    private double lastTurnUpdate = 0;
    private boolean useRelativeEncoderPosition = false;
    private @NotNull DriveState driveState = DriveState.TELEOP;
    private SwerveSetpoint lastSwerveSetpoint = new SwerveSetpoint(
            new ChassisSpeeds(0, 0, 0),
            new SwerveModuleState[]{
                    new SwerveModuleState(0, new Rotation2d(0)),
                    new SwerveModuleState(0, new Rotation2d(0)),
                    new SwerveModuleState(0, new Rotation2d(0)),
                    new SwerveModuleState(0, new Rotation2d(0))
            },
            new double[]{0, 0, 0, 0});
    private @Nullable HolonomicDriveController swerveAutoController;
    private double nextAllowedPrintError = 0;
    private @NotNull ChassisSpeeds nextChassisSpeeds = new ChassisSpeeds();
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


    {
        try {
            m_moduleStates = SwerveDriveKinematics.class.getDeclaredField("m_moduleStates");
            m_moduleStates.setAccessible(true);
        } catch (NoSuchFieldException e) {
            throw new RuntimeException(e);
        }
    }

    public Drive(DriveIO driveIO) {
        super();
        this.io = driveIO;

        setDriveState(DriveState.TELEOP);
    }

    private void resetAuto() {
        ProfiledPIDController autoTurnPIDController
                = new ProfiledPIDController(turnP.get(), turnI.get(), turnD.get(),
                new TrapezoidProfile.Constraints(1.7 * Math.PI, Math.PI * 3.5));
        autoTurnPIDController.enableContinuousInput(-Math.PI, Math.PI);
        autoTurnPIDController.setTolerance(Math.toRadians(1));

        swerveAutoController = new HolonomicDriveController(
                new PIDController(autoP.get(), autoI.get(), autoD.get()),
                new PIDController(autoP.get(), autoI.get(), autoD.get()),
                autoTurnPIDController);
        swerveAutoController.setEnabled(true);
        swerveAutoController.setTolerance(
                new Pose2d(ALLOWED_XY_ERROR_RAMSETE, ALLOWED_XY_ERROR_RAMSETE, Rotation2d.fromDegrees(10))); //TODO: Tune
    }

    /**
     * @return Returns requested drive wheel velocity in Meters per second
     * @return Returns requested drive wheel velocity in Meters per second
     */
    private double getSwerveDriveVelocity(int motorNum) {
        return inputs.driveMotorVelocities[motorNum];
    }

    public synchronized void setDriveState(@NotNull DriveState driveState) {
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

    private static Rotation2d getPredictedRobotAngleInLoopCenter() {
        return Robot.getRobotTracker().getGyroAngleAtTime(Timer.getFPGATimestamp()).toRotation2d()
                .plus(Rotation2d.fromDegrees(
                        Robot.getRobotTracker().getAngularVelocity() * (EXPECTED_TELEOP_DRIVE_DT * 4)));
    }

    PIDController drivePositionPidX = new PIDController(4.1, 0, 0.41);
    PIDController drivePositionPidY = new PIDController(4.1, 0, 0.41);

    {
        SmartDashboard.putData(drivePositionPidX);
        SmartDashboard.putData(drivePositionPidY);
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
                                                ControllerDriveInputs inputs) {

        Translation2d positionError = Robot.getRobotTracker().getLatestPose().getTranslation().minus(targetPosition);
        if (Math.abs(positionError.getX()) < PID_CONTROL_RANGE_AUTO_DRIVE_METERS
                && Math.abs(positionError.getY()) < PID_CONTROL_RANGE_AUTO_DRIVE_METERS) {
            if (Math.abs(positionError.getX()) < 0.02
                    && Math.abs(positionError.getY()) < 0.02) {
                setTurn(new ControllerDriveInputs(),
                        new State(targetAngle.getRadians(), 0),
                        Math.toRadians(1));
            } else {
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
            var robotTracker = Robot.getRobotTracker();
            realtimeTrajectoryStartVelocity = robotTracker.getVelocity();
            trajectoryToDrive = PathGenerator.generateTrajectory(
                    realtimeTrajectoryStartVelocity,
                    robotTracker.getLatestPose().getTranslation(),
                    targetPosition,
                    START_POS_PREDICT_AHEAD);
            realtimeTrajectoryStartTime = Timer.getFPGATimestamp() + START_POS_PREDICT_AHEAD;
            setDriveState(DriveState.WAITING_FOR_PATH);
        }
        if (driveState == DriveState.WAITING_FOR_PATH) {
            assert trajectoryToDrive != null;
            if (trajectoryToDrive.isDone()) {
                if (Timer.getFPGATimestamp() > realtimeTrajectoryStartTime) {
                    // Optional of the trajectory. Empty if the trajectory failed to be generated
                    var trajectory = trajectoryToDrive.join();

                    if (trajectory.isPresent()) {
                        setAutoPath(trajectory.get(), realtimeTrajectoryStartTime); // Sets the DriveState to RAMSETE
                        setAutoRotation(targetAngle);
                        if (Timer.getFPGATimestamp() + EXPECTED_TELEOP_DRIVE_DT > realtimeTrajectoryStartTime) {
                            DriverStation.reportError("Trajectory Generation was late by: "
                                    + (Timer.getFPGATimestamp() - realtimeTrajectoryStartTime) + "s", false);
                        }
                    } else {
                        nextChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                                DRIVE_HIGH_SPEED_M * inputs.getX(),
                                DRIVE_HIGH_SPEED_M * inputs.getY(),
                                inputs.getRotation() * MAX_TELEOP_TURN_SPEED,
                                getPredictedRobotAngleInLoopCenter());
                        kinematicLimit = KinematicLimits.NORMAL_DRIVING.kinematicLimit;
                        return false;
                    }
                }
            } else {
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

    private synchronized void swerveDrive(@NotNull ChassisSpeeds desiredRobotRelativeSpeeds, KinematicLimit kinematicLimit,
                                          double dt) {
        Pose2d robot_pose_vel = new Pose2d(desiredRobotRelativeSpeeds.vxMetersPerSecond * dt,
                desiredRobotRelativeSpeeds.vyMetersPerSecond * dt,
                Rotation2d.fromRadians(desiredRobotRelativeSpeeds.omegaRadiansPerSecond * dt));
        Twist2d twist_vel = IDENTITY_POSE.log(robot_pose_vel);

//        try {
//            SwerveModuleState[] prevStates = (SwerveModuleState[]) m_moduleStates.get(SWERVE_DRIVE_KINEMATICS);
//            for (int i = 0; i < 4; i++) {
//                if (prevStates[i] == null) {
//                    prevStates[i] = new SwerveModuleState();
//                }
//            }
//            m_moduleStates.set(SWERVE_DRIVE_KINEMATICS, prevStates);
//        } catch (IllegalAccessException e) {
//            throw new RuntimeException(e);
//        }


//        kinematicLimit = new KinematicLimit(
//                kinematicLimit.maxDriveVelocity(),
//                Math.min(DRIVE_FEEDFORWARD[0].maxAchievableAcceleration(Robot.getPowerDistribution().getVoltage(),
//                        Robot.getRobotTracker().getVelocity().getNorm()), kinematicLimit.maxDriveAcceleration()),
//                kinematicLimit.maxSteeringVelocity()
//        );
        ChassisSpeeds updated_chassis_speeds = new ChassisSpeeds(
                twist_vel.dx / dt, twist_vel.dy / dt, twist_vel.dtheta / dt);
        var newSwerveSetpoint =
                setpointGenerator.generateSetpoint(kinematicLimit, lastSwerveSetpoint, updated_chassis_speeds, dt);

//        double[] accels = new double[]{0, 0, 0, 0};
//
//
//        var newSwerveSetpoint = new SwerveSetpoint(new ChassisSpeeds(),
//                SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(updated_chassis_speeds), accels);

        synchronized (this) {
            lastSwerveSetpoint = newSwerveSetpoint;
        }

        setSwerveModuleStates(newSwerveSetpoint);
    }

    private synchronized void setSwerveModuleStates(SwerveSetpoint setpoint) {
        Logger.getInstance().recordOutput("Drive/Wanted Swerve Module States", setpoint.moduleStates());
        for (int i = 0; i < 4; i++) {
            var moduleState = setpoint.moduleStates()[i];
            double currentAngle = getWheelRotation(i);

            double angleDiff = getAngleDiff(moduleState.angle.getDegrees(), currentAngle);

            if (Math.abs(angleDiff) > ALLOWED_SWERVE_ANGLE_ERROR) {
                if (USE_CANCODERS) {
                    io.setSwerveMotorPosition(i, inputs.swerveMotorRelativePositions[i] + angleDiff);
                } else {
                    io.setSwerveMotorPosition(i, moduleState.angle.getDegrees());
                }
            } else {
                io.setSwerveMotorPosition(i, inputs.swerveMotorRelativePositions[i]);
            }

            setMotorSpeed(i, moduleState.speedMetersPerSecond, setpoint.wheelAccelerations()[i]);
            //setMotorSpeed(i, 0, 0);

            Logger.getInstance().recordOutput("Drive/SwerveModule " + i + " Wanted Angle", moduleState.angle.getDegrees());
            Logger.getInstance().recordOutput("Drive/SwerveModule " + i + " Wanted Speed", moduleState.speedMetersPerSecond);
            Logger.getInstance().recordOutput("Drive/SwerveModule " + i + " Wanted Acceleration",
                    setpoint.wheelAccelerations()[i]);
            Logger.getInstance().recordOutput("Drive/SwerveModule " + i + " Angle Error", angleDiff);
            Logger.getInstance().recordOutput("Drive/SwerveModule " + i + " Wanted State", moduleState);
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
        // Converts ffv voltage to percent output and sets it to motor
        io.setDriveMotorVoltage(module, ffv);

        Logger.getInstance().recordOutput("Drive/Out Volts Ks" + module, DRIVE_FEEDFORWARD[module].ks * Math.signum(velocity));
        Logger.getInstance().recordOutput("Drive/Out Volts Kv" + module, DRIVE_FEEDFORWARD[module].kv * velocity);
        Logger.getInstance().recordOutput("Drive/Out Volts Ka" + module, DRIVE_FEEDFORWARD[module].ka * acceleration);
        Logger.getInstance().recordOutput("Drive/Voltage Contrib to Accel" + module,
                ffv - DRIVE_FEEDFORWARD[module].calculate(getSwerveDriveVelocity(module)));

        double time = Logger.getInstance().getRealTimestamp() * SECONDS_PER_MICROSECOND;

        Logger.getInstance().recordOutput("Drive/Acceleration" + module,
                (lastModuleVelocities[module] - getSwerveDriveVelocity(module)) / (time - lastModuleTimes[module]));

        lastModuleVelocities[module] = getSwerveDriveVelocity(module);
        lastModuleTimes[module] = Logger.getInstance().getRealTimestamp() * SECONDS_PER_MICROSECOND;

        Logger.getInstance().recordOutput("Drive/Out Volts " + module, ffv);
        //swerveDriveMotors[module].setVoltage(10 * velocity/Constants.SWERVE_METER_PER_ROTATION);
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
                    targetHeading);

            if (Timer.getFPGATimestamp() - autoStartTime < currentAutoTrajectory.getTotalTimeSeconds()) {
                var currVel = new Translation2d(
                        goal.velocityMetersPerSecond * goal.poseMeters.getRotation().getCos(),
                        goal.velocityMetersPerSecond * goal.poseMeters.getRotation().getSin()
                );

                var prevVel = new Translation2d(
                        oldGoal.velocityMetersPerSecond * oldGoal.poseMeters.getRotation().getCos(),
                        oldGoal.velocityMetersPerSecond * oldGoal.poseMeters.getRotation().getSin()
                );

                var accel = currVel.minus(prevVel).div(EXPECTED_TELEOP_DRIVE_DT);
                var extraSpeed = accel.times(driveKa.get()).div(DRIVE_FEEDFORWARD[0].kv);
                var extraSpeedRobotRelative = extraSpeed.rotateBy(currentPose.getRotation());

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

    @Override
    public synchronized void update() {
        var lastTimeStep = inputs.driveIoTimestamp;
        io.updateInputs(inputs);
        Logger.getInstance().processInputs("Drive", inputs);

        switch (driveState) {
            case TURN, WAITING_FOR_PATH -> updateTurn();
            case HOLD -> setSwerveModuleStates(Constants.HOLD_MODULE_STATES);
            case STOP -> {
                nextChassisSpeeds = new ChassisSpeeds();
                kinematicLimit = KinematicLimits.NORMAL_DRIVING.kinematicLimit;
            }
            case RAMSETE -> updateRamsete();
            case AUTO_BALANCE -> autoBalance(ControllerDriveInputs.ZERO);
        }
        if (driveState != DriveState.HOLD && !DriverStation.isTest() && DriverStation.isEnabled()) {
            var dt = inputs.driveIoTimestamp - lastTimeStep;
            swerveDrive(nextChassisSpeeds, kinematicLimit, dt);
        }
    }

    /**
     * For auto use
     */
    public synchronized void setRotation(double angle, double allowedError) {
        setTurn(new ControllerDriveInputs(), new State(Math.toRadians(angle), 0), allowedError);
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
                Robot.getRobotTracker().getGyroAngle());


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

    synchronized public boolean isFinished() {
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
        if (useRelativeEncoderPosition) {
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
        setRotation(degrees, allowedError);
        while (!isFinished()) {
            //noinspection BusyWait
            Thread.sleep(20);
        }
    }

    public synchronized void autoBalance(@NotNull ControllerDriveInputs inputs) {
        var angle = Robot.getRobotTracker().getGyroYAngle();
        var angleMeasure = Math.toDegrees(angle);
        double angularVelocity = Robot.getRobotTracker().getAngularRollVelocity();

        Logger.getInstance().recordOutput("Drive/Auto Balance Angle", angleMeasure);
        Logger.getInstance().recordOutput("Drive/Auto Balance Angle Velocity", angularVelocity);


        double xVelocity;

        if (angleMeasure <= AUTO_BALANCE_COMPLETE_THRESHOLD && angleMeasure >= -AUTO_BALANCE_COMPLETE_THRESHOLD) {
            // Stops PID if within this range
            xVelocity = 0;
        } else if (Math.abs(angularVelocity) > Constants.ANGULAR_ACCELERATION_BALANCE_THRESHHOLD) {
            // Run backwards a little PID if velocity is too high
            xVelocity = Math.copySign(Constants.BALANCE_REVERSE_SPEED, -angleMeasure);
        } else {
            xVelocity = Math.copySign(balancePID.calculate(angleMeasure), angleMeasure);
        }

        Logger.getInstance().recordOutput("Drive/Auto Balance Velocity", xVelocity);


        nextChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                xVelocity,
                DRIVE_HIGH_SPEED_M * inputs.getY(),
                inputs.getRotation() * MAX_TELEOP_TURN_SPEED,
                Robot.getRobotTracker().getGyroAngle());
    }

    public void autoBalance() throws InterruptedException {
        setDriveState(DriveState.AUTO_BALANCE);
        while (DriverStation.isAutonomous()) {
            Thread.sleep(10);
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
