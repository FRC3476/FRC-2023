// Copyright 2019 FRC Team 3476 Code Orange

package frc.subsytem;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.utility.ControllerDriveInputs;
import frc.utility.net.editing.LiveEditableValue;
import frc.utility.swerve.SwerveSetpointGenerator;
import frc.utility.swerve.SwerveSetpointGenerator.KinematicLimit;
import frc.utility.swerve.SwerveSetpointGenerator.SwerveSetpoint;
import frc.utility.wpimodified.HolonomicDriveController;
import frc.utility.wpimodified.PIDController;
import org.jetbrains.annotations.Contract;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

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


    public void resetAuto() {
        swerveAutoControllerLock.lock();
        try {
            ProfiledPIDController autoTurnPIDController
                    = new ProfiledPIDController(turnP.get(), turnI.get(), turnD.get(),
                    new TrapezoidProfile.Constraints(999999999, 999999999));
            autoTurnPIDController.enableContinuousInput(-Math.PI, Math.PI);
            autoTurnPIDController.setTolerance(Math.toRadians(10));

            swerveAutoController = new HolonomicDriveController(
                    new PIDController(3, 0, 0),
                    new PIDController(3, 0, 0),
                    autoTurnPIDController);
            swerveAutoController.setTolerance(new Pose2d(0.5, 0.5, Rotation2d.fromDegrees(10))); //TODO: Tune
        } finally {
            swerveAutoControllerLock.unlock();
        }
    }

    public enum DriveState {
        TELEOP, TURN, HOLD, DONE, RAMSETE, STOP
    }

    private boolean useRelativeEncoderPosition = false;

    private static final @NotNull Drive INSTANCE = new Drive();

    public static @NotNull Drive getInstance() {
        return INSTANCE;
    }

    private final @NotNull PIDController turnPID;

    {
        turnPID = new PIDController(turnP.get(), turnI.get(), turnD.get());
        turnPID.enableContinuousInput(-Math.PI, Math.PI);
        turnPID.setIntegratorRange(-Math.PI * 2 * 4, Math.PI * 2 * 4);
    }

    private @NotNull DriveState driveState = DriveState.TELEOP;
    private volatile @NotNull Rotation2d wantedHeading = new Rotation2d();

    private boolean isAiming = false;

    /**
     * Motors that turn the wheels around. Uses Falcon500s
     */
    private final @NotNull CANSparkMax[] swerveMotors = new CANSparkMax[4];

    /**
     * Motors that are driving the robot around and causing it to move
     */
    private final @NotNull CANSparkMax[] swerveDriveMotors = new CANSparkMax[4];

    /**
     * Absolute Encoders for the motors that turn the wheel
     */

    private final @NotNull CANCoder[] swerveCanCoders = new CANCoder[4];

    private Drive() {
        super(Constants.DRIVE_PERIOD, 5);

        final @NotNull CANSparkMax leftFrontTalon, leftBackTalon, rightFrontTalon, rightBackTalon;
        final @NotNull CANCoder leftFrontCanCoder, leftBackCanCoder, rightFrontCanCoder, rightBackCanCoder;
        final @NotNull CANSparkMax leftFrontTalonSwerve, leftBackTalonSwerve, rightFrontTalonSwerve, rightBackTalonSwerve;
        // Swerve Drive Motors
        leftFrontTalon = new CANSparkMax(Constants.DRIVE_LEFT_FRONT_ID, MotorType.kBrushless);
        leftBackTalon = new CANSparkMax(Constants.DRIVE_LEFT_BACK_ID, MotorType.kBrushless);
        rightFrontTalon = new CANSparkMax(Constants.DRIVE_RIGHT_FRONT_ID, MotorType.kBrushless);
        rightBackTalon = new CANSparkMax(Constants.DRIVE_RIGHT_BACK_ID, MotorType.kBrushless);

        leftFrontTalon.setInverted(false);
        rightFrontTalon.setInverted(false);
        leftBackTalon.setInverted(false);
        rightBackTalon.setInverted(false);

        leftFrontTalonSwerve = new CANSparkMax(Constants.DRIVE_LEFT_FRONT_SWERVE_ID, MotorType.kBrushless);
        leftBackTalonSwerve = new CANSparkMax(Constants.DRIVE_LEFT_BACK_SWERVE_ID, MotorType.kBrushless);
        rightFrontTalonSwerve = new CANSparkMax(Constants.DRIVE_RIGHT_FRONT_SWERVE_ID, MotorType.kBrushless);
        rightBackTalonSwerve = new CANSparkMax(Constants.DRIVE_RIGHT_BACK_SWERVE_ID, MotorType.kBrushless);

        leftFrontCanCoder = new CANCoder(Constants.CAN_LEFT_FRONT_ID);
        leftBackCanCoder = new CANCoder(Constants.CAN_LEFT_BACK_ID);
        rightFrontCanCoder = new CANCoder(Constants.CAN_RIGHT_FRONT_ID);
        rightBackCanCoder = new CANCoder(Constants.CAN_RIGHT_BACK_ID);

        swerveMotors[0] = leftFrontTalonSwerve;
        swerveMotors[1] = leftBackTalonSwerve;
        swerveMotors[2] = rightFrontTalonSwerve;
        swerveMotors[3] = rightBackTalonSwerve;

        swerveDriveMotors[0] = leftFrontTalon;
        swerveDriveMotors[1] = leftBackTalon;
        swerveDriveMotors[2] = rightFrontTalon;
        swerveDriveMotors[3] = rightBackTalon;

        swerveCanCoders[0] = leftFrontCanCoder;
        swerveCanCoders[1] = leftBackCanCoder;
        swerveCanCoders[2] = rightFrontCanCoder;
        swerveCanCoders[3] = rightBackCanCoder;

        for (int i = 0; i < 4; i++) {
            // Sets swerveMotors PID
            swerveMotors[i].getPIDController().setP(Constants.SWERVE_DRIVE_P);
            swerveMotors[i].getPIDController().setD(Constants.SWERVE_DRIVE_D);
            swerveMotors[i].getPIDController().setI(Constants.SWERVE_DRIVE_I);
            swerveMotors[i].getPIDController().setFF(Constants.SWERVE_DRIVE_F);
            swerveMotors[i].getPIDController().setIZone(Constants.SWERVE_DRIVE_INTEGRAL_ZONE);

            // Sets current limits for motors
            swerveMotors[i].setSmartCurrentLimit(SWERVE_MOTOR_CURRENT_LIMIT);
            swerveMotors[i].enableVoltageCompensation(Constants.SWERVE_DRIVE_VOLTAGE_LIMIT);

            swerveDriveMotors[i].setSmartCurrentLimit(SWERVE_DRIVE_MOTOR_CURRENT_LIMIT);
            swerveDriveMotors[i].enableVoltageCompensation(Constants.SWERVE_DRIVE_VOLTAGE_LIMIT);

            // This makes motors brake when no RPM is set
            swerveDriveMotors[i].setIdleMode(IdleMode.kCoast);
            swerveMotors[i].setIdleMode(IdleMode.kCoast);
            swerveMotors[i].setInverted(true);

            swerveCanCoders[i].setStatusFramePeriod(CANCoderStatusFrame.VbatAndFaults, 200);
            swerveCanCoders[i].setStatusFramePeriod(CANCoderStatusFrame.SensorData, 20);
        }
        setDriveState(DriveState.TELEOP);
    }

    public void configCoast() {
        for (CANSparkMax swerveMotor : swerveMotors) {
            swerveMotor.setIdleMode(IdleMode.kCoast);
        }

        for (CANSparkMax swerveDriveMotor : swerveDriveMotors) {
            swerveDriveMotor.setIdleMode(IdleMode.kCoast);
        }
    }

    public void configBrake() {
        for (CANSparkMax swerveMotor : swerveMotors) {
            swerveMotor.setIdleMode(IdleMode.kBrake);
        }

        for (CANSparkMax swerveDriveMotor : swerveDriveMotors) {
            swerveDriveMotor.setIdleMode(IdleMode.kBrake);
        }
    }

    /**
     * @return the relative position of the selected swerve motor in degrees
     */
    private double getRelativeSwervePosition(int motorNum) {
        return swerveMotors[motorNum].getEncoder().getPosition() *
                Constants.SWERVE_MOTOR_POSITION_CONVERSION_FACTOR * 360;
    }

    /**
     * Set the target position of the selected swerve motor
     *
     * @param motorNum the selected swerve motor
     * @param position the target position in degrees (0-360)
     */
    private void setSwerveMotorPosition(int motorNum, double position) {
        swerveMotors[motorNum].getEncoder().setPosition((position / Constants.SWERVE_MOTOR_POSITION_CONVERSION_FACTOR) / 360);
    }


    /**
     * @return the relative position of the selected swerve motor in degrees
     */
    private double getSwerveDrivePosition(int motorNum) {
        return swerveDriveMotors[motorNum].getEncoder().getPosition()
                * Constants.SWERVE_DRIVE_MOTOR_REDUCTION * SWERVE_METER_PER_ROTATION;
    }

    /**
     * @return Returns requested drive wheel velocity in Meters per second
     */
    private double getSwerveDriveVelocity(int motorNum) {
        return swerveDriveMotors[motorNum].getEncoder().getVelocity()
                * Constants.SWERVE_DRIVE_MOTOR_REDUCTION
                * SWERVE_METER_PER_ROTATION;
    }

    public synchronized void setDriveState(@NotNull DriveState driveState) {
        this.driveState = driveState;
    }

    public void doHold() {
        setSwerveModuleStates(Constants.HOLD_MODULE_STATES);
    }

    public void swerveDrive(@NotNull ControllerDriveInputs inputs) {

        setDriveState(DriveState.TELEOP);


        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
                DRIVE_HIGH_SPEED_M * inputs.getX(),
                DRIVE_HIGH_SPEED_M * inputs.getY(),
                inputs.getRotation() * 7);
        swerveDrive(chassisSpeeds, KinematicLimits.NORMAL_DRIVING.kinematicLimit, EXPECTED_TELEOP_DRIVE_DT);
    }

    public void swerveDriveFieldRelative(@NotNull ControllerDriveInputs inputs) {

        setDriveState(DriveState.TELEOP);


        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                DRIVE_HIGH_SPEED_M * inputs.getX(),
                DRIVE_HIGH_SPEED_M * inputs.getY(),
                inputs.getRotation() * 7,
                RobotTracker.getInstance().getGyroAngle());
        swerveDrive(chassisSpeeds, KinematicLimits.NORMAL_DRIVING.kinematicLimit, EXPECTED_TELEOP_DRIVE_DT);
    }

    private SwerveSetpoint lastSwerveSetpoint = new SwerveSetpoint(
            new ChassisSpeeds(0, 0, 0),
            new SwerveModuleState[]{
                    new SwerveModuleState(0, new Rotation2d(0)),
                    new SwerveModuleState(0, new Rotation2d(0)),
                    new SwerveModuleState(0, new Rotation2d(0)),
                    new SwerveModuleState(0, new Rotation2d(0))
            },
            new double[]{0, 0, 0, 0});

    public void swerveDrive(@NotNull ChassisSpeeds desiredRobotRelativeSpeeds, KinematicLimit kinematicLimit, double dt) {
        Pose2d robot_pose_vel = new Pose2d(desiredRobotRelativeSpeeds.vxMetersPerSecond * dt,
                desiredRobotRelativeSpeeds.vyMetersPerSecond * dt,
                Rotation2d.fromRadians(desiredRobotRelativeSpeeds.omegaRadiansPerSecond * dt));
        Twist2d twist_vel = IDENTITY_POSE.log(robot_pose_vel);
        ChassisSpeeds updated_chassis_speeds = new ChassisSpeeds(
                twist_vel.dx / dt, twist_vel.dy / dt, twist_vel.dtheta / dt);
        var newSwerveSetpoint =
                setpointGenerator.generateSetpoint(kinematicLimit, lastSwerveSetpoint, updated_chassis_speeds, dt);

        synchronized (this) {
            lastSwerveSetpoint = newSwerveSetpoint;
        }

        setSwerveModuleStates(newSwerveSetpoint);
    }


    public synchronized void setSwerveModuleStates(SwerveSetpoint setpoint) {
        for (int i = 0; i < 4; i++) {
            var moduleState = setpoint.moduleStates()[i];
            double currentAngle = getWheelRotation(i);

            double angleDiff = getAngleDiff(moduleState.angle.getDegrees(), currentAngle);

            setSwerveMotorPosition(i, getRelativeSwervePosition(i) + angleDiff);

            setMotorSpeed(i, moduleState.speedMetersPerSecond, setpoint.wheelAccelerations()[i]);

            logData("SwerveModule " + i + " Angle", moduleState.angle.getDegrees());
            logData("SwerveModule " + i + " Speed", moduleState.speedMetersPerSecond);
            logData("SwerveModule " + i + " Acceleration", setpoint.wheelAccelerations()[i]);
            logData("SwerveModule " + i + " Angle Error", angleDiff);
        }
    }

    /**
     * @param targetAngle  The angle we want to be at (0-360)
     * @param currentAngle The current angle of the module (0-360)
     * @return the shortest angle between the two angles
     */
    public double getAngleDiff(double targetAngle, double currentAngle) {
        double angleDiff = targetAngle - currentAngle;
        if (angleDiff > 180) {
            angleDiff -= 360;
        }

        if (angleDiff < -180) {
            angleDiff += 360;
        }
        return angleDiff;
    }

    /**
     * Sets the motor voltage
     *
     * @param module   The module to set the voltage on
     * @param velocity The target velocity
     */
    public void setMotorSpeed(int module, double velocity, double acceleration) {
        if (module < 0 || module > DRIVE_FEEDFORWARD.length) {
            throw new IllegalArgumentException("Module must be between 0 and 3");
        }


        double ffv = DRIVE_FEEDFORWARD[module].calculate(velocity, acceleration);
        // Converts ffv voltage to percent output and sets it to motor
        swerveDriveMotors[module].setVoltage(ffv);
        logData("Out Volts " + module, ffv);
        //swerveDriveMotors[module].setVoltage(10 * velocity/Constants.SWERVE_METER_PER_ROTATION);
    }

    /**
     * Set the robot's rotation when driving a path. This is meant to be used in the auto builder. Code that wants to use this
     * should use {@link Drive#setAutoRotation(Rotation2d)} instead.
     *
     * @param angle The angle to set the robot to
     */
    public void setAutoRotation(double angle) {
        setAutoRotation(Rotation2d.fromDegrees(angle));
    }

    double autoStartTime;

    private final ReentrantLock swerveAutoControllerLock = new ReentrantLock();
    private @Nullable HolonomicDriveController swerveAutoController;
    boolean swerveAutoControllerInitialized = false;

    public void setAutoPath(Trajectory trajectory) {
        currentAutoTrajectoryLock.lock();
        try {
            swerveAutoControllerInitialized = false;
            setDriveState(DriveState.RAMSETE);
            this.currentAutoTrajectory = trajectory;
            autoStartTime = Timer.getFPGATimestamp();
        } finally {
            currentAutoTrajectoryLock.unlock();
        }
    }

    Trajectory currentAutoTrajectory;
    final Lock currentAutoTrajectoryLock = new ReentrantLock();
    volatile Rotation2d autoTargetHeading;

    private double nextAllowedPrintError = 0;

    @SuppressWarnings("ProhibitedExceptionCaught")
    private void updateRamsete() {
        currentAutoTrajectoryLock.lock();
        try {
            if (!swerveAutoControllerInitialized) {
                resetAuto();
                assert currentAutoTrajectory != null;
                swerveAutoControllerInitialized = true;
            }

            Trajectory.State goal = currentAutoTrajectory.sample(Timer.getFPGATimestamp() - autoStartTime);
            Rotation2d targetHeading = autoTargetHeading;

            try {
                if (swerveAutoController == null) {
                    DriverStation.reportError("swerveAutoController is null",
                            Thread.getAllStackTraces().get(Thread.currentThread()));
                    resetAuto();
                }
                ChassisSpeeds adjustedSpeeds = swerveAutoController.calculate(
                        RobotTracker.getInstance().getLatestPose(),
                        goal,
                        targetHeading);

                swerveDrive(adjustedSpeeds, KinematicLimits.NORMAL_DRIVING.kinematicLimit, EXPECTED_AUTO_DRIVE_DT);
                if (swerveAutoController.atReference()
                        && (Timer.getFPGATimestamp() - autoStartTime) >= currentAutoTrajectory.getTotalTimeSeconds()) {
                    setDriveState(DriveState.DONE);
                    stopMovement();
                }
            } catch (NullPointerException exception) {
                if (Timer.getFPGATimestamp() > nextAllowedPrintError) {
                    exception.printStackTrace();
                    nextAllowedPrintError = Timer.getFPGATimestamp() + 2;
                }
            }
        } finally {
            currentAutoTrajectoryLock.unlock();
        }
    }

    /**
     * @param rotationGoal The goal to aim at (in radians)
     * @return The speed to turn at (in radians/s)
     */
    private double getTurnPidDeltaSpeed(@NotNull TrapezoidProfile.State rotationGoal, boolean limitSpeed) {
        turnPID.setSetpoint(rotationGoal.position);

        if (Timer.getFPGATimestamp() - 0.2 > lastTurnUpdate) {
            turnPID.reset();
        } else if (turnPID.getPositionError() > Math.toRadians(7)) {
            // This is basically an I-Zone
            turnPID.resetI();
        }
        lastTurnUpdate = Timer.getFPGATimestamp();
        double pidSpeed =
                turnPID.calculate(RobotTracker.getInstance().getGyroAngle().getRadians()) + rotationGoal.velocity;

        if (limitSpeed) {
            pidSpeed = Math.copySign(Math.min(Math.abs(pidSpeed), Constants.TURN_SPEED_LIMIT_WHILE_AIMING), pidSpeed);
        }
        return pidSpeed;
    }


    public void setAutoRotation(@NotNull Rotation2d rotation) {
        currentAutoTrajectoryLock.lock();
        try {
            autoTargetHeading = rotation;
        } finally {
            currentAutoTrajectoryLock.unlock();
        }
        System.out.println("New Auto Rotation" + rotation.getDegrees());
    }

    public double getAutoElapsedTime() {
        return Timer.getFPGATimestamp() - autoStartTime;
    }

    @Override
    public void update() {
        DriveState snapDriveState;
        synchronized (this) {
            snapDriveState = driveState;
        }

        switch (snapDriveState) {
            case TURN -> updateTurn();
            case HOLD -> doHold();
            case STOP -> swerveDrive(
                    new ChassisSpeeds(0, 0, 0),
                    KinematicLimits.NORMAL_DRIVING.kinematicLimit, EXPECTED_AUTO_DRIVE_DT
            );
            case RAMSETE -> updateRamsete();
        }
    }

    synchronized public boolean isAiming() {
        return isAiming;
    }

    public void setRotation(Rotation2d angle) {
        wantedHeading = angle;
        driveState = DriveState.TURN;
        isAiming = !isTurningDone();
    }

    public void setRotation(double angle) {
        setRotation(Rotation2d.fromDegrees(angle));
    }


    public boolean isTurningDone() {
        Rotation2d currentHeading = RobotTracker.getInstance().getGyroAngle();
        synchronized (this) {
            double error = wantedHeading.minus(currentHeading).getDegrees();
            return (Math.abs(error) < Constants.MAX_TURN_ERROR);
        }
    }

    /**
     * Default method when the x and y velocity and the target heading are not passed
     */
    private void updateTurn() {
        updateTurn(new ControllerDriveInputs(0, 0, 0), wantedHeading, Math.toRadians(Constants.MAX_TURN_ERROR));
    }

    double lastTurnUpdate = 0;

    /**
     * This method takes in x and y velocity as well as the target heading to calculate how much the robot needs to turn in order
     * to face a target
     * <p>
     * xVelocity and yVelocity are in m/s
     *
     * @param controllerDriveInputs The x and y velocity of the robot (rotation is ignored)
     * @param targetHeading         The target heading the robot should face
     */
    public void updateTurn(ControllerDriveInputs controllerDriveInputs, @NotNull Rotation2d targetHeading,
                           double turnErrorRadians) {
        updateTurn(controllerDriveInputs, new State(targetHeading.getRadians(), 0), turnErrorRadians);
    }

    /**
     * This method takes in x and y velocity as well as the target heading to calculate how much the robot needs to turn in order
     * to face a target
     * <p>
     * xVelocity and yVelocity are in m/s
     *
     * @param controllerDriveInputs The x and y velocity of the robot (rotation is ignored)
     * @param goal                  The target state at the end of the turn (in radians, radians/s)
     */
    public void updateTurn(ControllerDriveInputs controllerDriveInputs, State goal, double turnErrorRadians) {
        synchronized (this) {
            if (driveState != DriveState.TURN) setDriveState(DriveState.TELEOP);
        }


        double pidDeltaSpeed = getTurnPidDeltaSpeed(goal,
                !(controllerDriveInputs.getX() == 0 && controllerDriveInputs.getY() == 0));

//        System.out.println(
//                "turn error: " + Math.toDegrees(turnPID.getPositionError()) + " delta speed: " + Math.toDegrees(pidDeltaSpeed));


        swerveDrive(ChassisSpeeds.fromFieldRelativeSpeeds(
                        controllerDriveInputs.getX() * DRIVE_HIGH_SPEED_M * 0.45,
                        controllerDriveInputs.getY() * DRIVE_HIGH_SPEED_M * 0.45,
                        pidDeltaSpeed,
                        RobotTracker.getInstance().getGyroAngle()),
                KinematicLimits.NORMAL_DRIVING.kinematicLimit,
                EXPECTED_TELEOP_DRIVE_DT);

        if (Math.abs(goal.position - RobotTracker.getInstance().getGyroAngle().getRadians()) < turnErrorRadians) {
            synchronized (this) {
                isAiming = false;
                if (this.driveState == DriveState.TURN) {
                    this.driveState = DriveState.DONE;
                }
            }
        } else {
            synchronized (this) {
                isAiming = true;
            }
        }

        double curSpeed = RobotTracker.getInstance().getAngularVelocity();
        logData("Turn Position Error", Math.toDegrees(turnPID.getPositionError()));
        logData("Turn Actual Speed", curSpeed);
        logData("Turn PID Command", pidDeltaSpeed);
        logData("Turn PID Setpoint Position", goal.position);
        logData("Turn PID Setpoint Velocity", goal.velocity);
        logData("Turn PID Measurement", RobotTracker.getInstance().getGyroAngle().getRadians());
    }

    public void stopMovement() {
        setDriveState(DriveState.STOP);
    }

    synchronized public boolean isFinished() {
        return driveState == DriveState.STOP || driveState == DriveState.DONE || driveState == DriveState.TELEOP;
    }

    @Override
    public void selfTest() {

    }

    @Override
    public void logData() {
        for (int i = 0; i < 4; i++) {
            double relPos = getRelativeSwervePosition(i) % 360;
            if (relPos < 0) relPos += 360;
            logData("Swerve Motor " + i + " Relative Position", relPos);
            logData("Swerve Motor " + i + " Absolute Position", getWheelRotation(i));
            logData("Drive Motor " + i + " Velocity", getSwerveDriveVelocity(i) / 60.0d);
            logData("Drive Motor " + i + " Current", swerveDriveMotors[i].getOutputCurrent());
            logData("Swerve Motor " + i + " Current", swerveMotors[i].getOutputCurrent());
            logData("Swerve Motor " + i + " Temp", swerveMotors[i].getMotorTemperature());
            logData("Drive Motor " + i + " Temp", swerveDriveMotors[i].getMotorTemperature());
        }
        logData("Drive State", driveState.toString());
    }


    /**
     * Returns the angle/position of the requested encoder module
     *
     * @param moduleNumber the module to get the angle of
     * @return angle in degrees of the module (0-360)
     */
    public double getWheelRotation(int moduleNumber) {
        if (useRelativeEncoderPosition) {
            double relPos = getRelativeSwervePosition(moduleNumber) % 360;
            if (relPos < 0) relPos += 360;
            return relPos;
        } else {
            return swerveCanCoders[moduleNumber].getAbsolutePosition();
        }
    }

    /**
     * Returns how far the encoder has measured the wheel has turned since the last reset
     *
     * @param moduleNumber the module to get the position of
     * @return distance in meters
     */
    public double getDrivePosition(int moduleNumber) {
        return swerveMotors[moduleNumber].getEncoder().getPosition()
                * SWERVE_METER_PER_ROTATION;
    }

    @Contract(pure = true)
    public SwerveModulePosition @NotNull [] getModulePositions() {
        SwerveModulePosition[] swerveModulePositions = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            swerveModulePositions[i] = new SwerveModulePosition(
                    getDrivePosition(i),
                    new Rotation2d(getWheelRotation(i)));
        }
        return swerveModulePositions;
    }

    @Contract(pure = true)
    public SwerveModuleState @NotNull [] getModuleStates() {
        SwerveModuleState[] swerveModuleStates = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            swerveModuleStates[i] = new SwerveModuleState(
                    getSwerveDriveVelocity(i) / 60.0d * SWERVE_METER_PER_ROTATION,
                    new Rotation2d(getWheelRotation(i)));
        }
        return swerveModuleStates;
    }


    public void setAbsoluteZeros() {
        for (int i = 0; i < swerveCanCoders.length; i++) {
            CANCoder swerveCanCoder = swerveCanCoders[i];
            System.out.println(i + " Setting Zero " + swerveCanCoder.configGetMagnetOffset() + " -> 0");
            swerveCanCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
            swerveCanCoder.configMagnetOffset(-(swerveCanCoder.getAbsolutePosition() - swerveCanCoder.configGetMagnetOffset()));
        }
    }

    public void turnToAngle(double degrees) throws InterruptedException {
        System.out.println("Turning to " + degrees);
        setRotation(degrees);
        while (!isTurningDone()) {
            //noinspection BusyWait
            Thread.sleep(20);
        }
    }
}
