package frc.subsytem.robottracker;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.dacubeking.AutoBuilder.robot.sender.pathpreview.RobotPositionSender;
import com.dacubeking.AutoBuilder.robot.sender.pathpreview.RobotState;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
import frc.subsytem.AbstractSubsystem;
import frc.subsytem.robottracker.GyroInputs.Entry;
import frc.utility.geometry.MutableTranslation2d;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Optional;
import java.util.concurrent.ScheduledThreadPoolExecutor;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.locks.ReentrantReadWriteLock;

import static frc.robot.Constants.*;

public final class RobotTracker extends AbstractSubsystem {

    private final ReentrantReadWriteLock lock = new ReentrantReadWriteLock();
    private final @NotNull WPI_Pigeon2 gyroSensor = new WPI_Pigeon2(PIGEON_CAN_ID, "rio");
    private final @NotNull Rotation3d ROTATION_IDENTITY = new Rotation3d();

    /**
     * The pose of the robot at the last time the odometry was updated.
     */
    private @NotNull Pose2d latestPose = new Pose2d();

    private @NotNull Rotation2d gyroOffset = new Rotation2d();

    private @NotNull Rotation2d rotation2d = new Rotation2d();
    private double angularRate = 0;


    private final Matrix<N3, N1> defaultVisionMeasurementStdDevs = VecBuilder.fill(0.9, 0.9, 0.9);

    private final SwerveDrivePoseEstimator swerveDriveOdometry;

    private final SwerveDriveOdometry noVisionOdometry;

    private static final double VELOCITY_MEASUREMENT_WINDOW = 0.5;

    private final TimeInterpolatableBuffer<Pose2d> poseBufferForVelocity = TimeInterpolatableBuffer.createBuffer(
            Pose2d::interpolate, 1.5);


    /**
     * Acceleration of the robot in field relative coordinates.
     */
    private @NotNull Translation3d acceleration = new Translation3d();

    /**
     * Velocity of the robot in field relative coordinates
     */
    private @NotNull Translation2d velocity = new Translation2d();

    private double lastTimestamp = 0;
    private final @NotNull TimeInterpolatableBuffer<Rotation3d> gyroHistory
            = TimeInterpolatableBuffer.createBuffer(Rotation3d::interpolate, 1.5); //abt 1.5 seconds (ms * 300)

    private final @NotNull TimeInterpolatableBuffer<Translation3d> accelerationHistory
            = TimeInterpolatableBuffer.createBuffer(Translation3d::interpolate, 1.5); //abt 1.5 seconds (ms * 300)

    public RobotTracker() {
        super(3);


        gyroInputs.updateInputs(gyroSensor);
        Logger.getInstance().processInputs("Gyro", gyroInputs);
        noVisionOdometry = new SwerveDriveOdometry(
                SWERVE_DRIVE_KINEMATICS,
                gyroInputs.rotation2d,
                Robot.getDrive().getModulePositions()
        );

        swerveDriveOdometry = new SwerveDrivePoseEstimator(
                SWERVE_DRIVE_KINEMATICS,
                gyroInputs.rotation2d,
                Robot.getDrive().getModulePositions(),
                new Pose2d(),
                VecBuilder.fill(0.1, 0.1, 0.1),
                defaultVisionMeasurementStdDevs
        );

        new ScheduledThreadPoolExecutor(1).scheduleAtFixedRate(this::updateGyroHistory, 0, 1,
                TimeUnit.MILLISECONDS);
    }


    private record VisionMeasurement(Pose2d pose, double timestamp, Optional<Matrix<N3, N1>> visionMeasurementStds) {
    }

    private final List<VisionMeasurement> visionMeasurements = Collections.synchronizedList(new ArrayList<>());

    public void addVisionMeasurement(@NotNull Pose2d visionMeasurement, double timestamp) {
        visionMeasurements.add(new VisionMeasurement(visionMeasurement, timestamp, Optional.empty()));
    }

    public void addVisionMeasurement(@NotNull Pose2d visionMeasurement, double timestamp, Matrix<N3, N1> visionMeasurementStds) {
        visionMeasurements.add(new VisionMeasurement(visionMeasurement, timestamp, Optional.of(visionMeasurementStds)));
    }

    private final GyroInputs gyroInputs = new GyroInputs();


    // Created here to avoid garbage collection
    private final short[] ba_xyz = new short[3];
    private final double[] quaternion = new double[4];

    private @Nullable Rotation3d lastRotation;
    private @Nullable Translation3d lastAcceleration;

    private void updateGyroHistory() {
        double time = Timer.getFPGATimestamp();

        gyroSensor.get6dQuaternion(quaternion);
        gyroSensor.getBiasedAccelerometer(ba_xyz);

        var rotW = quaternion[0];
        var rotX = quaternion[1];
        var rotY = quaternion[2];
        var rotZ = quaternion[3];

        // we need to transform the axis:
        // axis that we want <- what it is on the pigeon
        // these follow the right hand rule
        // x <- y
        // y <- -x
        // z <- z

        // axis convention of the pigeon: https://store.ctr-electronics.com/content/user-manual/Pigeon2%20User's%20Guide.pdf#page=20,

        var rotationFieldToRobot = new Rotation3d(new Quaternion(rotW, rotY, -rotX, rotZ));

        //ba_xyz is in fixed point notation (Q2.14) in units of g
        var x = toFloat(ba_xyz[0]) * GRAVITY;
        var y = toFloat(ba_xyz[1]) * GRAVITY;
        var z = toFloat(ba_xyz[2]) * GRAVITY;
        var accel = new Translation3d(y, -x, z) // transform the axis (see above) (robot frame)
                .rotateBy(rotationFieldToRobot.unaryMinus()); // rotate the acceleration to the field frame

        synchronized (gyroInputs) {
            if (lastAcceleration == null || !lastAcceleration.equals(accel)) {
                gyroInputs.accelerations.add(new Entry<>(time, accel));
                lastAcceleration = accel;
            }

            if (lastRotation == null || !lastRotation.equals(rotationFieldToRobot)) {
                gyroInputs.rotations.add(new Entry<>(time, rotationFieldToRobot));
                lastRotation = rotationFieldToRobot;
            }
        }
    }

    @Override
    public void update() {
        try {
            Thread.sleep(100);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        synchronized (gyroInputs) {
            gyroInputs.updateInputs(gyroSensor);
            Logger.getInstance().processInputs("Gyro", gyroInputs);
            lock.writeLock().lock();
            try {
                for (Entry<Translation3d> translation3dEntry : gyroInputs.accelerations) {
                    accelerationHistory.addSample(translation3dEntry.timestamp(), translation3dEntry.value());
                }
                for (Entry<Rotation3d> rotation3dEntry : gyroInputs.rotations) {
                    gyroHistory.addSample(rotation3dEntry.timestamp(), rotation3dEntry.value());
                }
                gyroInputs.accelerations.clear();
                gyroInputs.rotations.clear();

                angularRate = gyroInputs.gyroYawVelocity;
                acceleration = accelerationHistory.getInternalBuffer().lastEntry().getValue();
                rotation2d = gyroInputs.rotation2d;
            } finally {
                lock.writeLock().unlock();
            }
        }

        double timestamp = Timer.getFPGATimestamp();
        SwerveModulePosition[] modulePositions = Robot.getDrive().getModulePositions();
        swerveDriveOdometry.updateWithTime(timestamp, rotation2d, modulePositions);

        // Copy the vision measurements into a local variable so that we don't have to lock the list
        final VisionMeasurement[] visionMeasurementsCopy;
        synchronized (visionMeasurements) {
            visionMeasurementsCopy = visionMeasurements.toArray(new VisionMeasurement[0]);
            visionMeasurements.clear();
        }

        for (VisionMeasurement visionMeasurement : visionMeasurementsCopy) {
            swerveDriveOdometry.addVisionMeasurement(visionMeasurement.pose(), visionMeasurement.timestamp(),
                    visionMeasurement.visionMeasurementStds().orElse(defaultVisionMeasurementStdDevs));
        }

        @Nullable Translation2d velocity = null;

        // Calculate the velocity of the robot
        //noVisionOdometry is always relative to the startup angle of the robot
        noVisionOdometry.update(rotation2d, modulePositions);
        lock.readLock().lock();
        try {
            var pastPose = poseBufferForVelocity.getSample(timestamp - VELOCITY_MEASUREMENT_WINDOW);
            if (pastPose.isPresent()) {
                var currentPose = noVisionOdometry.getPoseMeters();
                var avgVelocity = pastPose.get().getTranslation().minus(currentPose.getTranslation());
                var averageVelocity = avgVelocity.div(VELOCITY_MEASUREMENT_WINDOW);

                // Create mutable objects to avoid creating new objects
                final var mutDeltaVelocity = new MutableTranslation2d();
                final var tempAccel = new MutableTranslation2d();
                final var lastAccel = new MutableTranslation2d();
                double lastTime = timestamp - VELOCITY_MEASUREMENT_WINDOW;


                final var optionalInitialAccel = accelerationHistory.getSample(lastTime);
                optionalInitialAccel.ifPresent(translation3d -> lastAccel.set(translation3d.getX(), translation3d.getY()));

                // Calculate the average deltaVelocity over the last 0.5 seconds
                for (var entry : accelerationHistory.getInternalBuffer().tailMap(lastTime, false).entrySet()) {
                    double time = Math.max(entry.getKey(), timestamp); // Don't go into the future
                    if (time < lastTime) { // if for some reason the times are out of order
                        continue;
                    }
                    var accel = entry.getValue();
                    tempAccel.set(accel.getX(), accel.getY()).plus(lastAccel).times(1.0 / 2.0).times(time - lastTime);
                    mutDeltaVelocity.plus(tempAccel);

                    lastAccel.set(accel.getX(), accel.getY());
                    lastTime = time;
                }

                if (lastTime < timestamp) {
                    // Add the last bit of deltaVelocity (assume it's constant from the last sample to now)
                    tempAccel.set(acceleration.getX(), acceleration.getY());

                    tempAccel.times(timestamp - lastTime);
                    mutDeltaVelocity.plus(tempAccel);
                }


                // The average deltaVelocity over our measurement window
                var deltaVelocity = mutDeltaVelocity.getTranslation2d();

                // https://www.desmos.com/calculator/szqs5g5d6i

                velocity = averageVelocity.plus(deltaVelocity.times(VELOCITY_MEASUREMENT_WINDOW / 2))
                        .rotateBy(swerveDriveOdometry.getEstimatedPosition().getRotation().minus(rotation2d));
            }
        } finally {
            lock.readLock().unlock();
        }

        lock.writeLock().lock();
        try {
            latestPose = swerveDriveOdometry.getEstimatedPosition();
            if (velocity != null) {
                this.velocity = velocity;
            }
            Pose2d noVisionPose = noVisionOdometry.getPoseMeters();
            poseBufferForVelocity.addSample(timestamp, noVisionPose);
            this.lastTimestamp = timestamp;
        } finally {
            lock.writeLock().unlock();
        }
    }

    /**
     * Get the latest pose of the robot. This pose is updated every 20ms and is corrected for drift using vision measurements.
     *
     * @return The latest pose of the robot.
     */
    public @NotNull Pose2d getLatestPose() {
        lock.readLock().lock();
        try {
            return latestPose;
        } finally {
            lock.readLock().unlock();
        }
    }

    /**
     * The (field centric) velocity of the robot at the last time the odometry was updated. (m/s)
     */
    public @NotNull Translation2d getVelocity() {
        lock.readLock().lock();
        try {
            return velocity;
        } finally {
            lock.readLock().unlock();
        }
    }

    /**
     * @return The angular velocity of the robot in radians per second. (CCW is positive)
     */
    public double getAngularVelocity() {
        lock.readLock().lock();
        try {
            return angularRate;
        } finally {
            lock.readLock().unlock();
        }
    }

    public @NotNull Rotation2d getGyroAngle() {
        lock.readLock().lock();
        try {
            return rotation2d.plus(gyroOffset);
        } finally {
            lock.readLock().unlock();
        }
    }

    private Vector<N3> POSITIVE_Z = VecBuilder.fill(0, 0, 1);

    public @NotNull Rotation3d getGyroAngleAtTime(double time) {
        lock.readLock().lock();
        try {
            return gyroHistory.getSample(time).orElse(ROTATION_IDENTITY)
                    .plus(new Rotation3d(POSITIVE_Z, gyroOffset.getRadians()));
        } finally {
            lock.readLock().unlock();
        }
    }


    public @NotNull Translation3d getAcceleration() {
        lock.readLock().lock();
        try {
            return acceleration;
        } finally {
            lock.readLock().unlock();
        }
    }

    public @NotNull Translation2d get2dAcceleration() {
        return getAcceleration().toTranslation2d();
    }

    public void resetPose(@NotNull Pose2d pose) {
        lock.writeLock().lock();
        try {
            swerveDriveOdometry.resetPosition(rotation2d, Robot.getDrive().getModulePositions(), pose);
            gyroOffset = pose.getRotation().minus(rotation2d);
        } finally {
            lock.writeLock().unlock();
        }
    }

    public boolean isOnRedSide() {
        return getLatestPose().getX() < FIELD_WIDTH_METERS / 2;
    }


    /**
     * Coverts data in fixed point notation to a double
     *
     * @param fixedPoint in fixed point notation Q2.14. eg. 16384 = 1G
     * @return the value in Gs
     */
    private double toFloat(short fixedPoint) {
        return fixedPoint / 16384.0;
    }


    @Override
    public void selfTest() {

    }

    @Override
    public void logData() {
        Logger.getInstance().recordOutput("RobotTracker/rotation", getGyroAngle().getDegrees());
        Logger.getInstance().recordOutput("RobotTracker/x", getLatestPose().getTranslation().getX());
        Logger.getInstance().recordOutput("RobotTracker/y", getLatestPose().getTranslation().getY());
        Logger.getInstance().recordOutput("RobotTracker/NoVisionPose", noVisionOdometry.getPoseMeters());
        Logger.getInstance().recordOutput("RobotTracker/VisionPose", getLatestPose());
        Logger.getInstance().recordOutput("RobotTracker/velocityX", getVelocity().getX());
        Logger.getInstance().recordOutput("RobotTracker/velocityY", getVelocity().getY());
        Logger.getInstance().recordOutput("RobotTracker/accelerationX", getAcceleration().getX());
        Logger.getInstance().recordOutput("RobotTracker/accelerationY", getAcceleration().getY());
        Logger.getInstance().recordOutput("RobotTracker/accelerationZ", getAcceleration().getZ());
        Logger.getInstance().recordOutput("RobotTracker/Velocity", getVelocity().getNorm());

        RobotPositionSender.addRobotPosition(new RobotState(getLatestPose(), getVelocity().getX(),
                getVelocity().getY(), getAngularVelocity(), lastTimestamp));
    }
}