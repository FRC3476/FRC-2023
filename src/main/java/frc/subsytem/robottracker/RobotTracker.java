package frc.subsytem.robottracker;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.dacubeking.AutoBuilder.robot.sender.pathpreview.RobotPositionSender;
import com.dacubeking.AutoBuilder.robot.sender.pathpreview.RobotState;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N4;
import frc.robot.Robot;
import frc.subsytem.AbstractSubsystem;
import frc.subsytem.robottracker.GyroInputs.Entry;
import frc.utility.geometry.MutableTranslation3d;
import frc.utility.wpimodified.SwerveDriveOdometry;
import frc.utility.wpimodified.SwerveDrivePoseEstimator;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;
import org.littletonrobotics.junction.Logger;

import java.util.*;
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
    private @NotNull Rotation3d rotation3d = new Rotation3d();

    private double angularRate = 0;


    public static final Matrix<N4, N1> DEFAULT_VISION_DEVIATIONS = VecBuilder.fill(0.03, 0.03, 0.03, Math.toRadians(3));

    private final SwerveDrivePoseEstimator swerveDriveOdometry;

    private final SwerveDriveOdometry noVisionOdometry;

    private static final double VELOCITY_MEASUREMENT_WINDOW = 0.5;

    private final TimeInterpolatableBuffer<Pose3d> poseBufferForVelocity = TimeInterpolatableBuffer.createBuffer(
            Pose3d::interpolate, 1.5);


    /**
     * Acceleration of the robot in field relative coordinates.
     */
    private @NotNull Translation3d acceleration = new Translation3d();

    /**
     * Velocity of the robot in field relative coordinates
     */
    private @NotNull Translation3d velocity = new Translation3d();

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
                gyroInputs.rotation3d,
                Robot.getDrive().getModulePositions()
        );

        swerveDriveOdometry = new SwerveDrivePoseEstimator(
                SWERVE_DRIVE_KINEMATICS,
                gyroInputs.rotation3d,
                Robot.getDrive().getModulePositions(),
                new Pose3d(),
                VecBuilder.fill(0.1, 0.1, 0.05, 0.01),
                DEFAULT_VISION_DEVIATIONS
        );

        new ScheduledThreadPoolExecutor(1).scheduleAtFixedRate(this::updateGyroHistory, 0, 1,
                TimeUnit.MILLISECONDS);
    }


    private record VisionMeasurement(Pose3d pose, double timestamp, Optional<Matrix<N4, N1>> visionMeasurementStds) {
    }

    private record CompletedVisionMeasurement(Pose3d pose, Matrix<N4, N1> visionMeasurementStds, Pose3d preUpdatePose) {
    }

    private final List<VisionMeasurement> visionMeasurements = Collections.synchronizedList(new ArrayList<>());
    private final TreeMap<Double, List<CompletedVisionMeasurement>> pastVisionMeasurements = new TreeMap<>();

    public void addVisionMeasurement(@NotNull Pose3d visionMeasurement, double timestamp) {
        visionMeasurements.add(new VisionMeasurement(visionMeasurement, timestamp, Optional.empty()));
    }

    public void addVisionMeasurement(@NotNull Pose3d visionMeasurement, double timestamp, Matrix<N4, N1> visionMeasurementStds) {
        visionMeasurements.add(new VisionMeasurement(visionMeasurement, timestamp, Optional.of(visionMeasurementStds)));
    }

    private final GyroInputs gyroInputs = new GyroInputs();


    // Created here to avoid garbage collection
    private final short[] ba_xyz = new short[3];
    private final double[] quaternion = new double[4];

    private @Nullable Rotation3d lastRotation;
    private @Nullable Translation3d lastAcceleration;

    private void updateGyroHistory() {
        double time = Logger.getInstance().getRealTimestamp() * SECONDS_PER_MICROSECOND;

        gyroSensor.get6dQuaternion(quaternion);
        gyroSensor.getBiasedAccelerometer(ba_xyz);

        var rotationFieldToRobot = new Rotation3d(GyroInputs.getQuaternion(quaternion)); // rotation from field to robot (robot
        // frame)

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

    private double maxAllowedPoseError = 0.5;
    private int mismatchedVelocityCount = 0;
    private boolean isVelocityMismatched = false;
    private double velocityMismatchTime = 0;

    {
        Logger.getInstance().recordOutput("Velocity Mismatch Count", mismatchedVelocityCount);
    }

    @Override
    public void update() {
        double timestamp = Robot.getDrive().getIoTimestamp();

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
                acceleration = accelerationHistory.getSample(timestamp).orElse(new Translation3d());
                rotation3d = gyroHistory.getSample(timestamp).orElse(gyroInputs.rotation3d);
                rotation2d = rotation3d.toRotation2d();
            } finally {
                lock.writeLock().unlock();
            }
        }

        SwerveModulePosition[] modulePositions = Robot.getDrive().getModulePositions();
        if (isVelocityMismatched) {
            var oldVelocity = velocity;
            var newVelocity = velocity.plus(getChangeInVelocity(lastTimestamp, timestamp));
            var averageVelocity = oldVelocity.plus(newVelocity).times(0.5);

            swerveDriveOdometry.updateWithTime(timestamp, rotation3d, averageVelocity, lastTimestamp - timestamp);

            velocityMismatchTime += timestamp - lastTimestamp;
        } else {
            swerveDriveOdometry.updateWithTime(timestamp, rotation3d, modulePositions);
        }

        synchronized (visionMeasurements) {
            for (VisionMeasurement visionMeasurement : visionMeasurements) {
                double poseError = visionMeasurement.pose.getTranslation().getDistance(visionMeasurement.pose.getTranslation());
                if (poseError > maxAllowedPoseError) {
                    continue;
                }

                var visionMeasurementStds = visionMeasurement.visionMeasurementStds().orElse(DEFAULT_VISION_DEVIATIONS);

                pastVisionMeasurements.putIfAbsent(visionMeasurement.timestamp(), new ArrayList<>());
                pastVisionMeasurements.get(visionMeasurement.timestamp()).add(
                        new CompletedVisionMeasurement(visionMeasurement.pose, visionMeasurementStds,
                                swerveDriveOdometry.getEstimatedPosition3d())
                );

                swerveDriveOdometry.addVisionMeasurement(visionMeasurement.pose(), visionMeasurement.timestamp(),
                        visionMeasurement.visionMeasurementStds().orElse(DEFAULT_VISION_DEVIATIONS));
            }
            visionMeasurements.clear();
        }

        pastVisionMeasurements.headMap(timestamp - MISMATCH_LOOK_BACK_TIME).clear();


        @Nullable Translation3d velocity = null;

        // Calculate the velocity of the robot
        //noVisionOdometry is always relative to the startup angle of the robot
        noVisionOdometry.update(rotation2d, modulePositions);
        lock.readLock().lock();
        try {
            Rotation3d startUpRotationToField = swerveDriveOdometry.getEstimatedPosition3d().getRotation().minus(rotation3d);
            var velocityOdometryBackedOptional = getVelocityAtTime(timestamp);

            if (isVelocityMismatched) {
                velocity = this.velocity.plus(getChangeInVelocity(lastTimestamp, timestamp));

                if (velocityOdometryBackedOptional.isPresent()) {
                    var velocityError = velocityOdometryBackedOptional.get().minus(velocity);
                    var velocityErrorMagnitude = velocityError.getNorm();
                    Logger.getInstance().recordOutput("Velocity Error", velocityErrorMagnitude);

                    if (velocityErrorMagnitude < MISMATCH_VELOCITY_ERROR_THRESHOLD || velocityMismatchTime > MAX_VELOCITY_MISMATCH_TIME) {
                        isVelocityMismatched = false;
                        velocityMismatchTime = 0;
                    }
                }
            } else {
                if (velocityOdometryBackedOptional.isPresent()) {
                    velocity = velocityOdometryBackedOptional.get().rotateBy(startUpRotationToField);
                }

                var oldVelocityOptional = getVelocityAtTime(timestamp - MISMATCH_LOOK_BACK_TIME);

                if (oldVelocityOptional.isPresent() && velocityOdometryBackedOptional.isPresent()) {
                    var velocityChangeToPresent = getChangeInVelocity(timestamp - MISMATCH_LOOK_BACK_TIME, timestamp);
                    var expectedPresentVelocity = oldVelocityOptional.get().plus(velocityChangeToPresent);
                    var velocityError = expectedPresentVelocity.minus(velocityOdometryBackedOptional.get());
                    var velocityErrorMagnitude = velocityError.getNorm();
                    Logger.getInstance().recordOutput("Velocity Error", velocityErrorMagnitude);

                    if (velocityErrorMagnitude > MISMATCH_VELOCITY_ERROR_THRESHOLD) {
                        mismatchedVelocityCount++;
                        Logger.getInstance().recordOutput("Velocity Mismatch Count", mismatchedVelocityCount);

                        isVelocityMismatched = true;
                        velocityMismatchTime = 0;

                        var lastVisionUpdateToUndo = pastVisionMeasurements.firstEntry();
                        if (lastVisionUpdateToUndo != null) {
                            swerveDriveOdometry.undoVisionMeasurement(lastVisionUpdateToUndo.getValue().get(0).pose(),
                                    lastVisionUpdateToUndo.getKey());
                            // If the key exists, then the array list can't be empty, so it is safe to get the first element
                            // The first element is the list is also the first vision measurement applied at that time, so it is the
                            // one to undo with.
                        }

                        if (swerveDriveOdometry.rollbackOdometry(timestamp - MISMATCH_LOOK_BACK_TIME)) {
                            // Update the current velocity to not use the wheel measurements
                            velocity = oldVelocityOptional.get().plus(velocityChangeToPresent).rotateBy(startUpRotationToField);

                            MutableTranslation3d oldVelocity = new MutableTranslation3d(oldVelocityOptional.get());
                            // The velocity from the getVelocityAtTime method is relative to the robot's startup angle, so we need to
                            // rotate it to the field frame
                            oldVelocity.rotateBy(startUpRotationToField);

                            MutableTranslation3d newVelocity = new MutableTranslation3d();
                            MutableTranslation3d avgVelocity = new MutableTranslation3d();

                            double lastTime = timestamp - MISMATCH_LOOK_BACK_TIME;
                            // Bring the odometry up to the present time
                            for (double time = lastTime + NOMINAL_DT; time < timestamp; time += NOMINAL_DT) {
                                newVelocity.set(getChangeInVelocity(lastTime, time))
                                        // The velocity from the getChangeInVelocity method is relative to the robot's startup angle,
                                        // so we need to rotate it to the field frame
                                        .rotateBy(startUpRotationToField)
                                        .plus(oldVelocity); // Add the old velocity to get the new velocity (already in the field frame)
                                avgVelocity.set(oldVelocity).plus(newVelocity).times(0.5);

                                swerveDriveOdometry.updateWithTime(time, rotation3d, avgVelocity.getTranslation3d(),
                                        time - lastTime);

                                oldVelocity.set(newVelocity);
                                lastTime = time;
                            }

                            // Add the last sample
                            newVelocity.set(velocity).plus(oldVelocity);
                            avgVelocity.set(oldVelocity).plus(newVelocity).times(0.5);
                            swerveDriveOdometry.updateWithTime(timestamp, rotation3d, avgVelocity.getTranslation3d(),
                                    timestamp - lastTime);

                            // Reapply the vision measurements
                            for (var entry : pastVisionMeasurements.entrySet()) {
                                for (var visionMeasurement : entry.getValue()) {
                                    swerveDriveOdometry.addVisionMeasurement(visionMeasurement.pose(), entry.getKey(),
                                            visionMeasurement.visionMeasurementStds());
                                }
                            }
                        }
                    }
                }
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
            Pose3d noVisionPose = noVisionOdometry.getPoseMeters3d();
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
            return velocity.toTranslation2d();
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
        Logger.getInstance().recordOutput("RobotTracker/Is Velocity Mismatched", isVelocityMismatched);
        Logger.getInstance().recordOutput("RobotTracker/Velocity Mismatch Time", velocityMismatchTime);


        RobotPositionSender.addRobotPosition(new RobotState(getLatestPose(), getVelocity().getX(),
                getVelocity().getY(), getAngularVelocity(), lastTimestamp));
    }


    private Translation3d getChangeInVelocity(double startTime, double endTime) {
        // Create mutable objects to avoid creating new objects
        final var mutDeltaVelocity = new MutableTranslation3d();
        final var tempAccel = new MutableTranslation3d();
        final var lastAccel = new MutableTranslation3d();


        final var optionalInitialAccel = accelerationHistory.getSample(startTime);
        optionalInitialAccel.ifPresent(
                translation3d -> lastAccel.set(translation3d.getX(), translation3d.getY(), translation3d.getZ()));

        for (var entry : accelerationHistory.getInternalBuffer().tailMap(startTime, false).entrySet()) {
            double time = entry.getKey();

            var accel = entry.getValue();

            if (time > endTime) { // We've gone past the end time
                accel = accelerationHistory.getSample(endTime).orElse(accel); // Interpolates the accel to the end time
                time = endTime;
            }

            tempAccel.set(accel.getX(), accel.getY(), accel.getZ()).plus(lastAccel).times(0.5).times(time - startTime);
            mutDeltaVelocity.plus(tempAccel);

            lastAccel.set(accel.getX(), accel.getY(), accel.getZ());
            startTime = time;

            if (time >= endTime) {
                break;
            }
        }

        if (startTime < endTime) {
            // Add the last bit of deltaVelocity (assume it's constant from the last sample to now)
            tempAccel.set(lastAccel);

            tempAccel.times(endTime - startTime);
            mutDeltaVelocity.plus(tempAccel);
        }


        // The deltaVelocity over our measurement window
        return mutDeltaVelocity.getTranslation3d();
    }

    /**
     * Gets the velocity of the robot at a given time
     *
     * @param timestamp the time to get the velocity at
     * @return the velocity of the robot at the given time relative to the startup angle of the robot
     */
    private Optional<Translation3d> getVelocityAtTime(double timestamp) {
        var pastPose = poseBufferForVelocity.getSample(timestamp - VELOCITY_MEASUREMENT_WINDOW);
        var currentPose = poseBufferForVelocity.getSample(timestamp);
        if (pastPose.isPresent() && currentPose.isPresent()) {
            var avgVelocity = pastPose.get().getTranslation().minus(currentPose.get().getTranslation());
            var averageVelocity = avgVelocity.div(VELOCITY_MEASUREMENT_WINDOW);

            // The change in velocity over our measurement window
            var deltaVelocity = getChangeInVelocity(timestamp - VELOCITY_MEASUREMENT_WINDOW, timestamp);

            // https://www.desmos.com/calculator/szqs5g5d6i

            return Optional.of(averageVelocity.plus(deltaVelocity.times(0.5)));
        }
        return Optional.empty();
    }
}