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

    {
        gyroSensor.configMountPose(0, 0, 0);
    }

    private final @NotNull Rotation3d ROTATION_IDENTITY = new Rotation3d();

    /**
     * The pose of the robot at the last time the odometry was updated.
     */
    private @NotNull Pose2d latestPose = new Pose2d();

    /**
     * The 3d pose of the robot at the last time the odometry was updated.
     */
    private @NotNull Pose3d latestPose3d = new Pose3d();
    private @NotNull Rotation2d gyroAngle2d = new Rotation2d();
    private @NotNull Rotation3d gyroAngle3d = new Rotation3d();

    /**
     * Angular Rate in degrees per second
     */
    private double angularRate = 0;

    /**
     * Angular Acceleration in degrees per second squared
     */
    private double angularAcceleration = 0;


    public static final Matrix<N4, N1> DEFAULT_VISION_DEVIATIONS = VecBuilder.fill(0.2, 0.2, 0.2, Math.toRadians(15));

    private final SwerveDrivePoseEstimator swerveDriveOdometry;

    private final SwerveDriveOdometry noVisionOdometry;

    private static final double VELOCITY_MEASUREMENT_WINDOW = 0.4;

    private final TimeInterpolatableBuffer<Pose3d> poseBufferForVelocity = TimeInterpolatableBuffer.createBuffer(
            Pose3d::interpolate, 1.5);

    private @NotNull Rotation3d gyroOffset = new Rotation3d();
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
            = TimeInterpolatableBuffer.createBuffer(Rotation3d::interpolate, 1.5);

    private final @NotNull TimeInterpolatableBuffer<Translation3d> accelerationHistory
            = TimeInterpolatableBuffer.createBuffer(Translation3d::interpolate, 1.5);

    private final @NotNull TimeInterpolatableBuffer<Translation3d> velocityHistory
            = TimeInterpolatableBuffer.createBuffer(Translation3d::interpolate, 15);
    private final @NotNull TimeInterpolatableBuffer<Translation3d> velocityOffsetHistory
            = TimeInterpolatableBuffer.createBuffer(Translation3d::interpolate, 15);

    public RobotTracker() {
        super();


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
    private final double[] g_xyz = new double[3];
    private final double[] quaternion = new double[4];

    private final short[] last_ba_xyz = new short[3];
    private final double[] last_g_xyz = new double[3];
    private final double[] last_quaternion = new double[4];
    private @Nullable Rotation3d lastRotation;
    private @Nullable Translation3d lastAcceleration;

    private void updateGyroHistory() {
        double time = Logger.getInstance().getRealTimestamp() * SECONDS_PER_MICROSECOND;

        gyroSensor.get6dQuaternion(quaternion);
        gyroSensor.getBiasedAccelerometer(ba_xyz);
        gyroSensor.getGravityVector(g_xyz);

        if (Arrays.equals(quaternion, last_quaternion) && Arrays.equals(ba_xyz, last_ba_xyz)
                && Arrays.equals(g_xyz, last_g_xyz)) {
            // Exit early if nothing has changed (don't create new objects)
            return;
        }

        System.arraycopy(quaternion, 0, last_quaternion, 0, 4);
        System.arraycopy(ba_xyz, 0, last_ba_xyz, 0, 3);
        System.arraycopy(g_xyz, 0, last_g_xyz, 0, 3);


        var rotationFieldToRobot = new Rotation3d(GyroInputs.getQuaternion(quaternion)); // rotation from field to robot (robot
        // frame)


        // we need to transform the axis:
        // axis that we want <- what it is on the pigeon
        // these follow the right hand rule
        // x <- y
        // y <- -x
        // z <- z

        // axis convention of the pigeon: https://store.ctr-electronics.com/content/user-manual/Pigeon2%20User's%20Guide.pdf#page=20

        var g_x = g_xyz[0];
        var g_y = g_xyz[1];
        var g_z = g_xyz[2];

        var gravity = new Translation3d(g_y, -g_x, g_z); // transform the axis (see above) (robot frame)
        gravity = gravity.times(GRAVITY / gravity.getNorm());

        //ba_xyz is in fixed point notation (Q2.14) in units of g
        var x = toFloat(ba_xyz[0]) * GRAVITY;
        var y = toFloat(ba_xyz[1]) * GRAVITY;
        var z = toFloat(ba_xyz[2]) * GRAVITY;
        var accel = new Translation3d(y, -x, z) // transform the axis (see above) (robot frame)
                .minus(gravity)
                // TODO: Subtract out acceleration due to rotation
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

    private Translation3d velocityOffset = new Translation3d();
    private Translation3d accumulatedVelocityError = new Translation3d();

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
                // Add the gyro measurements from the last iteration to the history
                for (Entry<Rotation3d> rotation3dEntry : gyroInputs.rotations) {
                    gyroHistory.addSample(rotation3dEntry.timestamp(), rotation3dEntry.value());
                }
                for (Entry<Translation3d> translation3dEntry : gyroInputs.accelerations) {
                    accelerationHistory.addSample(translation3dEntry.timestamp(), translation3dEntry.value());
                }

                // Finds angular acceleration using current and last gyro velocities
                angularAcceleration = (gyroInputs.gyroYawVelocity - angularRate) / (timestamp - lastTimestamp);
                angularRate = gyroInputs.gyroYawVelocity;


                // Get the gyro measurements for the timestamp of our swerve module positions
                acceleration = accelerationHistory.getSample(timestamp).orElse(new Translation3d());
                gyroAngle3d = gyroHistory.getSample(timestamp).orElse(gyroInputs.rotation3d);
                gyroAngle2d = gyroAngle3d.toRotation2d();

                // Integrate the acceleration to get the velocity for the velocity history
                var lastVelocityEntry = velocityHistory.getInternalBuffer().lastEntry();
                for (Entry<Translation3d> velocityEntry : gyroInputs.accelerations) {
                    double accelerationTime = velocityEntry.timestamp();
                    if (lastVelocityEntry == null) {
                        // If there is no last velocity, then assume the velocity is 0
                        // (It doesn't matter if this is correct as the velocity will be corrected by the offset)
                        velocityHistory.addSample(accelerationTime, new Translation3d());
                    } else {
                        var lastVelocity = lastVelocityEntry.getValue();
                        var changeInVelocity = getChangeInVelocity(lastVelocityEntry.getKey(), accelerationTime);
                        var newVelocity = lastVelocity.plus(changeInVelocity);
                        velocityHistory.addSample(accelerationTime, newVelocity);
                        lastVelocityEntry = velocityHistory.getInternalBuffer().lastEntry();
                    }
                }

                // Clear the gyro histories
                gyroInputs.accelerations.clear();
                gyroInputs.rotations.clear();
            } finally {
                lock.writeLock().unlock();
            }
        }

        // The rotation transformation from the frame of all our gyro measurements (which are relative to the startup angle) to
        // the field frame
        Rotation3d startUpRotationToField = swerveDriveOdometry.getEstimatedPosition3d().getRotation().minus(gyroAngle3d);

        SwerveModulePosition[] modulePositions = Robot.getDrive().getModulePositions();

        @Nullable Translation3d velocity;

        // Calculate the velocity of the robot
        //noVisionOdometry is always relative to the startup angle of the robot
        noVisionOdometry.update(gyroAngle3d, modulePositions);


        lock.readLock().lock();
        try {
            // Get the velocity of the robot from the odometry (first derivative)
            var averageVelocity = getVelocityAverageVelocityInPeriodFromOdometry(
                    timestamp - VELOCITY_MEASUREMENT_WINDOW, timestamp)
                    .orElse(new Translation3d())
                    .rotateBy(startUpRotationToField);

            // Get the velocity of the robot from the acceleration (first integration)
            var averageVelocityAcceleration = getVelocityAverageVelocityInPeriodFromAcceleration(
                    timestamp - VELOCITY_MEASUREMENT_WINDOW, timestamp)
                    .rotateBy(startUpRotationToField)
                    .plus(velocityOffset);


            // Difference between the integrated velocity from the accelerations and the velocity from the odometry
            var velocityError = averageVelocity.minus(averageVelocityAcceleration);
            var velocityErrorMagnitude = velocityError.getNorm();

            Logger.getInstance().recordOutput("RobotTracker/Average Velocity", averageVelocity.getNorm());
            Logger.getInstance().recordOutput("RobotTracker/Average Velocity X", averageVelocity.getX());
            Logger.getInstance().recordOutput("RobotTracker/Average Velocity Y", averageVelocity.getY());
            Logger.getInstance().recordOutput("RobotTracker/Average Velocity Z", averageVelocity.getZ());

            Logger.getInstance().recordOutput("RobotTracker/Velocity Error", velocityErrorMagnitude);
            Logger.getInstance().recordOutput("RobotTracker/Velocity Error X", velocityError.getX());
            Logger.getInstance().recordOutput("RobotTracker/Velocity Error Y", velocityError.getY());
            Logger.getInstance().recordOutput("RobotTracker/Velocity Error Z", velocityError.getZ());

            if (isVelocityMismatched && false) {
                // We're in mismatched mode, so we shouldn't use the odometry velocity
                if (velocityErrorMagnitude < MISMATCH_VELOCITY_ERROR_THRESHOLD || velocityMismatchTime > MAX_VELOCITY_MISMATCH_TIME) {
                    // We no longer meet the mismatched criteria, so we should switch back to using the odometry velocity
                    isVelocityMismatched = false;
                    if (velocityMismatchTime > MAX_VELOCITY_MISMATCH_TIME) {
                        // we have been mismatched for too long, so we should reset the velocity offset
                        // b/c we've probably drifted too far
                        velocityOffset = velocityOffset.plus(velocityError);
                        velocityOffsetHistory.addSample(timestamp, velocityOffset);
                    }
                    velocityMismatchTime = 0;
                    // Reset accumulated error, so we don't immediately switch back to mismatched mode
                    accumulatedVelocityError = accumulatedVelocityError.times(0);
                }
            } else {
                accumulatedVelocityError = accumulatedVelocityError.times(ACCUMULATED_ERROR_DECAY); // Decay accumulated error
                accumulatedVelocityError = accumulatedVelocityError.plus(velocityError); // Add new error

                var accumulatedVelocityErrorMagnitude = accumulatedVelocityError.getNorm();

                velocityOffset = velocityOffset.plus(velocityError.times(1)); // Add new error to offset
                // (theoretically, we would want to scale this by some factor, less than 1, but the accelerometer isn't centered
                // around the rotation axis, so we get a lot of error that accumulates, when we are rotating if we do that, so we
                // just add the error to the offset. By scaling it by 1, we're essentially doing the same thing we were trying
                // to do earlier with velocity latency compensation, but it's actually implemented correctly this time.)
                velocityOffsetHistory.addSample(timestamp, velocityOffset);

                // Old velocity from that we'd use in the case of a mismatch (from integration of acceleration)
                var oldVelocityOptional = velocityHistory.getSample(timestamp - MISMATCH_LOOK_BACK_TIME);

                // Old velocity offset from that we'd use in the case of a mismatch (from integration of acceleration)
                @Nullable var oldVelocityOffset =
                        velocityOffsetHistory.getInternalBuffer().lowerEntry(timestamp - MISMATCH_LOOK_BACK_TIME);

                if (oldVelocityOptional.isPresent() &&
                        oldVelocityOffset != null && accumulatedVelocityErrorMagnitude > MISMATCH_ACCUMULATED_VELOCITY_ERROR_THRESHOLD) {
                    mismatchedVelocityCount++;
                    Logger.getInstance().recordOutput("RobotTracker/Velocity Mismatch Count", mismatchedVelocityCount);

                    isVelocityMismatched = true;
                    velocityMismatchTime = 0;

                    // Clear velocity updates from the odometry in the past MISMATCH_LOOK_BACK_TIME
                    velocityOffset = oldVelocityOffset.getValue();
                    velocityOffsetHistory.getInternalBuffer().tailMap(timestamp - MISMATCH_LOOK_BACK_TIME, false).clear();

                    // Undo the vision measurements that were applied during the mismatch
                    var lastVisionUpdateToUndo = pastVisionMeasurements.firstEntry();
                    if (lastVisionUpdateToUndo != null) {
                        swerveDriveOdometry.undoVisionMeasurement(lastVisionUpdateToUndo.getValue().get(0).pose(),
                                lastVisionUpdateToUndo.getKey());
                        // If the key exists, then the array list can't be empty, so it is safe to get the first element
                        // The first element is the list is also the first vision measurement applied at that time, so it is the
                        // one to undo with.
                    }

                    if (swerveDriveOdometry.rollbackOdometry(timestamp - MISMATCH_LOOK_BACK_TIME)) {
                        // We successfully rolled back the odometry, so we can now update it with the correct velocities from
                        // the past MISMATCH_LOOK_BACK_TIME till lastTimestamp (lastTimestamp - timestamp happens when the pose
                        // estimator updates normally)
                        MutableTranslation3d newVelocity = new MutableTranslation3d();
                        double lastTime = timestamp - MISMATCH_LOOK_BACK_TIME;
                        // Bring the odometry up to lastTimestamp
                        for (double time = lastTime + NOMINAL_DT; time < lastTimestamp; time += NOMINAL_DT) {
                            newVelocity.set(getVelocityAverageVelocityInPeriodFromAcceleration(lastTime, time))
                                    // The velocity from the history method is relative to the robot's startup angle,
                                    // so we need to rotate it to the field frame
                                    .rotateBy(startUpRotationToField)
                                    .plus(velocityOffset);

                            swerveDriveOdometry.updateWithTime(time, getGyroAngleAtTime(time), newVelocity.getTranslation3d(),
                                    time - lastTime);

                            lastTime = time;
                        }

                        // Add the last sample
                        if (lastTime < lastTimestamp) {
                            newVelocity.set(getVelocityAverageVelocityInPeriodFromAcceleration(lastTime, lastTimestamp)
                                    // The velocity from the history method is relative to the robot's startup angle,
                                    // so we need to rotate it to the field frame
                                    .rotateBy(startUpRotationToField)
                                    .plus(velocityOffset));
                            swerveDriveOdometry.updateWithTime(timestamp, getGyroAngleAtTime(timestamp),
                                    newVelocity.getTranslation3d(),
                                    timestamp - lastTime);
                        }
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

            velocity = velocityHistory.getSample(timestamp).orElse(new Translation3d())
                    .rotateBy(startUpRotationToField)
                    .plus(velocityOffset);

            Logger.getInstance().recordOutput("RobotTracker/Velocity Offset X", velocityOffset.getX());
            Logger.getInstance().recordOutput("RobotTracker/Velocity Offset Y", velocityOffset.getY());
            Logger.getInstance().recordOutput("RobotTracker/Velocity Offset Z", velocityOffset.getZ());

            var rawVelocity = velocityHistory.getSample(timestamp).orElse(new Translation3d())
                    .rotateBy(startUpRotationToField);

            Logger.getInstance().recordOutput("RobotTracker/Raw Velocity X", rawVelocity.getX());
            Logger.getInstance().recordOutput("RobotTracker/Raw Velocity Y", rawVelocity.getY());
            Logger.getInstance().recordOutput("RobotTracker/Raw Velocity Z", rawVelocity.getZ());

            Logger.getInstance().recordOutput("RobotTracker/Accumulated Error X", accumulatedVelocityError.getX());
            Logger.getInstance().recordOutput("RobotTracker/Accumulated Error Y", accumulatedVelocityError.getY());
            Logger.getInstance().recordOutput("RobotTracker/Accumulated Error Z", accumulatedVelocityError.getZ());
        } finally {
            lock.readLock().unlock();
        }


        if (isVelocityMismatched) {
            // If the velocity is mismatched, then we need to calculate the average velocity over the time period
            // from the acceleration history
            var averageVelocity =
                    getVelocityAverageVelocityInPeriodFromAcceleration(lastTimestamp, timestamp)
                            .rotateBy(startUpRotationToField)
                            .plus(velocityOffset);

            // Update the odometry with the average velocity over the time period
            swerveDriveOdometry.updateWithTime(timestamp, gyroAngle3d, averageVelocity, lastTimestamp - timestamp);

            // Increment the velocity mismatch time
            velocityMismatchTime += timestamp - lastTimestamp;
        } else {
            // If the velocity is not mismatched, then we can just integrate the module positions
            swerveDriveOdometry.updateWithTime(timestamp, gyroAngle3d, modulePositions);
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

        // Remove old vision measurements we don't need anymore
        pastVisionMeasurements.headMap(timestamp - MISMATCH_LOOK_BACK_TIME).clear();


        lock.writeLock().lock();
        try {
            latestPose3d = swerveDriveOdometry.getEstimatedPosition3d();
            latestPose = latestPose3d.toPose2d();
            if (velocity != null) {
                this.velocity = velocity;
            }
            Pose3d noVisionPose = noVisionOdometry.getPoseMeters3d();
            poseBufferForVelocity.addSample(timestamp, noVisionPose);
            gyroOffset = startUpRotationToField;
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
     * Get the latest pose of the robot. This pose is updated every 20ms and is corrected for drift using vision measurements.
     *
     * @return The latest pose of the robot.
     */
    public @NotNull Pose3d getLatestPose3d() {
        lock.readLock().lock();
        try {
            return latestPose3d;
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

    public double getAngularAcceleration() {
        lock.readLock().lock();
        try {

        } finally {
            lock.readLock().unlock();
        }
    }

    public @NotNull Rotation2d getGyroAngle() {
        lock.readLock().lock();
        try {
            return latestPose.getRotation();
        } finally {
            lock.readLock().unlock();
        }
    }

    private Vector<N3> POSITIVE_Z = VecBuilder.fill(0, 0, 1);

    public @NotNull Rotation3d getGyroAngleAtTime(double time) {
        lock.readLock().lock();
        try {
            return gyroHistory.getSample(time).orElse(ROTATION_IDENTITY)
                    .plus(gyroOffset);
        } finally {
            lock.readLock().unlock();
        }
    }


    public @NotNull Translation3d getAcceleration() {
        lock.readLock().lock();
        try {
            return acceleration.rotateBy(gyroOffset);
        } finally {
            lock.readLock().unlock();
        }
    }

    public @NotNull Translation2d get2dAcceleration() {
        return getAcceleration().toTranslation2d();
    }

    public void resetPose(@NotNull Pose3d pose) {
        lock.writeLock().lock();
        try {
            swerveDriveOdometry.resetPosition(gyroInputs.rotation3d, Robot.getDrive().getModulePositions(), pose);
            gyroOffset = pose.getRotation().minus(gyroInputs.rotation3d);
        } finally {
            lock.writeLock().unlock();
        }
    }


    public void resetPose(@NotNull Pose2d pose2d) {
        resetPose(new Pose3d(pose2d));
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
        Logger.getInstance().recordOutput("RobotTracker/Rotation", getGyroAngle().getDegrees());
        Logger.getInstance().recordOutput("RobotTracker/Rotation Rad", getGyroAngle().getRadians());
        Logger.getInstance().recordOutput("RobotTracker/x", getLatestPose().getTranslation().getX());
        Logger.getInstance().recordOutput("RobotTracker/y", getLatestPose().getTranslation().getY());
        Logger.getInstance().recordOutput("RobotTracker/NoVisionPose", noVisionOdometry.getPoseMeters());
        Logger.getInstance().recordOutput("RobotTracker/VisionPose", getLatestPose());
        Logger.getInstance().recordOutput("RobotTracker/NoVisionPose3d", noVisionOdometry.getPoseMeters3d());
        Logger.getInstance().recordOutput("RobotTracker/VisionPose3d", getLatestPose3d());
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


    /**
     * @param startTime The time to start the integration at
     * @param endTime   The time to end the integration at
     * @return The change in velocity over the time interval
     */
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
     * @param startTime the time to start the measurement window
     * @param endTime   the time to end the measurement window
     * @return the velocity of the robot at the given time relative to the startup angle of the robot
     */
    private Optional<Translation3d> getVelocityAverageVelocityInPeriodFromOdometry(double startTime, double endTime) {
        var pastPose = poseBufferForVelocity.getSample(startTime);
        var currentPose = poseBufferForVelocity.getSample(endTime);
        if (pastPose.isPresent() && currentPose.isPresent()) {
            var avgVelocity = pastPose.get().getTranslation().minus(currentPose.get().getTranslation());
            var averageVelocity = avgVelocity.div(VELOCITY_MEASUREMENT_WINDOW);
            return Optional.of(averageVelocity);
        }
        return Optional.empty();
    }

    private Translation3d getVelocityAverageVelocityInPeriodFromAcceleration(double startTime, double endTime) {
        double deltaTime = endTime - startTime;
        // Create mutable objects to avoid creating new objects
        final var mutDeltaPosition = new MutableTranslation3d();
        final var tempVelocity = new MutableTranslation3d();
        final var lastVelocity = new MutableTranslation3d();


        final var optionalInitialAccel = velocityHistory.getSample(startTime);
        optionalInitialAccel.ifPresent(
                translation3d -> lastVelocity.set(translation3d.getX(), translation3d.getY(), translation3d.getZ()));

        for (var entry : velocityHistory.getInternalBuffer().tailMap(startTime, false).entrySet()) {
            double time = entry.getKey();

            var accel = entry.getValue();

            if (time > endTime) { // We've gone past the end time
                accel = velocityHistory.getSample(endTime).orElse(accel); // Interpolates the accel to the end time
                time = endTime;
            }

            tempVelocity.set(accel.getX(), accel.getY(), accel.getZ()).plus(lastVelocity).times(0.5).times(time - startTime);
            mutDeltaPosition.plus(tempVelocity);

            lastVelocity.set(accel.getX(), accel.getY(), accel.getZ());
            startTime = time;

            if (time >= endTime) {
                break;
            }
        }

        if (startTime < endTime) {
            // Add the last bit of deltaVelocity (assume it's constant from the last sample to now)
            tempVelocity.set(lastVelocity);

            tempVelocity.times(endTime - startTime);
            mutDeltaPosition.plus(tempVelocity);
        }


        // The deltaVelocity over our measurement window
        return mutDeltaPosition.div(deltaTime).getTranslation3d();
    }
}