package frc.subsytem.robottracker;

import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.dacubeking.AutoBuilder.robot.sender.pathpreview.RobotPositionSender;
import com.dacubeking.AutoBuilder.robot.sender.pathpreview.RobotState;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
import frc.subsytem.AbstractSubsystem;
import frc.subsytem.robottracker.GyroInputs.Entry;
import frc.utility.geometry.MutableTranslation3d;
import frc.utility.wpimodified.SwerveDrivePoseEstimator;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean;

import java.util.*;
import java.util.concurrent.ScheduledThreadPoolExecutor;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.locks.ReentrantReadWriteLock;

import static frc.robot.Constants.*;
import static frc.utility.OrangeUtility.fixCoords;
import static java.lang.Double.isNaN;

public final class RobotTracker extends AbstractSubsystem {
    public static final double GYRO_VELOCITY_MEASUREMENT_WINDOW = 0.04;
    private final ReentrantReadWriteLock lock = new ReentrantReadWriteLock();
    private final @NotNull WPI_Pigeon2 gyroSensor = new WPI_Pigeon2(PIGEON_CAN_ID, "*");

    {
        gyroSensor.configMountPose(0, 0, 0);
        gyroSensor.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_10_SixDeg_Quat, 10, 100);
        gyroSensor.setStatusFramePeriod(PigeonIMU_StatusFrame.BiasedStatus_6_Accel, 10, 100);
        gyroSensor.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_6_SensorFusion, 10, 100);
        gyroSensor.setStatusFramePeriod(PigeonIMU_StatusFrame.BiasedStatus_2_Gyro, 10, 100);
        gyroSensor.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_3_GeneralAccel, 10, 100);
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
     * Angular Roll Rate in degrees per second
     */
    private double angularRollRate = 0;

    public static final Matrix<N4, N1> REALSENSE_DEFAULT_VISION_DEVIATIONS = VecBuilder.fill(0.07, 0.07, 0.05, Math.toRadians(7));
    public static final Matrix<N4, N1> LIMELIGHT_DEFAULT_VISION_DEVIATIONS = VecBuilder.fill(0.3, 0.3, 0.3, Math.toRadians(45));


    private final SwerveDrivePoseEstimator swerveDriveOdometry;

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

        swerveDriveOdometry = new SwerveDrivePoseEstimator(
                SWERVE_DRIVE_KINEMATICS,
                gyroInputs.rotation3d,
                Robot.getDrive().getModulePositions(),
                new Pose3d(),
                VecBuilder.fill(0.1, 0.1, 0.1, 0.01),
                REALSENSE_DEFAULT_VISION_DEVIATIONS
        );

        new ScheduledThreadPoolExecutor(1).scheduleAtFixedRate(this::updateGyroHistory, 0, 10,
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
        double time = Logger.getInstance().getRealTimestamp() * SECONDS_PER_MICROSECOND - 0.05;

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

    long angleUpdates = 0;
    long accelerationUpdates = 0;

    private double gyroYVelocity = 0;
    private double gyroYAngle = 0;

    private final LoggedDashboardBoolean isGyroConnected = new LoggedDashboardBoolean("Gyro/Connected", false);

    {
        Logger.getInstance().registerDashboardInput(isGyroConnected);
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
                angleUpdates += gyroInputs.rotations.size();
                Logger.getInstance().recordOutput("RobotTracker/Angle Updates", angleUpdates);

                accelerationUpdates += gyroInputs.accelerations.size();
                Logger.getInstance().recordOutput("RobotTracker/Acceleration Updates", accelerationUpdates);

                isGyroConnected.set(gyroInputs.rotations.size() > 0 || gyroInputs.accelerations.size() > 0);

                angularRollRate = gyroInputs.gyroRollVelocity;
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

                var lastestEntry = gyroHistory.getInternalBuffer().lastEntry();

                if (lastestEntry != null) {
                    double latestTime = lastestEntry.getKey();
                    {
                        Translation3d up = new Translation3d(0, 0, 1);
                        var rotated = up.rotateBy(getGyroAngleAtTime(latestTime));
                        gyroYAngle = -Math.atan2(rotated.getX(), rotated.getZ());
                    }

                    Translation3d up = new Translation3d(0, 0, 1);
                    var rotated = up.rotateBy(getGyroAngleAtTime(latestTime - GYRO_VELOCITY_MEASUREMENT_WINDOW));
                    var prevGyroYAngle = -Math.atan2(rotated.getX(), rotated.getZ());

                    gyroYVelocity = (gyroYAngle - prevGyroYAngle) / GYRO_VELOCITY_MEASUREMENT_WINDOW;
                }
            } finally {
                lock.writeLock().unlock();
            }
        }

        // The rotation transformation from the frame of all our gyro measurements (which are relative to the startup angle) to
        // the field frame
        Rotation3d startUpRotationToField = swerveDriveOdometry.getEstimatedPosition3d().getRotation().minus(gyroAngle3d);

        SwerveModulePosition[] modulePositions = Robot.getDrive().getModulePositions();

        swerveDriveOdometry.updateWithTime(timestamp, gyroAngle3d, modulePositions);


        synchronized (visionMeasurements) {
            for (VisionMeasurement visionMeasurement : visionMeasurements) {
//                double poseError = lastPose.getTranslation().getDistance(visionMeasurement.pose.getTranslation());
//                if (poseError > maxAllowedPoseError) {
//                    continue;
//                }

                if (visionMeasurement.pose.getX() > FIELD_WIDTH_METERS - HALF_ROBOT_WIDTH
                        || visionMeasurement.pose.getX() < HALF_ROBOT_WIDTH
                        || visionMeasurement.pose.getY() > FIELD_HEIGHT_METERS - HALF_ROBOT_LENGTH + FIELD_HEIGHT_METERS / 2
                        || visionMeasurement.pose.getY() < HALF_ROBOT_LENGTH - FIELD_HEIGHT_METERS / 2
                        || visionMeasurement.pose.getZ() > 0.75) {
                    // ensure that the vision measurement is within the field
                    continue;
                }

                var visionMeasurementStds = visionMeasurement.visionMeasurementStds().orElse(REALSENSE_DEFAULT_VISION_DEVIATIONS);

                pastVisionMeasurements.putIfAbsent(visionMeasurement.timestamp(), new ArrayList<>());
                pastVisionMeasurements.get(visionMeasurement.timestamp()).add(
                        new CompletedVisionMeasurement(visionMeasurement.pose, visionMeasurementStds,
                                swerveDriveOdometry.getEstimatedPosition3d())
                );

                var stds = visionMeasurement.visionMeasurementStds().orElse(REALSENSE_DEFAULT_VISION_DEVIATIONS);
                // var reducedStds = VecBuilder.fill(stds.get(0, 0), stds.get(1, 0), stds.get(3, 0));

                // var visionPose = visionMeasurement.pose();
                // visionPose = new Pose3d(visionPose.getTranslation(), visionPose.getRotation());

                swerveDriveOdometry.addVisionMeasurement(visionMeasurement.pose(), visionMeasurement.timestamp(),
                        stds);
            }
            visionMeasurements.clear();
        }

        ChassisSpeeds chassisSpeeds = SWERVE_DRIVE_KINEMATICS.toChassisSpeeds(Robot.getDrive().getModuleStates());

        Translation3d velocity = new Translation3d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond, 0)
                .rotateBy(swerveDriveOdometry.getEstimatedPosition3d().getRotation());

        // Remove old vision measurements we don't need anymore
        pastVisionMeasurements.headMap(timestamp - MISMATCH_LOOK_BACK_TIME).clear();


        lock.writeLock().lock();
        try {
            var newPose = swerveDriveOdometry.getEstimatedPosition3d();
            var translation = newPose.getTranslation();
            if (isNaN(translation.getX()) || isNaN(translation.getY()) || isNaN(translation.getZ())) {
                swerveDriveOdometry.resetPosition(gyroAngle3d, modulePositions, latestPose3d);
                DriverStation.reportError("Reset swerve Drive Odometry because NaN was detected in the translation", false);
            } else {
                latestPose3d = newPose;
            }

            if (isNaN(translation.getX()) || isNaN(translation.getY()) || isNaN(translation.getZ())) {
                swerveDriveOdometry.resetPosition(gyroAngle3d, modulePositions, new Pose3d());
                DriverStation.reportError("Reset swerve Drive Odometry because NaN was detected in the translation 2nd Time",
                        false);
                latestPose3d = new Pose3d();
            }
            latestPose = latestPose3d.toPose2d();
            if (velocity != null) {
                this.velocity = velocity;
            }
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
     * @return The angular velocity of the robot in degrees per second. (CCW is positive)
     */
    public double getAngularVelocity() {
        lock.readLock().lock();
        try {
            return angularRate;
        } finally {
            lock.readLock().unlock();
        }
    }


    public double getAngularRollVelocity() {
        lock.readLock().lock();
        try {
            return angularRollRate;
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

    private final Vector<N3> POSITIVE_Z = VecBuilder.fill(0, 0, 1);

    public @NotNull Rotation3d getGyroAngleAtTime(double time) {
        lock.readLock().lock();
        try {
            return gyroOffset.plus(gyroHistory.getSample(time).orElse(ROTATION_IDENTITY));
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
        if (!Robot.isOnMainThread()) {
            Robot.runOnMainThread(() -> resetPose(pose));
            return;
        }
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
        Logger.getInstance().recordOutput("RobotTracker/VisionPose", getLatestPose());
        Logger.getInstance().recordOutput("RobotTracker/VisionPose3d", fixCoords(getLatestPose3d()));
        Logger.getInstance().recordOutput("RobotTracker/velocityX", getVelocity().getX());
        Logger.getInstance().recordOutput("RobotTracker/velocityY", getVelocity().getY());
        Logger.getInstance().recordOutput("RobotTracker/accelerationX", getAcceleration().getX());
        Logger.getInstance().recordOutput("RobotTracker/accelerationY", getAcceleration().getY());
        Logger.getInstance().recordOutput("RobotTracker/accelerationZ", getAcceleration().getZ());
        Logger.getInstance().recordOutput("RobotTracker/acceleration", getAcceleration().getNorm());
        Logger.getInstance().recordOutput("RobotTracker/Velocity", getVelocity().getNorm());

        Logger.getInstance().recordOutput("RobotTracker/Rotation X", Math.toDegrees(getLatestPose3d().getRotation().getX()));
        Logger.getInstance().recordOutput("RobotTracker/Rotation Y", Math.toDegrees(getLatestPose3d().getRotation().getY()));
        Logger.getInstance().recordOutput("RobotTracker/Rotation Z", Math.toDegrees(getLatestPose3d().getRotation().getZ()));

        var currGyroAngle = getGyroAngleAtTime(Timer.getFPGATimestamp());

        Logger.getInstance().recordOutput("RobotTracker/Raw Rotation X", Math.toDegrees(currGyroAngle.getX()));
        Logger.getInstance().recordOutput("RobotTracker/Raw Rotation Y", Math.toDegrees(currGyroAngle.getY()));
        Logger.getInstance().recordOutput("RobotTracker/Raw Rotation Z", Math.toDegrees(currGyroAngle.getZ()));

        Logger.getInstance().recordOutput("RobotTracker/Y Axis Angle", Math.toDegrees(gyroYAngle));
        Logger.getInstance().recordOutput("RobotTracker/Y Axis Angle", Math.toDegrees(gyroYVelocity));


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

    public double getGyroYVelocity() {
        return gyroYVelocity;
    }

    public double getGyroYAngle() {
        return gyroYAngle;
    }
}