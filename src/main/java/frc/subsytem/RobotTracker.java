package frc.subsytem;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.utility.geometry.MutableTranslation2d;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

import java.util.concurrent.ScheduledThreadPoolExecutor;
import java.util.concurrent.locks.ReentrantReadWriteLock;

import static frc.robot.Constants.PIGEON_CAN_ID;
import static frc.robot.Constants.SWERVE_DRIVE_KINEMATICS;

public final class RobotTracker extends AbstractSubsystem {

    private final ReentrantReadWriteLock lock = new ReentrantReadWriteLock();
    private final @NotNull WPI_Pigeon2 gyroSensor = new WPI_Pigeon2(PIGEON_CAN_ID, "rio");
    ;


    private static final @NotNull RobotTracker instance = new RobotTracker();

    public static @NotNull RobotTracker getInstance() {
        return RobotTracker.instance;
    }

    /**
     * The pose of the robot at the last time the odometry was updated.
     */
    private @NotNull Pose2d latestPose = new Pose2d();

    /**
     * The pose of the robot at the last time the odometry was updated. (without vision updates)
     */
    private @NotNull Pose2d noVisionPose = new Pose2d();

    private @NotNull Rotation2d gyroOffset = new Rotation2d();

    private double gyroRollVelocity = 0;
    private double gyroPitchVelocity = 0;
    private double gyroYawVelocity = 0;
    private double lastGyroPitch = 0;
    private double lastGyroRoll = 0;


    private final SwerveDrivePoseEstimator swerveDriveOdometry = new SwerveDrivePoseEstimator(
            SWERVE_DRIVE_KINEMATICS,
            gyroSensor.getRotation2d(),
            Drive.getInstance().getModulePositions(),
            new Pose2d(),
            VecBuilder.fill(0.1, 0.1, 0.1),
            VecBuilder.fill(0.9, 0.9, 0.9)
    );

    private final SwerveDriveOdometry noVisionOdometry = new SwerveDriveOdometry(
            SWERVE_DRIVE_KINEMATICS,
            gyroSensor.getRotation2d(),
            Drive.getInstance().getModulePositions()
    );

    private @NotNull Translation2d noVisionOdometryOffset = new Translation2d();


    private final TimeInterpolatableBuffer<Pose2d> poseBufferForVelocity = TimeInterpolatableBuffer.createBuffer(
            Pose2d::interpolate, 0.5);


    private @NotNull Translation2d acceleration = new Translation2d();
    private @NotNull Translation2d velocity = new Translation2d();
    private final @NotNull TimeInterpolatableBuffer<Rotation2d> gyroHistory
            = TimeInterpolatableBuffer.createBuffer(Rotation2d::interpolate, 1.5); //abt 1.5 seconds (ms * 300)

    private final @NotNull TimeInterpolatableBuffer<Translation2d> accelerationHistory
            = TimeInterpolatableBuffer.createBuffer(Translation2d::interpolate, 1.5); //abt 1.5 seconds (ms * 300)

    private RobotTracker() {
        super(Constants.ROBOT_TRACKER_PERIOD, 3);
        new ScheduledThreadPoolExecutor(1).scheduleAtFixedRate(this::updateGyroHistory, 0, 10,
                java.util.concurrent.TimeUnit.MILLISECONDS);
    }

    double[] xyz_dps = new double[3];
    double[] ba_xyz = new double[3];

    private void updateGyroHistory() {

        lock.writeLock().lock();
        try {
            double time = Timer.getFPGATimestamp();
            // TODO: confirm that the system timestamp is the same as the timestamp from Timer.getFPGATimestamp()
            gyroHistory.addSample(time, new Rotation2d(gyroSensor.getYaw()));

            gyroSensor.getRawGyro(xyz_dps);
            gyroYawVelocity = xyz_dps[2];
            gyroRollVelocity = xyz_dps[0];
            gyroPitchVelocity = xyz_dps[1];
            lastGyroPitch = gyroSensor.getPitch();
            lastGyroRoll = gyroSensor.getRoll();
            short[] ba_xyz = new short[3];
            gyroSensor.getBiasedAccelerometer(ba_xyz);

            acceleration = new Translation2d(ba_xyz[0], ba_xyz[1]).rotateBy(gyroSensor.getRotation2d().unaryMinus());
        } finally {
            lock.writeLock().unlock();
        }
    }

    private static final Pose2d ZERO_POSE = new Pose2d();

    private static final double VELOCITY_MEASUREMENT_WINDOW = 0.5;

    @Override
    public void update() {
        double timestamp = Timer.getFPGATimestamp();
        SwerveModulePosition[] modulePositions = Drive.getInstance().getModulePositions();
        swerveDriveOdometry.updateWithTime(timestamp, gyroSensor.getRotation2d(), modulePositions);

        @Nullable Translation2d velocity = null;

        // Calculate the velocity of the robot
        noVisionOdometry.update(gyroSensor.getRotation2d(), modulePositions);
        lock.readLock().lock();
        try {
            var pastPose = poseBufferForVelocity.getSample(timestamp - VELOCITY_MEASUREMENT_WINDOW);
            if (pastPose.isPresent()) {
                var currentPose = noVisionOdometry.getPoseMeters();
                var avgVelocity = pastPose.get().getTranslation().minus(currentPose.getTranslation());
                var averageVelocity = avgVelocity.div(VELOCITY_MEASUREMENT_WINDOW);

                // Create mutable objects to avoid creating new objects
                final MutableTranslation2d totalAccel = new MutableTranslation2d();
                final MutableTranslation2d tempAccel = new MutableTranslation2d();
                double lastTime = timestamp - VELOCITY_MEASUREMENT_WINDOW;


                // Calculate the average acceleration over the last 0.5 seconds
                for (var entry : accelerationHistory.getInternalBuffer().tailMap(lastTime, false).entrySet()) {
                    double time = Math.max(entry.getKey(), timestamp); // Don't go into the future
                    if (time < lastTime) {
                        continue;
                    }
                    tempAccel.set(entry.getValue()).times(time - lastTime);
                    totalAccel.plus(entry.getValue());
                    lastTime = time;
                }

                if (lastTime < timestamp) {
                    // Add the last bit of acceleration (assume it's constant from the last sample to now)
                    tempAccel.set(acceleration);

                    tempAccel.times(timestamp - lastTime);
                    totalAccel.plus(tempAccel);
                }


                // The average acceleration over our measurement window
                var acceleration = totalAccel.div(VELOCITY_MEASUREMENT_WINDOW).getTranslation2d();

                // Start with V_i = V_f - at
                // v_avg = (v_i + v_f) / 2
                // v_avg = (v_f - at + v_f) / 2
                // v_avg = v_f - (at / 2)
                // v_avg + (at / 2) = v_f

                // Calculate the velocity of the robot

                velocity = averageVelocity.plus(acceleration.times(VELOCITY_MEASUREMENT_WINDOW / 2));
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
            this.noVisionPose = noVisionPose;
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
     * Get the latest pose of the robot. This pose is updated every 20ms and is not corrected for drift using vision
     * measurements.
     *
     * @return The latest pose of the robot.
     */
    public @NotNull Pose2d getNoVisionPose() {
        lock.readLock().lock();
        try {
            return noVisionPose;
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
            return -Math.toRadians(gyroSensor.getRate()); // getRate() returns with cw positive, we want ccw positive
        } finally {
            lock.readLock().unlock();
        }
    }

    public @NotNull Rotation2d getGyroAngle() {
        lock.readLock().lock();
        try {
            return gyroSensor.getRotation2d().plus(gyroOffset);
        } finally {
            lock.readLock().unlock();
        }
    }

    public @NotNull Translation2d getAcceleration() {
        lock.readLock().lock();
        try {
            return acceleration;
        } finally {
            lock.readLock().unlock();
        }
    }

    public void resetPose(@NotNull Pose2d pose) {
        lock.writeLock().lock();
        try {
            swerveDriveOdometry.resetPosition(gyroSensor.getRotation2d(), Drive.getInstance().getModulePositions(), pose);
            noVisionOdometryOffset = pose.getTranslation().minus(noVisionPose.getTranslation());
            gyroOffset = pose.getRotation().minus(gyroSensor.getRotation2d());
        } finally {
            lock.writeLock().unlock();
        }
    }


    @Override
    public void selfTest() {

    }

    @Override
    public void logData() {
        logData("rotation", getGyroAngle().getDegrees());
    }
}