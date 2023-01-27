package frc.utility;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.spline.Spline.ControlVector;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryGenerator.ControlVectorList;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;

import java.util.ArrayList;
import java.util.Optional;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ForkJoinPool;

import static frc.robot.Constants.*;

public class PathGenerator {
    private PathGenerator() {}

    //set initial velocity, copy it then
    private static final double MAX_VELOCITY = 4;
    private static final double MAX_ACCELERATION = 3;

    private static final ArrayList<TrajectoryConstraint> constraints = new ArrayList<>();

    static {
        constraints.add(new CentripetalAccelerationConstraint(2.3));
    }

    public static CompletableFuture<Optional<Trajectory>> generateTrajectory(
            Translation2d robotVelocity, Translation2d robotTranslation, Translation2d targetPosition,
            double startPosPredictAhead) {
        // Implicitly determine the goal vector based on the position of the target position
        double dir = targetPosition.getX() > Constants.FIELD_WIDTH_METERS / 2 ? END_VECTOR_LEN : -END_VECTOR_LEN; //based on
        // alliance flip end vector
        return generateTrajectory(robotVelocity, robotTranslation, targetPosition, new Translation2d(dir, 0),
                startPosPredictAhead);
    }

    /**
     * Get a trajectory to drive the current robot state to end position
     *
     * @param robotVelocity        the current robot velocity
     * @param robotTranslation     the current robot position
     * @param targetPosition       the target position
     * @param endDir               the direction of the end vector
     * @param startPosPredictAhead the amount of time to predict the robot position ahead (to account for delay in generating the
     *                             path)
     * @return A future that will contain the trajectory
     */
    public static CompletableFuture<Optional<Trajectory>> generateTrajectory(
            Translation2d robotVelocity, Translation2d robotTranslation, Translation2d targetPosition, Translation2d endDir,
            double startPosPredictAhead) {
        var trajectoryFuture = new CompletableFuture<Optional<Trajectory>>();
        return trajectoryFuture.completeAsync(() -> {
            var startPos = robotTranslation.plus(robotVelocity.times(startPosPredictAhead));
            double robotVelocityNorm = robotVelocity.getNorm();
            ControlVectorList controlVectors = new ControlVectorList();
            controlVectors.add(new ControlVector(
                    new double[]{startPos.getX(), robotVelocity.getX() * VELOCITY_VECTOR_LEN_SCALE, 0},
                    new double[]{startPos.getY(), robotVelocity.getY() * VELOCITY_VECTOR_LEN_SCALE, 0}
            ));
            controlVectors.add(new ControlVector(
                    new double[]{targetPosition.getX(), endDir.getX(), 0},
                    new double[]{targetPosition.getX(), endDir.getY(), 0}
            ));
            Trajectory trajectory;
            TrajectoryConfig config = new TrajectoryConfig(MAX_VELOCITY, MAX_ACCELERATION);
            config.setKinematics(Constants.SWERVE_DRIVE_KINEMATICS);
            config.setStartVelocity(robotVelocityNorm);
            config.setEndVelocity(0);
            config.addConstraints(constraints);
            try {
                trajectory = TrajectoryGenerator.generateTrajectory(controlVectors, config);
            } catch (Exception e) {
                DriverStation.reportError("Failed to generate trajectory: " + e.getMessage(), e.getStackTrace());
                return Optional.empty();
            }
            if (trajectory.getStates().size() == 0) {
                DriverStation.reportError("Failed to generate trajectory: no states", false);
                return Optional.empty();
            } else if (Math.abs(
                    trajectory.getStates().get(0).velocityMetersPerSecond - robotVelocityNorm) > MAX_VELOCITY_ERROR_NEW_PATH) {
                DriverStation.reportError("Failed to generate trajectory: initial velocity too far off actual", false);
                // If we're getting this we likely need to increase VELOCITY_VECTOR_LEN_SCALE so that the path is more
                // straight, at the beginning of it
                return Optional.empty();
            }
            if (isTrajectoryInBounds(trajectory)) {
                return Optional.of(trajectory);
            } else {
                DriverStation.reportError("Failed to generate trajectory: trajectory goes out of bounds", false);
                return Optional.empty();
            }
        }, ForkJoinPool.commonPool());
    }

    public static boolean isTrajectoryInBounds(Trajectory trajectory) {
        for (var state : trajectory.getStates()) {
            // Check bounds of the field
            if (state.poseMeters.getTranslation().getX() > FIELD_WALL_RIGHT_X - HALF_ROBOT_WIDTH ||
                    state.poseMeters.getTranslation().getX() < FIELD_WALL_LEFT_X + HALF_ROBOT_WIDTH ||
                    state.poseMeters.getTranslation().getY() > FIELD_WALL_TOP_Y - HALF_ROBOT_LENGTH ||
                    state.poseMeters.getTranslation().getY() < FIELD_WALL_BOTTOM_Y + HALF_ROBOT_LENGTH) {
                return false;
            }

            // Check bounds of the grids
            if (state.poseMeters.getTranslation().getY() > GRIDS_START_Y - HALF_ROBOT_LENGTH &&
                    (state.poseMeters.getTranslation().getX() > GRIDS_RED_X - HALF_ROBOT_WIDTH ||
                            state.poseMeters.getTranslation().getX() < GRIDS_BLUE_X + HALF_ROBOT_WIDTH)) {
                return false;
            }
        }
        return true;
    }
}
