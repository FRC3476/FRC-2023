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
import org.joml.Intersectiond;
import org.joml.PolygonsIntersection;

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

    public static final float FIELD_HEIGHT_METERS = (float) Constants.FIELD_HEIGHT_METERS;
    public static final float FIELD_WIDTH_METERS = (float) Constants.FIELD_WIDTH_METERS;


    public static boolean isTrajectoryInBounds(Trajectory trajectory) {
        Trajectory.State prev = trajectory.getStates().get(0);
        for (var state : trajectory.getStates()) {
            for (var testPoint : testPoints) {
                float x = testPoint[0] + (float) state.poseMeters.getTranslation().getX();
                float y = testPoint[1] + (float) state.poseMeters.getTranslation().getY();

                if (!intersectionTest.testPoint(x, y)) {
                    return false;
                }
            }

            // Do intersection test between to detect if we're touching the boundary between the 2 protected zones.
            // Since it's long and thin we can't just do intersection tests with the corners of the robot.
            if (Intersectiond.testAabAab(
                    //robot bounds
                    //lower point
                    prev.poseMeters.getTranslation().getX() - HALF_ROBOT_WIDTH, //x
                    prev.poseMeters.getTranslation().getY() - HALF_ROBOT_LENGTH, //y
                    0, //z
                    //upper point
                    prev.poseMeters.getTranslation().getX() + HALF_ROBOT_WIDTH, //x
                    prev.poseMeters.getTranslation().getY() + HALF_ROBOT_LENGTH, //y
                    1, //z

                    //target bounds
                    //lower point
                    GRIDS_RED_X, //x
                    GRIDS_START_Y, //y
                    0, //z
                    //upper point
                    GRIDS_RED_X + COMMUNITY_BOARDER_LENGTH, //x
                    GRIDS_START_Y + COMMUNITY_BOARDER_HEIGHT, //y
                    1 //z
            ) || Intersectiond.testAabAab(
                    //robot bounds
                    //lower point
                    prev.poseMeters.getTranslation().getX() - HALF_ROBOT_WIDTH, //x
                    prev.poseMeters.getTranslation().getY() - HALF_ROBOT_LENGTH, //y
                    0, //z
                    //upper point
                    prev.poseMeters.getTranslation().getX() + HALF_ROBOT_WIDTH, //x
                    prev.poseMeters.getTranslation().getY() + HALF_ROBOT_LENGTH, //y
                    1, //z

                    //target bounds
                    //lower point
                    GRIDS_BLUE_X - COMMUNITY_BOARDER_LENGTH, //x
                    GRIDS_START_Y, //y
                    0, //z
                    //upper point
                    GRIDS_RED_X, //x
                    GRIDS_START_Y + COMMUNITY_BOARDER_HEIGHT, //y
                    1 //z
            )) {
                return false;
            }
        }
        return true;
    }

    private static final float[] vertices = new float[]{
            //x, y
            //Field boundary
            0, -FIELD_HEIGHT_METERS / 2,
            0, FIELD_HEIGHT_METERS / 2,
            FIELD_WIDTH_METERS, FIELD_HEIGHT_METERS / 2,
            FIELD_WIDTH_METERS, -FIELD_HEIGHT_METERS / 2,

            //Remove the Grids on the red side
            0, GRIDS_START_Y,
            0, FIELD_HEIGHT_METERS / 2,
            GRIDS_RED_X, FIELD_HEIGHT_METERS / 2,
            GRIDS_RED_X, GRIDS_START_Y,

            //Remove the Grids on the blue side
            GRIDS_BLUE_X, GRIDS_START_Y,
            GRIDS_BLUE_X, FIELD_HEIGHT_METERS / 2,
            FIELD_WIDTH_METERS, FIELD_HEIGHT_METERS / 2,
            FIELD_WIDTH_METERS, GRIDS_START_Y,

            //Remove the charging station on the red side
            CHARGING_STATION_RED_LOWER_LEFT_X, CHARGING_STATION_LOWER_LEFT_Y,
            CHARGING_STATION_RED_LOWER_LEFT_X, CHARGING_STATION_LOWER_LEFT_Y + CHARGING_STATION_HEIGHT,
            CHARGING_STATION_RED_LOWER_LEFT_X + CHARGING_STATION_WIDTH, CHARGING_STATION_LOWER_LEFT_Y + CHARGING_STATION_HEIGHT,
            CHARGING_STATION_RED_LOWER_LEFT_X + CHARGING_STATION_WIDTH, CHARGING_STATION_LOWER_LEFT_Y,

            //Remove the charging station on the blue side
            CHARGING_STATION_BLUE_LOWER_LEFT_X, CHARGING_STATION_LOWER_LEFT_Y,
            CHARGING_STATION_BLUE_LOWER_LEFT_X, CHARGING_STATION_LOWER_LEFT_Y + CHARGING_STATION_HEIGHT,
            CHARGING_STATION_BLUE_LOWER_LEFT_X + CHARGING_STATION_WIDTH, CHARGING_STATION_LOWER_LEFT_Y + CHARGING_STATION_HEIGHT,
            CHARGING_STATION_BLUE_LOWER_LEFT_X + CHARGING_STATION_WIDTH, CHARGING_STATION_LOWER_LEFT_Y,
    };

    private static final int[] polygonStartIndices = new int[]{
            0, 4, 8, 12, 16
    };


    private static final float[][] testPoints = new float[][]{
            {-HALF_ROBOT_LENGTH, -HALF_ROBOT_WIDTH},
            {-HALF_ROBOT_LENGTH, HALF_ROBOT_WIDTH},
            {HALF_ROBOT_LENGTH, HALF_ROBOT_WIDTH},
            {HALF_ROBOT_LENGTH, -HALF_ROBOT_WIDTH}
    };


    private static final PolygonsIntersection intersectionTest = new PolygonsIntersection(
            vertices,
            polygonStartIndices,
            vertices.length / 2
    );
}
