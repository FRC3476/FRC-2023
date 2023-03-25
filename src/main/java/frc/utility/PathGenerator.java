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
import frc.robot.Robot;
import org.joml.Intersectiond;
import org.joml.PolygonsIntersection;

import java.util.ArrayList;
import java.util.Optional;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import static frc.robot.Constants.*;

public class PathGenerator {

    public static final double CHARGING_STATION_CENTER_Y = 1.269;
    public static final double CREATE_EXTRA_POINT_X_THRESHOLD = 3.7;
    public static final double EXTRA_POINT_VECTOR_LENGTH = 2;
    public static final double EXTRA_POINT_X = 2.915;

    private PathGenerator() {}

    //set initial velocity, copy it then
    private static final double MAX_VELOCITY = 3.5;
    private static final ArrayList<TrajectoryConstraint> constraints = new ArrayList<>();

    private static ExecutorService threadPoolExecutor;

    static {
        constraints.add(new CentripetalAccelerationConstraint(5));
        threadPoolExecutor = Executors.newSingleThreadExecutor();
    }

    public static CompletableFuture<Optional<Trajectory>> generateTrajectory(
            Translation2d robotVelocity, Translation2d robotTranslation, Translation2d targetPosition,
            double startPosPredictAhead, boolean singleStationPickup, double maxAcceleration) {
        Translation2d endDirTranslation;
        if (singleStationPickup) {
            endDirTranslation = new Translation2d(0, -END_VECTOR_LEN);
        } else {
            // Implicitly determine the goal vector based on the position of the target position
            endDirTranslation = new Translation2d(
                    //based on alliance flip end vector
                    targetPosition.getX() > Constants.FIELD_WIDTH_METERS / 2 ? END_VECTOR_LEN : -END_VECTOR_LEN,
                    0
            );
        }
        return generateTrajectory(robotVelocity, robotTranslation, targetPosition, endDirTranslation,
                startPosPredictAhead, Robot.isRed(), maxAcceleration);
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
            double startPosPredictAhead, boolean isRedAlliance, double maxAcceleration) {
        var trajectoryFuture = new CompletableFuture<Optional<Trajectory>>();
        return trajectoryFuture.completeAsync(() -> {
            var startPos = robotTranslation.plus(robotVelocity.times(startPosPredictAhead));
            double robotVelocityNorm = robotVelocity.getNorm();

            var velocityToUse = robotVelocity;
            if (robotVelocityNorm < 0.02) {
                var posToTarget = robotTranslation.minus(targetPosition);
                velocityToUse = posToTarget.div(posToTarget.getNorm() * 10); // Make the length 1/10
            }

            ControlVectorList controlVectors = new ControlVectorList();
            controlVectors.add(new ControlVector(
                    new double[]{startPos.getX(), velocityToUse.getX() * VELOCITY_VECTOR_LEN_SCALE, 0},
                    new double[]{startPos.getY(), velocityToUse.getY() * VELOCITY_VECTOR_LEN_SCALE, 0}
            ));

            double yCordToCreate = robotTranslation.getY() > CHARGING_STATION_CENTER_Y ? 3.2 : -0.7019;

            // Create an extra point in between the charge station and wall if we're far enough away from the scoring zone
            if (isRedAlliance && endDir.getX() < 0) {
                // We're trying to score on the red side
                if (startPos.getX() > CREATE_EXTRA_POINT_X_THRESHOLD) {
                    controlVectors.add(new ControlVector(
                            new double[]{EXTRA_POINT_X, -EXTRA_POINT_VECTOR_LENGTH, 0},
                            new double[]{yCordToCreate, 0, 0}
                    ));
                }
            } else if (!isRedAlliance && endDir.getX() > 0) {
                // We're trying to score on the blue side
                if (startPos.getX() < FIELD_WIDTH_METERS - CREATE_EXTRA_POINT_X_THRESHOLD) {
                    controlVectors.add(new ControlVector(
                            new double[]{FIELD_WIDTH_METERS - EXTRA_POINT_X, EXTRA_POINT_VECTOR_LENGTH, 0},
                            new double[]{yCordToCreate, 0, 0}
                    ));
                }
            }


            controlVectors.add(new ControlVector(
                    new double[]{targetPosition.getX(), endDir.getX(), 0},
                    new double[]{targetPosition.getY(), endDir.getY(), 0}
            ));
            Trajectory trajectory;
            TrajectoryConfig config = new TrajectoryConfig(Math.max(MAX_VELOCITY, robotVelocityNorm), maxAcceleration);
            //config.setKinematics(Constants.SWERVE_DRIVE_KINEMATICS); // The Kinematics has mutable state, so we can't use the same one
            config.setStartVelocity(robotVelocityNorm);
            config.setEndVelocity(0);
            config.addConstraints(constraints);
            try {
                trajectory = TrajectoryGenerator.generateTrajectory(controlVectors, config);
            } catch (Exception e) {
                DriverStation.reportError("Failed to generate trajectory: " + e.getMessage(), e.getStackTrace());
                return Optional.empty();
            }
//            System.out.println(trajectory.getTotalTimeSeconds());
//            System.out.println(trajectory.sample(0));
//            System.out.println(trajectory.sample(trajectory.getTotalTimeSeconds()));
//            System.out.println(trajectory.sample(trajectory.getTotalTimeSeconds() / 2));
            if (trajectory.getStates().size() == 0) {
                DriverStation.reportError("Failed to generate trajectory: no states", false);
                return Optional.empty();
            } else if (false && Math.abs(
                    trajectory.getStates().get(0).velocityMetersPerSecond - robotVelocityNorm) > MAX_VELOCITY_ERROR_NEW_PATH) {
                DriverStation.reportError("Failed to generate trajectory: initial velocity too far off actual. " +
                        "Wanted Start Velocity: " + robotVelocityNorm +
                        "Actual Start Velocity: " + trajectory.getStates().get(0).velocityMetersPerSecond, false);
                // If we're getting this we likely need to increase VELOCITY_VECTOR_LEN_SCALE so that the path is more
                // straight, at the beginning of it
                return Optional.empty();
            }
            if (true || isTrajectoryInBounds(trajectory)) {
                return Optional.of(trajectory);
            } else {
                DriverStation.reportError("Failed to generate trajectory: trajectory goes out of bounds", false);
                return Optional.empty();
            }
        }, threadPoolExecutor);
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
                    System.out.println("Out of bounds at " + state);
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
                    GRIDS_RED_X + COMMUNITY_BORDER_LENGTH, //x
                    GRIDS_START_Y + COMMUNITY_BORDER_HEIGHT, //y
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
                    GRIDS_BLUE_X - COMMUNITY_BORDER_LENGTH, //x
                    GRIDS_START_Y, //y
                    0, //z
                    //upper point
                    GRIDS_RED_X, //x
                    GRIDS_START_Y + COMMUNITY_BORDER_HEIGHT, //y
                    1 //z
            )) {
                System.out.println("Hitting thing at " + state);
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
