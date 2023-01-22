package frc.utility;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.spline.Spline.ControlVector;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryGenerator.ControlVectorList;
import edu.wpi.first.math.trajectory.constraint.CentripetalAccelerationConstraint;
import frc.robot.Constants;

import java.util.concurrent.CompletableFuture;

public class PathGenerator {

    public static final double END_VECTOR_LEN = 0.5;
    public static final int VELOCTY_VECTOR_LEN_SCALE = 1;

    private PathGenerator() {}

    private static final TrajectoryConfig realtimeConfig = new TrajectoryConfig(3, 2);

    static {
        realtimeConfig.addConstraint(new CentripetalAccelerationConstraint(2.3));
        realtimeConfig.setKinematics(Constants.SWERVE_DRIVE_KINEMATICS);
    }

    public static CompletableFuture<Trajectory> generateTrajectory(
            Translation2d robotVelocity, Translation2d robotTranslation, Translation2d targetPosition,
            double startPosPredictAhead) {
        // Implicitly determine the position based on the position of the target position
        double dir = targetPosition.getX() > Constants.FIELD_WIDTH_METERS / 2 ? END_VECTOR_LEN : -END_VECTOR_LEN;
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
    public static CompletableFuture<Trajectory> generateTrajectory(
            Translation2d robotVelocity, Translation2d robotTranslation, Translation2d targetPosition, Translation2d endDir,
            double startPosPredictAhead) {
        var trajectoryFuture = new CompletableFuture<Trajectory>();
        return trajectoryFuture.completeAsync(() -> {
            var startPos = robotTranslation.plus(robotVelocity.times(startPosPredictAhead));
            ControlVectorList controlVectors = new ControlVectorList();
            controlVectors.add(new ControlVector(
                    new double[]{startPos.getX(), robotVelocity.getX() * VELOCTY_VECTOR_LEN_SCALE, 0},
                    new double[]{startPos.getY(), robotVelocity.getY() * VELOCTY_VECTOR_LEN_SCALE, 0}
            ));
            controlVectors.add(new ControlVector(
                    new double[]{targetPosition.getX(), endDir.getX(), 0},
                    new double[]{targetPosition.getX(), endDir.getY(), 0}
            ));
            return TrajectoryGenerator.generateTrajectory(controlVectors, realtimeConfig);
        });
    }
}
