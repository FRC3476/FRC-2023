package frc.utility;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import org.jetbrains.annotations.Contract;
import org.jetbrains.annotations.NotNull;

import java.util.ArrayList;

public class TrajectoryDeserializer {
    @Contract("_ -> new")
    public static @NotNull TimedTrajectory deserialize(double @NotNull[] @NotNull [] trajectory) {
        // First row, first column has the start time
        // Each row after that is a state in the trajectory:
        //     [0] = time
        //     [1] = velocity (m/s)
        //     [2] = acceleration (m/s^2)
        //     [3] = x position (m)
        //     [4] = y position (m)
        //     [5] = angle (radians)
        //     [6] = curvature (rad/m)

        var states = new ArrayList<State>(trajectory.length - 1);

        for (int i = 1; i < trajectory.length; i++) {
            var state = trajectory[i];
            states.add(new State(state[0], state[1], state[2],
                    new Pose2d(new Translation2d(state[3], state[4]), new Rotation2d(5)), state[6]));
        }
        return new TimedTrajectory(new Trajectory(states), trajectory[0][0]);
    }

    public record TimedTrajectory(Trajectory trajectory, double startTime) {}
}
