package frc.subsytem;

import edu.wpi.first.math.util.Units;
import frc.robot.Robot;
import org.jetbrains.annotations.NotNull;

public class MechanismStateManager extends AbstractSubsystem {
    record MechanismState(double elevatorPositionMeters, double telescopingArmPositionMeters, double grabberAngleDegrees) {
        public double grabberAngleRadians() {
            return Math.toRadians(grabberAngleDegrees);
        }
    }


    public enum MechanismStates {
        STOWED(new MechanismState(0.0, 0.0, 56 + 90 - 20)),
        LOW_SCORING(new MechanismState(0.0, 0.0, 0.0)),
        MIDDLE_SCORING(new MechanismState(Units.inchesToMeters(36), Units.inchesToMeters(17.5), 0.0)),
        HIGH_SCORING(new MechanismState(Units.inchesToMeters(44), Units.inchesToMeters(17.5), 45.0)),
        STATION_PICKUP(new MechanismState(0.0, 0.0, 0.0)),
        FLOOR_PICKUP(new MechanismState(0.0, 0.0, 0.0));

        private final MechanismState state;

        MechanismStates(MechanismState state) {
            this.state = state;
        }
    }


    private @NotNull MechanismState currentWantedState = MechanismStates.STOWED.state;

    public void setState(@NotNull MechanismStates state) {
        currentWantedState = state.state;
    }

    private @NotNull MechanismState lastRequestedState = new MechanismState(-1, -1, -1);

    @Override
    public void update() {
        //TODO: Make sure we're not going to hit anything
        var maxReachablePosition = currentWantedState;


        if (maxReachablePosition.elevatorPositionMeters() != lastRequestedState.elevatorPositionMeters()) {
            Robot.getElevator().setPosition(maxReachablePosition.elevatorPositionMeters());
        }
        if (maxReachablePosition.telescopingArmPositionMeters() != lastRequestedState.telescopingArmPositionMeters()) {
            Robot.getTelescopingArm().setPosition(maxReachablePosition.telescopingArmPositionMeters());
        }

        if (maxReachablePosition.grabberAngleRadians() != lastRequestedState.grabberAngleRadians()) {
            Robot.getGrabber().setPosition(maxReachablePosition.grabberAngleDegrees());
        }

        lastRequestedState = maxReachablePosition;
    }
}
