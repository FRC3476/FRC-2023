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
        STOWED(new MechanismState(0.01, 0.01, 56 + 90 - 23)),
        LOW_SCORING(new MechanismState(0.01, Units.inchesToMeters(10), 0)),

        MIDDLE_SCORING(new MechanismState(Units.inchesToMeters(36), Units.inchesToMeters(17), 0)),
        HIGH_SCORING(new MechanismState(Units.inchesToMeters(44), Units.inchesToMeters(17), 45.0)),
        //HIGH_SCORING(new MechanismState(Units.inchesToMeters(0.02), Units.inchesToMeters(17), 56 + 90 - 20)),
        STATION_PICKUP(new MechanismState(Units.inchesToMeters(0), 0.01, 0.0)),
        FLOOR_PICKUP(new MechanismState(0.01, Units.inchesToMeters(10), 0.0));

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
