package frc.subsytem;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.Robot;
import org.jetbrains.annotations.NotNull;

public class MechanismStateManager extends AbstractSubsystem {
    record MechanismState(double elevatorPositionMeters, double telescopingArmPositionMeters, double grabberAngleDegrees) {
        public double grabberAngleRadians() {
            return Math.toRadians(grabberAngleDegrees);
        }
    }


    public enum MechanismStates {
        STOWED(coordinatesToMechanismState(0, 0, 0)),
        LOW_SCORING(coordinatesToMechanismState(.2, 0, 0)),
        MIDDLE_SCORING(coordinatesToMechanismState(.4, .3, 0)),
        HIGH_SCORING(coordinatesToMechanismState(.7, .7, 0)),
        STATION_PICKUP(coordinatesToMechanismState(.7, .7, 0)),
        FLOOR_PICKUP(coordinatesToMechanismState(.1, 0, -45));

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

    /**
     * Will find elevator, grabber, and telescoping arm positions to that the end of the grabber reaches the coordinates specified. Coordinates
     * in meters with an orgin at the grabber position with a completely retracted arm and completely lowered elevator and a
     * horizontally level wrist. Origin measured from the tip of the grabber in the center.
     */
    private static MechanismState coordinatesToMechanismState(double x, double y, double wristAngleDegrees) {

        // Move elevator second to satisfy y keeping in mind that wrist already satisfies some of y
        double elevatorYOnlyMovement =
                y - Constants.GRABBER_LENGTH * Math.sin(Math.toRadians(wristAngleDegrees));

        // Converts the y component of the elevator into an the actual amount the elevator needs to drive on the tilt
        double elevatorRealMovement = elevatorYOnlyMovement / Math.sin(Constants.ELEVATOR_TILT_RADIANS);

        // Finds the amount the elevator moves in the x direction
        double elevatorXOnlyMovement = elevatorYOnlyMovement / Math.tan(Constants.ELEVATOR_TILT_RADIANS);

        // Move arm third to satisfy x keeping in mind that the elevator and grabber already satisfies some of x
        double telescopingArmMovement = x - elevatorXOnlyMovement
                - Constants.GRABBER_LENGTH * Math.cos(Math.toRadians(wristAngleDegrees));

        return new MechanismState(elevatorRealMovement, telescopingArmMovement, wristAngleDegrees);
    }

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
