package frc.subsytem;

import frc.robot.Constants;
import frc.robot.Robot;
import org.jetbrains.annotations.NotNull;

public class MechanismStateManager extends AbstractSubsystem {

    public record MechanismStateSubsystemPositions(double elevatorPositionMeters, double telescopingArmPositionMeters, double grabberAngleDegrees) {
        public double grabberAngleRadians() {
            return Math.toRadians(grabberAngleDegrees);
        }
    }

    public record MechanismStateCoordinates(double xMeters, double yMeters, double grabberAngleDegrees) {
        public double grabberAngleRadians() {
            return Math.toRadians(grabberAngleDegrees);
        }
    }


    public enum MechanismStates {
        STOWED(new MechanismStateCoordinates(0, 0, 0)),
        LOW_SCORING(new MechanismStateCoordinates(.2, 0, 0)),
        MIDDLE_SCORING(new MechanismStateCoordinates(.3, .2, 0)),
        HIGH_SCORING(new MechanismStateCoordinates(.7, .7, 0)),
        STATION_PICKUP(new MechanismStateCoordinates(.7, .7, 0)),
        FLOOR_PICKUP(new MechanismStateCoordinates(.2, 0, 45));

        private final MechanismStateCoordinates state;

        MechanismStates(MechanismStateCoordinates state) {
            this.state = state;
        }
    }


    private @NotNull MechanismStateManager.MechanismStateCoordinates currentWantedState = MechanismStates.STOWED.state;

    public void setState(@NotNull MechanismStates state) {
        currentWantedState = state.state;
    }

    private @NotNull MechanismStateManager.MechanismStateCoordinates lastRequestedState = new MechanismStateCoordinates(-1, -1, -1);

    /**
     * Will find elevator, grabber, and telescoping arm positions to that the end of the grabber reaches the coordinates specified. Coordinates
     * in meters with an orgin at the grabber position with a completely retracted arm and completely lowered elevator and a
     * horizontally level wrist. Origin measured from the tip of the grabber in the center.
     */
    public static MechanismStateSubsystemPositions coordinatesToSubsystemPositions(MechanismStateCoordinates mechanismState) {

        // Move elevator second to satisfy y keeping in mind that wrist already satisfies some of y
        double elevatorYOnlyMovement =
                mechanismState.yMeters - Constants.GRABBER_LENGTH * Math.sin(Math.toRadians(mechanismState.grabberAngleDegrees));

        // Converts the y component of the elevator into an the actual amount the elevator needs to drive on the tilt
        double elevatorRealMovement = elevatorYOnlyMovement / Math.sin(Constants.ELEVATOR_TILT_RADIANS);

        // Finds the amount the elevator moves in the x direction
        double elevatorXOnlyMovement = elevatorYOnlyMovement / Math.tan(Constants.ELEVATOR_TILT_RADIANS);

        // Move arm third to satisfy x keeping in mind that the elevator and grabber already satisfies some of x
        double telescopingArmMovement = mechanismState.xMeters - elevatorXOnlyMovement
                - Constants.GRABBER_LENGTH * Math.cos(Math.toRadians(mechanismState.grabberAngleDegrees));

        return new MechanismStateSubsystemPositions(elevatorRealMovement, telescopingArmMovement, mechanismState.grabberAngleDegrees);
    }

    public static MechanismStateCoordinates limitCoordinates(MechanismStateCoordinates mechanismState) {
        double mutableX = mechanismState.xMeters;
        double mutableY = mechanismState.yMeters;
        double mutableWristAngle = mechanismState.grabberAngleDegrees;

        if (mutableWristAngle > Constants.MAX_WRIST_ANGLE) {
            mutableWristAngle = Constants.MAX_WRIST_ANGLE;
        } else if (mutableWristAngle < Constants.MIN_WRIST_ANGLE) {
            mutableWristAngle = Constants.MIN_WRIST_ANGLE;
        }

        // Check to make sure desires x, y, and wristAngle are not outside of the allowed range
        double wristX = Constants.GRABBER_LENGTH * Math.cos(Math.toRadians(mutableWristAngle));
        double wristY = Constants.GRABBER_LENGTH * Math.sin(Math.toRadians(mutableWristAngle));

        // Add constraints for allowed y for certain wrist angles
        if (mutableWristAngle > 0) {
            if (mutableY > Constants.MAX_Y) {
                mutableY = Constants.MAX_Y;
            } else if (mutableY < Constants.MAX_Y + wristY) {
                // Makes sure that elevator doesn't try to go below allowed amount so it can tilt the grabber down to meet the max y
                mutableY = Constants.MAX_Y + wristY;
            }
        } else {
            if (mutableY > Constants.MAX_Y - wristY) {
                // Makes sure that elevator doesn't try to go above allowed amount so it can tilt the grabber up to meet the min y
                mutableY = Constants.MAX_Y - wristY;
            } else if (mutableY < Constants.MIN_Y) {
                mutableY = Constants.MIN_Y;
            }
        }

        // Calculate the minX and maxX based on desired y
        double dynamicMinX = Constants.BASE_MIN_X + (mutableY / Math.tan(Constants.ELEVATOR_TILT_RADIANS));
        double dynamicMaxX = Constants.BASE_MAX_X + (mutableY / Math.tan(Constants.ELEVATOR_TILT_RADIANS));

        // Add constraints for allowed x for certain wrist angles
        if (mutableWristAngle > 90 || mutableWristAngle < -90) {
            if (mutableX > dynamicMaxX + wristX) {
                mutableX = dynamicMaxX + wristX;
            } else if (mutableX < dynamicMinX) {
                // Makes sure that elevator doesn't try to go below allowed amount so it can tilt the grabber up to meet the min X
                mutableX = dynamicMinX;
            }
        } else {
            if (mutableX > dynamicMaxX) {
                mutableX = dynamicMaxX;
            } else if (mutableX < dynamicMinX - wristX) {
                mutableX = dynamicMinX - wristX;
            }
        }

        return new MechanismStateCoordinates(mutableX, mutableY, mutableWristAngle);
    }


    @Override
    public void update() {
        var desiredStateCoordinates = currentWantedState;
        MechanismStateCoordinates limitedStateCoordinates = limitCoordinates(desiredStateCoordinates);
        MechanismStateSubsystemPositions limitedStatePositions = coordinatesToSubsystemPositions(limitedStateCoordinates);

        // Check for changes
        if (desiredStateCoordinates.xMeters != lastRequestedState.xMeters
            || desiredStateCoordinates.yMeters != lastRequestedState.yMeters
            || desiredStateCoordinates.grabberAngleDegrees != lastRequestedState.grabberAngleDegrees) {

            // Drive Subsystems
            Robot.getElevator().setPosition(limitedStatePositions.elevatorPositionMeters);
            Robot.getTelescopingArm().setPosition(limitedStatePositions.telescopingArmPositionMeters);
            Robot.getGrabber().setPosition(limitedStatePositions.grabberAngleDegrees);
        }

        lastRequestedState = desiredStateCoordinates;
    }
}
