package frc.subsytem;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.Robot;
import org.jetbrains.annotations.NotNull;
import org.littletonrobotics.junction.Logger;

import static frc.robot.Constants.GRABBER_LENGTH;
import static frc.robot.Constants.MAX_WRIST_ANGLE;

public class MechanismStateManager extends AbstractSubsystem {

    public record MechanismStateSubsystemPositions(double elevatorPositionMeters, double telescopingArmPositionMeters,
                                                   double grabberAngleDegrees) {
        public double grabberAngleRadians() {
            return Math.toRadians(grabberAngleDegrees);
        }

        @Override
        public String toString() {
            return "Elevator Pos: " + elevatorPositionMeters + " Arm Pos: " + telescopingArmPositionMeters + " Angle: " + grabberAngleDegrees;
        }
    }

    public record MechanismStateCoordinates(double xMeters, double yMeters, double grabberAngleDegrees) {
        public double grabberAngleRadians() {
            return Math.toRadians(grabberAngleDegrees);
        }

        @Override
        public String toString() {
            return "X: " + xMeters + " Y: " + yMeters + " Angle: " + grabberAngleDegrees;
        }
    }


    public enum MechanismStates {
        STOWED(new MechanismStateCoordinates(-0.489, 0.249, MAX_WRIST_ANGLE - 2)),
        LOW_SCORING(new MechanismStateCoordinates(Units.inchesToMeters(12), Units.inchesToMeters(6), 0)),
        MIDDLE_SCORING(new MechanismStateCoordinates(Units.inchesToMeters(16), Units.inchesToMeters(45), -15)),
        HIGH_SCORING(new MechanismStateCoordinates(Units.inchesToMeters(36), Units.inchesToMeters(57), 55)),
        STATION_PICKUP(new MechanismStateCoordinates(0.531, 2.3 - 0.0025, 5.9)),
        FLOOR_PICKUP(new MechanismStateCoordinates(0, 0, -7));
        private final MechanismStateCoordinates state;

        MechanismStates(MechanismStateCoordinates state) {
            this.state = state;
        }
    }


    private @NotNull MechanismStateManager.MechanismStateCoordinates currentWantedState = MechanismStates.STOWED.state;

    public void setState(@NotNull MechanismStates state) {
        currentWantedState = state.state;
    }

    MechanismStateCoordinates lastNotStowState = MechanismStates.STOWED.state;

    public void setState(@NotNull MechanismStateCoordinates state) {
        currentWantedState = state;
        if (state.equals(MechanismStates.STOWED.state)) {
            lastNotStowState = state;
        }
    }

    private @NotNull MechanismStateManager.MechanismStateCoordinates lastRequestedState = new MechanismStateCoordinates(-1, -1,
            -1);

    /**
     * Will find elevator, grabber, and telescoping arm positions to that the end of the grabber reaches the coordinates
     * specified. Coordinates in meters with an orgin at the grabber position with a completely retracted arm and completely
     * lowered elevator and a horizontally level wrist. Origin measured from the tip of the grabber in the center.
     */
    public static MechanismStateSubsystemPositions coordinatesToSubsystemPositions(MechanismStateCoordinates mechanismState) {
        double x = mechanismState.xMeters() + Constants.GRABBER_LENGTH;
        double y = mechanismState.yMeters();
        double angleRad = mechanismState.grabberAngleRadians();

        // Move elevator second to satisfy y keeping in mind that wrist already satisfies some of y
        double elevatorYOnlyMovement = y - Constants.GRABBER_LENGTH * Math.sin(angleRad);

        // Converts the y component of the elevator into an the actual amount the elevator needs to drive on the tilt
        double elevatorRealMovement = elevatorYOnlyMovement / Math.sin(Constants.ELEVATOR_TILT_RADIANS);

        // Finds the amount the elevator moves in the x direction
        double elevatorXOnlyMovement = elevatorYOnlyMovement / Math.tan(Constants.ELEVATOR_TILT_RADIANS);

        // Move arm third to satisfy x keeping in mind that the elevator and grabber already satisfies some of x
        double telescopingArmMovement = x - elevatorXOnlyMovement - Constants.GRABBER_LENGTH * Math.cos(angleRad);

        return new MechanismStateSubsystemPositions(elevatorRealMovement, telescopingArmMovement, Math.toDegrees(angleRad));
    }

    public static MechanismStateCoordinates limitCoordinates(MechanismStateCoordinates mechanismState) {
        return limitCoordinates(mechanismState, Robot.getMechanismStateManager().getCurrentCoordinates());
    }

    private static final double GRABBER_ZERO_X_OFFSET = GRABBER_LENGTH;

    public static MechanismStateCoordinates limitCoordinates(MechanismStateCoordinates mechanismState,
                                                             MechanismStateCoordinates currentMechanismState) {
        double mutableX = mechanismState.xMeters;
        double mutableY = mechanismState.yMeters;
        double mutableWristAngle = mechanismState.grabberAngleDegrees;


        double maxXGivenCurrentY;
        double minYGivenCurrentX;

        double currWristX = Constants.GRABBER_LENGTH * Math.cos(Math.toRadians(currentMechanismState.grabberAngleRadians()));
        double currWristY = Constants.GRABBER_LENGTH * Math.sin(Math.toRadians(currentMechanismState.grabberAngleRadians()));


        double minYOfCurrentArm =
                currentMechanismState.yMeters - currWristY;
        if (!Robot.getMechanismStateManager().lastNotStowState.equals(MechanismStates.FLOOR_PICKUP.state)) {
            if (!Robot.getMechanismStateManager().lastNotStowState.equals(MechanismStates.STATION_PICKUP.state)) {
                double targetArmHeight = mutableY - Math.sin(Math.toRadians(mutableWristAngle));

                if (minYOfCurrentArm < Units.inchesToMeters(24)) {
                    mutableX = Math.min(0.087, mutableX);
                    mutableWristAngle = Math.max(mutableWristAngle, 90);
                }

                mutableY = targetArmHeight + Math.sin(currentMechanismState.grabberAngleRadians());

                if (Math.max(currentMechanismState.xMeters, currentMechanismState.xMeters - currWristX) > 0.087) {
                    mutableY = Math.max(Units.inchesToMeters(24) + currWristY, mutableY);
                    mutableY = Math.max(Units.inchesToMeters(24), mutableY);
                }
            } else {
                if (minYOfCurrentArm < Units.inchesToMeters(3 * 12 + 4)) {
                    mutableX = Math.min(0, mutableX);
                }

                double targetArmHeight = mutableY - Math.sin(Math.toRadians(mutableWristAngle));

                if (currentMechanismState.xMeters + currWristX > 1.02) {
                    mutableY = Math.max(Units.inchesToMeters(3 * 12 + 2) - currWristY, mutableY);
                    mutableY = Math.max(Units.inchesToMeters(3 * 12 + 2), mutableY);
                    mutableWristAngle = Math.max(mutableWristAngle, 90);
                }

                mutableY = targetArmHeight + Math.sin(currentMechanismState.grabberAngleRadians());
            }
        }


        if (mutableWristAngle > MAX_WRIST_ANGLE) {
            mutableWristAngle = MAX_WRIST_ANGLE;
        } else if (mutableWristAngle < Constants.MIN_WRIST_ANGLE) {
            mutableWristAngle = Constants.MIN_WRIST_ANGLE;
        }

        // Check to make sure desires x, y, and wristAngle are not outside of the allowed range

        double wristX = Constants.GRABBER_LENGTH * Math.cos(Math.toRadians(mutableWristAngle));
        double wristY = Constants.GRABBER_LENGTH * Math.sin(Math.toRadians(mutableWristAngle));

        Logger.getInstance().recordOutput("MechanismStateManager/Wrist X", wristX);
        Logger.getInstance().recordOutput("MechanismStateManager/Wrist Y", wristY);

        // Add constraints for allowed y for certain wrist angles
        if (mutableY > Constants.MAX_Y + wristY) {
            mutableY = Constants.MAX_Y + wristY;
        } else if (mutableY < Constants.MIN_Y) {
            // Makes sure that elevator doesn't try to go below allowed amount so it can tilt the grabber down to meet the max y
            mutableY = Constants.MIN_Y;
        } else if (mutableY < wristY) {
            mutableY = wristY;
        }


        // Calculate the minX and maxX based on desired y
        double dynamicMinX =
                ((mutableY - wristY) / Math.tan(Constants.ELEVATOR_TILT_RADIANS)) - Constants.GRABBER_LENGTH + wristX;
        double dynamicMaxX =
                ((mutableY - wristY) / Math.tan(
                        Constants.ELEVATOR_TILT_RADIANS)) - Constants.GRABBER_LENGTH + wristX + Constants.BASE_MAX_X;

        Logger.getInstance().recordOutput("MechanismStateManager/Dynamic Min X", dynamicMinX);
        Logger.getInstance().recordOutput("MechanismStateManager/Dynamic Max X", dynamicMaxX);


        if (mutableX < dynamicMinX) {
            mutableX = dynamicMinX;
        } else if (mutableX > dynamicMaxX) {
            mutableX = dynamicMaxX;
        }

        return new MechanismStateCoordinates(mutableX, mutableY, mutableWristAngle);
    }

    /**
     * Gets current mechanism state based off encoder values Assumes that each system is zeroed on startup
     */
    private MechanismStateCoordinates getCurrentCoordinates() {
        // Find x coordinate
        double x = -Constants.GRABBER_LENGTH;

        // Determine how much the elevator contributes to x
        x += Math.cos(Constants.ELEVATOR_TILT_RADIANS) * Robot.getElevator().getPosition();

        // Determine how much the grabber contributes to x
        x += Constants.GRABBER_LENGTH * Math.cos(Math.toRadians(Robot.getGrabber().getPivotDegrees()));

        // Determine how much the telescoping arm contributes y
        x += Robot.getTelescopingArm().getPosition();

        // Find y coordinate
        double y = 0;

        // Determine how much the elevator contributes to y
        y += Math.sin(Constants.ELEVATOR_TILT_RADIANS) * Robot.getElevator().getPosition();

        // Determine how much the grabber contributes to y
        y += Constants.GRABBER_LENGTH * Math.sin(Math.toRadians(Robot.getGrabber().getPivotDegrees()));

        double wristAngle = Robot.getGrabber().getPivotDegrees();

        return new MechanismStateCoordinates(x, y, wristAngle);
    }

    @Override
    public void update() {

        MechanismStateCoordinates currentCoordinates = getCurrentCoordinates();
        MechanismStateCoordinates limitedStateCoordinates = limitCoordinates(currentWantedState, currentCoordinates);

        Logger.getInstance().recordOutput("MechanismStateManager/Desired State X", currentWantedState.xMeters);
        Logger.getInstance().recordOutput("MechanismStateManager/Desired State Y", currentWantedState.yMeters);
        Logger.getInstance().recordOutput("MechanismStateManager/Desired State Angle",
                currentWantedState.grabberAngleDegrees);

        Logger.getInstance().recordOutput("MechanismStateManager/Limited State X", limitedStateCoordinates.xMeters);
        Logger.getInstance().recordOutput("MechanismStateManager/Limited State Y", limitedStateCoordinates.yMeters);
        Logger.getInstance().recordOutput("MechanismStateManager/Limited State Angle",
                limitedStateCoordinates.grabberAngleDegrees);


        // Check for changes
        if (limitedStateCoordinates.xMeters != lastRequestedState.xMeters
                || limitedStateCoordinates.yMeters != lastRequestedState.yMeters
                || limitedStateCoordinates.grabberAngleDegrees != lastRequestedState.grabberAngleDegrees) {

            MechanismStateSubsystemPositions limitedStatePositions = coordinatesToSubsystemPositions(limitedStateCoordinates);
            // Drive Subsystems
            Robot.getElevator().setPosition(limitedStatePositions.elevatorPositionMeters);
            Robot.getTelescopingArm().setPosition(limitedStatePositions.telescopingArmPositionMeters);
            Robot.getGrabber().setPosition(limitedStatePositions.grabberAngleDegrees);

            Logger.getInstance().recordOutput("MechanismStateManager/Elevator Position",
                    limitedStatePositions.elevatorPositionMeters);
            Logger.getInstance().recordOutput("MechanismStateManager/Arm Position",
                    limitedStatePositions.telescopingArmPositionMeters);
            Logger.getInstance().recordOutput("MechanismStateManager/Grabber Position",
                    limitedStatePositions.grabberAngleDegrees);
        }

        lastRequestedState = limitedStateCoordinates;

        Logger.getInstance().recordOutput("MechanismStateManager/Current X", currentCoordinates.xMeters);
        Logger.getInstance().recordOutput("MechanismStateManager/Current Y", currentCoordinates.yMeters);
        Logger.getInstance().recordOutput("MechanismStateManager/Current Grabber Degrees",
                currentCoordinates.grabberAngleDegrees);

        MechanismStateSubsystemPositions currentPositions = coordinatesToSubsystemPositions(currentCoordinates);
        Logger.getInstance().recordOutput("MechanismStateManager/Recalculated Elevator Position",
                currentPositions.elevatorPositionMeters);
        Logger.getInstance().recordOutput("MechanismStateManager/Recalculated Telescoping Position",
                currentPositions.telescopingArmPositionMeters());
        Logger.getInstance().recordOutput("MechanismStateManager/Recalculated Grabber Angle",
                currentPositions.grabberAngleDegrees);
    }
}
