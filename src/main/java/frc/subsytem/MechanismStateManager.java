package frc.subsytem;

import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.utility.OrangeUtility;
import org.jetbrains.annotations.NotNull;
import org.littletonrobotics.junction.Logger;

import static frc.robot.Constants.GRABBER_LENGTH;
import static frc.robot.Constants.MAX_WRIST_ANGLE;

public class MechanismStateManager extends AbstractSubsystem {

    public static final double SCORING_KEEPOUT_Y = 0.92;
    public static final double SCORING_KEEPOUT_X = 0.21;
    public static final double PICKUP_KEEPOUT_ELEVATOR_DISTANCE = 1.17;
    public static final double KEEPOUT_HYSTERESIS = 0.02;

    boolean isAtFinalPosition = false;

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

        public double grabberX() {
            return Math.cos(grabberAngleRadians()) * GRABBER_LENGTH;
        }

        public double grabberY() {
            return Math.sin(grabberAngleRadians()) * GRABBER_LENGTH;
        }

        @Override
        public String toString() {
            return "X: " + xMeters + " Y: " + yMeters + " Angle: " + grabberAngleDegrees;
        }

        @Override
        public boolean equals(Object o) {
            return epsilonEquals(0, 0.01, 1);
        }

        public boolean epsilonEquals(Object o, double epsilon, double angleEpsilon) {
            if (this == o) return true;
            if (o == null || getClass() != o.getClass()) return false;
            MechanismStateCoordinates that = (MechanismStateCoordinates) o;
            return OrangeUtility.doubleEqual(that.xMeters, xMeters, epsilon) &&
                    OrangeUtility.doubleEqual(that.yMeters, yMeters, epsilon) &&
                    OrangeUtility.doubleEqual(that.grabberAngleDegrees, grabberAngleDegrees, angleEpsilon);
        }
    }


    public enum MechanismStates {
        STOWED(new MechanismStateCoordinates(-0.489, 0.249, MAX_WRIST_ANGLE - 2)),
        LOW_SCORING(new MechanismStateCoordinates(0.08, 0.1, 0)),
        CUBE_MIDDLE_SCORING(new MechanismStateCoordinates(Units.inchesToMeters(16), Units.inchesToMeters(40), 0)),
        CONE_MIDDLE_SCORING(new MechanismStateCoordinates(Units.inchesToMeters(13), Units.inchesToMeters(47.5), 0)),
        CONE_HIGH_SCORING(new MechanismStateCoordinates(Units.inchesToMeters(36), Units.inchesToMeters(57), 65)),
        CUBE_HIGH_SCORING(new MechanismStateCoordinates(Units.inchesToMeters(36), Units.inchesToMeters(54), 33)),
        STATION_PICKUP(new MechanismStateCoordinates(0.531, 2.3 - 0.015, 12)),
        FLOOR_PICKUP(new MechanismStateCoordinates(0.08, 0.06, 0));
        private final MechanismStateCoordinates state;

        MechanismStates(MechanismStateCoordinates state) {
            this.state = state;
        }
    }


    private @NotNull MechanismStateManager.MechanismStateCoordinates currentWantedState = MechanismStates.STOWED.state;

    MechanismStates lastNotStowState = MechanismStates.STOWED;

    MechanismStates lastState = MechanismStates.STOWED;

    public synchronized void setState(@NotNull MechanismStates state) {
        setState(state.state);
        if (state != MechanismStates.STOWED) {
            lastNotStowState = state;
            System.out.println("New State: " + state.name());
        }
        lastState = state;
    }


    public @NotNull MechanismStateCoordinates getCurrentWantedState() {
        return currentWantedState;
    }

    public synchronized void setState(@NotNull MechanismStateCoordinates state) {
        // Don't set the lastNotStowState here so that the limits remain active if the arcade mode is used to move the mechanism
        currentWantedState = state;
        synchronized (this) {
            isAtFinalPosition = false;
        }
    }

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

    private static final double GRABBER_ZERO_X_OFFSET = GRABBER_LENGTH;

    public static MechanismStateCoordinates limitCoordinates(MechanismStateCoordinates mechanismState) {
        //wanted states
        double mutableX = mechanismState.xMeters;
        double mutableY = mechanismState.yMeters;
        double mutableWristAngle = mechanismState.grabberAngleDegrees;


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
    private synchronized MechanismStateCoordinates getCurrentCoordinates() {
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
    public synchronized void update() {
        MechanismStateCoordinates currentCoordinates = getCurrentCoordinates();
        MechanismStateSubsystemPositions currentPositions = coordinatesToSubsystemPositions(getCurrentCoordinates());

        MechanismStateCoordinates limitedStateCoordinates = limitCoordinates(currentWantedState);


        MechanismStateSubsystemPositions limitedStatePositions = coordinatesToSubsystemPositions(limitedStateCoordinates);
        if (!(lastNotStowState == MechanismStates.FLOOR_PICKUP || lastNotStowState == MechanismStates.LOW_SCORING)) {
            if (lastNotStowState != MechanismStates.STATION_PICKUP) {
                // Use scoring keepouts
                double armEndX = limitedStateCoordinates.xMeters - limitedStateCoordinates.grabberX();
                double armEndY = limitedStateCoordinates.yMeters - limitedStateCoordinates.grabberY();
                double grabberAngle = limitedStateCoordinates.grabberAngleDegrees;

                double realArmEndX = currentCoordinates.xMeters() - currentCoordinates.grabberX();
                double realArmEndY = currentCoordinates.yMeters() - currentCoordinates.grabberY();

                if (realArmEndX > SCORING_KEEPOUT_X) {
                    armEndY = Math.max(armEndY, SCORING_KEEPOUT_Y + KEEPOUT_HYSTERESIS);
                }
                if (realArmEndY < SCORING_KEEPOUT_Y) {
                    armEndX = Math.min(armEndX, SCORING_KEEPOUT_X - KEEPOUT_HYSTERESIS);

                    double remainingX = Math.max(0, SCORING_KEEPOUT_X - realArmEndX - 0.25);
                    double remainingY = SCORING_KEEPOUT_Y - realArmEndY + KEEPOUT_HYSTERESIS * 3;
                    assert remainingY >= 0;
                    assert remainingX >= 0;

                    double minAngleRad = Math.min(
                            Math.acos(Math.min(1, remainingX / GRABBER_LENGTH)), //allow horizontal if far enough
                            Math.atan2(remainingY, remainingX) //don't hit the corner of the keepouts
                    );

                    grabberAngle = Math.max(grabberAngle, Math.toDegrees(minAngleRad));
                }

                limitedStateCoordinates = new MechanismStateCoordinates(
                        armEndX + Math.cos(Math.toRadians(grabberAngle)) * GRABBER_LENGTH,
                        armEndY + Math.sin(Math.toRadians(grabberAngle)) * GRABBER_LENGTH,
                        grabberAngle);
//                limitedStateCoordinates = limitCoordinates(limitedStateCoordinates);
                limitedStatePositions = coordinatesToSubsystemPositions(limitedStateCoordinates);
            } else {
                //use pickup keepouts
                double elevatorPosMeters = limitedStatePositions.elevatorPositionMeters();
                double telescopingPosMeters = limitedStatePositions.telescopingArmPositionMeters();
                double grabberAngleDegrees = limitedStateCoordinates.grabberAngleDegrees();

                if (currentPositions.elevatorPositionMeters < PICKUP_KEEPOUT_ELEVATOR_DISTANCE) {
                    // If the elevator is too low, don't allow the grabber to pivot down too much
                    grabberAngleDegrees = Math.max(grabberAngleDegrees, 90);
                }

                if (currentPositions.grabberAngleDegrees < 90) {
                    // Don't allow the elevator to go up too much if the pivot isn't up enough
                    elevatorPosMeters = Math.max(elevatorPosMeters, PICKUP_KEEPOUT_ELEVATOR_DISTANCE);
                }


                limitedStatePositions = new MechanismStateSubsystemPositions(elevatorPosMeters, telescopingPosMeters,
                        grabberAngleDegrees);
            }
        }


        Logger.getInstance().recordOutput("MechanismStateManager/Desired State X", currentWantedState.xMeters);
        Logger.getInstance().recordOutput("MechanismStateManager/Desired State Y", currentWantedState.yMeters);
        Logger.getInstance().recordOutput("MechanismStateManager/Desired State Angle",
                currentWantedState.grabberAngleDegrees);

        Logger.getInstance().recordOutput("MechanismStateManager/Limited State X", limitedStateCoordinates.xMeters);
        Logger.getInstance().recordOutput("MechanismStateManager/Limited State Y", limitedStateCoordinates.yMeters);
        Logger.getInstance().recordOutput("MechanismStateManager/Limited State Angle",
                limitedStateCoordinates.grabberAngleDegrees);


        // Check for changes


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

        Logger.getInstance().recordOutput("MechanismStateManager/Current X", currentCoordinates.xMeters);
        Logger.getInstance().recordOutput("MechanismStateManager/Current Y", currentCoordinates.yMeters);
        Logger.getInstance().recordOutput("MechanismStateManager/Current Grabber Degrees",
                currentCoordinates.grabberAngleDegrees);

        Logger.getInstance().recordOutput("MechanismStateManager/Recalculated Elevator Position",
                currentPositions.elevatorPositionMeters);
        Logger.getInstance().recordOutput("MechanismStateManager/Recalculated Telescoping Position",
                currentPositions.telescopingArmPositionMeters());
        Logger.getInstance().recordOutput("MechanismStateManager/Recalculated Grabber Angle",
                currentPositions.grabberAngleDegrees);


        var currentGoal = limitCoordinates(currentWantedState);
        boolean isAtFinalPosition = currentGoal.epsilonEquals(currentCoordinates, 0.1, 5);

        Logger.getInstance().recordOutput("MechanismStateManager/X Error", currentGoal.xMeters() - currentCoordinates.xMeters());
        Logger.getInstance().recordOutput("MechanismStateManager/Y Error", currentGoal.yMeters() - currentCoordinates.yMeters());
        Logger.getInstance().recordOutput("MechanismStateManager/Angle Error",
                currentGoal.grabberAngleDegrees() - currentCoordinates.grabberAngleDegrees());

        Logger.getInstance().recordOutput("MechanismStateManager/isAtFinalPosition",
                isAtFinalPosition);


        synchronized (this) {
            this.isAtFinalPosition = isAtFinalPosition;
        }
    }

    /**
     * For auto use only. Blocks the thread until the mechanism is at the correct position
     *
     * @param extraDelayMs extra delay to add after the mechanism is at the correct position
     * @throws InterruptedException if the thread is interrupted
     */
    public void waitTillMechAtFinalPos(long extraDelayMs) throws InterruptedException {
        while (true) {
            synchronized (this) {
                if (isAtFinalPosition) {
                    break;
                }
            }
            Thread.sleep(10);
        }
        Thread.sleep(extraDelayMs);
    }

    public void waitTillMechAtFinalPos() throws InterruptedException {
        waitTillMechAtFinalPos(0);
    }
}
