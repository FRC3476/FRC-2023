package frc.subsytem;

import frc.robot.Constants;
import frc.utility.MechanismState;

/**
 * Coordinates the elevator, telescoping arm, and grabber subsystems
 */
public class SystemCoordinator extends AbstractSubsystem {

    private final Elevator elevator;
    private final TelescopingArm telescopingArm;
    private final Grabber grabber;

    private static SystemCoordinator instance;

    public static SystemCoordinator getInstance() {
        if (instance == null) {
            instance = new SystemCoordinator(Constants.SYSTEM_COORDINATOR_PERIOD, 5);
        }

        return instance;
    }
    public SystemCoordinator(int period, int loggingInterval) {
        super(period, loggingInterval);
        elevator = Elevator.getInstance();
        telescopingArm = TelescopingArm.getInstance();
        grabber = Grabber.getInstance();
    }

    /**
     * Will drive the elevator, grabber, and telescoping arm to that the end of the grabber reaches these coordinates
     * Coordinates in meters with an orgin at the grabber position with a completely retracted arm and completely lowered
     * elevator and a horizontally level wrist. Orgin measured from the tip of the grabber in the center.
     * @param mechanismState - A mechanism state that includes and x, y, z, and wrist angle
     */
    public void goToCoordinates(MechanismState mechanismState) {


        // Move elevator first

        // Move arm second

        // Move
    }

    private MechanismState findCurrentMechanismState() {
        // Find x coordinate
        double x = 0;

        // Determine how much the elevator contributes to x
        x += Math.cos(Constants.ELEVATOR_TILT_RADIANS) * elevator.getPosition();

        // Determine how much the grabber contributes to x
        x += Constants.GRABBER_LENGTH * Math.cos(Math.toDegrees(grabber.getPivotAngle()));

        // Find y coordinate
        double y = 0;

        // Determine how much the elevator contributes to y
        y += Math.sin(Constants.ELEVATOR_TILT_RADIANS) * elevator.getPosition();

        // Determine how much the telescoping arm contributes y
        y += telescopingArm.getPosition();

        // Determine how much the grabber contributes to y
        y += Constants.GRABBER_LENGTH * Math.sin(Math.toDegrees(grabber.getPivotAngle()));

        double wristAngle = grabber.getPivotAngle();

        return new MechanismState(x, y, wristAngle);
    }
}