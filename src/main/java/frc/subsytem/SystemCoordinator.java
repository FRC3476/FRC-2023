package frc.subsytem;

import frc.robot.Constants;
import frc.utility.MechanismState;

public class SystemCoordinator extends AbstractSubsystem {

    private static SystemCoordinator instance;
    private final Grabber grabber;
    private Elevator elevator;
    private TelescopingArm telescopingArm;

    private SystemCoordinator() {
        super(Constants.SYSTEM_COORDINATOR_PERIOD, 5);
        elevator = Elevator.getInstance();
        telescopingArm = TelescopingArm.getInstance();
        grabber = Grabber.getInstance();
    }

    /**
     * Coordinates the elevator, telescoping arm, and grabber subsystems
     */
    public static SystemCoordinator getInstance() {
        if (instance == null) {
            instance = new SystemCoordinator();
        }

        return instance;
    }

    /**
     * Will drive the elevator, grabber, and telescoping arm to that the end of the grabber reaches these coordinates
     * Coordinates in meters with an orgin at the grabber position with a completely retracted arm and completely lowered
     * elevator and a horizontally level wrist. Orgin measured from the tip of the grabber in the center.
     *
     * @param desiredState - A mechanism state that includes and x, y, z, and wrist angle
     */
    public void goToCoordinates(MechanismState desiredState) {

        // Move the wrist first to satisfy wristAngle
        grabber.setPosition(desiredState.getWristAngle());

        // Move elevator second to satisfy y keeping in mind that wrist already satisfies some of y
        double elevatorYMovement = desiredState.getyCoordinate() - Constants.GRABBER_LENGTH * Math.sin(Math.toRadians(desiredState.getWristAngle()));
        elevator.setPosition(elevatorYMovement / Math.sin(Constants.ELEVATOR_TILT_RADIANS));

        // Move arm third to satisfy x keeping in mind that the elevator already satisfies some of x
        telescopingArm.setPosition(desiredState.getxCoordinate() - (elevatorYMovement / Math.tan(Constants.ELEVATOR_TILT_RADIANS))
                - Constants.GRABBER_LENGTH * Math.cos(Math.toRadians(desiredState.getWristAngle())));

    }

    /**
     * Gets current mechanism state based off encoder values
     * Assumes that each system is zeroed on startup
     */
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
