package frc.subsytem;

import frc.robot.Constants;

public class SystemCoordinator extends AbstractSubsystem {

    private final Elevator elevator = Elevator.getInstance();
    private final TelescopingArm telescopingArm = TelescopingArm.getInstance();
    private final Grabber grabber = Grabber.getInstance();

    private double xCoordinate;
    private double yCoordinate;
    private double zCoordinate;

    private static SystemCoordinator instance;

    public static SystemCoordinator getInstance() {
        if (instance == null) {
            instance = new SystemCoordinator(Constants.SYSTEM_COORDINATOR_PERIOD, 5);
        }

        return instance;
    }
    public SystemCoordinator(int period, int loggingInterval) {
        super(period, loggingInterval);
    }

    /**
     * Will drive the elevator, grabber, and telescoping arm to that the end of the grabber reaches these coordinates
     * Coordinates in meters with an orgin at the grabber position with a completely retracted arm and completely lowered
     * elevator and a horizontally level wrist.
     * @param x - x coordinate in meters
     * @param y - y coordinate in meters
     * @param z - z coordinate in meters
     */
    public void goToCoordinates(double x, double y, double z) {
        // Move elevator first

    }
}
