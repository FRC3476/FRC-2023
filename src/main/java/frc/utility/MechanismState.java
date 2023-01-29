package frc.utility;

public class MechanismState {

    private double xCoordinate;
    private double yCoordinate;
    private double wristAngle;

    /**
     * Represents a state that the elevator-arm-grabber mechanism can be in
     * @param x - coordinate in meters
     * @param y - coordiante in meters
     * @param wristAngle - wrist angle in meters
     */
    public MechanismState(double x, double y, double wristAngle) {
        xCoordinate = x;
        yCoordinate = y;
        this.wristAngle = wristAngle;
    }

    public double getxCoordinate() {
        return xCoordinate;
    }

    public double getyCoordinate() {
        return yCoordinate;
    }

    public double getWristAngle() {
        return wristAngle;
    }
}
