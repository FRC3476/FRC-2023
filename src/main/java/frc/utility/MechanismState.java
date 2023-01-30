package frc.utility;

import frc.robot.Constants;

public class MechanismState {

    private double xCoordinate;
    private double yCoordinate;
    private double wristAngle;

    // TODO: Find real max and min values
    private final static double MAX_X = 0;
    private final static double MIN_X = 0;
    private final static double MAX_Y = 0;
    private final static double MIN_Y = 0;
    private final static double MIN_WRIST_ANGLE = 0;
    private final static double MAX_WRIST_ANGLE = 0;

    /**
     * Represents a state that the elevator-arm-grabber mechanism can be in
     * @param x - coordinate in meters
     * @param y - coordiante in meters
     * @param wristAngle - wrist angle in degrees
     */
    public MechanismState(double x, double y, double wristAngle) {
        if(wristAngle > MAX_WRIST_ANGLE) {
            wristAngle = MAX_WRIST_ANGLE;
        } else if (wristAngle < MIN_WRIST_ANGLE) {
            wristAngle = MIN_WRIST_ANGLE;
        }

        // Check to make sure desires x, y, and wristAngle are not outside of the allowed range
        double wristX = Constants.GRABBER_LENGTH * Math.cos(Math.toRadians(wristAngle));
        double wristY = Constants.GRABBER_LENGTH * Math.sin(Math.toRadians(wristAngle));

        // Add constraints for allowed x for certain wrist angles
        if(wristAngle > 90 && wristAngle < 270) {
            if(x > MAX_X + wristX) {
                x = MAX_X + wristX;
            } else if(x < MIN_X) {
                // Makes sure that elevator doesn't try to go below allowed amount so it can tilt the grabber up to meet the min X
                x = MIN_X;
            }
        } else {
            if(x > MAX_X) {
                x = MAX_X;
            } else if(x < MIN_X - wristX) {
                x = MIN_X - wristX;
            }
        }

        // Add constraints for allowed y for certain wrist angles
        if(wristAngle > 0) {
            if(y > MAX_Y) {
                y = MAX_Y;
            } else if(y < MIN_Y + wristY) {
                // Makes sure that elevator doesn't try to go below allowed amount so it can tilt the grabber down to meet the max y
                y = MIN_Y + wristY;
            }
        } else {
            if(y > MAX_Y - wristY) {
                // Makes sure that elevator doesn't try to go above allowed amount so it can tilt the grabber up to meet the min y
                y = MAX_Y - wristY;
            } else if(y < MIN_Y) {
                y = MIN_Y;
            }
        }

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
