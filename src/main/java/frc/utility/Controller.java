// Copyright 2019 FRC Team 3476 Code Orange

package frc.utility;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import org.jetbrains.annotations.Contract;

/**
 * This class stores the int sent back from the Driver Station and uses it to check for rising or falling edges
 */
public class Controller extends Joystick {

    public static final class XboxButtons {
        public static final int A = 1;
        public static final int B = 2;
        public static final int X = 3;
        public static final int Y = 4;
        public static final int LEFT_BUMPER = 5;
        public static final int RIGHT_BUMPER = 6;
        public static final int START = 7;
        public static final int BACK = 8;
        public static final int LEFT_CLICK = 9;
        public static final int RIGHT_CLICK = 10;
    }

    public static final class XboxAxes {
        public static final int LEFT_X = 0;
        public static final int LEFT_Y = 1;
        public static final int LEFT_TRIGGER = 2;
        public static final int RIGHT_TRIGGER = 3;
        public static final int RIGHT_X = 4;
        public static final int RIGHT_Y = 5;
    }

    /*
     * The Driver Station sends back an int(32 bits) for buttons Shifting 1 left
     * (button - 1) times and ANDing it with the int sent from the Driver
     * Station will either give you 0 or a number not zero if it is true
     */
    private int oldButtons;
    private int currentButtons;
    private int axisCount, povCount;
    private double[] oldAxis;
    private double[] currentAxis;
    private int[] currentPOV;

    public Controller(int port) {
        super(port);
        axisCount = DriverStation.getStickAxisCount(port);
        povCount = DriverStation.getStickPOVCount(port);
        oldAxis = new double[axisCount];
        currentAxis = new double[axisCount];
        currentPOV = new int[povCount];
    }

    /**
     * Only works if update() is called in each iteration
     *
     * @param button Joystick button ID
     * @return Falling edge state of the button
     */
    @Contract(pure = true)
    public boolean getFallingEdge(int button) {
        boolean oldVal = getButtonState(button, oldButtons);
        boolean currentVal = getButtonState(button, currentButtons);
        return oldVal && !currentVal;
    }

    /**
     * Only works if update() is called in each iteration
     *
     * @param button Joystick button ID
     * @return Rising edge state of the button
     */
    @Contract(pure = true)
    public boolean getRisingEdge(int button) {
        boolean oldVal = getButtonState(button, oldButtons);
        boolean currentVal = getButtonState(button, currentButtons);
        return !oldVal && currentVal;
    }

    @Contract(pure = true)
    public boolean getRisingEdge(int axis, double threshold) {
        if (axis <= axisCount) {
            boolean oldVal = oldAxis[axis] > threshold;
            boolean currentVal = currentAxis[axis] > threshold;
            return !oldVal && currentVal;
        }
        return false;
    }

    @Contract(pure = true)
    public boolean getFallingEdge(int axis, double threshold) {
        if (axis <= axisCount) {
            boolean oldVal = oldAxis[axis] > threshold;
            boolean currentVal = currentAxis[axis] > threshold;
            return oldVal && !currentVal;
        }
        return false;
    }

    /**
     * This method needs to be called for each iteration of the teleop loop
     */
    public void update() {
        oldButtons = currentButtons;
        currentButtons = DriverStation.getStickButtons(getPort());

        // Reset the axis and pov arrays if the number of axes or povs changes
        if (axisCount != DriverStation.getStickAxisCount(getPort())) {
            axisCount = DriverStation.getStickAxisCount(getPort());
            oldAxis = new double[axisCount];
            currentAxis = new double[axisCount];
        }
        if (povCount != DriverStation.getStickPOVCount(getPort())) {
            povCount = DriverStation.getStickPOVCount(getPort());
            currentPOV = new int[povCount];
        }

        // Copy the current axis into the old axis array
        System.arraycopy(currentAxis, 0, oldAxis, 0, axisCount);

        // Get the current axis and pov values
        for (int i = 0; i < axisCount; i++) {
            currentAxis[i] = DriverStation.getStickAxis(getPort(), i);
        }

        // Get the current pov values
        for (int i = 0; i < povCount; i++) {
            currentPOV[i] = DriverStation.getStickPOV(getPort(), i);
        }
    }

    @Override
    @Contract(pure = true)
    public boolean getRawButton(int button) {
        return getButtonState(button, currentButtons);
    }

    @Override
    @Contract(pure = true)
    public double getRawAxis(int axis) {
        try {
            if (axis <= axisCount && axis >= 0) {
                return currentAxis[axis];
            }
        } catch (ArrayIndexOutOfBoundsException e) {
            //System.out.println("Axis out of bounds " + axis + "/" + axisCount);
        }
        return 0;
    }

    @Override
    @Contract(pure = true)
    public int getPOV(int pov) {
        if (pov < povCount && pov >= 0) {
            return currentPOV[pov];
        }
        return -1;
    }

    @SuppressWarnings("SpellCheckingInspection")
    @Contract(pure = true)
    public int getAxesAsPOV(int x, int y, boolean xinv, boolean yinv) {
        try {
            if (x <= axisCount && x >= 0 && y <= axisCount && y >= 0) {
                int xval = (int) Math.round(currentAxis[x]) * (xinv ? -1 : 1),
                        yval = (int) Math.round(currentAxis[y]) * (yinv ? -1 : 1);
                if (xval == 0 && yval == 0) return -1;
                int val = (int) (Math.atan2(yval, xval) * 180.0 / Math.PI);
                if (val < 0) val += 360;
                return val;
            }
        } catch (ArrayIndexOutOfBoundsException e) { //noinspection ProhibitedExceptionCaught
            //System.out.println("Axes out of bounds " + x + " " + y + "/" + axisCount);
        }
        return -1;
    }

    public boolean getButtonState(int button, int state) {
        return ((0x1 << (button - 1)) & state) != 0;
    }
}
