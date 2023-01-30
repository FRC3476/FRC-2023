package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import frc.utility.Controller;
import org.jetbrains.annotations.Contract;
import org.jetbrains.annotations.NotNull;
import org.joml.Intersectiond;
import org.joml.Vector2d;

import static frc.robot.Constants.*;

public class ScoringPositionManager {

    private static final double CENTER_CUBE_SCORING_PLATFORM_TO_NODE_Y = 0.56;
    /**
     * The Y coordinate of the center of the cube scoring platforms
     */
    private static final double[] CUBE_SCORING_Y_CENTER = {-0.415, 1.26, 2.94};
    private static ScoringPositionManager instance = new ScoringPositionManager();

    public static ScoringPositionManager getInstance() {
        return instance;
    }

    private ScoringPositionManager() {
    }


    public enum PositionType {
        CONE, CUBE, BOTH
    }


    /**
     * Represents scoring position on the field. TOP, MIDDLE, BOTTOM represent the three rows of scoring positions. (TOP is the
     * one with the highest score value)
     * <p>
     * LEFT, CENTER, RIGHT are the three columns of scoring positions. <b>LEFT/RIGHT is relative to the direction the robot would
     * be facing when it is scoring in that position</b>.
     * <p>
     * On the red alliance, the LEFT is a more negative Y value, and the RIGHT is a more positive Y value.
     * <p>
     * On the blue alliance, the LEFT is a more positive Y value, and the RIGHT is a more negative Y value.
     */
    public enum SelectedPosition {
        // TODO: make sure these are correct
        BOTTOM_LEFT(1, PositionType.BOTH),
        BOTTOM_MIDDLE(2, PositionType.BOTH),
        BOTTOM_RIGHT(3, PositionType.BOTH),

        MIDDLE_LEFT(5, PositionType.CONE),
        MIDDLE_MIDDLE(6, PositionType.CUBE),
        MIDDLE_RIGHT(7, PositionType.CONE),

        TOP_LEFT(9, PositionType.CONE),
        TOP_MIDDLE(10, PositionType.CUBE),
        TOP_RIGHT(11, PositionType.CONE);


        /**
         * @return The level of the scoring position. 0 is the lowest, 2 is the highest.
         */
        public int getLevel() {
            return ordinal() / 3; //always rounds down
        }

        /**
         * @return 0 for left, 1 for middle, 2 for right
         */
        public int getSide() {
            return ordinal() % 3;
        }

        public final int buttonPanelIndex;
        public final PositionType positionType;

        SelectedPosition(int buttonPanelIndex, PositionType positionType) {
            this.buttonPanelIndex = buttonPanelIndex;
            this.positionType = positionType;
        }
    }

    private SelectedPosition selectedPosition = SelectedPosition.TOP_LEFT;

    /**
     * @param buttonPanel the controller that is used to select the position
     * @return if the position was changed
     */
    public boolean updateSelectedPosition(Controller buttonPanel) {
        SelectedPosition oldSelectedPosition = selectedPosition;
        for (SelectedPosition value : SelectedPosition.values()) {
            if (buttonPanel.getRisingEdge(value.buttonPanelIndex)) {
                selectedPosition = value;
            }
        }
        return oldSelectedPosition != selectedPosition;
    }

    public SelectedPosition getSelectedPosition() {
        return selectedPosition;
    }

    public void setSelectedPosition(SelectedPosition selectedPosition) {
        this.selectedPosition = selectedPosition;
    }

    /**
     * @param selectedPosition The position to get the Y coordinate of
     * @param isRedAlliance    Whether the robot is on the red alliance
     * @return The Y coordinate of the scoring location relative to the center of the cube scoring locations
     */
    @Contract(pure = true)
    public static double getGridRelativeY(@NotNull SelectedPosition selectedPosition, boolean isRedAlliance) {
        double y = (selectedPosition.getSide() - 1) * CENTER_CUBE_SCORING_PLATFORM_TO_NODE_Y;
        if (!isRedAlliance) {
            y *= -1;
        }
        return y;
    }

    /**
     * @param selectedPosition Currently selected position
     * @param isRedAlliance    Whether the robot is on the red alliance
     * @return An array of 3 Y coordinates of where the robot could be placed in order to score in the selected position
     */
    @Contract(pure = true)
    public static double @NotNull [] getPossibleFieldYs(@NotNull SelectedPosition selectedPosition, boolean isRedAlliance) {
        double[] possibleYs = new double[3];
        double y = getGridRelativeY(selectedPosition, isRedAlliance);
        for (int i = 0; i < 3; i++) {
            possibleYs[i] = y + CUBE_SCORING_Y_CENTER[i];
        }
        return possibleYs;
    }

    /**
     * Returns the Y coordinate that the robot should be at in order to score in the selected position that the driver most likely
     * wants based on their current position and velocity.
     * <p>
     * The code uses the following algorithm: It generates a line from the robot's current position to the robot's current
     * position + the robot's current velocity. It then finds the intersection of that line with the three possible Y coordinates
     * of the scoring position. It then finds the cosest of the the three possible Y coordinates to the intersection.
     *
     * @param selectedPosition Currently selected position
     * @param isRedAlliance    Whether the robot is on the red alliance
     * @param robotPosition    The position of the robot
     * @param robotVelocity    The velocity of the robot
     * @return The Y position the robot should be at in order to score in the selected position that the driver most likely wants
     */
    @Contract(pure = true)
    public static double getBestFieldY(@NotNull SelectedPosition selectedPosition, boolean isRedAlliance,
                                       @NotNull Translation2d robotPosition, @NotNull Translation2d robotVelocity) {
        double intersectionXLine;

        if (isRedAlliance) {
            intersectionXLine = GRIDS_RED_X + INTERSECTION_TEST_LINE_X_OFFSET;
        } else {
            intersectionXLine = GRIDS_BLUE_X - INTERSECTION_TEST_LINE_X_OFFSET;
        }

        Vector2d intersection = new Vector2d();
        Intersectiond.intersectLineLine(
                // Create a line that includes the robot's position and is in the direction of the robot's velocity
                robotPosition.getX(), robotPosition.getY(),
                robotPosition.getX() + robotVelocity.getX(), robotPosition.getY() + robotVelocity.getY(),

                // Create a line  parallel to the y axis that is at the intersectionXLine
                intersectionXLine, 0,
                intersectionXLine, 3,
                intersection
        );

        double[] possibleYs = getPossibleFieldYs(selectedPosition, isRedAlliance);

        double bestY = possibleYs[0];

        // Find the closest possible Y to the intersection that we found.
        for (double possibleY : possibleYs) {
            if (Math.abs(possibleY - intersection.y) < Math.abs(bestY - intersection.y)) {
                bestY = possibleY;
            }
        }

        return bestY;
    }
}
