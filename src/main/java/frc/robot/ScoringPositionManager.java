package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import frc.utility.Controller;
import org.jetbrains.annotations.Contract;
import org.jetbrains.annotations.NotNull;
import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;
import org.littletonrobotics.junction.networktables.LoggedDashboardString;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import static frc.robot.Constants.IS_PRACTICE;

public class ScoringPositionManager {

    private static final double CENTER_CUBE_SCORING_PLATFORM_TO_NODE_Y = 0.56;
    /**
     * The Y coordinate of the center of the cube scoring platforms
     */
    private static final double[] CUBE_SCORING_Y_CENTER = {
            -0.415,
            1.26,
            2.94
    };
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
        BOTTOM_LEFT(3, PositionType.BOTH),
        BOTTOM_MIDDLE(2, PositionType.BOTH),
        BOTTOM_RIGHT(1, PositionType.BOTH),

        MIDDLE_LEFT(7, PositionType.CONE),
        MIDDLE_MIDDLE(6, PositionType.CUBE),
        MIDDLE_RIGHT(5, PositionType.CONE),

        TOP_LEFT(9, PositionType.CONE),
        TOP_MIDDLE(12, PositionType.CUBE),
        TOP_RIGHT(11, PositionType.CONE);

        /**
         * @return The level of the scoring position. 0 is the lowest, 2 is the highest.
         */
        public int getLevel() {
            return ordinal() / 3; //always rounds down
        }

        /**
         * @return The column of the scoring position.
         */
        public ScoringDirection getScoringDirection() {
            return ScoringDirection.values()[ordinal() % 3];
        }

        public final int buttonPanelIndex;
        public final PositionType positionType;

        SelectedPosition(int buttonPanelIndex, PositionType positionType) {
            this.buttonPanelIndex = buttonPanelIndex;
            this.positionType = positionType;
        }
    }

    private SelectedPosition selectedPosition = SelectedPosition.TOP_LEFT;

    private final LoggedDashboardBoolean[] selectedPositions = new LoggedDashboardBoolean[9];

    {
        for (SelectedPosition value : SelectedPosition.values()) {
            selectedPositions[value.ordinal()] = new LoggedDashboardBoolean("Selected Position " + value.name(), false);
        }
    }

    private PositionType wantedPositionType = PositionType.CONE;

    LoggedDashboardBoolean isCone = new LoggedDashboardBoolean("Is Cone", true);
    LoggedDashboardBoolean isCube = new LoggedDashboardBoolean("Is Cube", false);
    LoggedDashboardString wantedScoringPosition = new LoggedDashboardString("Wanted Scoring Position", selectedPosition.name());

    LoggedDashboardBoolean doesWantedPositionTypeMatchSelectedPositionType =
            new LoggedDashboardBoolean("Does Wanted Position Type Match Selected Position Type", true);


    private final List<LoggedDashboardNumber> yPositionErrors = List.of(
            new LoggedDashboardNumber("Y Position Error Grid 0", 0),
            new LoggedDashboardNumber("Y Position Error Grid 1", 0),
            new LoggedDashboardNumber("Y Position Error Grid 2", 0)
    );

    public PositionType getWantedPositionType() {
        return wantedPositionType;
    }

    public boolean doesWantedPositionTypeMatchSelectedPositionType() {
        if (selectedPosition.positionType == PositionType.BOTH) {
            return true;
        }
        return wantedPositionType == selectedPosition.positionType;
    }

    /**
     * @param buttonPanel the controller that is used to select the position
     * @return if the position was changed
     */
    public boolean updateSelectedPosition(Controller buttonPanel) {
        SelectedPosition oldSelectedPosition = selectedPosition;
        for (SelectedPosition value : SelectedPosition.values()) {
            if (buttonPanel.getRisingEdge(value.buttonPanelIndex)) {
                selectedPosition = value;

                setCone(selectedPosition.positionType == PositionType.CONE);
            }
        }

        if (buttonPanel.getRisingEdge(4)) {
            wantedPositionType = PositionType.CONE;
            setCone(true);
        } else if (buttonPanel.getRisingEdge(8)) {
            wantedPositionType = PositionType.CUBE;
            setCone(false);
        }

        wantedScoringPosition.set(selectedPosition.name());

        if (oldSelectedPosition != selectedPosition) {
            selectedPositions[oldSelectedPosition.ordinal()].set(false);
            selectedPositions[selectedPosition.ordinal()].set(true);
            doesWantedPositionTypeMatchSelectedPositionType.set(doesWantedPositionTypeMatchSelectedPositionType());
        }

        double robotY = Robot.getRobotTracker().getLatestPose3d().getY();

        double[] possibleFieldYs = getPossibleFieldYs(selectedPosition, Robot.isRed());
        for (int i = 0; i < 3; i++) {
            yPositionErrors.get(i).set(robotY - possibleFieldYs[i]);
        }


        return oldSelectedPosition != selectedPosition;
    }

    private void setCone(boolean isCone) {
        this.isCone.set(isCone);
        this.isCube.set(!isCone);
        wantedPositionType = isCone ? PositionType.CONE : PositionType.CUBE;
        doesWantedPositionTypeMatchSelectedPositionType.set(doesWantedPositionTypeMatchSelectedPositionType());
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
        double offset = IS_PRACTICE ? 0.012 : 0;

        double y = (selectedPosition.getScoringDirection().ordinal() - 1) * CENTER_CUBE_SCORING_PLATFORM_TO_NODE_Y + offset;
        if (!isRedAlliance) {
            y *= -1;
        }
        return y;
    }

    enum AllianceSide {
        RED, BLUE
    }

    /**
     * Position of the scoring column relative to the robot. <b>LEFT/RIGHT is relative to the direction the robot would be facing
     * when it is scoring in that position</b>.
     */
    enum ScoringDirection {
        LEFT, MIDDLE, RIGHT
    }

    // Map<AllianceSide, Map<Grid Index (Always starts from -y), Map<Scoring Column, Offset>>>
    static final Map<AllianceSide, HashMap<Integer, HashMap<ScoringDirection, Double>>> yScoringOffsets;

    static {
        yScoringOffsets = Map.of(
                AllianceSide.RED, new HashMap<>() {{
                    /*
                    RED          SCORING TABLE           BLUE
                    -                                    -
                    -                                    -
                    -                                    -

                    -                                    -
                    -                                    -
                    -                                    -

                    R                                    -
                    M                                    -
                    L                                    -
                    */
                    put(0, new HashMap<>() {{
                        put(ScoringDirection.LEFT, -0.018);
                        put(ScoringDirection.MIDDLE, 0.0);
                        put(ScoringDirection.RIGHT, 0.0);
                    }});


                    /*
                    RED          SCORING TABLE           BLUE
                    -                                    -
                    -                                    -
                    -                                    -

                    R                                    -
                    M                                    -
                    L                                    -

                    -                                    -
                    -                                    -
                    -                                    -
                    */
                    put(1, new HashMap<>() {{
                        put(ScoringDirection.LEFT, 0.0);
                        put(ScoringDirection.MIDDLE, 0.0);
                        put(ScoringDirection.RIGHT, 0.0);
                    }});

                    /*
                    RED          SCORING TABLE           BLUE
                    R                                    -
                    M                                    -
                    L                                    -

                    -                                    -
                    -                                    -
                    -                                    -

                    -                                    -
                    -                                    -
                    -                                    -
                    */
                    put(2, new HashMap<>() {{
                        put(ScoringDirection.LEFT, 0.0);
                        put(ScoringDirection.MIDDLE, 0.0);
                        put(ScoringDirection.RIGHT, 0.05);
                    }});
                }},

                AllianceSide.BLUE, new HashMap<>() {{
                    /*
                    RED          SCORING TABLE           BLUE
                    -                                    -
                    -                                    -
                    -                                    -

                    -                                    -
                    -                                    -
                    -                                    -

                    -                                    L
                    -                                    M
                    -                                    R
                    */
                    put(0, new HashMap<>() {{
                        put(ScoringDirection.LEFT, 0.0);
                        put(ScoringDirection.MIDDLE, 0.0);
                        put(ScoringDirection.RIGHT, -0.04);
                    }});
                    /*
                    RED          SCORING TABLE           BLUE
                    -                                    -
                    -                                    -
                    -                                    -

                    -                                    L
                    -                                    M
                    -                                    R

                    -                                    -
                    -                                    -
                    -                                    -
                    */
                    put(1, new HashMap<>() {{
                        put(ScoringDirection.LEFT, 0.0);
                        put(ScoringDirection.MIDDLE, 0.0);
                        put(ScoringDirection.RIGHT, 0.0);
                    }});
                    /*
                    RED          SCORING TABLE           BLUE
                    -                                    L
                    -                                    M
                    -                                    R

                    -                                    -
                    -                                    -
                    -                                    -

                    -                                    -
                    -                                    -
                    -                                    -
                    */
                    put(2, new HashMap<>() {{
                        put(ScoringDirection.LEFT, 0.0);
                        put(ScoringDirection.MIDDLE, 0.0);
                        put(ScoringDirection.RIGHT, 0.0);
                    }});
                }}
        );
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
            possibleYs[i] = CUBE_SCORING_Y_CENTER[i] +
                    y +
                    yScoringOffsets
                            .get(isRedAlliance ? AllianceSide.RED : AllianceSide.BLUE)
                            .get(i)
                            .get(selectedPosition.getScoringDirection());
        }
        return possibleYs;
    }

    record BestFieldY(double y, int gridIndex) {}

    /**
     * Returns the Y coordinate that the robot should be at in order to score in the selected position that the driver most likely
     * wants based on their current position and velocity.
     * <p>
     * The code uses the following algorithm: It generates a line from the robot's current position to the robot's current
     * position + the robot's current velocity. It then finds the intersection of that line with the three possible Y coordinates
     * of the scoring position. It then finds the closest of the three possible Y coordinates to the intersection.
     *
     * @param selectedPosition Currently selected position
     * @param isRedAlliance    Whether the robot is on the red alliance
     * @param robotPosition    The position of the robot
     * @param robotVelocity    The velocity of the robot
     * @return The Y position the robot should be at in order to score in the selected position that the driver most likely wants
     */
    @Contract(pure = true)
    public static BestFieldY getBestFieldY(@NotNull SelectedPosition selectedPosition, boolean isRedAlliance,
                                           @NotNull Translation2d robotPosition, @NotNull Translation2d robotVelocity,
                                           boolean forceGrid, int forcedGridIndex) {

        var predictedRobotY = robotPosition.getY() + robotVelocity.getY() * 0.5;
        double[] possibleYs = CUBE_SCORING_Y_CENTER;
        double bestY = possibleYs[0];
        int chosenGridIndex = 0;

        if (forceGrid) {
            bestY = possibleYs[forcedGridIndex];
            chosenGridIndex = forcedGridIndex;
        } else {
            // Find the closest possible Y to the intersection that we found.
            for (int i = 0; i < 3; i++) {
                double possibleY = possibleYs[i];
                if (Math.abs(possibleY - predictedRobotY) < Math.abs(bestY - predictedRobotY)) {
                    bestY = possibleY;
                    chosenGridIndex = i;
                }
            }
        }

        bestY = bestY +
                getGridRelativeY(selectedPosition, isRedAlliance) +
                yScoringOffsets
                        .get(isRedAlliance ? AllianceSide.RED : AllianceSide.BLUE)
                        .get(chosenGridIndex)
                        .get(selectedPosition.getScoringDirection());


        return new BestFieldY(bestY, chosenGridIndex);
    }
}
