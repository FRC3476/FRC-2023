package frc.subsytem;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTablesJNI;
import frc.robot.Constants;
import org.jetbrains.annotations.NotNull;

/**
 * Gives pose estimates based on april tag vision
 */
public class VisionHandler extends AbstractSubsystem {

    // Network tables
    private final @NotNull NetworkTableInstance networkTableInstance;
    private NetworkTable visionTable;

    private static VisionHandler instance;

    public static VisionHandler getInstance() {
        if(instance == null) {
            instance = new VisionHandler(Constants.VISION_HANDLER_PERIOD, 5);
        }

        return instance;
    }
    public VisionHandler(int period, int loggingInterval) {
        super(period, loggingInterval);

        // Network table init
        networkTableInstance = NetworkTableInstance.getDefault();
        visionTable = networkTableInstance.getTable("Vision");
    }

    @Override
    public void update() {

    }

    @Override
    public void selfTest() {

    }

    @Override
    public void logData() {

    }
}
