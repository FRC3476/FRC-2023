package frc.subsytem;

import edu.wpi.first.networktables.*;
import frc.robot.Constants;
import org.jetbrains.annotations.NotNull;

import java.util.EnumSet;

/**
 * Gives pose estimates based on april tag vision
 */
public class VisionHandler extends AbstractSubsystem {

    // Network tables
    private final @NotNull NetworkTableInstance networkTableInstance;
    private NetworkTable visionTable;
    private NetworkTable visionMiscTable;
    private NetworkTable configTable;

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
        visionMiscTable = networkTableInstance.getTable("Vision Misc");
        configTable = networkTableInstance.getTable("Vision Config");

        // Send out connection flag to april tags processor
        visionMiscTable.getEntry("Connection Flag").setBoolean(true);

        configTable.getEntry("Exposure").setDouble(100);
        configTable.getEntry("Camera Type").setDouble(0);
        configTable.getEntry("X Resolution").setDouble(1280);
        configTable.getEntry("Y Resolution").setDouble(800);
        configTable.getEntry("Framerate").setDouble(30);
        configTable.getEntry("Threads").setDouble(4);
        configTable.getEntry("Do Stream").setBoolean(false);

        visionTable.addListener(EnumSet.of(NetworkTableEvent.Kind.kValueAll), (table, key, event) -> {

        });
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
