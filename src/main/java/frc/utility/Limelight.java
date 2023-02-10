package frc.utility;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import frc.subsytem.AbstractSubsystem;
import org.jetbrains.annotations.Contract;
import org.jetbrains.annotations.NotNull;
import org.joml.Vector2d;

import java.util.EnumSet;
import java.util.HashMap;
import java.util.concurrent.locks.ReentrantReadWriteLock;

/**
 * This class is used to get data from the limelight network tables
 */
public final class Limelight extends AbstractSubsystem {
    public static final double[] EMPTY_DOUBLE_ARRAY = new double[0];
    final @NotNull NetworkTable limelightTable;
    public final @NotNull NetworkTable limelightGuiTable;

    private static final HashMap<String, Limelight> LIMELIGHT_MAP = new HashMap<>();
    private static final ReentrantReadWriteLock LIMELIGHT_MAP_LOCK = new ReentrantReadWriteLock();

    public static @NotNull Limelight getInstance() {
        return getInstance("limelight");
    }


    public enum LimelightResolution {
        k320x240(320, 240),
        k960x720(960, 720);

        LimelightResolution(int width, int height) {
            this.width = width;
            this.height = height;
        }

        final int width;
        final int height;
    }

    public LimelightResolution cameraResolution = LimelightResolution.k320x240;

    public void setCameraResolution(LimelightResolution resolution) {
        cameraResolution = resolution;
    }


    public static @NotNull Limelight getInstance(String name) {
        LIMELIGHT_MAP_LOCK.readLock().lock();
        try {
            if (LIMELIGHT_MAP.containsKey(name)) {
                return LIMELIGHT_MAP.get(name);
            }
        } finally {
            LIMELIGHT_MAP_LOCK.readLock().unlock();
        }

        LIMELIGHT_MAP_LOCK.writeLock().lock();
        try {
            if (LIMELIGHT_MAP.containsKey(name)) {
                return LIMELIGHT_MAP.get(name);
            } else {
                Limelight limelight = new Limelight(name);
                LIMELIGHT_MAP.put(name, limelight);
                return limelight;
            }
        } finally {
            LIMELIGHT_MAP_LOCK.writeLock().unlock();
        }
    }

    @Override
    public void selfTest() {

    }

    @Override
    public void logData() {

    }


    /**
     * Limelight’s LED states
     */
    public enum LedMode {
        /**
         * use mode set in the current pipeline.
         */
        DEFAULT(0),
        /**
         * Force off
         */
        OFF(1),
        /**
         * Force blinking
         */
        BLINK(2),
        /**
         * Force on
         */
        ON(3);

        LedMode(int i) {
            this.i = i;
        }

        private final int i;
    }

    /**
     * limelight’s operation modes
     */
    public enum CamMode {
        VISION_PROCESSOR(0),
        /**
         * Increases exposure, disables vision processing
         */
        DRIVER_CAMERA(1);

        private final int i;

        CamMode(int i) {
            this.i = i;
        }
    }

    /**
     * limelight’s streaming modes
     */
    public enum StreamingMode {
        /**
         * Side-by-side streams if a webcam is attached to Limelight
         */
        STANDARD(0),
        /**
         * The secondary camera stream is placed in the lower-right corner of the primary camera stream
         */
        PIP_MAIN(1),
        /**
         * The primary camera stream is placed in the lower-right corner of the secondary camera stream
         */
        PIP_SECONDARY(2);

        private final int i;

        StreamingMode(int i) {
            this.i = i;
        }
    }

    private Limelight(String name) {
        super();
        limelightTable = NetworkTableInstance.getDefault().getTable(name);
        limelightGuiTable = NetworkTableInstance.getDefault().getTable(name + "gui");

        NetworkTableInstance.getDefault().addListener(limelightTable.getEntry("tl"),
                EnumSet.of(NetworkTableEvent.Kind.kValueRemote), event -> lastUpdate = Timer.getFPGATimestamp());
    }


    /**
     * @return Whether the limelight has any valid targets
     */
    public boolean isTargetVisible() {
        return limelightTable.getEntry("tv").getDouble(0) == 1 && isConnected();
    }

    double lastUpdate = 0;

    public boolean isConnected() {
        //System.out.println(Timer.getFPGATimestamp() - lastUpdate);
        return Timer.getFPGATimestamp() - lastUpdate < 2;
    }

    /**
     * @return Horizontal Offset From Crosshair To Target (LL1: -27 degrees to 27 degrees | LL2: -29.8 to 29.8 degrees)
     */
    public double getHorizontalOffset() {
        return limelightTable.getEntry("tx").getDouble(0) - 1;
    }

    /**
     * @return Vertical Offset From Crosshair To Target (LL1: -20.5 degrees to 20.5 degrees | LL2: -24.85 to 24.85 degrees)
     */
    public double getVerticalOffset() {
        return limelightTable.getEntry("ty").getDouble(0);
    }

    /**
     * @return Target Area (0% of image to 100% of image)
     */
    public double getTargetArea() {
        return limelightTable.getEntry("ta").getDouble(0);
    }

    /**
     * @return Skew or rotation (-90 degrees to 0 degrees)
     */
    public double getTargetSkew() {
        return limelightTable.getEntry("ts").getDouble(0);
    }

    /**
     * @return The pipeline’s latency contribution (ms). Add at least 11ms for image capture latency.
     */
    public double getLatency() {
        return limelightTable.getEntry("tl").getDouble(0);
    }

    /**
     * @return The timeStamp of the last processed frame
     */
    public double getTimestamp() {
        //return Timer.getFPGATimestamp();
        return lastUpdate - ((getLatency()) / 1000);
    }

    /**
     * Sets limelight’s LED state
     */
    public void setLedMode(@NotNull LedMode ledMode) {
        if (limelightGuiTable.getEntry("forceledon").getBoolean(false)) {
            limelightTable.getEntry("ledMode").setNumber(LedMode.ON.i);
        } else {
            limelightTable.getEntry("ledMode").setNumber(ledMode.i);
        }
    }

    /**
     * Sets limelight’s operation mode
     */
    public void setCamMode(@NotNull CamMode camMode) {
        if (limelightGuiTable.getEntry("forceledon").getBoolean(false)) {
            limelightTable.getEntry("camMode").setNumber(0);
        } else {
            limelightTable.getEntry("camMode").setNumber(camMode.i);
        }
    }

    /**
     * Sets limelight’s current pipeline
     *
     * @param pipeline Select pipeline 0...9
     */
    public void setPipeline(int pipeline) {
        limelightTable.getEntry("pipeline").setNumber(pipeline);
    }

    /**
     * Sets limelight’s streaming mode
     */
    public void setStreamingMode(@NotNull StreamingMode streamingMode) {
        limelightTable.getEntry("stream").setNumber(streamingMode.i);
    }

    /**
     * Allows users to take snapshots during a match
     *
     * @param takeSnapshots Take two snapshots per second
     */
    public void takeSnapshots(boolean takeSnapshots) {
        if (takeSnapshots) {
            limelightTable.getEntry("snapshot").setNumber(1);
        } else {
            limelightTable.getEntry("snapshot").setNumber(0);
        }
    }

    @Contract(" -> new")
    public @NotNull Vector2d getTargetPosInCameraPixels() {
        return new Vector2d(
                (getHorizontalOffset() / 29.8) * (cameraResolution.width / 2.0) + (cameraResolution.width / 2.0),
                (getVerticalOffset() / 24.85) * (cameraResolution.height / 2.0) + (cameraResolution.height / 2.0)
        );
    }

    /**
     * Top right is (0,0)
     *
     * @return The position of all the corners in the camera frame.
     */
    public Vector2d[] getCorners() {
        double[] corners = limelightTable.getEntry("tcornxy").getDoubleArray(EMPTY_DOUBLE_ARRAY);
        Vector2d[] processedCorners = new Vector2d[corners.length / 2];
        for (int i = 0; i < corners.length; i += 2) {
            if (i + 1 < corners.length) {
                processedCorners[i / 2] = new Vector2d(corners[i], corners[i + 1]);
            }
        }
        return processedCorners;
    }

    public boolean areCornersTouchingEdge() {
        Vector2d[] corners = getCorners();
        for (Vector2d corner : corners) {
            if (corner.x < 30 || corner.x > cameraResolution.width - 30 || corner.y < 30 || corner.y > cameraResolution.height) {
                return true;
            }
        }
        return false;
    }
}