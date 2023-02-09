package frc.subsytem;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public abstract class AbstractSubsystem {
    private final int period;
    private final int loggingInterval;
    private int logInterval;
    private @NotNull ThreadSignal signal = ThreadSignal.PAUSED;
    public final @NotNull String subsystemName;
    private @Nullable Thread thisThread;
    private final @NotNull NetworkTable loggingTable;

    private static final List<AbstractSubsystem> subsystems = Collections.synchronizedList(new ArrayList<>());


    public enum ThreadSignal {
        ALIVE, PAUSED, DEAD
    }

    /**
     * @param period The period when calling update
     */
    public AbstractSubsystem(int period, int loggingInterval) {
        this.period = period;
        this.subsystemName = this.getClass().getSimpleName();
        this.loggingInterval = loggingInterval;
        this.loggingTable = NetworkTableInstance.getDefault().getTable(subsystemName);
    }

    public AbstractSubsystem(int period) {
        this(period, Constants.DEFAULT_PERIODS_PER_LOG);
    }

    public void selfTest() {}

    public void logData() {}

    /**
     * @deprecated Use {@link Logger#recordOutput(String, long)} instead
     */
    @Deprecated(forRemoval = true)
    public void logData(@NotNull String key, @NotNull Object value) {
        loggingTable.getEntry(key).setValue(value);
    }

    public void pause() {
        signal = ThreadSignal.PAUSED;
    }

    public void kill() {
        signal = ThreadSignal.DEAD;
    }

    public void start() {
        subsystems.add(this);
    }

    public static void tick() {
        for (AbstractSubsystem subsystem : subsystems) {
            double startTime = Timer.getFPGATimestamp();
            if (subsystem.signal == ThreadSignal.ALIVE) {
                subsystem.update();

                subsystem.logInterval++;
                if (subsystem.logInterval > subsystem.loggingInterval) {
                    subsystem.logData();
                    subsystem.logInterval = 0;
                }
            }
            double executionTimeMS = (Timer.getFPGATimestamp() - startTime) * 1000;
            subsystem.logData("Execution Time", executionTimeMS);
        }
    }


    /**
     * This function will be called repeatedly when the thread is alive. The period will be whatever you defined when creating the
     * object
     */
    public void update() {

    }
}
