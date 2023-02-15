package frc.subsytem;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;
import org.littletonrobotics.junction.Logger;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public abstract class AbstractSubsystem {
    private final int loggingInterval;
    private int ticksSinceLastLog;
    private @NotNull ThreadSignal signal = ThreadSignal.PAUSED;
    public final @NotNull String subsystemName;
    private @Nullable Thread thisThread;
    private final @NotNull NetworkTable loggingTable;

    private static final List<AbstractSubsystem> subsystems = Collections.synchronizedList(new ArrayList<>());


    public enum ThreadSignal {
        ALIVE, PAUSED, DEAD
    }

    /**
     *
     */
    public AbstractSubsystem(int loggingInterval) {
        this.subsystemName = this.getClass().getSimpleName();
        this.loggingInterval = loggingInterval;
        this.loggingTable = NetworkTableInstance.getDefault().getTable(subsystemName);
    }

    public AbstractSubsystem() {
        this(Constants.DEFAULT_PERIODS_PER_LOG);
    }

    public void selfTest() {}

    public void logData() {}

    public void pause() {
        signal = ThreadSignal.PAUSED;
    }

    public void kill() {
        signal = ThreadSignal.DEAD;
    }

    public void start() {
        subsystems.add(this);
        signal = ThreadSignal.ALIVE;
    }

    public static void tick() {
        for (AbstractSubsystem subsystem : subsystems) {
            double startTime = Logger.getInstance().getRealTimestamp();
            if (subsystem.signal == ThreadSignal.ALIVE) {
                subsystem.update();

                subsystem.ticksSinceLastLog++;
                if (subsystem.ticksSinceLastLog > subsystem.loggingInterval) {
                    subsystem.logData();
                    subsystem.ticksSinceLastLog = 0;
                }
            }
            double executionTimeMS = (Logger.getInstance().getRealTimestamp() - startTime) * 0.001; //micoseconds to ms
            Logger.getInstance().recordOutput("SubsystemExecutionTimes/" + subsystem.subsystemName + " Execution Time",
                    executionTimeMS);
        }
    }


    /**
     * This function will be called repeatedly when the thread is alive. The period will be whatever you defined when creating the
     * object
     */
    public void update() {

    }
}
