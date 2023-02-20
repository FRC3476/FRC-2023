package frc.utility.net.editing;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import org.jetbrains.annotations.NotNull;

import java.util.EnumSet;
import java.util.function.Function;

public class LiveEditableValue<T> {
    private volatile T value;
    private final @NotNull NetworkTableEntry entry;

    private final @NotNull Function<? super T, Object> onWrite;

    /**
     * @param defaultValue The default value to use if the entry is not changed / the initial value set to the table
     * @param entry        The entry to listen to
     * @param onNTChange   The function to call when the entry is changed through the network table to convert the value to
     *                     {@link T}
     * @param onWrite      The function to convert to a NT object when {@link #set(T)} is called
     */
    public LiveEditableValue(@NotNull T defaultValue, @NotNull NetworkTableEntry entry,
                             @NotNull Function<Object, ? extends T> onNTChange,
                             @NotNull Function<? super T, Object> onWrite) {
        this.onWrite = onWrite;
        this.value = defaultValue;
        this.entry = entry;
        entry.setValue(onWrite.apply(defaultValue));
        NetworkTableInstance.getDefault().addListener(entry, EnumSet.of(NetworkTableEvent.Kind.kValueRemote),
                (NetworkTableEvent) -> value = onNTChange.apply(entry.getValue().getValue()));
    }

    /**
     * Create a new {@link LiveEditableValue} with the onNTChange and onWrite to simply read and write the object to NT without
     * any conversion
     *
     * @param defaultValue The default value to use if the entry is not changed / the initial value set to the table
     * @param entry        The entry to listen to
     */
    public LiveEditableValue(T defaultValue, NetworkTableEntry entry) {
        this(defaultValue, entry, value -> (T) value, value -> value);
    }

    /**
     * @return The current value
     */
    public T get() {
        return value;
    }

    /**
     * @param value The new value to set
     */
    public void set(T value) {
        this.value = value;
        entry.setValue(onWrite.apply(value));
    }
}
