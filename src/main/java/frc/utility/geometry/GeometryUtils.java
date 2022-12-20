package frc.utility.geometry;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import org.jetbrains.annotations.Contract;
import org.jetbrains.annotations.NotNull;

public final class GeometryUtils {
    private GeometryUtils() {}


    /**
     * Uses atan2 to compute the angle of a translation.
     *
     * @param translation The translation2d to compute the angle of.
     * @return The angle of the translation. (in radians)
     */
    @Contract(value = "_ -> new", pure = true)
    public static @NotNull Rotation2d angleOf(@NotNull Translation2d translation) {
        return new Rotation2d(Math.atan2(translation.getY(), translation.getX()));
    }
}
