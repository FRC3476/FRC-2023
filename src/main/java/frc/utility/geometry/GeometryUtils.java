package frc.utility.geometry;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import org.jetbrains.annotations.Contract;
import org.jetbrains.annotations.NotNull;

public final class GeometryUtils {
    private GeometryUtils() {}

    /**
     * Returns the squared distance between two points.
     *
     * @param first  The first point.
     * @param second The second point.
     * @return The squared distance between the two points.
     */
    @SuppressWarnings("ParameterName")
    public static double dist2(@NotNull Translation2d first, @NotNull Translation2d second) {
        return Math.pow(first.getX() - second.getX(), 2) + Math.pow(first.getY() - second.getY(), 2);
    }

    /**
     * Returns the squared distance between two points.
     *
     * @param first  The first point.
     * @param second The second point.
     * @return The squared distance between the two points.
     */
    @SuppressWarnings("ParameterName")
    public static double dist2(@NotNull Translation2d first, @NotNull MutableTranslation2d second) {
        return Math.pow(first.getX() - second.getX(), 2) + Math.pow(first.getY() - second.getY(), 2);
    }

    /**
     * Returns the squared distance between two points.
     *
     * @param first  The first point.
     * @param second The second point.
     * @return The squared distance between the two points.
     */
    @SuppressWarnings("ParameterName")
    public static double dist2(@NotNull MutableTranslation2d first, @NotNull Translation2d second) {
        return Math.pow(first.getX() - second.getX(), 2) + Math.pow(first.getY() - second.getY(), 2);
    }

    /**
     * Returns the squared distance between two points.
     *
     * @param first  The first point.
     * @param second The second point.
     * @return The squared distance between the two points.
     */
    @SuppressWarnings("ParameterName")
    public static double dist2(@NotNull MutableTranslation2d first, @NotNull MutableTranslation2d second) {
        return Math.pow(first.getX() - second.getX(), 2) + Math.pow(first.getY() - second.getY(), 2);
    }

    public static double dist2(@NotNull Translation2d translation2d) {
        double x = translation2d.getX();
        double y = translation2d.getY();
        return x * x + y * y;
    }

    public static double dist2(@NotNull Translation3d translation3d) {
        double x = translation3d.getX();
        double y = translation3d.getY();
        double z = translation3d.getZ();
        return x * x + y * y + z * z;
    }


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
