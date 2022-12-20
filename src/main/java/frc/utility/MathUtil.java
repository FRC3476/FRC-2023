package frc.utility;

import edu.wpi.first.math.geometry.Translation2d;
import frc.utility.geometry.MutableTranslation2d;
import org.jetbrains.annotations.NotNull;

public final class MathUtil {
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
}
