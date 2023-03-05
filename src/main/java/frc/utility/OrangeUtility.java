// Copyright 2019 FRC Team 3476 Code Orange

package frc.utility;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import frc.utility.geometry.MutableTranslation2d;
import org.jetbrains.annotations.NotNull;

import java.util.regex.Matcher;
import java.util.regex.Pattern;

import static frc.robot.Constants.FIELD_HEIGHT_METERS;

public final class OrangeUtility {

    /**
     * Extracts the double value from a string.
     *
     * @param mess the string to be parsed
     * @return the double value extracted from the string
     */
    public static double cleanDoubleParse(String mess)// TODO: encapsulate
    // special replaceall
    {
        String result = mess;
        result = result.toUpperCase();
        result = result.replaceAll("[^\\d.\\-E]", "");// remove all illegal
        // characters
        int e = result.indexOf("E");
        if (e != -1) {
            String before = result.substring(0, e + 1), after = result.substring(e + 1);
            before = OrangeUtility.removeExtraInstances(before, "\\.");
            before = OrangeUtility.removeExtraInstances(before, "-");
            after = OrangeUtility.removeExtraInstances(after, "\\.");
            after = OrangeUtility.removeExtraInstances(after, "-");

            result = before + after;
        } else// no e
        {
            result = OrangeUtility.removeExtraInstances(result, "\\.");
            result = OrangeUtility.removeExtraInstances(result, "-");
        }

        return Double.parseDouble(result);
    }

    /**
     * Extracts the int value from a string.
     *
     * @param mess the string to be parsed
     * @return the int value extracted from the string
     */
    public static int cleanIntParse(String mess)// TODO: encapsulate special
    // replaceall
    {
        String result = mess;
        result = result.toUpperCase();
        result = result.replaceAll("[^\\d]", "");// remove all illegal
        // characters

        return Integer.parseInt(result);
    }

    /**
     * Keeps a value in a range by truncating it.
     *
     * @param toCoerce the value to coerce
     * @param high     the high value of the range
     * @param low      the low value of the range
     * @return the coerced value
     */
    public static double coerce(double toCoerce, double high, double low) {
        if (toCoerce > high) {
            return high;
        } else if (toCoerce < low) {
            return low;
        }
        return toCoerce;
    }

    /**
     * Donuts an input value so that a control loop can overcome backlash or friction.
     *
     * @param toDonut   the input value
     * @param threshold the backlash or friction scalar
     * @return the adjusted "input" value for evaluation by the control loop
     */
    public static double donut(double toDonut, double threshold) {
        if (toDonut == 0) {
            return 0;
        }
        if (toDonut > 0) {
            return toDonut + threshold;
        }
        return toDonut - threshold;
    }

    public static final double EPSILON = 1.0E-14;

    public static boolean doubleEqual(double a, double b) {
        return Math.abs(a - b) < EPSILON;
    }

    public static boolean doubleEqual(double a, double b, double epsilon) {
        return Math.abs(a - b) < epsilon;
    }

    public static double getArrayMax(double @NotNull [] array) {
        double result = array[0];
        for (double e : array) {
            if (e > result) {
                result = e;
            }
        }
        return result;
    }

    public static double getArrayMin(double @NotNull [] array) {
        double result = array[0];
        for (double e : array) {
            if (e < result) {
                result = e;
            }
        }
        return result;
    }

    public static boolean inRange(double test, double a, double b) {
        return OrangeUtility.inRange(test, new double[]{a, b});
    }

    public static boolean inRange(double test, double @NotNull [] range) {
        return test > OrangeUtility.getArrayMin(range) && test < OrangeUtility.getArrayMax(range);
    }

    public static double normalize(double toNormalize, double fromHigh, double fromLow, double toHigh, double toLow) {
        double factor = (toHigh - toLow) / (fromHigh - fromLow);
        double add = toLow - fromLow * factor;
        return toNormalize * factor + add;
    }

    public static double coercedNormalize(double rawValue, double minInput, double maxInput, double minOutput,
                                          double maxOutput) {
        if (rawValue < minInput) {
            return minOutput;
        } else if (rawValue > maxInput) {
            return maxOutput;
        }
        double norm = (Math.abs(rawValue) - minInput) / (maxInput - minInput);
        norm = Math.copySign(norm * (maxOutput - minOutput), rawValue) + minOutput;
        return norm;
    }


    /**
     * Removes all instances of toReplace after the first. If toReplace does not occur, input is returned unchanged.
     *
     * @param input        the String to operate on
     * @param replaceRegex the regex to remove extra instances of
     * @return the resulting string
     */
    public static @NotNull String removeExtraInstances(@NotNull String input, @NotNull String replaceRegex) {
        Matcher match = Pattern.compile(replaceRegex).matcher(input);
        if (match.find()) {
            return OrangeUtility.replaceAllPastIndex(input, replaceRegex, "", match.start());
        } else {
            return input;
        }
    }

    /**
     * Removes single line comments
     *
     * @param input            the String to operate on
     * @param commentDelimiter the String that denotes a comment
     * @return the modified String
     */
    public static @NotNull String removeSLComments(@NotNull String input, @NotNull String commentDelimiter) {
        int comdex = input.indexOf(commentDelimiter);
        int comend = input.indexOf("\n", comdex);
        while (comdex != -1) {
            if (comend != -1)// comend
            {
                input = input.substring(0, comdex) + input.substring(comend);
            } else// comment abbuts end of string
            {
                input = input.substring(0, comdex);
            }
            comdex = input.indexOf(commentDelimiter);
            comend = input.indexOf("\n", comdex);
        }
        return input;
    }

    /**
     * Calls input.replaceAll() on everything after the index Precondition: 0 <= index < input.length
     *
     * @param input       the input String
     * @param regex       the regex to be found
     * @param replacement the String to replace matching substrings
     * @param index       the index to start at (non-inclusive)
     * @return the resulting string
     */
    public static @NotNull String replaceAllPastIndex(@NotNull String input, @NotNull String regex, @NotNull String replacement,
                                                      int index) {
        return input.substring(0, index + 1) + input.substring(index + 1).replaceAll(regex, replacement);
    }

    public static double scalingDonut(double toDonut, double threshold, double clamp, double maxInput) {
        threshold = Math.abs(threshold);
        clamp = Math.abs(clamp);
        maxInput = Math.abs(maxInput);
        if (toDonut == 0) {
            return 0;
        }

        // range check
        if (toDonut > clamp || toDonut > maxInput) {
            return Math.min(clamp, maxInput);
        }
        if (toDonut < -clamp || toDonut < -maxInput) {
            return -Math.min(clamp, maxInput);
        }

        // calc factors
        double newRange = maxInput - threshold;
        double factor = newRange / maxInput;// maxInput is the old range
        // (maxInput - 0)

        if (toDonut > 0) {
            return toDonut * factor + threshold;
        }
        return toDonut * factor - threshold;
    }

    /**
     * Encapsulates Thread.sleep to make code more readable.
     *
     * @param millis the time to sleep
     */
    public static void sleep(long millis) {
        try {
            Thread.sleep(millis);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    /**
     * Returns the speed of the given entity in m/s.
     *
     * @return the speed of the given entity in m/s
     */
    public static double getSpeed(ChassisSpeeds speeds) {
        return Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    }

    public static MutableTranslation2d getTranslation2d(ChassisSpeeds speeds) {
        return new MutableTranslation2d(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
    }


    public static void waitTillTeleopStart() throws InterruptedException {
        while (DriverStation.isTeleop() && DriverStation.isEnabled()) {
            //noinspection BusyWait
            Thread.sleep(10);
        }
    }

    public static void assertTrue(boolean assertion) {
        if (!assertion) {
            throw new RuntimeException("Assertion Failed");
        }
    }

    /**
     * Fix the coordinates to they work with advantage kit
     */
    public static Pose3d fixCoords(Pose3d pose3d) {
        return new Pose3d(new Translation3d(pose3d.getX(), pose3d.getY() + FIELD_HEIGHT_METERS / 2, pose3d.getZ()),
                pose3d.getRotation());
    }
}
