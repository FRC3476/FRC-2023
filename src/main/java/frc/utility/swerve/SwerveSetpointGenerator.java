package frc.utility.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import org.jetbrains.annotations.NotNull;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import static frc.utility.MathUtil.*;

/**
 * Takes a prior setpoint (ChassisSpeeds), a desired setpoint (from a driver, or from a path follower), and outputs a new setpoint
 * that respects all the kinematic constraints on module rotation speed and wheel velocity/acceleration. By generating a new
 * setpoint every iteration, the robot will converge to the desired setpoint quickly while avoiding any intermediate state that is
 * kinematically infeasible (and can result in wheel slip or robot heading drift as a result).
 *
 * @author 254
 */
public class SwerveSetpointGenerator {


    private static final Rotation2d ROTATION_180 = new Rotation2d(Math.PI);

    private static final Twist2d IdentityTwist = new Twist2d(0, 0, 0);

    public record KinematicLimit(double maxDriveVelocity, double maxDriveAcceleration, double maxSteeringVelocity) {
    }

    private final SwerveDriveKinematics kinematics;

    public SwerveSetpointGenerator(final SwerveDriveKinematics kinematics) {
        this.kinematics = kinematics;
    }

    /**
     * Check if it would be faster to go to the opposite of the goal heading (and reverse drive direction).
     *
     * @param prevToGoal The rotation from the previous state to the goal state (i.e. prev.inverse().rotateBy(goal)).
     * @return True if the shortest path to achieve this rotation involves flipping the drive direction.
     */
    private boolean flipHeading(Rotation2d prevToGoal) {
        return Math.abs(getRadians(prevToGoal)) > Math.PI / 2.0;
    }

    private double unwrapAngle(double ref, double angle) {
        double diff = angle - ref;
        if (diff > Math.PI) {
            return angle - 2.0 * Math.PI;
        } else if (diff < -Math.PI) {
            return angle + 2.0 * Math.PI;
        } else {
            return angle;
        }
    }

    @FunctionalInterface
    private interface Function2d {
        double f(double x, double y);
    }

    /**
     * Find the root of the generic 2D parametric function 'func' using the regula falsi technique. This is a pretty naive way to
     * do root finding, but it's usually faster than simple bisection while being robust in ways that e.g. the Newton-Raphson
     * method isn't.
     *
     * @param func            The Function2d to take the root of.
     * @param x0              x value of the lower bracket.
     * @param y0              y value of the lower bracket.
     * @param f0              value of 'func' at x0, y0 (passed in by caller to save a call to 'func' during recursion)
     * @param x1              x value of the upper bracket.
     * @param y1              y value of the upper bracket.
     * @param f1              value of 'func' at x1, y1 (passed in by caller to save a call to 'func' during recursion)
     * @param iterations_left Number of iterations of root finding left.
     * @return The parameter value 's' that interpolating between 0 and 1 that corresponds to the (approximate) root.
     */
    private double findRoot(Function2d func, double x0, double y0, double f0, double x1, double y1, double f1,
                            int iterations_left) {
        if (iterations_left < 0 || epsilonEquals(f0, f1)) {
            return 1.0;
        }
        var s_guess = Math.max(0.0, Math.min(1.0, -f0 / (f1 - f0)));
        var x_guess = (x1 - x0) * s_guess + x0;
        var y_guess = (y1 - y0) * s_guess + y0;
        var f_guess = func.f(x_guess, y_guess);
        if (Math.signum(f0) == Math.signum(f_guess)) {
            // 0 and guess on same side of root, so use upper bracket.
            return s_guess + (1.0 - s_guess) * findRoot(func, x_guess, y_guess, f_guess, x1, y1, f1, iterations_left - 1);
        } else {
            // Use lower bracket.
            return s_guess * findRoot(func, x0, y0, f0, x_guess, y_guess, f_guess, iterations_left - 1);
        }
    }

    protected double findSteeringMaxS(double x0, double y0, double f0, double x1, double y1, double f1,
                                      double maxDeviation, int maxIterations) {
        f1 = unwrapAngle(f0, f1);
        double diff = f1 - f0;
        if (Math.abs(diff) <= maxDeviation) {
            // Can go all the way to s=1.
            return 1.0;
        }
        double offset = f0 + Math.signum(diff) * maxDeviation;
        Function2d func = (x, y) -> unwrapAngle(f0, Math.atan2(y, x)) - offset;
        return findRoot(func, x0, y0, f0 - offset, x1, y1, f1 - offset, maxIterations);
    }

    protected double findDriveMaxS(double x0, double y0, double f0, double x1, double y1, double f1, double maxVelStep,
                                   int max_iterations) {
        double diff = f1 - f0;
        if (Math.abs(diff) <= maxVelStep) {
            // Can go all the way to s=1.
            return 1.0;
        }
        double offset = f0 + Math.signum(diff) * maxVelStep;
        Function2d func = (x, y) -> Math.hypot(x, y) - offset;
        return findRoot(func, x0, y0, f0 - offset, x1, y1, f1 - offset, max_iterations);
    }

    protected double findDriveMaxS(double x0, double y0, double x1, double y1, double maxVelStep) {
        // Our drive velocity between s=0 and s=1 is quadratic in s:
        // v^2 = ((x1 - x0) * s + x0)^2 + ((y1 - y0) * s + y0)^2
        //     = a * s^2 + b * s + c
        // Where:
        //   a = (x1 - x0)^2 + (y1 - y0)^2
        //   b = 2 * x0 * (x1 - x0) + 2 * y0 * (y1 - y0)
        //   c = x0^2 + y0^2
        // We want to find where this quadratic results in a velocity that is > maxVelStep from our velocity at s=0:
        // sqrt(x0^2 + y0^2) +/- maxVelStep = ...quadratic...
        final double dx = x1 - x0;
        final double dy = y1 - y0;
        final double a = dx * dx + dy * dy;
        final double b = 2.0 * x0 * dx + 2.0 * y0 * dy;
        final double c = x0 * x0 + y0 * y0;
        final double v_limit_upper_2 = Math.pow(Math.hypot(x0, y0) + maxVelStep, 2.0);
        final double v_limit_lower_2 = Math.pow(Math.hypot(x0, y0) - maxVelStep, 2.0);
        return 0.0;
    }

    /**
     * Generate a new setpoint.
     *
     * @param limits       The kinematic limits to respect for this setpoint.
     * @param prevSetpoint The previous setpoint motion. Normally, you'd pass in the previous iteration setpoint instead of the
     *                     actual measured/estimated kinematic state.
     * @param desiredState The desired state of motion, such as from the driver sticks or a path following algorithm.
     * @param dt           The loop time.
     * @return A Setpoint object that satisfies all the KinematicLimit while converging to desiredState quickly.
     */
    public SwerveSetpoint generateSetpoint(final @NotNull SwerveSetpointGenerator.KinematicLimit limits,
                                           final @NotNull SwerveSetpoint prevSetpoint,
                                           ChassisSpeeds desiredState, final double dt) {
        final Translation2d[] modules = Constants.SWERVE_MODULE_LOCATIONS;

        SwerveModuleState[] desiredModuleState = kinematics.toSwerveModuleStates(desiredState);
        // Make sure desiredState respects velocity limits.
        if (limits.maxDriveVelocity > 0.0) {
            SwerveDriveKinematics.desaturateWheelSpeeds(desiredModuleState, limits.maxDriveVelocity);
            desiredState = kinematics.toChassisSpeeds(desiredModuleState);
        }

        // Special case: desiredState is a complete stop. In this case, module angle is arbitrary, so just use the previous angle.
        boolean need_to_steer = true;
        if (epsilonEquals(toTwist2d(desiredState), IdentityTwist)) {
            need_to_steer = false;
            for (int i = 0; i < modules.length; ++i) {
                desiredModuleState[i].angle = prevSetpoint.moduleStates[i].angle;
                desiredModuleState[i].speedMetersPerSecond = 0.0;
            }
        }

        // For each module, compute local Vx and Vy vectors.
        double[] prevVx = new double[modules.length];
        double[] prevVy = new double[modules.length];
        Rotation2d[] prevHeading = new Rotation2d[modules.length];
        double[] desiredVx = new double[modules.length];
        double[] desiredVy = new double[modules.length];
        Rotation2d[] desiredHeading = new Rotation2d[modules.length];
        boolean allModulesShouldFlip = true;
        for (int i = 0; i < modules.length; ++i) {
            prevVx[i] = prevSetpoint.moduleStates[i].angle.getCos() * prevSetpoint.moduleStates[i].speedMetersPerSecond;
            prevVy[i] = prevSetpoint.moduleStates[i].angle.getSin() * prevSetpoint.moduleStates[i].speedMetersPerSecond;
            prevHeading[i] = prevSetpoint.moduleStates[i].angle;
            if (prevSetpoint.moduleStates[i].speedMetersPerSecond < 0.0) {
                prevHeading[i] = prevHeading[i].rotateBy(ROTATION_180);
            }
            desiredVx[i] = desiredModuleState[i].angle.getCos() * desiredModuleState[i].speedMetersPerSecond;
            desiredVy[i] = desiredModuleState[i].angle.getSin() * desiredModuleState[i].speedMetersPerSecond;
            desiredHeading[i] = desiredModuleState[i].angle;
            if (desiredModuleState[i].speedMetersPerSecond < 0.0) {
                desiredHeading[i] = desiredHeading[i].plus(ROTATION_180);
            }
            if (allModulesShouldFlip) {
                double requiredRotationRad = Math.abs(
                        getRadians(
                                prevHeading[i].unaryMinus().rotateBy(desiredHeading[i])
                        ));
                if (requiredRotationRad < Math.PI / 2.0) {
                    allModulesShouldFlip = false;
                }
            }
        }
        if (allModulesShouldFlip &&
                !epsilonEquals(toTwist2d(prevSetpoint.chassisSpeeds), IdentityTwist) && //We're moving (previous state)
                !epsilonEquals(toTwist2d(desiredState), IdentityTwist)) { //We're moving (desired state)
            // It will (likely) be faster to stop the robot, rotate the modules in place to the complement of the desired
            // angle, and accelerate again.
            return generateSetpoint(limits, prevSetpoint, new ChassisSpeeds(), dt);
        }

        // Compute the deltas between start and goal. We can then interpolate from the start state to the goal state; then
        // find the amount we can move from start towards goal in this cycle such that no kinematic limit is exceeded.
        double dx = desiredState.vxMetersPerSecond - prevSetpoint.chassisSpeeds.vxMetersPerSecond;
        double dy = desiredState.vyMetersPerSecond - prevSetpoint.chassisSpeeds.vyMetersPerSecond;
        double dtheta = desiredState.omegaRadiansPerSecond - prevSetpoint.chassisSpeeds.omegaRadiansPerSecond;

        // 's' interpolates between start and goal. At 0, we are at prevState and at 1, we are at desiredState.
        double minS = 1.0;

        // In cases where an individual module is stopped, we want to remember the right steering angle to command (since
        // inverse kinematics doesn't care about angle, we can be opportunistically lazy).
        List<Optional<Rotation2d>> overrideSteering = new ArrayList<>(modules.length);
        // Enforce steering velocity limits. We do this by taking the derivative of steering angle at the current angle,
        // and then backing out the maximum interpolant between start and goal states. We remember the minimum across all modules, since
        // that is the active constraint.
        final double maxThetaStep = dt * limits.maxSteeringVelocity;
        for (int i = 0; i < modules.length; ++i) {
            if (!need_to_steer) {
                overrideSteering.add(Optional.of(prevSetpoint.moduleStates[i].angle));
                continue;
            }
            overrideSteering.add(Optional.empty());
            if (epsilonEquals(prevSetpoint.moduleStates[i].speedMetersPerSecond, 0.0)) {
                // If module is stopped, we know that we will need to move straight to the final steering angle, so limit based
                // purely on rotation in place.
                if (epsilonEquals(desiredModuleState[i].speedMetersPerSecond, 0.0)) {
                    // Goal angle doesn't matter. Just leave module at its current angle.
                    overrideSteering.set(i, Optional.of(prevSetpoint.moduleStates[i].angle));
                    continue;
                }

                var necessaryRotation = prevSetpoint.moduleStates[i].angle.unaryMinus().rotateBy(
                        desiredModuleState[i].angle);
                if (flipHeading(necessaryRotation)) {
                    necessaryRotation = necessaryRotation.rotateBy(ROTATION_180);
                }
                // getRadians() bounds to +/- Pi.
                final double numStepsNeeded = Math.abs(getRadians(necessaryRotation)) / maxThetaStep;

                if (numStepsNeeded <= 1.0) {
                    // Steer directly to goal angle.
                    overrideSteering.set(i, Optional.of(desiredModuleState[i].angle));
                    // Don't limit the global minS;
                    continue;
                } else {
                    // Adjust steering by maxThetaStep.
                    overrideSteering.set(i, Optional.of(prevSetpoint.moduleStates[i].angle.rotateBy(
                            Rotation2d.fromRadians(Math.signum(getRadians(necessaryRotation)) * maxThetaStep))));
                    minS = 0.0;
                    continue;
                }
            }
            if (minS == 0.0) {
                // s can't get any lower. Save some CPU.
                continue;
            }

            final int kMaxIterations = 8;
            double s = findSteeringMaxS(prevVx[i], prevVy[i], getRadians(prevHeading[i]),
                    desiredVx[i], desiredVy[i], getRadians(desiredHeading[i]),
                    maxThetaStep, kMaxIterations);
            minS = Math.min(minS, s);
        }

        // Enforce drive wheel acceleration limits.
        final double maxVelStep = dt * limits.maxDriveAcceleration;
        for (int i = 0; i < modules.length; ++i) {
            if (minS == 0.0) {
                // No need to carry on.
                break;
            }
            double vxMinS = minS == 1.0 ? desiredVx[i] : (desiredVx[i] - prevVx[i]) * minS + prevVx[i];
            double vyMinS = minS == 1.0 ? desiredVy[i] : (desiredVy[i] - prevVy[i]) * minS + prevVy[i];
            // Find the max s for this drive wheel. Search on the interval between 0 and minS, because we already know we can't go faster
            // than that.
            // TODO(for efficiency, do all this on v^2 to save a bunch of sqrts)
            // TODO(be smarter about root finding, since this is just a quadratic in s: ((xf-x0)*s+x0)^2+((yf-y0)*s+y0)^2)
            final int kMaxIterations = 10;
            double s = minS * findDriveMaxS(prevVx[i], prevVy[i], Math.hypot(prevVx[i], prevVy[i]),
                    vxMinS, vyMinS, Math.hypot(vxMinS, vyMinS),
                    maxVelStep, kMaxIterations);
            minS = Math.min(minS, s);
        }

        ChassisSpeeds limitedSpeeds = new ChassisSpeeds(
                prevSetpoint.chassisSpeeds.vxMetersPerSecond + minS * dx,
                prevSetpoint.chassisSpeeds.vyMetersPerSecond + minS * dy,
                prevSetpoint.chassisSpeeds.omegaRadiansPerSecond + minS * dtheta);

        var prevModuleInitialState = kinematics.toSwerveModuleStates(prevSetpoint.chassisSpeeds);
        var limitedStates = kinematics.toSwerveModuleStates(limitedSpeeds);
        double[] wheelAccelerations = new double[modules.length];

        for (int i = 0; i < modules.length; ++i) {
            wheelAccelerations[i] = (limitedStates[i].speedMetersPerSecond - prevModuleInitialState[i].speedMetersPerSecond) / dt;
        }

        for (int i = 0; i < modules.length; ++i) {
            final var maybeOverride = overrideSteering.get(i);
            if (maybeOverride.isPresent()) {
                var override = maybeOverride.get();
                if (flipHeading(limitedStates[i].angle.unaryMinus().rotateBy(override))) {
                    limitedStates[i].speedMetersPerSecond *= -1.0;
                    wheelAccelerations[i] *= -1.0;
                }
                limitedStates[i].angle = override;
            }
            final var deltaRotation = prevSetpoint.moduleStates[i].angle.unaryMinus().rotateBy(limitedStates[i].angle);
            if (flipHeading(deltaRotation)) {
                limitedStates[i].angle = limitedStates[i].angle.plus(ROTATION_180);
                limitedStates[i].speedMetersPerSecond *= -1.0;
                wheelAccelerations[i] *= -1.0;
            }
        }


        for (int i = 0; i < modules.length; ++i) {
            wheelAccelerations[i] = (limitedStates[i].speedMetersPerSecond - prevSetpoint.moduleStates[i].speedMetersPerSecond) / dt;
        }

        return new SwerveSetpoint(limitedSpeeds, limitedStates, wheelAccelerations);
    }


    public record SwerveSetpoint(ChassisSpeeds chassisSpeeds, SwerveModuleState[] moduleStates, double[] wheelAccelerations) {}
}
