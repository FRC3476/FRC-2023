// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.utility.wpimodified;

import edu.wpi.first.util.sendable.Sendable;

import java.lang.invoke.MethodHandle;
import java.lang.invoke.MethodHandles;
import java.lang.reflect.Field;

/**
 * Implements a PID control loop.
 */
public class PIDController extends edu.wpi.first.math.controller.PIDController implements Sendable, AutoCloseable {

    public PIDController(double kp, double ki, double kd) {
        super(kp, ki, kd);
    }

    public PIDController(double kp, double ki, double kd, double period) {
        super(kp, ki, kd, period);
    }

    // Use a method handle to get reduce the amount of overhead from using reflection
    private static final MethodHandle setTotalError;

    static {
        try {
            Field totalErrorField = edu.wpi.first.math.controller.PIDController.class.getDeclaredField("m_totalError");
            totalErrorField.setAccessible(true);
            setTotalError = MethodHandles.lookup().unreflectSetter(totalErrorField);
        } catch (NoSuchFieldException | IllegalAccessException e) {
            throw new RuntimeException(e);
        }
    }

    public void resetI() {
        try {
            setTotalError.invoke(this, 0.0);
        } catch (Throwable e) {
            throw new RuntimeException(e);
        }
    }
}
