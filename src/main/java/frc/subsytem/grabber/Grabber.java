package frc.subsytem.grabber;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.subsytem.AbstractSubsystem;
import org.littletonrobotics.junction.Logger;

import static frc.robot.Constants.*;

public class Grabber extends AbstractSubsystem {

    public static final double MIN_OPEN_TIME = 0.5;
    public static final double MIN_CLOSED_TIME = 0.2;
    public static final double CURRENT_SPIKE_TIME_THRESHOLD_S = 0.2;
    private final GrabberIO io;
    private final GrabberInputsAutoLogged inputs = new GrabberInputsAutoLogged();

    public Grabber(GrabberIO grabberIO) {
        super();
        this.io = grabberIO;
        trapezoidProfile = new TrapezoidProfile(Constants.GRABBER_PIVOT_CONSTRAINTS, new TrapezoidProfile.State(56 + 90 - 20, 0));
        setAutoGrab(false);
    }

    private TrapezoidProfile trapezoidProfile;
    private double trapezoidProfileStartTime = 0;
    private double finalGoalPosition = 0;

    /**
     * @param position The position to set the grabber (degrees)
     */
    public synchronized void setPosition(double position) {
        finalGoalPosition = position;
        trapezoidProfile = new TrapezoidProfile(Constants.GRABBER_PIVOT_CONSTRAINTS, new TrapezoidProfile.State(position, 0),
                new TrapezoidProfile.State(inputs.pivotPosition, inputs.pivotVelocity));
        trapezoidProfileStartTime = -1;
        Logger.getInstance().recordOutput("Pivot/Goal position", position);
    }

    private double pastVelocity = 0, pastTime = 0;

    @Override
    public synchronized void update() {
        io.updateInputs(inputs);
        Logger.getInstance().processInputs("Grabber", inputs);

        double currentTime = Timer.getFPGATimestamp();
        if (trapezoidProfileStartTime == -1) {
            trapezoidProfileStartTime = currentTime;
        }
        currentTime = 100000000;
        TrapezoidProfile.State state = trapezoidProfile.calculate(currentTime - trapezoidProfileStartTime);
        double acceleration = 0; // (state.velocity - pastVelocity) / (currentTime - pastTime);

        double arbFFVoltage = Constants.GRABBER_FEEDFORWARD.calculate(Math.toRadians(inputs.pivotPosition),
                state.velocity, acceleration);
        if (DriverStation.isTest()) {
            io.setPivotVoltage(Constants.GRABBER_FEEDFORWARD.calculate(Math.toRadians(inputs.pivotPosition), 0, 0));
        } else {
            if (Math.abs(inputs.pivotPosition - state.position) > 1) {
                io.setPivotPosition(state.position, arbFFVoltage);
            } else {
                io.setPivotVoltage(arbFFVoltage);
            }
        }

        pastVelocity = state.velocity;
        pastTime = currentTime;

        Logger.getInstance().recordOutput("Pivot/Wanted pos", state.position);
        Logger.getInstance().recordOutput("Pivot/Wanted vel", state.velocity);
        Logger.getInstance().recordOutput("Pivot/Wanted accel", acceleration);
        Logger.getInstance().recordOutput("Pivot/Total trapezoidProfile time", trapezoidProfile.totalTime());
        Logger.getInstance().recordOutput("Pivot/Profile length", currentTime - trapezoidProfileStartTime);
        Logger.getInstance().recordOutput("Pivot/TrapezoidProfile error", state.position - inputs.pivotPosition);
        Logger.getInstance().recordOutput("Pivot/Arb FF", arbFFVoltage);
    }

    public enum GrabState {
        OPEN(6),
        GRAB_CUBE(-4),
        GRAB_CONE(-12),
        IDLE(0);
        final double voltage;

        GrabState(double voltage) {
            this.voltage = voltage;
        }
    }

    GrabState lastGrabState = GrabState.IDLE;
    double allowedClosedTime = 0;
    double allowedOpenTime = 0;


    public synchronized void setGrabState(GrabState grabState) {
        if (!Robot.isOnMainThread()) {
            Robot.runOnMainThread(() -> setGrabState(grabState));
            return;
        }

        io.setGrabberVoltage(grabState.voltage);
        Logger.getInstance().recordOutput("Grabber/Grabber voltage", grabState.voltage);
        Logger.getInstance().recordOutput("Grabber/Grabber state", grabState.name());
        if (grabState != lastGrabState) {
            lastGrabState = grabState;
            if (grabState == GrabState.OPEN) {
                allowedOpenTime = Timer.getFPGATimestamp() + MIN_OPEN_TIME;
            } else {
                allowedClosedTime = Timer.getFPGATimestamp() + MIN_CLOSED_TIME;
            }

            grabbedAboveCurrentTime = Double.MAX_VALUE;
            openAboveCurrentTime = Double.MAX_VALUE;
        }
    }

    boolean isAutoGrabEnabled = false;

    public synchronized void setAutoGrab(boolean enabled) {
        if (!Robot.isOnMainThread()) {
            Robot.runOnMainThread(() -> setAutoGrab(enabled));
            return;
        }
        isAutoGrabEnabled = enabled && IS_AUTO_GRAB_ENABLED;
        io.setAutoGrab(enabled && IS_AUTO_GRAB_ENABLED);

        Logger.getInstance().recordOutput("Grabber/Limit Switch Enabled", enabled && IS_AUTO_GRAB_ENABLED);
    }

    public synchronized boolean isAutoGrabEnabled() {
        return isAutoGrabEnabled;
    }

    double grabbedAboveCurrentTime = 0;


    public synchronized boolean isGrabbed() {
        if (Math.abs(inputs.grabberCurrent) > GRABBED_CURRENT_THRESHOLD
                && (lastGrabState == GrabState.GRAB_CONE || lastGrabState == GrabState.GRAB_CUBE)
                && Timer.getFPGATimestamp() > allowedOpenTime
                && (inputs.grabberAbsolutePosition <= Constants.GRABBER_CLOSE_THRESHOLD_DEGREES || !USE_GRABBER_ENCODER)) {
            return Timer.getFPGATimestamp() > grabbedAboveCurrentTime;
        } else {
            grabbedAboveCurrentTime = Timer.getFPGATimestamp() + CURRENT_SPIKE_TIME_THRESHOLD_S;
        }
        return false;
    }


    double openAboveCurrentTime = 0;

    public synchronized boolean isOpen() {
        if (Math.abs(inputs.grabberCurrent) > GRABBED_CURRENT_THRESHOLD
                && (lastGrabState == GrabState.OPEN)
                && Timer.getFPGATimestamp() > allowedOpenTime
                && (inputs.grabberAbsolutePosition >= Constants.GRABBER_OPEN_THRESHOLD_DEGREES || !USE_GRABBER_ENCODER)) {
            return Timer.getFPGATimestamp() > openAboveCurrentTime;
        } else {
            openAboveCurrentTime = Timer.getFPGATimestamp() + CURRENT_SPIKE_TIME_THRESHOLD_S;
        }
        return false;
    }

    double startTime;

    public void waitTillGrabbed(double maxTime) throws InterruptedException {
        startTime = Timer.getFPGATimestamp();
        while (true) {
            synchronized (this) {
                if (isGrabbed()) {
                    break;
                }
            }

            Thread.sleep(10);

            if (Timer.getFPGATimestamp() - startTime > maxTime) {
                break;
            }
        }
    }

    public void setRollerVoltage(double voltage) {
        io.setRollerVoltage(voltage);
    }

    @Override
    public void logData() {
        SmartDashboard.putBoolean("Is Limit Switch Triggered", inputs.isLimitSwitchTriggered);
    }

    @Override
    public void selfTest() {

    }

    public double getPivotDegrees() {
        return inputs.pivotPosition;
    }
}
