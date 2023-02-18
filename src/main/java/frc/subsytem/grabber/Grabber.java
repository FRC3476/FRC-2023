package frc.subsytem.grabber;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.subsytem.AbstractSubsystem;
import org.littletonrobotics.junction.Logger;

public class Grabber extends AbstractSubsystem {

    private final GrabberIO io;
    private final GrabberInputsAutoLogged inputs = new GrabberInputsAutoLogged();

    public Grabber(GrabberIO grabberIO) {
        super();
        this.io = grabberIO;
        trapezoidProfile = new TrapezoidProfile(Constants.GRABBER_PIVOT_CONSTRAINTS, new TrapezoidProfile.State(56 + 90 - 20, 0));
    }

    private TrapezoidProfile trapezoidProfile;
    private double trapezoidProfileStartTime = 0;
    private double finalGoalPosition = 0;

    /**
     * @param position The position to set the grabber (degrees)
     */
    public void setPosition(double position) {
        finalGoalPosition = position;
        trapezoidProfile = new TrapezoidProfile(Constants.GRABBER_PIVOT_CONSTRAINTS, new TrapezoidProfile.State(position, 0),
                new TrapezoidProfile.State(inputs.pivotPosition, inputs.pivotVelocity));
        trapezoidProfileStartTime = -1;
        Logger.getInstance().recordOutput("Pivot/Goal position", position);
    }

    private double pastVelocity = 0, pastTime = 0;

    @Override
    public void update() {
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
        if (Math.abs(inputs.pivotPosition - state.position) > 1) {
            io.setPivotPosition(state.position, arbFFVoltage);
        } else {
            io.setPivotVoltage(arbFFVoltage);
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
        OPEN(2),
        GRAB_CUBE(-1),
        GRAB_CONE(-3),
        IDLE(0);
        final double voltage;

        GrabState(double voltage) {
            this.voltage = voltage;
        }
    }

    public void setGrabState(GrabState grabState) {
        io.setGrabberVoltage(grabState.voltage);
        Logger.getInstance().recordOutput("Grabber/Grabber voltage", grabState.voltage);
        Logger.getInstance().recordOutput("Grabber/Grabber state", grabState.name());
    }

    public void setRollerVoltage(double voltage) {
        io.setRollerVoltage(voltage);
    }

    @Override
    public void selfTest() {

    }

    public double getPivotDegrees() {
        return inputs.pivotPosition;
    }
}
