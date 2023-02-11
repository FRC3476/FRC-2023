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
        super(5);
        this.io = grabberIO;
    }

    private TrapezoidProfile trapezoidProfile =
            new TrapezoidProfile(Constants.GRABBER_PIVOT_CONSTRAINTS, new TrapezoidProfile.State());
    private double trapezoidProfileStartTime = 0;

    /**
     * @param position The position to set the grabber (degrees)
     */
    public void setPosition(double position) {
        trapezoidProfile = new TrapezoidProfile(Constants.GRABBER_PIVOT_CONSTRAINTS, new TrapezoidProfile.State(position, 0),
                new TrapezoidProfile.State(inputs.grabberPosition, inputs.grabberVelocity));
        trapezoidProfileStartTime = Timer.getFPGATimestamp();
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
        TrapezoidProfile.State state = trapezoidProfile.calculate(currentTime - trapezoidProfileStartTime);
        double acceleration = (state.velocity - pastVelocity) / (currentTime - pastTime);

        double arbFFVoltage = Constants.GRABBER_FEEDFORWARD.calculate(state.velocity, acceleration);
        io.setPivotPosition(state.position, arbFFVoltage);

        pastVelocity = state.velocity;
        pastTime = currentTime;

        Logger.getInstance().recordOutput("Pivot/Wanted pos", state.position);
        Logger.getInstance().recordOutput("Pivot/Wanted vel", state.velocity);
        Logger.getInstance().recordOutput("Pivot/Wanted accel", acceleration);
        Logger.getInstance().recordOutput("Pivot/Total trapezoidProfile time", trapezoidProfile.totalTime());
        Logger.getInstance().recordOutput("Pivot/Profile length", currentTime - trapezoidProfileStartTime);
        Logger.getInstance().recordOutput("Pivot/TrapezoidProfile error", state.position - inputs.pivotPosition);
        Logger.getInstance().recordOutput("Pivot/Arb FF voltage", arbFFVoltage);
    }

    public enum GrabState {
        OPEN(-3),
        GRAB_CUBE(3),
        GRAB_CONE(6);
        final double voltage;

        GrabState(double voltage) {
            this.voltage = voltage;
        }
    }

    public void setGrabState(GrabState grabState) {
        io.setGrabberVoltage(grabState.voltage);
    }

    public void setRollerVoltage(double voltage) {
        io.setRollerVoltage(voltage);
    }

    @Override
    public void selfTest() {

    }
}
