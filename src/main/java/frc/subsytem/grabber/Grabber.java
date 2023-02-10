package frc.subsytem.grabber;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.subsytem.AbstractSubsystem;
import org.littletonrobotics.junction.Logger;

public class Grabber extends AbstractSubsystem {

    private final GrabberIO grabberIO;
    private final GrabberInputsAutoLogged io = new GrabberInputsAutoLogged();

    private Grabber(GrabberIO grabberIO) {
        super(Constants.GRABBER_PERIOD, 5);
        this.grabberIO = grabberIO;
    }

    private TrapezoidProfile trapezoidProfile =
            new TrapezoidProfile(Constants.GRABBER_PIVOT_CONSTRAINTS, new TrapezoidProfile.State());
    private double trapezoidProfileStartTime = 0;

    /**
     * @param position The position to set the grabber (degrees)
     */
    public void setPosition(double position) {
        trapezoidProfile = new TrapezoidProfile(Constants.GRABBER_PIVOT_CONSTRAINTS, new TrapezoidProfile.State(position, 0),
                new TrapezoidProfile.State(io.grabberPosition, io.grabberVelocity));
        trapezoidProfileStartTime = Timer.getFPGATimestamp();
        Logger.getInstance().recordOutput("Pivot/Goal position", position);
    }

    private double pastVelocity = 0, pastTime = 0;

    @Override
    public void update() {
        grabberIO.updateInputs(io);
        Logger.getInstance().processInputs("Grabber", io);

        double currentTime = Timer.getFPGATimestamp();
        if (trapezoidProfileStartTime == -1) {
            trapezoidProfileStartTime = currentTime;
        }
        TrapezoidProfile.State state = trapezoidProfile.calculate(currentTime - trapezoidProfileStartTime);
        double acceleration = (state.velocity - pastVelocity) / (currentTime - pastTime);

        grabberIO.setPivotPosition(state.position, Constants.GRABBER_FEEDFORWARD.calculate(state.velocity, acceleration));

        pastVelocity = state.velocity;
        pastTime = currentTime;

        Logger.getInstance().recordOutput("Pivot/Wanted pos", state.position);
        Logger.getInstance().recordOutput("Pivot/Wanted vel", state.velocity);
        Logger.getInstance().recordOutput("Pivot/Wanted accel", acceleration);
        Logger.getInstance().recordOutput("Pivot/Total trapezoidProfile time", trapezoidProfile.totalTime());
        Logger.getInstance().recordOutput("Pivot/Profile length", currentTime - trapezoidProfileStartTime);
        Logger.getInstance().recordOutput("Pivot/TrapezoidProfile error", state.position - io.pivotPosition);
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
        grabberIO.setGrabberVoltage(grabState.voltage);
    }

    @Override
    public void selfTest() {

    }
}
