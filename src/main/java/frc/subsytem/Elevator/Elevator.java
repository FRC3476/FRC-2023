package frc.subsytem.Elevator;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.subsytem.AbstractSubsystem;
import org.littletonrobotics.junction.Logger;

public class Elevator extends AbstractSubsystem {

    private final ElevatorIO elevatorIO;

    private final ElevatorInputsAutoLogged io = new ElevatorInputsAutoLogged();

    public Elevator(ElevatorIO elevatorIO) {
        super(Constants.ELEVATOR_PERIOD, 5);
        this.elevatorIO = elevatorIO;
    }


    private TrapezoidProfile trapezoidProfile =
            new TrapezoidProfile(Constants.ELEVATOR_CONSTRAINTS, new TrapezoidProfile.State());
    private double trapezoidProfileStartTime = 0;

    /**
     * @param position The position to set the elevator (meters)
     */
    public void setPosition(double position) {
        trapezoidProfile = new TrapezoidProfile(Constants.ELEVATOR_CONSTRAINTS, new TrapezoidProfile.State(position, 0),
                new TrapezoidProfile.State(io.elevatorPosition, io.elevatorVelocity));
        trapezoidProfileStartTime = -1;
        Logger.getInstance().recordOutput("Elevator/Goal position", position);
    }

    private double pastVelocity = 0, pastTime = 0;

    /**
     * Controls elevator motor with percent output
     *
     * @param percent Spans from -1 to 1 where the extremes are full power and direction depends on the sign
     */
    public void setPercentOutput(double percent) {
        elevatorIO.setElevatorVoltage(percent * Constants.ELEVATOR_NOMINAL_VOLTAGE);
    }

    @Override
    public void update() {
        elevatorIO.updateInputs(io);
        Logger.getInstance().processInputs("Elevator", io);

        double currentTime = Timer.getFPGATimestamp();
        if (trapezoidProfileStartTime == -1) {
            trapezoidProfileStartTime = currentTime;
        }
        TrapezoidProfile.State state = trapezoidProfile.calculate(currentTime - trapezoidProfileStartTime);
        double acceleration = (state.velocity - pastVelocity) / (currentTime - pastTime);
        elevatorIO.setElevatorPosition(state.position, Constants.ELEVATOR_FEEDFORWARD.calculate(state.velocity, acceleration));

        pastVelocity = state.velocity;
        pastTime = currentTime;

        Logger.getInstance().recordOutput("Elevator/Wanted pos", state.position);
        Logger.getInstance().recordOutput("Elevator/Wanted vel", state.velocity);
        Logger.getInstance().recordOutput("Elevator/Wanted accel", acceleration);
        Logger.getInstance().recordOutput("Elevator/Total trapezoidProfile time", trapezoidProfile.totalTime());
        Logger.getInstance().recordOutput("Elevator/TrapezoidProfile time", currentTime - trapezoidProfileStartTime);
        Logger.getInstance().recordOutput("Elevator/TrapezoidProfile Error", state.position - io.elevatorPosition);
    }

    @Override
    public void selfTest() {

    }
}
