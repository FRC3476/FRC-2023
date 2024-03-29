package frc.subsytem.Elevator;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.subsytem.AbstractSubsystem;
import org.littletonrobotics.junction.Logger;

import static frc.robot.Constants.ELEVATOR_HOME_VOLTAGE;
import static frc.robot.Constants.ELEVATOR_MIN_HOME_TIME;
import static frc.utility.MathUtil.avg;

public class Elevator extends AbstractSubsystem {

    private final ElevatorIO io;

    private final ElevatorInputsAutoLogged inputs = new ElevatorInputsAutoLogged();

    public Elevator(ElevatorIO elevatorIO) {
        super(5);
        this.io = elevatorIO;
    }


    private TrapezoidProfile trapezoidProfile =
            new TrapezoidProfile(Constants.ELEVATOR_CONSTRAINTS, new TrapezoidProfile.State());
    private double trapezoidProfileStartTime = 0;

    /**
     * @param position The position to set the elevator (meters)
     */
    public void setPosition(double position) {
        if (position < Constants.ELEVATOR_LOWER_LIMIT) {
            position = Constants.ELEVATOR_LOWER_LIMIT;
        } else if (position > Constants.ELEVATOR_UPPER_LIMIT) {
            position = Constants.ELEVATOR_UPPER_LIMIT;
        }

        trapezoidProfile = new TrapezoidProfile(Constants.ELEVATOR_CONSTRAINTS, new TrapezoidProfile.State(position, 0),
                new TrapezoidProfile.State(inputs.elevatorPosition[0], inputs.elevatorVelocity[0]));
        trapezoidProfileStartTime = -1;
        Logger.getInstance().recordOutput("Elevator/Goal position", position);
    }

    private double pastVelocity = 0, pastTime = 0;

    private boolean homing = false;
    private double homeTime = 0;

    public void home() {
        homeTime = ELEVATOR_MIN_HOME_TIME;
        homing = true;
    }

    public void cancelHome() {
        homing = false;
    }

    @Override
    public void update() {
        io.updateInputs(inputs);
        Logger.getInstance().processInputs("Elevator", inputs);

        if (homing) {
            if (DriverStation.isEnabled()) {
                homeTime -= Constants.NOMINAL_DT;
                io.setElevatorVoltage(ELEVATOR_HOME_VOLTAGE);
                if (homeTime <= 0 && avg(inputs.elevatorCurrent) > Constants.ELEVATOR_STALLING_CURRENT) {
                    homing = false;
                    io.resetPosition(0);
                }
                Logger.getInstance().recordOutput("Elevator/Home time", homeTime);
            }
            return;
        }


        double currentTime = Timer.getFPGATimestamp();
        if (trapezoidProfileStartTime == -1) {
            trapezoidProfileStartTime = currentTime;
        }
        currentTime = 100000000;
        TrapezoidProfile.State state = trapezoidProfile.calculate(currentTime - trapezoidProfileStartTime);
        double acceleration = 0; //(state.velocity - pastVelocity) / (currentTime - pastTime);
        double arbFFVoltage = Constants.ELEVATOR_FEEDFORWARD.calculate(state.velocity, acceleration);
        if (DriverStation.isTest()) {
            io.setElevatorVoltage(Constants.ELEVATOR_FEEDFORWARD.calculate(0, 0));
        } else {
            io.setElevatorPosition(state.position, arbFFVoltage);
        }

        pastVelocity = state.velocity;
        pastTime = currentTime;

        Logger.getInstance().recordOutput("Elevator/Wanted pos", state.position);
        Logger.getInstance().recordOutput("Elevator/Wanted vel", state.velocity);
        Logger.getInstance().recordOutput("Elevator/Wanted accel", acceleration);
        Logger.getInstance().recordOutput("Elevator/Total trapezoidProfile time", trapezoidProfile.totalTime());
        Logger.getInstance().recordOutput("Elevator/TrapezoidProfile time", currentTime - trapezoidProfileStartTime);
        Logger.getInstance().recordOutput("Elevator/TrapezoidProfile Error", state.position - avg(inputs.elevatorPosition));
        Logger.getInstance().recordOutput("Elevator/FF voltage", arbFFVoltage);
    }

    public double getPosition() {
        return inputs.elevatorPosition[0];
    }
}
