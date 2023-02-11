package frc.subsytem.Elevator;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.subsytem.AbstractSubsystem;
import org.littletonrobotics.junction.Logger;

import static frc.robot.Constants.ELEVATOR_MIN_HOME_TIME;
import static frc.robot.Constants.MOTOR_SPEED_DECREASING_RATE;
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
                new TrapezoidProfile.State(avg(inputs.elevatorPosition), avg(inputs.elevatorVelocity)));
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
        io.setElevatorVoltage(percent * Constants.ELEVATOR_NOMINAL_VOLTAGE);
    }

    private boolean homing = false;
    private double homeTime = 0;

    public void homeElevator() {
        homeTime = ELEVATOR_MIN_HOME_TIME;
        homing = true;
    }

    @Override
    public void update() {
        io.updateInputs(inputs);
        Logger.getInstance().processInputs("Elevator", inputs);

        if (homing) {
            if (DriverStation.isEnabled()) {
                homeTime -= Constants.NOMINAL_DT;
                setPercentOutput(MOTOR_SPEED_DECREASING_RATE);
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
        double acceleration = (state.velocity - pastVelocity) / (currentTime - pastTime);
        double arbFFVoltage = Constants.ELEVATOR_FEEDFORWARD.calculate(state.velocity, acceleration);
        io.setElevatorPosition(state.position, arbFFVoltage);

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

    @Override
    public void selfTest() {

    }
}
