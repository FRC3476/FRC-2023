package frc.subsytem;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

public class Elevator extends AbstractSubsystem {
    private final CANSparkMax elevatorSparkMax;
    private final SparkMaxPIDController elevatorSparkMaxPIDController;
    private final static Elevator instance = new Elevator();

    public Elevator() {
        super(20, 5);
        elevatorSparkMax = new CANSparkMax(0, CANSparkMaxLowLevel.MotorType.kBrushless);
        elevatorSparkMaxPIDController = elevatorSparkMax.getPIDController();
        elevatorSparkMaxPIDController.setP(Constants.ELEVATOR_P);
        elevatorSparkMaxPIDController.setI(Constants.ELEVATOR_I);
        elevatorSparkMaxPIDController.setD(Constants.ELEVATOR_D);
    }

    static TrapezoidProfile trapezoidProfile =
            new TrapezoidProfile(new TrapezoidProfile.Constraints(Constants.elevatorConstraints.maxVelocity,
                    Constants.elevatorConstraints.maxAcceleration),
                    new TrapezoidProfile.State());
    static double setPositionPastTime = 0;

    /**
     * This method takes in meters
     */
    public void setPosition(double position) {
        trapezoidProfile = new TrapezoidProfile(Constants.elevatorConstraints, new TrapezoidProfile.State(position, 0),
                new TrapezoidProfile.State(elevatorSparkMax.getEncoder().getPosition() / Constants.ELEVATOR_POSITION_MULTIPLIER,
                        elevatorSparkMax.getEncoder().getVelocity() / Constants.ELEVATOR_POSITION_MULTIPLIER));
        setPositionPastTime = Timer.getFPGATimestamp();
        logData("Goal position", position);
    }

    double pastVelocity = 0, pastTime = 0;

    @Override
    public void update() {
        TrapezoidProfile.State state = trapezoidProfile.calculate(Timer.getFPGATimestamp() - setPositionPastTime);
        double acceleration = (state.velocity - pastVelocity) / (Timer.getFPGATimestamp() - pastTime);

        elevatorSparkMax.getPIDController().setReference(state.position * Constants.ELEVATOR_POSITION_MULTIPLIER,
                CANSparkMax.ControlType.kPosition, 0, Constants.elevatorFeedforward.calculate(state.velocity, acceleration),
                SparkMaxPIDController.ArbFFUnits.kVoltage);

        pastVelocity = state.velocity;
        pastTime = Timer.getFPGATimestamp();
    }

    @Override
    public void logData() {
        logData("Motor Position", elevatorSparkMax.getEncoder().getPosition() * Constants.ELEVATOR_POSITION_MULTIPLIER);
        logData("Current position of the trapezoidProfile", trapezoidProfile.calculate(Timer.getFPGATimestamp()));
        logData("Motor current", elevatorSparkMax.getOutputCurrent());
        logData("Motor temperature", elevatorSparkMax.getMotorTemperature());
        logData("Time in the trapezoidProfile", trapezoidProfile.totalTime());
        if (trapezoidProfile.isFinished(Timer.getFPGATimestamp())) {
            logData("Error from the trapezoidProfile", trapezoidProfile.calculate(Timer.getFPGATimestamp()));
        }
    }

    @Override
    public void selfTest() {

    }

    public static Elevator getInstance() {
        return instance;
    }
}
