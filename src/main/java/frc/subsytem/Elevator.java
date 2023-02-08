package frc.subsytem;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

import static frc.robot.Constants.ELEVATOR_ROTATIONS_PER_METER;
import static frc.robot.Constants.SECONDS_PER_MINUTE;

public class Elevator extends AbstractSubsystem {
    private final CANSparkMax elevatorSparkMax;
    private final static Elevator instance = new Elevator();

    public static Elevator getInstance() {
        return instance;
    }

    private Elevator() {
        super(Constants.ELEVATOR_PERIOD, 5);
        elevatorSparkMax = new CANSparkMax(0, CANSparkMaxLowLevel.MotorType.kBrushless);
        SparkMaxPIDController elevatorSparkMaxPIDController = elevatorSparkMax.getPIDController();
        elevatorSparkMaxPIDController.setP(Constants.ELEVATOR_P);
        elevatorSparkMaxPIDController.setI(Constants.ELEVATOR_I);
        elevatorSparkMaxPIDController.setD(Constants.ELEVATOR_D);
        elevatorSparkMax.enableVoltageCompensation(Constants.ELEVATOR_NOMINAL_VOLTAGE);
        elevatorSparkMax.setSmartCurrentLimit(Constants.ELEVATOR_SMART_CURRENT_LIMIT);
        elevatorSparkMax.getEncoder().setPositionConversionFactor(1.0 / ELEVATOR_ROTATIONS_PER_METER);
        elevatorSparkMax.getEncoder().setVelocityConversionFactor((1.0 / ELEVATOR_ROTATIONS_PER_METER) / SECONDS_PER_MINUTE);
    }

    private TrapezoidProfile trapezoidProfile =
            new TrapezoidProfile(Constants.ELEVATOR_CONSTRAINTS, new TrapezoidProfile.State());
    private double trapezoidProfileStartTime = 0;

    /**
     * @param position The position to set the elevator (meters)
     */
    public void setPosition(double position) {
        trapezoidProfile = new TrapezoidProfile(Constants.ELEVATOR_CONSTRAINTS, new TrapezoidProfile.State(position, 0),
                new TrapezoidProfile.State(elevatorSparkMax.getEncoder().getPosition(),
                        elevatorSparkMax.getEncoder().getVelocity()));
        trapezoidProfileStartTime = -1;
        logData("Goal position", position);
    }

    private double pastVelocity = 0, pastTime = 0;

    /**
     * Controls elevator motor with percent output
     *
     * @param percent Spans from -1 to 1 where the extremes are full power and direction depends on the sign
     */
    public void setPercentOutput(double percent) {
        elevatorSparkMax.setVoltage(percent * 12);
    }

    @Override
    public void update() {
        double currentTime = Timer.getFPGATimestamp();
        if (trapezoidProfileStartTime == -1) {
            trapezoidProfileStartTime = currentTime;
        }
        TrapezoidProfile.State state = trapezoidProfile.calculate(currentTime - trapezoidProfileStartTime);
        double acceleration = (state.velocity - pastVelocity) / (currentTime - pastTime);

        elevatorSparkMax.getPIDController().setReference(state.position,
                CANSparkMax.ControlType.kPosition, 0, Constants.ELEVATOR_FEEDFORWARD.calculate(state.velocity, acceleration),
                SparkMaxPIDController.ArbFFUnits.kVoltage);

        pastVelocity = state.velocity;
        pastTime = currentTime;

        logData("Wanted pos", state.position);
        logData("Wanted vel", state.velocity);
        logData("Wanted accel", acceleration);
        logData("Total trapezoidProfile time", trapezoidProfile.totalTime());
        logData("TrapezoidProfile time", currentTime - trapezoidProfileStartTime);
        logData("TrapezoidProfile Error", state.position
                - elevatorSparkMax.getEncoder().getPosition());
    }

    @Override
    public void logData() {
        logData("Motor Position", elevatorSparkMax.getEncoder().getPosition());
        logData("Motor Velocity", elevatorSparkMax.getEncoder().getVelocity());
        logData("Motor current", elevatorSparkMax.getOutputCurrent());
        logData("Motor temperature", elevatorSparkMax.getMotorTemperature());
    }

    @Override
    public void selfTest() {

    }
}
