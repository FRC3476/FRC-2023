package frc.subsytem;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

public class Grabber extends AbstractSubsystem {
    private final CANSparkMax pivotSparkMax;
    private final CANSparkMax grabberSparkMax;
    private static final Grabber instance = new Grabber();

    public static Grabber getInstance() {
        return instance;
    }

    private Grabber() {
        super(Constants.GRABBER_PERIOD, 5);
        pivotSparkMax = new CANSparkMax(0, CANSparkMaxLowLevel.MotorType.kBrushless);
        grabberSparkMax = new CANSparkMax(0, CANSparkMaxLowLevel.MotorType.kBrushless);
        SparkMaxPIDController pivotSparkMaxPIDController = pivotSparkMax.getPIDController();
        pivotSparkMaxPIDController.setP(Constants.GRABBER_P);
        pivotSparkMaxPIDController.setI(Constants.GRABBER_I);
        pivotSparkMaxPIDController.setD(Constants.GRABBER_D);
        pivotSparkMax.enableVoltageCompensation(Constants.GRABBER_NOMINAL_VOLTAGE);
        pivotSparkMax.setSmartCurrentLimit(Constants.PIVOT_SMART_CURRENT_LIMIT);
        grabberSparkMax.enableVoltageCompensation(Constants.GRABBER_NOMINAL_VOLTAGE);
        grabberSparkMax.setSmartCurrentLimit(Constants.GRABBER_SMART_CURRENT_LIMIT);
    }

    private TrapezoidProfile trapezoidProfile =
            new TrapezoidProfile(new TrapezoidProfile.Constraints(Constants.GRABBER_CONSTRAINTS.maxVelocity,
                    Constants.GRABBER_CONSTRAINTS.maxAcceleration),
                    new TrapezoidProfile.State());
    private double trapezoidProfileStartTime = 0;

    /**
     * @param position The position to set the grabber (degrees)
     */
    public void setPosition(double position) {
        trapezoidProfile = new TrapezoidProfile(Constants.GRABBER_CONSTRAINTS, new TrapezoidProfile.State(position, 0),
                new TrapezoidProfile.State(pivotSparkMax.getEncoder().getPosition() / Constants.GRABBER_ROTATIONS_PER_DEGREE,
                        pivotSparkMax.getEncoder().getVelocity() / Constants.GRABBER_ROTATIONS_PER_DEGREE));
        trapezoidProfileStartTime = Timer.getFPGATimestamp();
        logData("Goal position", position);
    }

    private double pastVelocity = 0, pastTime = 0;

    @Override
    public void update() {
        double currentTime = Timer.getFPGATimestamp();
        if (trapezoidProfileStartTime == -1) {
            trapezoidProfileStartTime = currentTime;
        }
        TrapezoidProfile.State state = trapezoidProfile.calculate(currentTime - trapezoidProfileStartTime);
        double acceleration = (state.velocity - pastVelocity) / (currentTime - pastTime);

        pivotSparkMax.getPIDController().setReference(state.position * Constants.GRABBER_ROTATIONS_PER_DEGREE,
                CANSparkMax.ControlType.kPosition, 0, Constants.GRABBER_FEEDFORWARD.calculate(state.velocity, acceleration),
                SparkMaxPIDController.ArbFFUnits.kVoltage);

        pastVelocity = state.velocity;
        pastTime = currentTime;

        logData("Wanted pos", state.position / Constants.GRABBER_ROTATIONS_PER_DEGREE);
        logData("Wanted vel", state.velocity / Constants.GRABBER_ROTATIONS_PER_DEGREE / 60);
        logData("Wanted accel", acceleration);
        logData("Total trapezoidProfile time", trapezoidProfile.totalTime());
        logData("Profile length", currentTime - trapezoidProfileStartTime);
        logData("TrapezoidProfile error", state.position
                - pivotSparkMax.getEncoder().getPosition());
    }

    @Override
    public void logData() {
        logData("Motor Position", pivotSparkMax.getEncoder().getPosition() / Constants.GRABBER_ROTATIONS_PER_DEGREE);
        logData("Motor Velocity", pivotSparkMax.getEncoder().getVelocity() / Constants.GRABBER_ROTATIONS_PER_DEGREE / 60);
        logData("Motor current", pivotSparkMax.getOutputCurrent());
        logData("Motor temperature", pivotSparkMax.getMotorTemperature());
    }

    public enum GrabState {
        OPEN(-3),
        GRAB_CUBE(3),
        GRAB_CONE(6);
        double voltage;

        GrabState(double voltage) {
            this.voltage = voltage;
        }
    }

    public void setGrabState(GrabState grabState) {
        grabberSparkMax.setVoltage(grabState.voltage);
    }

    @Override
    public void selfTest() {

    }
}
