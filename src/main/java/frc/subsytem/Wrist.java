package frc.subsytem;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import org.checkerframework.checker.units.qual.C;

public class Wrist extends AbstractSubsystem {
    private final CANSparkMax wristSparkMax;
    private final SparkMaxPIDController wristSparkMaxPIDController;
    private static final Wrist instance = new Wrist();

    public Wrist() {
        super(20, 5);
        wristSparkMax = new CANSparkMax(0, CANSparkMaxLowLevel.MotorType.kBrushless);
        wristSparkMaxPIDController = wristSparkMax.getPIDController();
        wristSparkMaxPIDController.setP(Constants.WRIST_P);
        wristSparkMaxPIDController.setI(Constants.WRIST_I);
        wristSparkMaxPIDController.setD(Constants.WRIST_D);
    }

    static TrapezoidProfile trapezoidProfile =
            new TrapezoidProfile(new TrapezoidProfile.Constraints(Constants.wristConstraints.maxVelocity,
                    Constants.wristConstraints.maxAcceleration),
                    new TrapezoidProfile.State());
    static double setPositionPastTime = 0;
    static double goalPosition;

    /**
     * This method takes in meters
     */
    public void setPosition(double position) {
        trapezoidProfile = new TrapezoidProfile(Constants.wristConstraints, new TrapezoidProfile.State(position, 0),
                new TrapezoidProfile.State(wristSparkMax.getEncoder().getPosition() / Constants.WRIST_POSITION_MULTIPLIER,
                        wristSparkMax.getEncoder().getVelocity() / Constants.WRIST_POSITION_MULTIPLIER));
        setPositionPastTime = Timer.getFPGATimestamp();
        goalPosition = position;
        logData("Goal position", position);
    }

    double pastVelocity = 0, pastTime = 0;

    @Override
    public void update() {
        TrapezoidProfile.State state = trapezoidProfile.calculate(Timer.getFPGATimestamp() - setPositionPastTime);
        double acceleration = (state.velocity - pastVelocity) / (Timer.getFPGATimestamp() - pastTime);

        wristSparkMax.getPIDController().setReference(state.position * Constants.WRIST_POSITION_MULTIPLIER,
                CANSparkMax.ControlType.kPosition, 0, Constants.wristFeedforward.calculate(state.velocity, acceleration),
                SparkMaxPIDController.ArbFFUnits.kVoltage);

        pastVelocity = state.velocity;
        pastTime = Timer.getFPGATimestamp();
    }

    @Override
    public void logData() {
        logData("Motor Position", wristSparkMax.getEncoder().getPosition() * Constants.ELEVATOR_POSITION_MULTIPLIER);
        logData("Goal position for the trapezoidProfile", goalPosition);
        logData("Motor current", wristSparkMax.getOutputCurrent());
        logData("Motor temperature", wristSparkMax.getMotorTemperature());
        logData("Time in the trapezoidProfile", trapezoidProfile.totalTime());
        if (trapezoidProfile.isFinished(Timer.getFPGATimestamp())) {
            logData("Error from the trapezoidProfile", trapezoidProfile.calculate(Timer.getFPGATimestamp()));
        }
    }

    @Override
    public void selfTest() {

    }

    public static Wrist getInstance() {
        return instance;
    }
}
