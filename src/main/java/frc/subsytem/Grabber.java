package frc.subsytem;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

public class Grabber extends AbstractSubsystem {
    private final CANSparkMax wristSparkMax;
    private final SparkMaxPIDController wristSparkMaxPIDController;
    private static final Grabber instance = new Grabber();
    public static Grabber getInstance() {
        return instance;
    }
    public Grabber() {
        super(Constants.WRIST_PERIOD, 5);
        wristSparkMax = new CANSparkMax(0, CANSparkMaxLowLevel.MotorType.kBrushless);
        wristSparkMaxPIDController = wristSparkMax.getPIDController();
        wristSparkMaxPIDController.setP(Constants.WRIST_P);
        wristSparkMaxPIDController.setI(Constants.WRIST_I);
        wristSparkMaxPIDController.setD(Constants.WRIST_D);
    }

    private TrapezoidProfile trapezoidProfile =
            new TrapezoidProfile(new TrapezoidProfile.Constraints(Constants.WRIST_CONSTRAINTS.maxVelocity,
                    Constants.WRIST_CONSTRAINTS.maxAcceleration),
                    new TrapezoidProfile.State());
    private double trapezoidProfileStartTime = 0;

    /**
     * @param position The position to set the grabber (radians/degrees)
     */
    public void setPosition(double position) {
        trapezoidProfile = new TrapezoidProfile(Constants.WRIST_CONSTRAINTS, new TrapezoidProfile.State(position, 0),
                new TrapezoidProfile.State(wristSparkMax.getEncoder().getPosition() / Constants.WRIST_POSITION_MULTIPLIER,
                        wristSparkMax.getEncoder().getVelocity() / Constants.WRIST_POSITION_MULTIPLIER));
        trapezoidProfileStartTime = Timer.getFPGATimestamp();
        logData("Goal position", position);
    }

    private double pastVelocity = 0, pastTime = 0, currentTime, acceleration;

    @Override
    public void update() {
        if(trapezoidProfileStartTime == -1) {
            currentTime = Timer.getFPGATimestamp();
        }
        TrapezoidProfile.State state = trapezoidProfile.calculate(currentTime - trapezoidProfileStartTime);
        acceleration = (state.velocity - pastVelocity) / (currentTime - pastTime);

        wristSparkMax.getPIDController().setReference(state.position * Constants.WRIST_POSITION_MULTIPLIER,
                CANSparkMax.ControlType.kPosition, 0, Constants.WRIST_FEEDFORWARD.calculate(state.velocity, acceleration),
                SparkMaxPIDController.ArbFFUnits.kVoltage);

        pastVelocity = state.velocity;
        pastTime = currentTime;

        logData("Wanted pos", state.position);
        logData("Wanted vel", state.velocity);
        logData("Wanted accel", acceleration);
        logData("Total trapezoidProfile time", trapezoidProfile.totalTime());
        logData("Profile length", Timer.getFPGATimestamp() - trapezoidProfileStartTime);
        if (trapezoidProfile.isFinished(Timer.getFPGATimestamp())) {
            logData("TrapezoidProfile error", trapezoidProfile.calculate(Timer.getFPGATimestamp()).position
                    - wristSparkMax.getEncoder().getPosition());
        }
    }

    @Override
    public void logData() {
        logData("Motor Position", wristSparkMax.getEncoder().getPosition() * Constants.ELEVATOR_POSITION_MULTIPLIER);
        logData("Motor current", wristSparkMax.getOutputCurrent());
        logData("Motor temperature", wristSparkMax.getMotorTemperature());
    }

    @Override
    public void selfTest() {

    }
}
