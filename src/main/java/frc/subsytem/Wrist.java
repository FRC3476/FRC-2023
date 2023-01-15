package frc.subsytem;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

public class Wrist extends AbstractSubsystem{
    Wrist wrist;
    static CANSparkMax sparkMax;
    static SparkMaxPIDController sparkMaxPIDController;

    public Wrist() {
        super(20, 5);
        wrist = new Wrist();
        sparkMax = new CANSparkMax(0, CANSparkMaxLowLevel.MotorType.kBrushless);
        sparkMaxPIDController = sparkMax.getPIDController();
        sparkMaxPIDController.setP(Constants.WRIST_P);
        sparkMaxPIDController.setI(Constants.WRIST_I);
        sparkMaxPIDController.setD(Constants.WRIST_D);
    }

    TrapezoidProfile trapezoidProfile =
            new TrapezoidProfile(new TrapezoidProfile.Constraints(Constants.WRIST_MAX_VELOCITY, Constants.WRIST_MAX_ACCELERATION),
                    new TrapezoidProfile.State());
    double setPositionPastTime = 0;
    public void setPosition(double position) {
        TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(Constants.WRIST_MAX_VELOCITY, Constants.WRIST_MAX_ACCELERATION);

        trapezoidProfile = new TrapezoidProfile(constraints, new TrapezoidProfile.State(position, 0),
                new TrapezoidProfile.State(sparkMax.getEncoder().getPosition() / Constants.WRIST_POSITION_MULTIPLIER,
                        sparkMax.getEncoder().getVelocity() / Constants.WRIST_POSITION_MULTIPLIER));
        setPositionPastTime = Timer.getFPGATimestamp();
    }
    double pastVelocity = 0, pastTime = 0;
    @Override
    public void update() {
        TrapezoidProfile.State state = trapezoidProfile.calculate(Timer.getFPGATimestamp() - setPositionPastTime);
        double acceleration = (state.velocity - pastVelocity) / (Timer.getFPGATimestamp() - pastTime);

        sparkMax.getPIDController().setReference(state.position * Constants.WRIST_POSITION_MULTIPLIER,
                CANSparkMax.ControlType.kPosition, 0, feedForward(state.velocity, acceleration),
                SparkMaxPIDController.ArbFFUnits.kVoltage);

        pastVelocity = state.velocity;
        pastTime = Timer.getFPGATimestamp();
    }

    public double feedForward(double velocity, double acceleration) {
        ElevatorFeedforward wristFeedforward = new ElevatorFeedforward(0, 0, 0, 0);
        return wristFeedforward.calculate(velocity, acceleration);
    }

    @Override
    public void logData() {
        logData("Motor Position", sparkMax.getEncoder().getPosition() * Constants.WRIST_POSITION_MULTIPLIER);
    }

    @Override
    public void selfTest() {

    }
}
