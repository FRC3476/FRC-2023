package frc.subsytem;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

public class Elevator extends AbstractSubsystem{
    static CANSparkMax sparkMax;
    static SparkMaxPIDController sparkMaxPIDController;

    public Elevator() {
        super(20, 5);
        sparkMax = new CANSparkMax(0, CANSparkMaxLowLevel.MotorType.kBrushless);
        sparkMaxPIDController = sparkMax.getPIDController();
        sparkMaxPIDController.setP(Constants.ELEVATOR_P);
        sparkMaxPIDController.setI(Constants.ELEVATOR_I);
        sparkMaxPIDController.setD(Constants.ELEVATOR_D);
    }

    static TrapezoidProfile trapezoidProfile =
            new TrapezoidProfile(new TrapezoidProfile.Constraints(Constants.ELEVATOR_MAX_VELOCITY, Constants.ELEVATOR_MAX_ACCELERATION),
                    new TrapezoidProfile.State());
    static double setPositionPastTime = 0;
    public static void setPosition(double position) {
        TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(Constants.ELEVATOR_MAX_VELOCITY, Constants.ELEVATOR_MAX_ACCELERATION);

        trapezoidProfile = new TrapezoidProfile(constraints, new TrapezoidProfile.State(position, 0),
                new TrapezoidProfile.State(sparkMax.getEncoder().getPosition() / Constants.ELEVATOR_POSITION_MULTIPLIER,
                        sparkMax.getEncoder().getVelocity() / Constants.ELEVATOR_POSITION_MULTIPLIER));
        setPositionPastTime = Timer.getFPGATimestamp();
    }
    double pastVelocity = 0, pastTime = 0;
    @Override
    public void update() {
        TrapezoidProfile.State state = trapezoidProfile.calculate(Timer.getFPGATimestamp() - setPositionPastTime);
        double acceleration = (state.velocity - pastVelocity) / (Timer.getFPGATimestamp() - pastTime);

        sparkMax.getPIDController().setReference(state.position * Constants.ELEVATOR_POSITION_MULTIPLIER,
                CANSparkMax.ControlType.kPosition, 0, feedForward(state.velocity, acceleration),
                SparkMaxPIDController.ArbFFUnits.kVoltage);

        pastVelocity = state.velocity;
        pastTime = Timer.getFPGATimestamp();
    }

    public double feedForward(double velocity, double acceleration) {
        ElevatorFeedforward elevatorFeedforward = new ElevatorFeedforward(0, 0, 0, 0);
        return elevatorFeedforward.calculate(velocity, acceleration);
    }

    @Override
    public void logData() {
        logData("Motor Position", sparkMax.getEncoder().getPosition() * Constants.ELEVATOR_POSITION_MULTIPLIER);
    }

    @Override
    public void selfTest() {

    }
}
