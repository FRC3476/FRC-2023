package frc.subsytem;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

public class TelescopingArm extends AbstractSubsystem{
    TelescopingArm telescopingArm;
    static CANSparkMax sparkMax;
    static SparkMaxPIDController sparkMaxPIDController;

    public TelescopingArm() {
        super(20, 5);
        telescopingArm = new TelescopingArm();
        sparkMax = new CANSparkMax(0, CANSparkMaxLowLevel.MotorType.kBrushless);
        sparkMaxPIDController = sparkMax.getPIDController();
        sparkMaxPIDController.setP(Constants.TELESCOPING_ARM_P);
        sparkMaxPIDController.setI(Constants.TELESCOPING_ARM_I);
        sparkMaxPIDController.setD(Constants.TELESCOPING_ARM_D);
    }

    TrapezoidProfile trapezoidProfile =
            new TrapezoidProfile(new TrapezoidProfile.Constraints(Constants.TELESCOPING_ARM_MAX_VELOCITY, Constants.TELESCOPING_ARM_MAX_ACCELERATION),
                    new TrapezoidProfile.State());
    double setPositionPastTime = 0;
    public void setPosition(double position) {
        TrapezoidProfile.Constraints constraints = new TrapezoidProfile.Constraints(Constants.TELESCOPING_ARM_MAX_VELOCITY, Constants.TELESCOPING_ARM_MAX_ACCELERATION);

        trapezoidProfile = new TrapezoidProfile(constraints, new TrapezoidProfile.State(position, 0),
                new TrapezoidProfile.State(sparkMax.getEncoder().getPosition() / Constants.TELESCOPING_ARM_POSITION_MULTIPLIER,
                        sparkMax.getEncoder().getVelocity() / Constants.TELESCOPING_ARM_POSITION_MULTIPLIER));
        setPositionPastTime = Timer.getFPGATimestamp();
    }
    double pastVelocity = 0, pastTime = 0;
    @Override
    public void update() {
        TrapezoidProfile.State state = trapezoidProfile.calculate(Timer.getFPGATimestamp() - setPositionPastTime);
        double acceleration = (state.velocity - pastVelocity) / (Timer.getFPGATimestamp() - pastTime);

        sparkMax.getPIDController().setReference(state.position * Constants.TELESCOPING_ARM_POSITION_MULTIPLIER,
                CANSparkMax.ControlType.kPosition, 0, feedForward(state.velocity, acceleration),
                SparkMaxPIDController.ArbFFUnits.kVoltage);

        pastVelocity = state.velocity;
        pastTime = Timer.getFPGATimestamp();
    }

    public double feedForward(double velocity, double acceleration) {
        ElevatorFeedforward telescopingArmFeedforward = new ElevatorFeedforward(0, 0, 0, 0);
        return telescopingArmFeedforward.calculate(velocity, acceleration);
    }

    @Override
    public void logData() {
        logData("Motor Position", sparkMax.getEncoder().getPosition() * Constants.TELESCOPING_ARM_POSITION_MULTIPLIER);
    }

    @Override
    public void selfTest() {

    }
}
