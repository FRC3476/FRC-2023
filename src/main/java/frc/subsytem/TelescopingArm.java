package frc.subsytem;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import org.checkerframework.checker.units.qual.C;

public class TelescopingArm extends AbstractSubsystem {
    private final CANSparkMax telescopingArmSparkMax;
    private final SparkMaxPIDController telescopingArmSparkMaxPIDController;
    private static final TelescopingArm instance = new TelescopingArm();

    public TelescopingArm() {
        super(20, 5);
        telescopingArmSparkMax = new CANSparkMax(0, CANSparkMaxLowLevel.MotorType.kBrushless);
        telescopingArmSparkMaxPIDController = telescopingArmSparkMax.getPIDController();
        telescopingArmSparkMaxPIDController.setP(Constants.TELESCOPING_ARM_P);
        telescopingArmSparkMaxPIDController.setI(Constants.TELESCOPING_ARM_I);
        telescopingArmSparkMaxPIDController.setD(Constants.TELESCOPING_ARM_D);
    }

    static TrapezoidProfile trapezoidProfile =
            new TrapezoidProfile(new TrapezoidProfile.Constraints(Constants.telescopingArmConstraints.maxVelocity,
                    Constants.telescopingArmConstraints.maxAcceleration),
                    new TrapezoidProfile.State());
    static double setPositionPastTime = 0;

    /**
     * This method takes in meters
     */
    public void setPosition(double position) {
        trapezoidProfile = new TrapezoidProfile(Constants.telescopingArmConstraints, new TrapezoidProfile.State(position, 0),
                new TrapezoidProfile.State(telescopingArmSparkMax.getEncoder().getPosition() / Constants.TELESCOPING_ARM_POSITION_MULTIPLIER,
                        telescopingArmSparkMax.getEncoder().getVelocity() / Constants.TELESCOPING_ARM_POSITION_MULTIPLIER));
        setPositionPastTime = Timer.getFPGATimestamp();
        logData("Goal position", position);
    }

    double pastVelocity = 0, pastTime = 0;

    @Override
    public void update() {
        TrapezoidProfile.State state = trapezoidProfile.calculate(Timer.getFPGATimestamp() - setPositionPastTime);
        double acceleration = (state.velocity - pastVelocity) / (Timer.getFPGATimestamp() - pastTime);

        telescopingArmSparkMax.getPIDController().setReference(state.position * Constants.TELESCOPING_ARM_POSITION_MULTIPLIER,
                CANSparkMax.ControlType.kPosition, 0, Constants.telescopingArmFeedforward.calculate(state.velocity, acceleration),
                SparkMaxPIDController.ArbFFUnits.kVoltage);

        pastVelocity = state.velocity;
        pastTime = Timer.getFPGATimestamp();
    }

    @Override
    public void logData() {
        logData("Motor Position", telescopingArmSparkMax.getEncoder().getPosition() * Constants.ELEVATOR_POSITION_MULTIPLIER);
        logData("Current position of the trapezoidProfile", trapezoidProfile.calculate(Timer.getFPGATimestamp()));
        logData("Motor current", telescopingArmSparkMax.getOutputCurrent());
        logData("Motor temperature", telescopingArmSparkMax.getMotorTemperature());
        logData("Time in the trapezoidProfile", trapezoidProfile.totalTime());
        if (trapezoidProfile.isFinished(Timer.getFPGATimestamp())) {
            logData("Error from the trapezoidProfile", trapezoidProfile.calculate(Timer.getFPGATimestamp()));
        }
    }

    @Override
    public void selfTest() {

    }

    public static TelescopingArm getInstance() {
        return instance;
    }
}
