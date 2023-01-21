package frc.subsytem;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

public class TelescopingArm extends AbstractSubsystem {
    private final CANSparkMax telescopingArmSparkMax;
    private final SparkMaxPIDController telescopingArmSparkMaxPIDController;
    private static final TelescopingArm instance = new TelescopingArm();

    public static TelescopingArm getInstance() {
        return instance;
    }

    public TelescopingArm() {
        super(Constants.TELESCOPING_ARM_PERIOD, 5);
        telescopingArmSparkMax = new CANSparkMax(0, CANSparkMaxLowLevel.MotorType.kBrushless);
        telescopingArmSparkMaxPIDController = telescopingArmSparkMax.getPIDController();
        telescopingArmSparkMaxPIDController.setP(Constants.TELESCOPING_ARM_P);
        telescopingArmSparkMaxPIDController.setI(Constants.TELESCOPING_ARM_I);
        telescopingArmSparkMaxPIDController.setD(Constants.TELESCOPING_ARM_D);
    }

    private TrapezoidProfile trapezoidProfile =
            new TrapezoidProfile(new TrapezoidProfile.Constraints(Constants.TELESCOPING_ARM_CONSTRAINTS.maxVelocity,
                    Constants.TELESCOPING_ARM_CONSTRAINTS.maxAcceleration),
                    new TrapezoidProfile.State());
    private double trapezoidProfileStartTime = 0;

    /**
     * @param position The position to set the telescoping arm (meters)
     */
    public void setPosition(double position) {
        trapezoidProfile = new TrapezoidProfile(Constants.TELESCOPING_ARM_CONSTRAINTS, new TrapezoidProfile.State(position, 0),
                new TrapezoidProfile.State(telescopingArmSparkMax.getEncoder().getPosition() / Constants.TELESCOPING_ARM_POSITION_MULTIPLIER,
                        telescopingArmSparkMax.getEncoder().getVelocity() / Constants.TELESCOPING_ARM_POSITION_MULTIPLIER));
        trapezoidProfileStartTime = -1;
        logData("Goal position", position);
    }

    private double pastVelocity = 0, pastTime = 0;

    @Override
    public void update() {
        double acceleration;
        double currentTime = Timer.getFPGATimestamp();
        if (trapezoidProfileStartTime == -1) {
            trapezoidProfileStartTime = currentTime;
        }
        TrapezoidProfile.State state = trapezoidProfile.calculate(currentTime - trapezoidProfileStartTime);
        acceleration = (state.velocity - pastVelocity) / (currentTime - pastTime);

        telescopingArmSparkMax.getPIDController().setReference(state.position * Constants.TELESCOPING_ARM_POSITION_MULTIPLIER,
                CANSparkMax.ControlType.kPosition, 0, Constants.TELESCOPING_ARM_FEEDFORWARD.calculate(state.velocity, acceleration),
                SparkMaxPIDController.ArbFFUnits.kVoltage);

        pastVelocity = state.velocity;
        pastTime = currentTime;

        logData("Wanted pos", state.position);
        logData("Wanted vel", state.velocity);
        logData("Wanted accel", acceleration);
        logData("Total trapezoidProfile time", trapezoidProfile.totalTime());
        logData("TrapezoidProfile time", currentTime - trapezoidProfileStartTime);
        logData("TrapezoidProfile error", state.position
                - telescopingArmSparkMax.getEncoder().getPosition());
    }

    @Override
    public void logData() {
        logData("Motor Position", telescopingArmSparkMax.getEncoder().getPosition() * Constants.ELEVATOR_POSITION_MULTIPLIER);
        logData("Motor Velocity", telescopingArmSparkMax.getEncoder().getVelocity());
        logData("Motor current", telescopingArmSparkMax.getOutputCurrent());
        logData("Motor temperature", telescopingArmSparkMax.getMotorTemperature());
    }

    @Override
    public void selfTest() {

    }
}
