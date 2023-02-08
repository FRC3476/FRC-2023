package frc.subsytem;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

import static frc.robot.Constants.SECONDS_PER_MINUTE;
import static frc.robot.Constants.TELESCOPING_ARM_ROTATIONS_PER_METER;

public class TelescopingArm extends AbstractSubsystem {
    private final CANSparkMax telescopingArmSparkMax;
    private static final TelescopingArm instance = new TelescopingArm();

    public static TelescopingArm getInstance() {
        return instance;
    }

    private TelescopingArm() {
        super(Constants.TELESCOPING_ARM_PERIOD, 5);
        telescopingArmSparkMax = new CANSparkMax(0, CANSparkMaxLowLevel.MotorType.kBrushless);
        SparkMaxPIDController telescopingArmSparkMaxPIDController = telescopingArmSparkMax.getPIDController();
        telescopingArmSparkMaxPIDController.setP(Constants.TELESCOPING_ARM_P);
        telescopingArmSparkMaxPIDController.setI(Constants.TELESCOPING_ARM_I);
        telescopingArmSparkMaxPIDController.setD(Constants.TELESCOPING_ARM_D);
        telescopingArmSparkMax.enableVoltageCompensation(Constants.TELESCOPING_ARM_NOMINAL_VOLTAGE);
        telescopingArmSparkMax.setSmartCurrentLimit(Constants.TELESCOPING_ARM_SMART_CURRENT_LIMIT);

        telescopingArmSparkMax.getEncoder().setPositionConversionFactor(1.0 / TELESCOPING_ARM_ROTATIONS_PER_METER);
        telescopingArmSparkMax.getEncoder().setVelocityConversionFactor(
                (1.0 / TELESCOPING_ARM_ROTATIONS_PER_METER) / SECONDS_PER_MINUTE);
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
                new TrapezoidProfile.State(
                        telescopingArmSparkMax.getEncoder().getPosition(),
                        telescopingArmSparkMax.getEncoder().getVelocity()));
        trapezoidProfileStartTime = -1;
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

        telescopingArmSparkMax.getPIDController().setReference(state.position,
                CANSparkMax.ControlType.kPosition, 0,
                Constants.TELESCOPING_ARM_FEEDFORWARD.calculate(state.velocity, acceleration),
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
        logData("Motor Position", telescopingArmSparkMax.getEncoder().getPosition());
        logData("Motor Velocity", telescopingArmSparkMax.getEncoder().getVelocity());
        logData("Motor current", telescopingArmSparkMax.getOutputCurrent());
        logData("Motor temperature", telescopingArmSparkMax.getMotorTemperature());
    }

    @Override
    public void selfTest() {

    }
}
