package frc.subsytem.telescopingarm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;
import frc.robot.Constants;

import static edu.wpi.first.wpilibj.RobotBase.isReal;
import static frc.robot.Constants.*;

public class TelescopingArmIOSparkMax extends TelescopingArmIO {
    private final CANSparkMax telescopingArmSparkMax;

    public TelescopingArmIOSparkMax() {
        telescopingArmSparkMax = new CANSparkMax(TELESCOPING_ARM_CAN_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
        SparkMaxPIDController telescopingArmSparkMaxPIDController = telescopingArmSparkMax.getPIDController();
        telescopingArmSparkMaxPIDController.setP(Constants.TELESCOPING_ARM_P);
        telescopingArmSparkMaxPIDController.setI(Constants.TELESCOPING_ARM_I);
        telescopingArmSparkMaxPIDController.setD(Constants.TELESCOPING_ARM_D);
        telescopingArmSparkMax.enableVoltageCompensation(Constants.TELESCOPING_ARM_NOMINAL_VOLTAGE);
        telescopingArmSparkMax.setSmartCurrentLimit(Constants.TELESCOPING_ARM_SMART_CURRENT_LIMIT);
        telescopingArmSparkMax.setSecondaryCurrentLimit(80);
        telescopingArmSparkMax.setInverted(true);

        telescopingArmSparkMax.getEncoder().setPositionConversionFactor(1.0 / TELESCOPING_ARM_ROTATIONS_PER_METER);
        telescopingArmSparkMax.getEncoder().setVelocityConversionFactor(
                (1.0 / TELESCOPING_ARM_ROTATIONS_PER_METER) / SECONDS_PER_MINUTE);
        resetTelescopingArmPosition(0);
        telescopingArmSparkMax.getPIDController().setOutputRange(-0.8, 0.6);
        if (isReal()) {
            telescopingArmSparkMax.burnFlash();
        }
    }


    @Override
    public void updateInputs(TelescopingArmInputs inputs) {
        inputs.position = telescopingArmSparkMax.getEncoder().getPosition();
        inputs.velocity = telescopingArmSparkMax.getEncoder().getVelocity();
        inputs.current = telescopingArmSparkMax.getOutputCurrent();
        inputs.voltage = telescopingArmSparkMax.getAppliedOutput() * Constants.GRABBER_NOMINAL_VOLTAGE;
        inputs.temp = telescopingArmSparkMax.getMotorTemperature();
    }

    @Override
    public void setTelescopingArmVoltage(double voltage) {
        telescopingArmSparkMax.getPIDController().setReference(voltage, ControlType.kVoltage);
    }

    @Override
    public void setTelescopingArmPosition(double position, double arbFFVoltage) {
        telescopingArmSparkMax.getPIDController().setReference(position, ControlType.kPosition, 0, arbFFVoltage,
                ArbFFUnits.kVoltage);
    }

    @Override
    public void resetTelescopingArmPosition(double position) {
        telescopingArmSparkMax.getEncoder().setPosition(position);
    }
}
