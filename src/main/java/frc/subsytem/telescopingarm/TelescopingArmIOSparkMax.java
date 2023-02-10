package frc.subsytem.telescopingarm;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxPIDController;
import frc.robot.Constants;

import static frc.robot.Constants.SECONDS_PER_MINUTE;
import static frc.robot.Constants.TELESCOPING_ARM_ROTATIONS_PER_METER;

public class TelescopingArmIOSparkMax extends TelescopingArmIO {
    private final CANSparkMax telescopingArmSparkMax;

    TelescopingArmIOSparkMax() {
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
}
