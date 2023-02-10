package frc.subsytem.grabber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxPIDController;
import frc.robot.Constants;

import static frc.robot.Constants.GRABBER_ROTATIONS_PER_DEGREE;
import static frc.robot.Constants.SECONDS_PER_MINUTE;

public class GrabberIOSparkMax extends GrabberIO {

    private final CANSparkMax pivotSparkMax;
    private final CANSparkMax grabberSparkMax;

    public GrabberIOSparkMax() {
        pivotSparkMax = new CANSparkMax(0, CANSparkMaxLowLevel.MotorType.kBrushless);
        grabberSparkMax = new CANSparkMax(0, CANSparkMaxLowLevel.MotorType.kBrushless);
        SparkMaxPIDController pivotSparkMaxPIDController = pivotSparkMax.getPIDController();
        pivotSparkMaxPIDController.setP(Constants.GRABBER_P);
        pivotSparkMaxPIDController.setI(Constants.GRABBER_I);
        pivotSparkMaxPIDController.setD(Constants.GRABBER_D);
        pivotSparkMax.enableVoltageCompensation(Constants.GRABBER_NOMINAL_VOLTAGE);
        pivotSparkMax.setSmartCurrentLimit(Constants.PIVOT_SMART_CURRENT_LIMIT);
        grabberSparkMax.enableVoltageCompensation(Constants.GRABBER_NOMINAL_VOLTAGE);
        grabberSparkMax.setSmartCurrentLimit(Constants.GRABBER_SMART_CURRENT_LIMIT);

        pivotSparkMax.getEncoder().setPositionConversionFactor(1.0 / GRABBER_ROTATIONS_PER_DEGREE);
        pivotSparkMax.getEncoder().setVelocityConversionFactor((1.0 / GRABBER_ROTATIONS_PER_DEGREE) / SECONDS_PER_MINUTE);
    }

    @Override
    public void updateInputs(GrabberInputsAutoLogged inputs) {
        inputs.pivotPosition = pivotSparkMax.getEncoder().getPosition();
        inputs.pivotVelocity = pivotSparkMax.getEncoder().getVelocity();
        inputs.pivotCurrent = pivotSparkMax.getOutputCurrent();
        inputs.pivotTemp = pivotSparkMax.getMotorTemperature();
        inputs.pivotVoltage = pivotSparkMax.getAppliedOutput() * Constants.GRABBER_NOMINAL_VOLTAGE;

        inputs.grabberPosition = grabberSparkMax.getEncoder().getPosition();
        inputs.grabberVelocity = grabberSparkMax.getEncoder().getVelocity();
        inputs.grabberCurrent = grabberSparkMax.getOutputCurrent();
        inputs.grabberTemp = grabberSparkMax.getMotorTemperature();
        inputs.grabberVoltage = grabberSparkMax.getAppliedOutput() * Constants.GRABBER_NOMINAL_VOLTAGE;
    }


    @Override
    public void setPivotVoltage(double voltage) {
        pivotSparkMax.getPIDController().setReference(voltage, CANSparkMax.ControlType.kVoltage);
    }

    @Override
    public void setPivotPosition(double position, double arbFFVoltage) {
        pivotSparkMax.getPIDController().setReference(position, CANSparkMax.ControlType.kPosition, 0, arbFFVoltage);
    }

    @Override
    public void setGrabberVoltage(double voltage) {
        grabberSparkMax.getPIDController().setReference(voltage, CANSparkMax.ControlType.kVoltage);
    }
}
