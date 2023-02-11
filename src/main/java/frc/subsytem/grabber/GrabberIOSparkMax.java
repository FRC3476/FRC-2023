package frc.subsytem.grabber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxPIDController;
import frc.robot.Constants;

import static frc.robot.Constants.*;

public class GrabberIOSparkMax extends GrabberIO {

    private final CANSparkMax pivotSparkMax;
    private final CANSparkMax grabberSparkMax;
    private final CANSparkMax rollerMainSparkMax;
    private final CANSparkMax rollerFollowerSparkMax;


    public GrabberIOSparkMax() {
        pivotSparkMax = new CANSparkMax(GRABBER_PIVOT_CAN_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
        grabberSparkMax = new CANSparkMax(GRABBER_CAN_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
        rollerMainSparkMax = new CANSparkMax(GRABBER_ROLLER_MAIN_CAN_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
        rollerFollowerSparkMax = new CANSparkMax(GRABBER_ROLLER_FOLLOWER_CAN_ID, CANSparkMaxLowLevel.MotorType.kBrushless);

        SparkMaxPIDController pivotSparkMaxPIDController = pivotSparkMax.getPIDController();
        pivotSparkMaxPIDController.setP(Constants.PIVOT_P);
        pivotSparkMaxPIDController.setI(Constants.PIVOT_I);
        pivotSparkMaxPIDController.setD(Constants.PIVOT_D);
        pivotSparkMax.enableVoltageCompensation(Constants.GRABBER_NOMINAL_VOLTAGE);
        pivotSparkMax.setSmartCurrentLimit(Constants.PIVOT_SMART_CURRENT_LIMIT);
        resetPivotPosition(34);

        grabberSparkMax.enableVoltageCompensation(Constants.GRABBER_NOMINAL_VOLTAGE);
        grabberSparkMax.setSmartCurrentLimit(Constants.GRABBER_SMART_CURRENT_LIMIT);

        rollerMainSparkMax.enableVoltageCompensation(Constants.GRABBER_NOMINAL_VOLTAGE);
        rollerMainSparkMax.setSmartCurrentLimit(Constants.GRABBER_ROLLER_SMART_CURRENT_LIMIT);

        rollerFollowerSparkMax.follow(rollerMainSparkMax, true);
        rollerFollowerSparkMax.enableVoltageCompensation(Constants.GRABBER_NOMINAL_VOLTAGE);
        rollerFollowerSparkMax.setSmartCurrentLimit(Constants.GRABBER_ROLLER_SMART_CURRENT_LIMIT);

        pivotSparkMax.getEncoder().setPositionConversionFactor(1.0 / PIVOT_ROTATIONS_PER_DEGREE);
        pivotSparkMax.getEncoder().setVelocityConversionFactor((1.0 / PIVOT_ROTATIONS_PER_DEGREE) / SECONDS_PER_MINUTE);
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

        inputs.rollerMainPosition = rollerMainSparkMax.getEncoder().getPosition();
        inputs.rollerMainVelocity = rollerMainSparkMax.getEncoder().getVelocity();
        inputs.rollerMainCurrent = rollerMainSparkMax.getOutputCurrent();
        inputs.rollerMainTemp = rollerMainSparkMax.getMotorTemperature();
        inputs.rollerMainVoltage = rollerMainSparkMax.getAppliedOutput() * Constants.GRABBER_NOMINAL_VOLTAGE;

        inputs.rollerFollowerPosition = rollerFollowerSparkMax.getEncoder().getPosition();
        inputs.rollerFollowerVelocity = rollerFollowerSparkMax.getEncoder().getVelocity();
        inputs.rollerFollowerCurrent = rollerFollowerSparkMax.getOutputCurrent();
        inputs.rollerFollowerTemp = rollerFollowerSparkMax.getMotorTemperature();
        inputs.rollerFollowerVoltage = rollerFollowerSparkMax.getAppliedOutput() * Constants.GRABBER_NOMINAL_VOLTAGE;
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

    @Override
    public void resetPivotPosition(double position) {
        pivotSparkMax.getEncoder().setPosition(position);
    }

    @Override
    public void setRollerVoltage(double voltage) {
        rollerMainSparkMax.getPIDController().setReference(voltage, CANSparkMax.ControlType.kVoltage);
    }
}
