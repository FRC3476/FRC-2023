package frc.subsytem.grabber;

import com.revrobotics.*;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.SparkMaxLimitSwitch.Type;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Constants;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

import static edu.wpi.first.wpilibj.RobotBase.isReal;
import static frc.robot.Constants.*;

public class GrabberIOSparkMax extends GrabberIO {

    private final CANSparkMax pivotSparkMax;
    private final CANSparkMax grabberSparkMax;
    private @Nullable CANSparkMax rollerSparkMax1;
    private @Nullable CANSparkMax rollerSparkMax2;
    private @Nullable SparkMaxAbsoluteEncoder pivotAbsoluteEncoder;

    private @NotNull SparkMaxLimitSwitch reverseLimitSwitch;


    public GrabberIOSparkMax() {
        pivotSparkMax = new CANSparkMax(GRABBER_PIVOT_CAN_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
        grabberSparkMax = new CANSparkMax(GRABBER_CAN_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
        pivotAbsoluteEncoder = pivotSparkMax.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);

        if (pivotAbsoluteEncoder == null) {
            pivotSparkMax.getEncoder().setPositionConversionFactor(1.0 / PIVOT_ROTATIONS_PER_DEGREE);
            pivotSparkMax.getEncoder().setVelocityConversionFactor((1.0 / PIVOT_ROTATIONS_PER_DEGREE) / SECONDS_PER_MINUTE);
        } else {
            pivotAbsoluteEncoder.setVelocityConversionFactor((1.0 / PIVOT_ROTATIONS_PER_DEGREE) / SECONDS_PER_MINUTE);
        }
        resetPivotPosition(MAX_WRIST_ANGLE);

        pivotSparkMax.enableVoltageCompensation(Constants.GRABBER_NOMINAL_VOLTAGE);
        pivotSparkMax.setSmartCurrentLimit(Constants.PIVOT_SMART_CURRENT_LIMIT);
        pivotSparkMax.getPIDController().setSmartMotionMaxAccel(GRABBER_PIVOT_CONSTRAINTS.maxAcceleration, 0);
        pivotSparkMax.getPIDController().setSmartMotionMaxVelocity(GRABBER_PIVOT_CONSTRAINTS.maxVelocity, 0);
        pivotSparkMax.getPIDController().setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
        pivotSparkMax.getPIDController().setSmartMotionAllowedClosedLoopError(2, 0);

        SparkMaxPIDController pivotSparkMaxPIDController = pivotSparkMax.getPIDController();
        pivotSparkMaxPIDController.setP(Constants.PIVOT_P);
        pivotSparkMaxPIDController.setI(Constants.PIVOT_I);
        pivotSparkMaxPIDController.setD(Constants.PIVOT_D);
        pivotSparkMaxPIDController.setIZone(Constants.PIVOT_IZONE);

        grabberSparkMax.enableVoltageCompensation(Constants.GRABBER_NOMINAL_VOLTAGE);
        grabberSparkMax.setSmartCurrentLimit(Constants.GRABBER_SMART_CURRENT_LIMIT);
        grabberSparkMax.setClosedLoopRampRate(0.75);
        reverseLimitSwitch = grabberSparkMax.getReverseLimitSwitch(Type.kNormallyClosed);

        if (GRABBER_WHEELS_USED) {
            rollerSparkMax1 = new CANSparkMax(GRABBER_ROLLER_MAIN_CAN_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
            rollerSparkMax2 = new CANSparkMax(GRABBER_ROLLER_FOLLOWER_CAN_ID, CANSparkMaxLowLevel.MotorType.kBrushless);

            rollerSparkMax1.enableVoltageCompensation(Constants.GRABBER_NOMINAL_VOLTAGE);
            rollerSparkMax1.setSmartCurrentLimit(Constants.GRABBER_ROLLER_SMART_CURRENT_LIMIT);

            rollerSparkMax2.enableVoltageCompensation(Constants.GRABBER_NOMINAL_VOLTAGE);
            rollerSparkMax2.setSmartCurrentLimit(Constants.GRABBER_ROLLER_SMART_CURRENT_LIMIT);
            rollerSparkMax2.setInverted(true);
        }

        if (isReal()) {
            pivotSparkMax.burnFlash();
            grabberSparkMax.burnFlash();
            if (GRABBER_WHEELS_USED) {
                rollerSparkMax1.burnFlash();
                rollerSparkMax2.burnFlash();
            }
        }
    }

    @Override
    public synchronized void updateInputs(GrabberInputsAutoLogged inputs) {
        if (pivotAbsoluteEncoder == null) {
            inputs.pivotPosition = pivotSparkMax.getEncoder().getPosition();
            inputs.pivotVelocity = pivotSparkMax.getEncoder().getVelocity();
        } else {
            inputs.pivotPosition = pivotAbsoluteEncoder.getPosition();
            inputs.pivotVelocity = pivotAbsoluteEncoder.getVelocity();
        }
        inputs.pivotCurrent = pivotSparkMax.getOutputCurrent();
        inputs.pivotTemp = pivotSparkMax.getMotorTemperature();
        inputs.pivotVoltage = pivotSparkMax.getAppliedOutput() * pivotSparkMax.getBusVoltage();

        inputs.grabberPosition = grabberSparkMax.getEncoder().getPosition();
        inputs.grabberVelocity = grabberSparkMax.getEncoder().getVelocity();
        inputs.grabberCurrent = grabberSparkMax.getOutputCurrent();
        inputs.grabberTemp = grabberSparkMax.getMotorTemperature();
        inputs.grabberVoltage = grabberSparkMax.getAppliedOutput() * grabberSparkMax.getBusVoltage();

        if (GRABBER_WHEELS_USED) {
            assert rollerSparkMax1 != null;
            assert rollerSparkMax2 != null;
            inputs.rollerMainPosition = rollerSparkMax1.getEncoder().getPosition();
            inputs.rollerMainVelocity = rollerSparkMax1.getEncoder().getVelocity();
            inputs.rollerMainCurrent = rollerSparkMax1.getOutputCurrent();
            inputs.rollerMainTemp = rollerSparkMax1.getMotorTemperature();
            inputs.rollerMainVoltage = rollerSparkMax1.getAppliedOutput() * rollerSparkMax1.getBusVoltage();

            inputs.rollerFollowerPosition = rollerSparkMax2.getEncoder().getPosition();
            inputs.rollerFollowerVelocity = rollerSparkMax2.getEncoder().getVelocity();
            inputs.rollerFollowerCurrent = rollerSparkMax2.getOutputCurrent();
            inputs.rollerFollowerTemp = rollerSparkMax2.getMotorTemperature();
            inputs.rollerFollowerVoltage = rollerSparkMax2.getAppliedOutput() * rollerSparkMax2.getBusVoltage();
        }

        inputs.isLimitSwitchTriggered = reverseLimitSwitch.isPressed();
    }

    @Override
    public void setPivotVoltage(double voltage) {
        pivotSparkMax.getPIDController().setReference(voltage, CANSparkMax.ControlType.kVoltage);
    }

    @Override
    public void setPivotPosition(double position, double arbFFVoltage) {
        pivotSparkMax.getPIDController().setReference(position, ControlType.kPosition, 0, arbFFVoltage);
    }

    @Override
    public void setGrabberVoltage(double current) {
        grabberSparkMax.getPIDController().setReference(current, ControlType.kVoltage);
    }

    @Override
    public void resetPivotPosition(double position) {
        pivotSparkMax.getEncoder().setPosition(position);
    }

    @Override
    public void setRollerVoltage(double voltage) {
        if (GRABBER_WHEELS_USED) {
            assert rollerSparkMax1 != null;
            assert rollerSparkMax2 != null;
            rollerSparkMax1.getPIDController().setReference(voltage, CANSparkMax.ControlType.kVoltage);
            rollerSparkMax2.getPIDController().setReference(voltage, ControlType.kVoltage);
        }
    }

    @Override
    public void setAutoGrab(boolean enabled) {
        reverseLimitSwitch.enableLimitSwitch(enabled);
    }
}
