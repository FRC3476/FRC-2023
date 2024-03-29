package frc.subsytem.grabber;

import com.revrobotics.*;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.SparkMaxLimitSwitch.Type;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;
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
    private @Nullable SparkMaxAbsoluteEncoder grabberAbsoluteEncoder;

    private @NotNull SparkMaxLimitSwitch reverseLimitSwitch;


    public GrabberIOSparkMax() {
        pivotSparkMax = new CANSparkMax(GRABBER_PIVOT_CAN_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
        grabberSparkMax = new CANSparkMax(GRABBER_CAN_ID, CANSparkMaxLowLevel.MotorType.kBrushless);

        pivotSparkMax.getEncoder().setPositionConversionFactor(1.0 / PIVOT_ROTATIONS_PER_DEGREE);
        pivotSparkMax.getEncoder().setVelocityConversionFactor((1.0 / PIVOT_ROTATIONS_PER_DEGREE) / SECONDS_PER_MINUTE);

        if (USE_PIVOT_ABSOLUTE_ENCODER) {
            pivotAbsoluteEncoder = pivotSparkMax.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
            pivotAbsoluteEncoder.setPositionConversionFactor(DEGREES_PER_ROTATION);
            pivotAbsoluteEncoder.setVelocityConversionFactor(DEGREES_PER_ROTATION / SECONDS_PER_MINUTE);
            resetPivotPosition(pivotAbsoluteEncoder.getPosition());
        } else {
            resetPivotPosition(MAX_WRIST_ANGLE);
        }
        pivotSparkMax.getPIDController().setFeedbackDevice(pivotSparkMax.getEncoder());
        pivotSparkMax.getPIDController().setPositionPIDWrappingEnabled(false);

        if (USE_GRABBER_ENCODER) {
            grabberAbsoluteEncoder = grabberSparkMax.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
            grabberAbsoluteEncoder.setPositionConversionFactor(DEGREES_PER_ROTATION);
            grabberAbsoluteEncoder.setVelocityConversionFactor(DEGREES_PER_ROTATION / SECONDS_PER_MINUTE);
            grabberAbsoluteEncoder.setInverted(true);
            resetGrabberPosition(grabberAbsoluteEncoder.getPosition());
        }

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
        grabberSparkMax.setIdleMode(IdleMode.kBrake);
        reverseLimitSwitch = grabberSparkMax.getReverseLimitSwitch(Type.kNormallyOpen);

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

        inputs.pivotPosition = pivotSparkMax.getEncoder().getPosition();
        inputs.pivotVelocity = pivotSparkMax.getEncoder().getVelocity();

        inputs.pivotRelativePosition = pivotSparkMax.getEncoder().getPosition();
        inputs.pivotRelativeVelocity = pivotSparkMax.getEncoder().getVelocity();

        inputs.pivotCurrent = pivotSparkMax.getOutputCurrent();
        inputs.pivotTemp = pivotSparkMax.getMotorTemperature();
        inputs.pivotVoltage = pivotSparkMax.getAppliedOutput() * pivotSparkMax.getBusVoltage();

        inputs.grabberPosition = grabberSparkMax.getEncoder().getPosition();
        inputs.grabberVelocity = grabberSparkMax.getEncoder().getVelocity();
        inputs.grabberCurrent = grabberSparkMax.getOutputCurrent();
        inputs.grabberTemp = grabberSparkMax.getMotorTemperature();
        inputs.grabberVoltage = grabberSparkMax.getAppliedOutput() * grabberSparkMax.getBusVoltage();
        inputs.grabberAppliedOutput = grabberSparkMax.getAppliedOutput();
        inputs.grabberBusVoltage = grabberSparkMax.getBusVoltage();

        if (USE_GRABBER_ENCODER) {
            assert grabberAbsoluteEncoder != null;
            inputs.grabberAbsolutePosition = grabberAbsoluteEncoder.getPosition() - 180; // Closed is 180 to avoid wrapping issues
        }

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
    public void resetGrabberPosition(double position) {
        grabberSparkMax.getEncoder().setPosition(position);
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
