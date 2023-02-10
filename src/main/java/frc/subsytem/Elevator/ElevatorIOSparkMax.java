package frc.subsytem.Elevator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxPIDController;
import frc.robot.Constants;

import static edu.wpi.first.wpilibj.RobotBase.isReal;
import static frc.robot.Constants.*;

public class ElevatorIOSparkMax extends ElevatorIO {
    private final CANSparkMax elevatorMain;
    private final CANSparkMax elevatorFollower;

    public ElevatorIOSparkMax() {
        elevatorMain = new CANSparkMax(ELEVATOR_MAIN_CAN_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
        elevatorFollower = new CANSparkMax(ELEVATOR_FOLLOWER_CAN_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
        initializeMotor(elevatorMain);
        initializeMotor(elevatorFollower);
        elevatorFollower.follow(elevatorMain);
    }

    private void initializeMotor(CANSparkMax motor) {
        if (isReal()) {
            motor.restoreFactoryDefaults();
        }
        motor.setIdleMode(IdleMode.kBrake);
        SparkMaxPIDController elevatorSparkMaxPIDController = motor.getPIDController();
        elevatorSparkMaxPIDController.setP(Constants.ELEVATOR_P);
        elevatorSparkMaxPIDController.setI(Constants.ELEVATOR_I);
        elevatorSparkMaxPIDController.setD(Constants.ELEVATOR_D);
        motor.enableVoltageCompensation(Constants.ELEVATOR_NOMINAL_VOLTAGE);
        motor.setSmartCurrentLimit(Constants.ELEVATOR_SMART_CURRENT_LIMIT);
        motor.getEncoder().setPositionConversionFactor(1.0 / ELEVATOR_ROTATIONS_PER_METER);
        motor.getEncoder().setVelocityConversionFactor((1.0 / ELEVATOR_ROTATIONS_PER_METER) / SECONDS_PER_MINUTE);

        if (isReal()) {
            motor.burnFlash();
        }
    }

    @Override
    public void updateInputs(ElevatorInputs inputs) {
        inputs.elevatorPosition = elevatorMain.getEncoder().getPosition();
        inputs.elevatorVelocity = elevatorMain.getEncoder().getVelocity();

        inputs.elevatorCurrent = new double[]{elevatorMain.getOutputCurrent(), elevatorFollower.getOutputCurrent()};
        inputs.elevatorTemp = new double[]{elevatorMain.getMotorTemperature(), elevatorFollower.getMotorTemperature()};
        inputs.elevatorVoltage = new double[]{elevatorMain.getAppliedOutput(), elevatorFollower.getAppliedOutput()};
    }

    @Override
    public void setElevatorVoltage(double voltage) {
        elevatorMain.getPIDController().setReference(voltage, CANSparkMax.ControlType.kVoltage);
    }

    @Override
    public void setElevatorPosition(double position, double arbFFVoltage) {
        elevatorMain.getPIDController().setReference(position, CANSparkMax.ControlType.kPosition, 0, arbFFVoltage,
                SparkMaxPIDController.ArbFFUnits.kVoltage);
    }
}
