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

        SparkMaxPIDController pidController = elevatorMain.getPIDController();
        pidController.setP(Constants.ELEVATOR_P);
        pidController.setI(Constants.ELEVATOR_I);
        pidController.setD(Constants.ELEVATOR_D);
        pidController.setIZone(Constants.ELEVATOR_IZONE);
        elevatorMain.getPIDController().setOutputRange(-0.62, 0.8);

        resetPosition(0);

        //initializeMotor(elevatorFollower);
        elevatorFollower.follow(elevatorMain);
        if (isReal()) {
            elevatorMain.burnFlash();
            elevatorFollower.burnFlash();
        }
    }

    private void initializeMotor(CANSparkMax motor) {
        if (isReal()) {
            motor.restoreFactoryDefaults();
        }
        motor.setIdleMode(IdleMode.kBrake);

        motor.enableVoltageCompensation(Constants.ELEVATOR_NOMINAL_VOLTAGE);
        motor.setSmartCurrentLimit(Constants.ELEVATOR_SMART_CURRENT_LIMIT);
        motor.getEncoder().setPositionConversionFactor(ELEVATOR_REDUCTION / ELEVATOR_ROTATIONS_PER_METER);
        motor.getEncoder().setVelocityConversionFactor((ELEVATOR_REDUCTION / ELEVATOR_ROTATIONS_PER_METER) / SECONDS_PER_MINUTE);
        motor.setInverted(false);
    }

    @Override
    public void updateInputs(ElevatorInputsAutoLogged inputs) {
        inputs.elevatorPosition = new double[]{elevatorMain.getEncoder().getPosition(),
                elevatorFollower.getEncoder().getPosition()};
        inputs.elevatorVelocity = new double[]{elevatorMain.getEncoder().getVelocity(),
                elevatorFollower.getEncoder().getPosition()};

        inputs.elevatorCurrent = new double[]{elevatorMain.getOutputCurrent(), elevatorFollower.getOutputCurrent()};
        inputs.elevatorTemp = new double[]{elevatorMain.getMotorTemperature(), elevatorFollower.getMotorTemperature()};
        inputs.elevatorVoltage = new double[]{elevatorMain.getAppliedOutput() * elevatorMain.getBusVoltage(),
                elevatorFollower.getAppliedOutput() * elevatorFollower.getBusVoltage()};
    }

    @Override
    public void setElevatorVoltage(double voltage) {
        elevatorMain.getPIDController().setReference(voltage, CANSparkMax.ControlType.kVoltage);
//        elevatorFollower.getPIDController().setReference(voltage, CANSparkMax.ControlType.kVoltage);
    }

    @Override
    public void setElevatorPosition(double position, double arbFFVoltage) {
        elevatorMain.getPIDController().setReference(position, CANSparkMax.ControlType.kPosition, 0, arbFFVoltage,
                SparkMaxPIDController.ArbFFUnits.kVoltage);
//        elevatorFollower.getPIDController().setReference(position, CANSparkMax.ControlType.kPosition, 0, arbFFVoltage,
//                SparkMaxPIDController.ArbFFUnits.kVoltage);
    }

    @Override
    public void resetPosition(double position) {
        elevatorMain.getEncoder().setPosition(position);
        //elevatorFollower.getEncoder().setPosition(position);
    }
}
