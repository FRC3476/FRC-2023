package frc.subsytem;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import frc.robot.Constants;

public class Intake extends AbstractSubsystem {

    TalonSRX intakeMotor;

    private static Intake instance;

    public static Intake getInstance() {
        if (instance == null) {
            instance = new Intake(Constants.INTAKE_PERIOD, 5);
        }

        return instance;
    }

    private Intake(int period, int loggingInterval) {
        super(period, loggingInterval);
        intakeMotor = new TalonSRX(Constants.INTAKE_CAN_ID);
        intakeMotor.setInverted(false);
        intakeMotor.setNeutralMode(NeutralMode.Brake);
        intakeMotor.configContinuousCurrentLimit(Constants.INTAKE_CURRENT_LIMIT);
    }

    /**
     * Set the intake output power.
     *
     * @param percent desired speed
     */

    public void setIntakePercentOutput(double percent) {
        intakeMotor.set(TalonSRXControlMode.PercentOutput, percent);
        logData("intake power (%)", percent);
        logData("intake motor current (amps)", intakeMotor.getStatorCurrent());
        logData("intake motor temperature (C)", intakeMotor.getTemperature());

    }

    @Override
    public void logData() {
        super.logData();
    }
}
