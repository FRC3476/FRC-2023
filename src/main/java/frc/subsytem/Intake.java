package frc.subsytem;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import frc.robot.Constants;

public class Intake extends AbstractSubsystem {
    CANSparkMax intake = new CANSparkMax(6, CANSparkMaxLowLevel.MotorType.kBrushless);

    private static Intake instance;

    public static Intake getInstance() {
        if (instance == null) {
            instance = new Intake(Constants.INTAKE_PERIOD, 5);
        }

        return instance;
    }

    private Intake(int period, int loggingInterval) {
        super(period, loggingInterval);
        intake.setInverted(false);
        intake.setIdleMode(CANSparkMax.IdleMode.kBrake);
        intake.setSmartCurrentLimit(Constants.INTAKE_CURRENT_LIMIT);
    }

    /**
     * Set the intake output power.
     *
     * @param percent desired speed
     */

    public void setIntakePercentOutput(double percent) {
        intake.set(percent);
        logData("intake power (%)", percent);
        logData("intake motor current (amps)", intake.getOutputCurrent());
        logData("intake motor temperature (C)", intake.getMotorTemperature());

    }

    @Override
    public void logData() {
        super.logData();
    }
}
