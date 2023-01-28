package frc.subsytem;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    }

    /**
     * Set the intake output power.
     *
     * @param percent desired speed
     * @param amps    current limit
     */

    public void setIntakeMotor(double percent, int amps) {
        intake.set(percent);
        intake.setSmartCurrentLimit(amps);
        logData("intake power (%)", percent);
        logData("intake motor current (amps)", intake.getOutputCurrent());
        logData("intake motor temperature (C)", intake.getMotorTemperature());

    }

    @Override
    public void logData() {
        super.logData();
    }
}
