package frc.subsytem;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class Arm extends AbstractSubsystem {
    private static Arm instance;
    CANSparkMax arm = new CANSparkMax(5, CANSparkMaxLowLevel.MotorType.kBrushless);

    public static Arm getInstance() {
        if (instance == null) {
            instance = new Arm(Constants.ARM_PERIOD, 5);
        }
        return instance;
    }

    private Arm(int period, int loggingInterval) {
        super(period, loggingInterval);
        arm.setInverted(true);
        arm.setIdleMode(CANSparkMax.IdleMode.kBrake);
        arm.setSmartCurrentLimit(Constants.ARM_CURRENT_LIMIT_A);
    }

    /**
     * Set the arm output power. Positive is out, negative is in.
     *
     * @param percent
     */
    public void setArmMotor(double percent) {
        arm.set(percent);
        logData("arm power (%)", percent);
        logData("arm motor current (amps)", arm.getOutputCurrent());
        logData("arm motor temperature (C)", arm.getMotorTemperature());
    }
}
