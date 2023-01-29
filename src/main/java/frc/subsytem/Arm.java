package frc.subsytem;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import frc.robot.Constants;

public class Arm extends AbstractSubsystem {

    private CANSparkMax arm;
    private static Arm instance;

    public static Arm getInstance() {
        if (instance == null) {
            instance = new Arm(Constants.ARM_PERIOD, 5);
        }
        return instance;
    }

    private Arm(int period, int loggingInterval) {
        super(period, loggingInterval);
        arm = new CANSparkMax(Constants.ARM_MOTOR_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
        arm.setInverted(true);
        arm.setIdleMode(CANSparkMax.IdleMode.kBrake);
        arm.setSmartCurrentLimit(Constants.ARM_CURRENT_LIMIT);
    }

    /**
     * Set the arm position
     *
     * @param percent
     */
    public void setArmMotor(double angle) {
        
    }
}
