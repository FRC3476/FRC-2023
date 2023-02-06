package frc.subsytem;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
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
     * @param rotation
     */
    public void setArmMotor(double rotation) {
        // Set the motor position to the desired motor rotation
        arm.getPIDController().setReference(rotation, CANSparkMax.ControlType.kPosition);
    }

    public void setArmPercent(double percent) {
        arm.getPIDController().setReference(percent * 12.0
                , CANSparkMax.ControlType.kVoltage);
    }

    /**
     * Get the arm rotation in degrees
     */
    public double getArmRotation() {
        return arm.getEncoder().getPosition();
    }

    @Override
    public void update() {
        super.update();
    }

    @Override
    public void logData() {
        logData("Arm Rotation", getArmRotation());
        logData("Arm Current", arm.getOutputCurrent());
    }
}
