package frc.subsytem;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants;

public class Arm extends AbstractSubsystem {

    private CANSparkMax arm;
    private static Arm instance;
    private static DutyCycleEncoder encoder;





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
        SparkMaxPIDController armPIDController = arm.getPIDController();
        armPIDController.setP(Constants.ARM_P);
        armPIDController.setI(Constants.ARM_I);
        armPIDController.setD(Constants.ARM_D);
        encoder = new DutyCycleEncoder(0);

        // Sync the relative encoder to the absolute encoder position
        arm.getEncoder().setPosition(encoder.get());

    }

    /**
     * Set the arm position
     *
     * @param angle
     */
    public void setArmMotor(double angle) {
        // Set the motor position to the desired angle
        arm.getPIDController().setReference(angle / 360, CANSparkMax.ControlType.kPosition);
    }
}
