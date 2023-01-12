package frc.subsytem;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxPIDController;
import frc.robot.Constants;
public class Elevator extends AbstractSubsystem{
    Elevator elevator;
    static CANSparkMax sparkMax;
    static SparkMaxPIDController sparkMaxPIDController;

    public Elevator() {
        super(20, 5);
        elevator = new Elevator();
        sparkMax = new CANSparkMax(0, CANSparkMaxLowLevel.MotorType.kBrushless);
        sparkMaxPIDController = sparkMax.getPIDController();
        sparkMaxPIDController.setP(Constants.ELEVATOR_P);
        sparkMaxPIDController.setI(Constants.ELEVATOR_I);
        sparkMaxPIDController.setD(Constants.ELEVATOR_D);
    }

    public void turnMotor(double position) {
        sparkMax.getEncoder().setPosition(position * Constants.Position_Multiplier);
    }

    @Override
    public void selfTest() {

    }

    @Override
    public void logData() {
        logData("Motor Position", sparkMax.getEncoder().getPosition() * Constants.Position_Multiplier);
    }
}
