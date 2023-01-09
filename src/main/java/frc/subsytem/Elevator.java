package frc.subsytem;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxPIDController;
import frc.robot.Constants; 
import org.checkerframework.checker.units.qual.C;

public class Elevator extends AbstractSubsystem{
    Elevator elevator;
    CANSparkMax sparkMax;
    SparkMaxPIDController sparkMaxPIDController;

    public Elevator() {
        super(20, 5);
        elevator = new Elevator();
        sparkMax = new CANSparkMax(0, CANSparkMaxLowLevel.MotorType.kBrushless);
        sparkMaxPIDController = sparkMax.getPIDController();
        sparkMaxPIDController.setP(Constants.ELEVATOR_P);
    }

    @Override
    public void selfTest() {

    }

    @Override
    public void logData() {

    }
}
