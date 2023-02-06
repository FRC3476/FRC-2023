package frc.subsytem;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

import java.util.concurrent.locks.ReentrantReadWriteLock;

public class Elevator extends AbstractSubsystem {

    private final CANSparkMax elevatorSparkMaster;
    private final CANSparkMax elevatorSparkSlave;

    private ElevatorState currentElevatorState = ElevatorState.OFF;

    private double percentOutput = 0;
    private double positionSetpoint = Constants.ELEVATOR_LOWER_LIMIT;

    private final static Elevator instance = new Elevator();

    public static Elevator getInstance() {
        return instance;
    }

    private Elevator() {
        super(Constants.ELEVATOR_PERIOD, 5);
        elevatorSparkMaster = new CANSparkMax(Constants.ELEVATOR_MASTER_SPARK_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
        elevatorSparkSlave = new CANSparkMax(Constants.ELEVATOR_SLAVE_SPARK_ID, CANSparkMaxLowLevel.MotorType.kBrushless);
        SparkMaxPIDController elevatorSparkMaxPIDController = elevatorSparkMaster.getPIDController();
        elevatorSparkMaxPIDController.setP(Constants.ELEVATOR_P);
        elevatorSparkMaxPIDController.setI(Constants.ELEVATOR_I);
        elevatorSparkMaxPIDController.setD(Constants.ELEVATOR_D);

        elevatorSparkMaster.enableVoltageCompensation(Constants.ELEVATOR_NOMINAL_VOLTAGE);
        elevatorSparkMaster.setSmartCurrentLimit(Constants.ELEVATOR_SMART_CURRENT_LIMIT);
        elevatorSparkMaster.setInverted(false);
        elevatorSparkMaster.getEncoder().setPositionConversionFactor(Constants.ELEVATOR_REDUCTION);

        elevatorSparkSlave.follow(elevatorSparkMaster);
    }

    /**
     * @param position The position to set the elevator (meters)
     */
    public void setPosition(double position) {
        if(position < Constants.ELEVATOR_LOWER_LIMIT) {
            position = Constants.ELEVATOR_LOWER_LIMIT;
        } else if (position > Constants.ELEVATOR_UPPER_LIMIT) {
            position = Constants.ELEVATOR_UPPER_LIMIT;
        }

        positionSetpoint = position;
    }

    /**
     * @return Elevator position in meters
     */
    public double getPosition() {
        return elevatorSparkMaster.getEncoder().getPosition() / Constants.ELEVATOR_ROTATIONS_PER_METER;
    }

    private void zeroEncoder() {
        elevatorSparkMaster.getEncoder().setPosition(0);
    }

    private boolean hasStalledIntoBottom = false;
    private double minRunTime = -1;

    public void resetHoming() {
        hasStalledIntoBottom = false;
        minRunTime = -1;
    }

    public void elevatorStallIntoBottom() {
        if (minRunTime == -1) minRunTime = Timer.getFPGATimestamp() + Constants.MOTOR_STARTING_TIME;
        if(hasStalledIntoBottom){
            setPercentOutput(0);
        } else {
            setPercentOutput(Constants.MOTOR_SPEED_DECREASING_RATE);
        }
        if (Math.abs(elevatorSparkMaster.getOutputCurrent()) > Constants.STALLING_CURRENT && Timer.getFPGATimestamp() > minRunTime){
            hasStalledIntoBottom = true;
            zeroEncoder();

        }
    }

    private double pastVelocity = 0, pastTime = 0;

    /**
     * Controls elevator motor with percent output
     * @param percent Spans from -1 to 1 where the extremes are full power and direction depends on the sign
     */
    public void setPercentOutput(double percent) {
        percentOutput = percent;
    }

    public void setElevatorState(ElevatorState elevatorState) {
        currentElevatorState = elevatorState;
    }

    @Override
    public void update() {
        switch(currentElevatorState) {
            case TELEOP -> {
                elevatorSparkMaster.getPIDController().setReference(positionSetpoint, CANSparkMax.ControlType.kPosition);
                logData("Elevator Position Setpoint", positionSetpoint);
            }

            case TEST -> {
                elevatorSparkMaster.getPIDController().setReference(percentOutput, CANSparkMax.ControlType.kDutyCycle);
                logData("Elevator percent output", percentOutput);
            }

            case OFF -> {
                elevatorSparkMaster.getPIDController().setReference(0, CANSparkMax.ControlType.kDutyCycle);
                logData("Elevator percent output", 0);
            }
        }
    }

    @Override
    public void logData() {
        logData("Motor Position", elevatorSparkMaster.getEncoder().getPosition() / Constants.ELEVATOR_ROTATIONS_PER_METER);
        logData("Motor Velocity", elevatorSparkMaster.getEncoder().getVelocity() / Constants.ELEVATOR_ROTATIONS_PER_METER / 60);
        logData("Motor current", elevatorSparkMaster.getOutputCurrent());
        logData("Motor temperature", elevatorSparkMaster.getMotorTemperature());
        logData("Elevator State", currentElevatorState.toString());
    }

    @Override
    public void selfTest() {

    }

    public enum ElevatorState {
        TELEOP,
        TEST,
        OFF
    }
}
