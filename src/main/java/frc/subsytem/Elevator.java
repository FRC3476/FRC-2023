package frc.subsytem;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

public class Elevator extends AbstractSubsystem {
    private final CANSparkMax elevatorSparkMaster;
    private final CANSparkMax elevatorSparkSlave;

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

        elevatorSparkSlave.follow(elevatorSparkMaster);
    }

    private TrapezoidProfile trapezoidProfile =
            new TrapezoidProfile(new TrapezoidProfile.Constraints(Constants.ELEVATOR_CONSTRAINTS.maxVelocity,
                    Constants.ELEVATOR_CONSTRAINTS.maxAcceleration),
                    new TrapezoidProfile.State());
    private double trapezoidProfileStartTime = 0;

    /**
     * @param position The position to set the elevator (meters)
     */
    public void setPosition(double position) {
        trapezoidProfile = new TrapezoidProfile(Constants.ELEVATOR_CONSTRAINTS, new TrapezoidProfile.State(position, 0),
                new TrapezoidProfile.State(elevatorSparkMaster.getEncoder().getPosition() / Constants.ELEVATOR_ROTATIONS_PER_METER,
                        elevatorSparkMaster.getEncoder().getVelocity() / Constants.ELEVATOR_ROTATIONS_PER_METER / 60));
        trapezoidProfileStartTime = -1;
        logData("Goal position", position);
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
        elevatorSparkMaster.getPIDController().setReference(percent, CANSparkMax.ControlType.kDutyCycle);
    }

    @Override
    public void update() {

        double currentTime = Timer.getFPGATimestamp();
        if (trapezoidProfileStartTime == -1) {
            trapezoidProfileStartTime = currentTime;
        }
        TrapezoidProfile.State state = trapezoidProfile.calculate(currentTime - trapezoidProfileStartTime);
        double acceleration = (state.velocity - pastVelocity) / (currentTime - pastTime);

        elevatorSparkMaster.getPIDController().setReference(state.position * Constants.ELEVATOR_ROTATIONS_PER_METER,
                CANSparkMax.ControlType.kPosition, 0, Constants.ELEVATOR_FEEDFORWARD.calculate(state.velocity, acceleration),
                SparkMaxPIDController.ArbFFUnits.kVoltage);

        pastVelocity = state.velocity;
        pastTime = currentTime;

        logData("Wanted pos", state.position);
        logData("Wanted vel", state.velocity);
        logData("Wanted accel", acceleration);
        logData("Total trapezoidProfile time", trapezoidProfile.totalTime());
        logData("TrapezoidProfile time", currentTime - trapezoidProfileStartTime);
        logData("TrapezoidProfile Error", state.position
                - elevatorSparkMaster.getEncoder().getPosition());


    }

    @Override
    public void logData() {
        logData("Motor Position", elevatorSparkMaster.getEncoder().getPosition() / Constants.ELEVATOR_ROTATIONS_PER_METER);
        logData("Motor Velocity", elevatorSparkMaster.getEncoder().getVelocity() / Constants.ELEVATOR_ROTATIONS_PER_METER / 60);
        logData("Motor current", elevatorSparkMaster.getOutputCurrent());
        logData("Motor temperature", elevatorSparkMaster.getMotorTemperature());
    }

    @Override
    public void selfTest() {

    }
}
