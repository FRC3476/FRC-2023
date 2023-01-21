package frc.subsytem;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

public class Elevator extends AbstractSubsystem {
    private final CANSparkMax elevatorSparkMax;
    private final SparkMaxPIDController elevatorSparkMaxPIDController;
    private final static Elevator instance = new Elevator();
    public static Elevator getInstance() {
        return instance;
    }
    public Elevator() {
        super(Constants.ELEVATOR_PERIOD, 5);
        elevatorSparkMax = new CANSparkMax(0, CANSparkMaxLowLevel.MotorType.kBrushless);
        elevatorSparkMaxPIDController = elevatorSparkMax.getPIDController();
        elevatorSparkMaxPIDController.setP(Constants.ELEVATOR_P);
        elevatorSparkMaxPIDController.setI(Constants.ELEVATOR_I);
        elevatorSparkMaxPIDController.setD(Constants.ELEVATOR_D);
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
                new TrapezoidProfile.State(elevatorSparkMax.getEncoder().getPosition() / Constants.ELEVATOR_POSITION_MULTIPLIER,
                        elevatorSparkMax.getEncoder().getVelocity() / Constants.ELEVATOR_POSITION_MULTIPLIER));
        trapezoidProfileStartTime = -1;
        logData("Goal position", position);
    }

    private double pastVelocity = 0, pastTime = 0;

    @Override
    public void update() {
        double acceleration;
        double currentTime = Timer.getFPGATimestamp();
        if(trapezoidProfileStartTime == -1) {
            trapezoidProfileStartTime = Timer.getFPGATimestamp();
        }
        TrapezoidProfile.State state = trapezoidProfile.calculate(currentTime - trapezoidProfileStartTime);
        acceleration = (state.velocity - pastVelocity) / (currentTime - pastTime);

        elevatorSparkMax.getPIDController().setReference(state.position * Constants.ELEVATOR_POSITION_MULTIPLIER,
                CANSparkMax.ControlType.kPosition, 0, Constants.ELEVATOR_FEEDFORWARD.calculate(state.velocity, acceleration),
                SparkMaxPIDController.ArbFFUnits.kVoltage);

        pastVelocity = state.velocity;
        pastTime = currentTime;

        logData("Wanted pos", state.position);
        logData("Wanted vel", state.velocity);
        logData("Wanted accel", acceleration);
        logData("Total trapezoidProfile time", trapezoidProfile.totalTime());
        logData("TrapezoidProfile time", currentTime - trapezoidProfileStartTime);
        logData("TrapezoidProfile Error", trapezoidProfile.calculate(currentTime).position
                - elevatorSparkMax.getEncoder().getPosition());
    }

    @Override
    public void logData() {
        logData("Motor Position", elevatorSparkMax.getEncoder().getPosition() * Constants.ELEVATOR_POSITION_MULTIPLIER);
        logData("Motor Velocity", elevatorSparkMax.getEncoder().getVelocity());
        logData("Motor current", elevatorSparkMax.getOutputCurrent());
        logData("Motor temperature", elevatorSparkMax.getMotorTemperature());
    }

    @Override
    public void selfTest() {

    }
}
