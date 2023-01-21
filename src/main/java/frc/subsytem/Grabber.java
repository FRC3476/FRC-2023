package frc.subsytem;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

public class Grabber extends AbstractSubsystem {
    private final CANSparkMax grabberSparkMax;
    private final SparkMaxPIDController grabberSparkMaxPIDController;
    private static final Grabber instance = new Grabber();
    private final Solenoid solenoid;
    public static Grabber getInstance() {
        return instance;
    }
    public Grabber() {
        super(Constants.GRABBER_PERIOD, 5);
        solenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
        grabberSparkMax = new CANSparkMax(0, CANSparkMaxLowLevel.MotorType.kBrushless);
        grabberSparkMaxPIDController = grabberSparkMax.getPIDController();
        grabberSparkMaxPIDController.setP(Constants.GRABBER_P);
        grabberSparkMaxPIDController.setI(Constants.GRABBER_I);
        grabberSparkMaxPIDController.setD(Constants.GRABBER_D);
    }

    private TrapezoidProfile trapezoidProfile =
            new TrapezoidProfile(new TrapezoidProfile.Constraints(Constants.GRABBER_CONSTRAINTS.maxVelocity,
                    Constants.GRABBER_CONSTRAINTS.maxAcceleration),
                    new TrapezoidProfile.State());
    private double trapezoidProfileStartTime = 0;

    /**
     * @param position The position to set the grabber (degrees)
     */
    public void setPosition(double position) {
        trapezoidProfile = new TrapezoidProfile(Constants.GRABBER_CONSTRAINTS, new TrapezoidProfile.State(position, 0),
                new TrapezoidProfile.State(grabberSparkMax.getEncoder().getPosition() / Constants.GRABBER_POSITION_MULTIPLIER,
                        grabberSparkMax.getEncoder().getVelocity() / Constants.GRABBER_POSITION_MULTIPLIER));
        trapezoidProfileStartTime = Timer.getFPGATimestamp();
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

        grabberSparkMax.getPIDController().setReference(state.position * Constants.GRABBER_POSITION_MULTIPLIER,
                CANSparkMax.ControlType.kPosition, 0, Constants.GRABBER_FEEDFORWARD.calculate(state.velocity, acceleration),
                SparkMaxPIDController.ArbFFUnits.kVoltage);

        pastVelocity = state.velocity;
        pastTime = currentTime;

        logData("Wanted pos", state.position);
        logData("Wanted vel", state.velocity);
        logData("Wanted accel", acceleration);
        logData("Total trapezoidProfile time", trapezoidProfile.totalTime());
        logData("Profile length", currentTime - trapezoidProfileStartTime);
        logData("TrapezoidProfile error", trapezoidProfile.calculate(currentTime).position
                - grabberSparkMax.getEncoder().getPosition());
    }

    @Override
    public void logData() {
        logData("Motor Position", grabberSparkMax.getEncoder().getPosition() * Constants.GRABBER_POSITION_MULTIPLIER);
        logData("Motor Velocity", grabberSparkMax.getEncoder().getVelocity());
        logData("Motor current", grabberSparkMax.getOutputCurrent());
        logData("Motor temperature", grabberSparkMax.getMotorTemperature());
    }

    public void toggle() {
        solenoid.toggle();
    }

    @Override
    public void selfTest() {

    }
}
