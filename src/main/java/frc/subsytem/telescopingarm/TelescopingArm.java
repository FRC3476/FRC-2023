package frc.subsytem.telescopingarm;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.subsytem.AbstractSubsystem;
import org.littletonrobotics.junction.Logger;

public class TelescopingArm extends AbstractSubsystem {

    private final TelescopingArmIO io;
    private final TelescopingArmInputsAutoLogged inputs = new TelescopingArmInputsAutoLogged();

    public TelescopingArm(TelescopingArmIO telescopingArmIO) {
        super(5);
        this.io = telescopingArmIO;
    }

    private TrapezoidProfile trapezoidProfile =
            new TrapezoidProfile(Constants.TELESCOPING_ARM_CONSTRAINTS, new TrapezoidProfile.State());
    private double trapezoidProfileStartTime = 0;

    /**
     * @param position The position to set the telescoping arm (meters)
     */
    public void setPosition(double position) {
        trapezoidProfile = new TrapezoidProfile(Constants.TELESCOPING_ARM_CONSTRAINTS, new TrapezoidProfile.State(position, 0),
                new TrapezoidProfile.State(
                        inputs.position,
                        inputs.velocity));
        trapezoidProfileStartTime = -1;
        Logger.getInstance().recordOutput("Telescoping Arm/Goal position", position);
    }

    private double pastVelocity = 0, pastTime = 0;

    @Override
    public void update() {
        io.updateInputs(inputs);
        Logger.getInstance().processInputs("Telescoping Arm", inputs);

        double currentTime = Timer.getFPGATimestamp();
        if (trapezoidProfileStartTime == -1) {
            trapezoidProfileStartTime = currentTime;
        }
        TrapezoidProfile.State state = trapezoidProfile.calculate(currentTime - trapezoidProfileStartTime);
        double acceleration = (state.velocity - pastVelocity) / (currentTime - pastTime);

        io.setTelescopingArmPosition(state.position,
                Constants.TELESCOPING_ARM_FEEDFORWARD.calculate(state.velocity, acceleration));

        pastVelocity = state.velocity;
        pastTime = currentTime;

        Logger.getInstance().recordOutput("Telescoping Arm/Wanted pos", state.position);
        Logger.getInstance().recordOutput("Telescoping Arm/Wanted vel", state.velocity);
        Logger.getInstance().recordOutput("Telescoping Arm/Wanted accel", acceleration);
        Logger.getInstance().recordOutput("Telescoping Arm/Total trapezoidProfile time", trapezoidProfile.totalTime());
        Logger.getInstance().recordOutput("Telescoping Arm/TrapezoidProfile time", currentTime - trapezoidProfileStartTime);
        Logger.getInstance().recordOutput("Telescoping Arm/TrapezoidProfile error", state.position - inputs.position);
    }

    @Override
    public void selfTest() {

    }

    public double getPosition() {
        return inputs.position;
    }
}
