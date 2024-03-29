package frc.subsytem.telescopingarm;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.subsytem.AbstractSubsystem;
import org.littletonrobotics.junction.Logger;

import static frc.robot.Constants.*;

public class TelescopingArm extends AbstractSubsystem {

    private final TelescopingArmIO io;
    private final TelescopingArmInputsAutoLogged inputs = new TelescopingArmInputsAutoLogged();

    public TelescopingArm(TelescopingArmIO telescopingArmIO) {
        super();
        this.io = telescopingArmIO;
    }

    private TrapezoidProfile trapezoidProfile =
            new TrapezoidProfile(Constants.TELESCOPING_ARM_CONSTRAINTS, new TrapezoidProfile.State());
    private double trapezoidProfileStartTime = 0;

    /**
     * @param position The position to set the telescoping arm (meters)
     */
    public void setPosition(double position) {
        position = Math.max(Constants.BASE_MIN_X, Math.min(Constants.BASE_MAX_X, position));
        trapezoidProfile = new TrapezoidProfile(Constants.TELESCOPING_ARM_CONSTRAINTS, new TrapezoidProfile.State(position, 0),
                new TrapezoidProfile.State(
                        inputs.position,
                        inputs.velocity));
        trapezoidProfileStartTime = -1;
        Logger.getInstance().recordOutput("Telescoping Arm/Goal position", position);
    }

    private boolean homing = false;
    private double homeTime = 0;

    public void home() {
        homeTime = TELESCOPING_ARM_MIN_HOME_TIME;
        homing = true;
    }

    public void cancelHome() {
        homing = false;
    }

    private double pastVelocity = 0, pastTime = 0;

    @Override
    public void update() {
        io.updateInputs(inputs);
        Logger.getInstance().processInputs("Telescoping Arm", inputs);

        if (homing) {
            if (DriverStation.isEnabled()) {
                homeTime -= Constants.NOMINAL_DT;
                io.setTelescopingArmVoltage(TELESCOPING_ARM_HOME_VOLTAGE);
                if (homeTime <= 0 && inputs.current > TELESCOPING_ARM_STALLING_CURRENT) {
                    homing = false;
                    io.resetTelescopingArmPosition(0);
                }
                Logger.getInstance().recordOutput("Telescoping Arm/Home time", homeTime);
            }
            return;
        }

        double currentTime = Timer.getFPGATimestamp();
        if (trapezoidProfileStartTime == -1) {
            trapezoidProfileStartTime = currentTime;
        }
        currentTime = 100000000;
        TrapezoidProfile.State state = trapezoidProfile.calculate(currentTime - trapezoidProfileStartTime);
        double acceleration = 0; // (state.velocity - pastVelocity) / (currentTime - pastTime);

        if (DriverStation.isTest()) {
            io.setTelescopingArmVoltage(Constants.TELESCOPING_ARM_FEEDFORWARD.calculate(0, 0));
        } else {
            if (Math.abs(state.position - inputs.position) > Constants.TELESCOPING_ARM_ALLOWED_ERROR) {
                io.setTelescopingArmPosition(state.position,
                        Constants.TELESCOPING_ARM_FEEDFORWARD.calculate(state.velocity, acceleration));
            } else {
                io.setTelescopingArmVoltage(Constants.TELESCOPING_ARM_FEEDFORWARD.calculate(state.velocity, acceleration));
            }
        }

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
