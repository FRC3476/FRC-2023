// Copyright 2019 FRC Team 3476 Code Orange

package frc.subsytem.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.subsytem.AbstractSubsystem;

import static frc.robot.Constants.*;

public final class Drive extends AbstractSubsystem {
    private final DriveIO io;
    private final DriveInputsAutoLogged inputs = new DriveInputsAutoLogged();

    public Drive(DriveIO driveIO) {
        super();
        this.io = driveIO;
    }

    public synchronized double getIoTimestamp() {
        return inputs.driveIoTimestamp;
    }

    public double getDrivePosition(int moduleNumber) {
        return inputs.driveMotorPositions[moduleNumber];
    }

    public double getWheelRotation(int moduleNumber) {
        if (USE_RELATIVE_ENCODER_POSITION) {
            double relPos = inputs.swerveMotorRelativePositions[moduleNumber] % 360;
            if (relPos < 0) relPos += 360;
            return relPos;
        } else {
            return inputs.swerveMotorAbsolutePositions[moduleNumber];
        }
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] swerveModulePositions = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            swerveModulePositions[i] = new SwerveModulePosition(
                    getDrivePosition(i),
                    Rotation2d.fromDegrees(getWheelRotation(i)));
        }
        return swerveModulePositions;
    }

    private double getSwerveDriveVelocity(int motorNum) {
        return inputs.driveMotorVelocities[motorNum];
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] swerveModuleStates = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            swerveModuleStates[i] = new SwerveModuleState(
                    getSwerveDriveVelocity(i),
                    Rotation2d.fromDegrees(getWheelRotation(i)));
        }
        return swerveModuleStates;
    }

    public synchronized void setDriveVoltageCompLevel(double voltage) {
        io.setDriveVoltageCompLevel(voltage);
    }

    public synchronized void resetAbsoluteZeros() {
        io.resetAbsoluteZeros();
    }

    public synchronized void setBrakeMode(boolean brake) {
        io.setBrakeMode(brake);
    }

    public synchronized void resetPeriodicFrames() {
        io.resetPeriodicFrames();
    }
}
