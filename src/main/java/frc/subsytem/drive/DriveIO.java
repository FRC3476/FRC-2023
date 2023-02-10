package frc.subsytem.drive;

import org.littletonrobotics.junction.AutoLog;

public abstract class DriveIO {
    @AutoLog
    public static class DriveInputs {
        double[] driveMotorPositions = new double[4];
        double[] driveMotorVelocities = new double[4];

        double[] swerveMotorAbsolutePositions = new double[4];
        double[] swerveMotorRelativePositions = new double[4];

        double[] driveMotorCurrents = new double[4];
        double[] swerveMotorCurrents = new double[4];

        double[] driveMotorTemps = new double[4];
        double[] swerveMotorTemps = new double[4];

        double[] driveMotorVoltages = new double[4];
        double[] swerveMotorVoltages = new double[4];
    }

    /**
     * Set the break mode of the drive and turn motors
     *
     * @param enable True to set brake mode, false to set coast mode
     */
    protected void setBrakeMode(boolean enable) {}

    /**
     * Set the target position of the selected swerve motor
     *
     * @param motorNum the selected swerve motor
     * @param position the target position in degrees (0-360)
     */
    protected void setSwerveMotorPosition(int motorNum, double position) {}


    /**
     * Set the target position of the selected swerve motor
     *
     * @param motorNum the selected swerve motor
     * @param voltage  the wanted voltage
     */
    protected void setSwerveMotorVoltage(int motorNum, double voltage) {}


    /**
     * Set the target voltage of the selected swerve motor
     *
     * @param motorNum the selected swerve motor
     * @param voltage  the target voltage in volts
     */
    protected void setDriveMotorVoltage(int motorNum, double voltage) {}


    public void resetAbsoluteZeros() {}

    public void updateInputs(DriveInputs inputs) {}
}