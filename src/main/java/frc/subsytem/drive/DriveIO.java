package frc.subsytem.drive;

import org.littletonrobotics.junction.AutoLog;

public abstract class DriveIO {
    @AutoLog
    public static class DriveInputs {
        /**
         * absolute position of the drive motor in meters
         */
        double[] driveMotorPositions = new double[4];
        /**
         * velocity of the drive motor in meters per second
         */
        double[] driveMotorVelocities = new double[4];

        /**
         * absolute position of the swerve motor in degrees (0-360)
         */
        double[] swerveMotorAbsolutePositions = new double[4];
        /**
         * relative position of the swerve motor in degrees (0-360)
         */
        double[] swerveMotorRelativePositions = new double[4];

        /**
         * current of the drive motor in amps
         */
        double[] driveMotorCurrents = new double[4];

        /**
         * current of the swerve motor in amps
         */
        double[] swerveMotorCurrents = new double[4];

        /**
         * temperature of the drive motor in degrees Celsius
         */
        double[] driveMotorTemps = new double[4];

        /**
         * temperature of the swerve motor in degrees Celsius
         */
        double[] swerveMotorTemps = new double[4];

        /**
         * voltage of the drive motor in volts
         */
        double[] driveMotorVoltages = new double[4];

        /**
         * voltage of the swerve motor in volts
         */
        double[] swerveMotorVoltages = new double[4];


        double[] swerveMotorFaults = new double[4];
        double[] driveMotorFaults = new double[4];


        /**
         * timestamp of drive in seconds
         */
        double driveIoTimestamp = 0.0;
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

    protected void updateInputs(DriveInputsAutoLogged inputs) {}

    /**
     * Set the drive voltage compensation level
     *
     * @param voltage the maximum nominal voltage
     */
    public void setDriveVoltageCompLevel(double voltage) {}

    public void resetPeriodicFrames() {}
}
