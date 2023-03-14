package frc.subsytem.drive;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.StickyFaults;
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

        ErrorCode[] swerveMotorFaults = new ErrorCode[4];
        ErrorCode[] driveMotorFaults = new ErrorCode[4];


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
    protected void setSwerveMotorCurrent(int motorNum, double voltage) {}


    /**
     * Set the target voltage of the selected swerve motor
     *
     * @param motorNum the selected swerve motor
     * @param voltage  the target voltage in volts
     */
    protected void setDriveMotorCurrent(int motorNum, double voltage) {}


    public void resetAbsoluteZeros() {}

    protected void updateInputs(DriveInputsAutoLogged inputs) {}

    /**
     * Set the drive voltage compensation level
     *
     * @param voltage the maximum nominal voltage
     */
    public void setDriveVoltageCompLevel(double voltage) {}
}
