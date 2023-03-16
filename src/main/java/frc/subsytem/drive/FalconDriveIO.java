package frc.subsytem.drive;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import frc.robot.Constants;
import org.jetbrains.annotations.NotNull;
import org.littletonrobotics.junction.Logger;

import java.util.concurrent.locks.ReentrantLock;

import static frc.robot.Constants.*;

public class FalconDriveIO extends DriveIO {
    /**
     * Motors that turn the wheels around. Uses Falcon500s
     */
    private final @NotNull TalonFX[] swerveMotors = new TalonFX[4];
    /**
     * Motors that are driving the robot around and causing it to move
     */
    private final @NotNull TalonFX[] swerveDriveMotors = new TalonFX[4];
    /**
     * Absolute Encoders for the motors that turn the wheel
     */

    private final @NotNull CANCoder[] swerveCanCoders;
    private final ReentrantLock swerveAutoControllerLock = new ReentrantLock();

    public FalconDriveIO() {
        final @NotNull TalonFX leftFrontTalon, leftBackTalon, rightFrontTalon, rightBackTalon;
        final @NotNull TalonFX leftFrontTalonSwerve, leftBackTalonSwerve, rightFrontTalonSwerve, rightBackTalonSwerve;

        SupplyCurrentLimitConfiguration swerveMotorCurrentLimit = new SupplyCurrentLimitConfiguration();
        swerveMotorCurrentLimit.currentLimit = SWERVE_MOTOR_CURRENT_LIMIT;
        SensorVelocityMeasPeriod measurementPeriod = SensorVelocityMeasPeriod.Period_20Ms;
        NeutralMode mode = NeutralMode.Coast;

        // Swerve Drive Motors
        leftFrontTalon = new TalonFX(Constants.DRIVE_LEFT_FRONT_ID);
        leftBackTalon = new TalonFX(Constants.DRIVE_LEFT_BACK_ID);
        rightFrontTalon = new TalonFX(Constants.DRIVE_RIGHT_FRONT_ID);
        rightBackTalon = new TalonFX(Constants.DRIVE_RIGHT_BACK_ID);

        leftFrontTalon.setInverted(false);
        rightFrontTalon.setInverted(false);
        leftBackTalon.setInverted(false);
        rightBackTalon.setInverted(false);

        leftFrontTalonSwerve = new TalonFX(Constants.DRIVE_LEFT_FRONT_SWERVE_ID);
        leftBackTalonSwerve = new TalonFX(Constants.DRIVE_LEFT_BACK_SWERVE_ID);
        rightFrontTalonSwerve = new TalonFX(Constants.DRIVE_RIGHT_FRONT_SWERVE_ID);
        rightBackTalonSwerve = new TalonFX(Constants.DRIVE_RIGHT_BACK_SWERVE_ID);


        swerveMotors[0] = leftFrontTalonSwerve;
        swerveMotors[1] = leftBackTalonSwerve;
        swerveMotors[2] = rightFrontTalonSwerve;
        swerveMotors[3] = rightBackTalonSwerve;

        swerveDriveMotors[0] = leftFrontTalon;
        swerveDriveMotors[1] = leftBackTalon;
        swerveDriveMotors[2] = rightFrontTalon;
        swerveDriveMotors[3] = rightBackTalon;

        if (USE_CANCODERS) {
            final @NotNull CANCoder leftFrontCanCoder, leftBackCanCoder, rightFrontCanCoder, rightBackCanCoder;

            leftFrontCanCoder = new CANCoder(Constants.CAN_LEFT_FRONT_ID);
            leftBackCanCoder = new CANCoder(Constants.CAN_LEFT_BACK_ID);
            rightFrontCanCoder = new CANCoder(Constants.CAN_RIGHT_FRONT_ID);
            rightBackCanCoder = new CANCoder(Constants.CAN_RIGHT_BACK_ID);

            swerveCanCoders = new CANCoder[4];
            swerveCanCoders[0] = leftFrontCanCoder;
            swerveCanCoders[1] = leftBackCanCoder;
            swerveCanCoders[2] = rightFrontCanCoder;
            swerveCanCoders[3] = rightBackCanCoder;
        } else {
            swerveCanCoders = null;
        }


        for (int i = 0; i < 4; i++) {
            // Sets swerveMotors PID
            swerveMotors[i].config_kP(0, Constants.SWERVE_DRIVE_P);
            swerveMotors[i].config_kD(0, Constants.SWERVE_DRIVE_D);
            swerveMotors[i].config_kI(0, Constants.SWERVE_DRIVE_I);
            swerveMotors[i].config_kF(0, Constants.SWERVE_DRIVE_F);
            swerveMotors[i].config_IntegralZone(0, Constants.SWERVE_DRIVE_INTEGRAL_ZONE);

            // Sets current limits for motors

            swerveMotors[i].configSupplyCurrentLimit(swerveMotorCurrentLimit);
            swerveMotors[i].enableVoltageCompensation(true);
            swerveMotors[i].configVoltageCompSaturation(Constants.SWERVE_DRIVE_VOLTAGE_LIMIT_AUTO);

            swerveMotors[i].setStatusFramePeriod(StatusFrame.Status_1_General, 10);
            swerveMotors[i].setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 101);

            swerveDriveMotors[i].configSupplyCurrentLimit(swerveMotorCurrentLimit);
            swerveDriveMotors[i].enableVoltageCompensation(true);
            swerveDriveMotors[i].configVoltageCompSaturation(SWERVE_DRIVE_VOLTAGE_LIMIT_AUTO);
            swerveDriveMotors[i].configVelocityMeasurementPeriod(measurementPeriod);

            swerveDriveMotors[i].setNeutralMode(mode);
            swerveMotors[i].setNeutralMode(mode);

            swerveMotors[i].setInverted(true);

            swerveCanCoders[i].setStatusFramePeriod(CANCoderStatusFrame.VbatAndFaults, 200);
            swerveCanCoders[i].setStatusFramePeriod(CANCoderStatusFrame.SensorData, 20);
            swerveCanCoders[i].configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        }
    }

    @Override
    protected void updateInputs(DriveInputsAutoLogged inputs) {
        inputs.driveMotorPositions = new double[4];
        inputs.driveMotorVelocities = new double[4];
        inputs.swerveMotorAbsolutePositions = new double[4];
        inputs.swerveMotorRelativePositions = new double[4];
        inputs.driveMotorCurrents = new double[4];
        inputs.swerveMotorCurrents = new double[4];
        inputs.driveMotorTemps = new double[4];
        inputs.swerveMotorTemps = new double[4];
        inputs.driveMotorVoltages = new double[4];
        inputs.swerveMotorVoltages = new double[4];
        StickyFaults stickyFaults = new StickyFaults();

        inputs.driveIoTimestamp = Logger.getInstance().getRealTimestamp() * SECONDS_PER_MICROSECOND;

        for (int i = 0; i < 4; i++) {
            inputs.driveMotorPositions[i] =
                    swerveDriveMotors[i].getSelectedSensorPosition() / FALCON_ENCODER_TICKS_PER_ROTATIONS * SWERVE_DRIVE_MOTOR_REDUCTION;
            inputs.driveMotorVelocities[i] =
                    swerveDriveMotors[i].getSelectedSensorVelocity() * FALCON_ENCODER_TICKS_PER_100_MS_TO_RPM
                            * SWERVE_DRIVE_MOTOR_REDUCTION * FALCON_ENCODER_TICKS_PER_ROTATIONS;
            inputs.driveMotorCurrents[i] = swerveDriveMotors[i].getOutputCurrent();
            inputs.driveMotorTemps[i] = swerveDriveMotors[i].getTemperature();
            inputs.driveMotorVoltages[i] = swerveDriveMotors[i].getBusVoltage() * swerveDriveMotors[i].getMotorOutputVoltage();
            inputs.swerveMotorAbsolutePositions[i] = swerveCanCoders[i].getAbsolutePosition();

            inputs.swerveMotorCurrents[i] = swerveMotors[i].getOutputCurrent();
            inputs.swerveMotorTemps[i] = swerveMotors[i].getTemperature();
            inputs.swerveMotorVoltages[i] = swerveMotors[i].getBusVoltage() * swerveMotors[i].getMotorOutputVoltage();
            inputs.swerveMotorRelativePositions[i] =
                    swerveMotors[i].getSelectedSensorPosition() / FALCON_ENCODER_TICKS_PER_ROTATIONS
                            * SWERVE_MOTOR_POSITION_CONVERSION_FACTOR * 360;
            inputs.driveMotorFaults[i] = swerveDriveMotors[i].getStickyFaults(stickyFaults).ordinal();
            inputs.swerveMotorFaults[i] = swerveMotors[i].getStickyFaults(stickyFaults).ordinal();
        }
    }

    private boolean isBreaking = false;

    @Override
    protected void setBrakeMode(boolean enable) {
        if (isBreaking != enable) {
            NeutralMode mode = enable ? NeutralMode.Brake : NeutralMode.Coast;
            for (TalonFX swerveMotor : swerveMotors) {
                swerveMotor.setNeutralMode(mode);
            }
            for (TalonFX swerveDriveMotor : swerveDriveMotors) {
                swerveDriveMotor.setNeutralMode(mode);
            }
            isBreaking = enable;
        }
    }

    /**
     * Set the target position of the selected swerve motor
     *
     * @param motorNum the selected swerve motor
     * @param position the target position in degrees (0-360)
     */
    @Override
    protected void setSwerveMotorPosition(int motorNum, double position) {
        swerveMotors[motorNum].set(ControlMode.Position,
                position * FALCON_ENCODER_TICKS_PER_ROTATIONS / SWERVE_MOTOR_POSITION_CONVERSION_FACTOR / 360);
    }

    /**
     * Set the target position of the selected swerve motor
     *
     * @param motorNum the selected swerve motor
     * @param voltage  the wanted voltage
     */
    @Override
    protected void setDriveMotorVoltage(int motorNum, double voltage) {
        swerveMotors[motorNum].set(ControlMode.PercentOutput, voltage);
    }


    @Override
    protected void setSwerveMotorVoltage(int motorNum, double voltage) {
        swerveDriveMotors[motorNum].set(ControlMode.PercentOutput, voltage);
    }

    public void resetAbsoluteZeros() {
        for (int i = 0; i < swerveCanCoders.length; i++) {
            CANCoder swerveCanCoder = swerveCanCoders[i];
            System.out.println(i + " Setting Zero " + swerveCanCoder.configGetMagnetOffset() + " -> 0");
            swerveCanCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
            swerveCanCoder.configMagnetOffset(
                    -(swerveCanCoder.getAbsolutePosition() - swerveCanCoder.configGetMagnetOffset())
            );
        }
    }

    public void setDriveVoltageCompLevel(double voltage) {
        SupplyCurrentLimitConfiguration currentLimitConfiguration = new SupplyCurrentLimitConfiguration();
        currentLimitConfiguration.currentLimit = voltage;
        for (TalonFX driveMotor : swerveDriveMotors) {
            if (voltage <= 0) {
                driveMotor.enableVoltageCompensation(false);
            } else if (voltage > 0) {
                driveMotor.configSupplyCurrentLimit(currentLimitConfiguration);
            }
        }
    }
}
