package frc.subsytem.drive;

import com.ctre.phoenixpro.configs.Slot0Configs;
import com.ctre.phoenixpro.configs.VoltageConfigs;
import com.ctre.phoenixpro.controls.CoastOut;
import com.ctre.phoenixpro.controls.StaticBrake;
import com.ctre.phoenixpro.hardware.CANcoder;
import com.ctre.phoenixpro.hardware.TalonFX;
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

    private final @NotNull CANcoder[] swerveCanCoders;
    private final ReentrantLock swerveAutoControllerLock = new ReentrantLock();

    public FalconDriveIO() {
        final @NotNull TalonFX leftFrontTalon, leftBackTalon, rightFrontTalon, rightBackTalon;
        final @NotNull TalonFX leftFrontTalonSwerve, leftBackTalonSwerve, rightFrontTalonSwerve, rightBackTalonSwerve;

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
            final @NotNull CANcoder leftFrontCanCoder, leftBackCanCoder, rightFrontCanCoder, rightBackCanCoder;

            leftFrontCanCoder = new CANcoder(Constants.CAN_LEFT_FRONT_ID, "*");
            leftBackCanCoder = new CANcoder(Constants.CAN_LEFT_BACK_ID, "*");
            rightFrontCanCoder = new CANcoder(Constants.CAN_RIGHT_FRONT_ID, "*");
            rightBackCanCoder = new CANcoder(Constants.CAN_RIGHT_BACK_ID, "*");

            swerveCanCoders = new CANcoder[4];
            swerveCanCoders[0] = leftFrontCanCoder;
            swerveCanCoders[1] = leftBackCanCoder;
            swerveCanCoders[2] = rightFrontCanCoder;
            swerveCanCoders[3] = rightBackCanCoder;
        } else {
            swerveCanCoders = null;
        }


        for (int i = 0; i < 4; i++) {
            // Sets swerveMotors PID
            Slot0Configs PIDConfigs = new Slot0Configs();
            PIDConfigs.kP = SWERVE_DRIVE_P;
            PIDConfigs.kI = SWERVE_DRIVE_I;
            PIDConfigs.kD = SWERVE_DRIVE_D;
            PIDConfigs.kS = 0;
            PIDConfigs.kV = 0;

            swerveMotors[i].getConfigurator().apply(PIDConfigs);

            // Sets current limits for motors
            VoltageConfigs voltageConfigs = new VoltageConfigs();
            voltageConfigs.PeakForwardVoltage = SWERVE_MOTOR_CURRENT_LIMIT;
            voltageConfigs.SupplyVoltageTimeConstant = SWERVE_DRIVE_VOLTAGE_LIMIT_AUTO;
            swerveMotors[i].getConfigurator().apply(voltageConfigs);
            swerveDriveMotors[i].getConfigurator().apply(voltageConfigs);

            swerveMotors[i].setStatusFramePeriod(StatusFrame.Status_1_General, 10);
            swerveMotors[i].setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 101);

            CoastOut coastOut = new CoastOut();
            swerveDriveMotors[i].setControl(coastOut);
            swerveMotors[i].setControl(coastOut);

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

        inputs.driveIoTimestamp = Logger.getInstance().getRealTimestamp() * SECONDS_PER_MICROSECOND;

        for (int i = 0; i < 4; i++) {
            inputs.driveMotorPositions[i] =
                    swerveDriveMotors[i].getPosition().getValue() / FALCON_ENCODER_TICKS_PER_ROTATIONS * SWERVE_DRIVE_MOTOR_REDUCTION;
            inputs.driveMotorVelocities[i] =
                    swerveDriveMotors[i].getVelocity().getValue() * FALCON_ENCODER_TICKS_PER_100_MS_TO_RPM
                            * SWERVE_DRIVE_MOTOR_REDUCTION * FALCON_ENCODER_TICKS_PER_ROTATIONS;
            inputs.driveMotorCurrents[i] = swerveDriveMotors[i].getSupplyCurrent().getValue();
            inputs.driveMotorTemps[i] = swerveDriveMotors[i].getDeviceTemp().getValue();
            inputs.driveMotorVoltages[i] = swerveDriveMotors[i].getSupplyVoltage().getValue(); // Can't find bus voltage
            inputs.swerveMotorAbsolutePositions[i] = swerveCanCoders[i].getAbsolutePosition().getValue();

            inputs.swerveMotorCurrents[i] = swerveMotors[i].getSupplyCurrent().getValue();
            inputs.swerveMotorTemps[i] = swerveMotors[i].getDeviceTemp().getValue();
            inputs.swerveMotorVoltages[i] = swerveMotors[i].getSupplyVoltage().getValue(); // Can't find bus voltage
            inputs.swerveMotorRelativePositions[i] =
                    swerveMotors[i].getPosition().getValue() / FALCON_ENCODER_TICKS_PER_ROTATIONS
                            * SWERVE_MOTOR_POSITION_CONVERSION_FACTOR * 360;
            inputs.driveMotorFaults[i] = swerveDriveMotors[i]. //Which faults do we need here
            inputs.swerveMotorFaults[i] = swerveMotors[i]. //Which faults do we need here
        }
    }

    private boolean isBreaking = false;

    @Override
    protected void setBrakeMode(boolean enable) {
        if (isBreaking != enable) {
            CoastOut coastOut = new CoastOut();
            StaticBrake staticBrake = new StaticBrake();
            for (TalonFX swerveMotor : swerveMotors) {
                swerveMotor.setControl(enable ? staticBrake : coastOut);
            }
            for (TalonFX swerveDriveMotor : swerveDriveMotors) {
                swerveDriveMotor.setControl(enable ? staticBrake : coastOut);
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
        swerveMotors[motorNum].setRotorPosition(position * FALCON_ENCODER_TICKS_PER_ROTATIONS
                / SWERVE_MOTOR_POSITION_CONVERSION_FACTOR / 360);
    }

    /**
     * Set the target position of the selected swerve motor
     *
     * @param motorNum the selected swerve motor
     * @param voltage  the wanted voltage
     */
    @Override
    protected void setDriveMotorVoltage(int motorNum, double voltage) {
        swerveMotors[motorNum].set(voltage);
    }


    @Override
    protected void setSwerveMotorVoltage(int motorNum, double voltage) {
        swerveDriveMotors[motorNum].set(voltage);
    }

    public void resetAbsoluteZeros() {
        for (int i = 0; i < swerveCanCoders.length; i++) {
            CANcoder swerveCanCoder = swerveCanCoders[i];
            System.out.println(i + " Setting Zero " + swerveCanCoder.get() + " -> 0");
            swerveCanCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
            swerveCanCoder.configMagnetOffset(
                    -(swerveCanCoder.getAbsolutePosition() - swerveCanCoder.configGetMagnetOffset())
            );
        }
    }

    public void setDriveVoltageCompLevel(double voltage) {
        VoltageConfigs voltageConfigs = new VoltageConfigs();
        voltageConfigs.PeakForwardVoltage = SWERVE_MOTOR_CURRENT_LIMIT;
        voltageConfigs.SupplyVoltageTimeConstant = SWERVE_DRIVE_VOLTAGE_LIMIT_AUTO;
        for (TalonFX driveMotor : swerveDriveMotors) {
            if (voltage <= 0) {
                voltageConfigs.PeakForwardVoltage = 0;
                voltageConfigs.SupplyVoltageTimeConstant = 0;
            }
            driveMotor.getConfigurator().apply(voltageConfigs);
        }
    }
}
