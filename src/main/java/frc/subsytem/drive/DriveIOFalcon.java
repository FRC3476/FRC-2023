package frc.subsytem.drive;

import com.ctre.phoenixpro.configs.*;
import com.ctre.phoenixpro.controls.PositionVoltage;
import com.ctre.phoenixpro.controls.TorqueCurrentFOC;
import com.ctre.phoenixpro.controls.VoltageOut;
import com.ctre.phoenixpro.hardware.CANcoder;
import com.ctre.phoenixpro.hardware.TalonFX;
import com.ctre.phoenixpro.signals.FeedbackSensorSourceValue;
import com.ctre.phoenixpro.signals.InvertedValue;
import com.ctre.phoenixpro.signals.NeutralModeValue;
import frc.robot.Constants;
import org.jetbrains.annotations.NotNull;
import org.littletonrobotics.junction.Logger;

import java.util.concurrent.locks.ReentrantLock;

import static frc.robot.Constants.*;

public class DriveIOFalcon extends DriveIO {
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

    public DriveIOFalcon() {
        if (!USE_CANCODERS) {
            throw new IllegalStateException("CANCoders are not enabled");
        }

        final @NotNull TalonFX leftFrontTalon, leftBackTalon, rightFrontTalon, rightBackTalon;
        final @NotNull TalonFX leftFrontTalonSwerve, leftBackTalonSwerve, rightFrontTalonSwerve, rightBackTalonSwerve;

        // NOTE: The "*" canbus means use any canivore bus

        // Swerve Drive Motors
        leftFrontTalon = new TalonFX(Constants.DRIVE_LEFT_FRONT_ID, "*");
        leftBackTalon = new TalonFX(Constants.DRIVE_LEFT_BACK_ID, "*");
        rightFrontTalon = new TalonFX(Constants.DRIVE_RIGHT_FRONT_ID, "*");
        rightBackTalon = new TalonFX(Constants.DRIVE_RIGHT_BACK_ID, "*");

        leftFrontTalon.setInverted(false);
        rightFrontTalon.setInverted(false);
        leftBackTalon.setInverted(false);
        rightBackTalon.setInverted(false);

        leftFrontTalonSwerve = new TalonFX(Constants.DRIVE_LEFT_FRONT_SWERVE_ID, "*");
        leftBackTalonSwerve = new TalonFX(Constants.DRIVE_LEFT_BACK_SWERVE_ID, "*");
        rightFrontTalonSwerve = new TalonFX(Constants.DRIVE_RIGHT_FRONT_SWERVE_ID, "*");
        rightBackTalonSwerve = new TalonFX(Constants.DRIVE_RIGHT_BACK_SWERVE_ID, "*");


        swerveMotors[0] = leftFrontTalonSwerve;
        swerveMotors[1] = leftBackTalonSwerve;
        swerveMotors[2] = rightFrontTalonSwerve;
        swerveMotors[3] = rightBackTalonSwerve;

        swerveDriveMotors[0] = leftFrontTalon;
        swerveDriveMotors[1] = leftBackTalon;
        swerveDriveMotors[2] = rightFrontTalon;
        swerveDriveMotors[3] = rightBackTalon;

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


        for (int i = 0; i < 4; i++) {
            swerveMotors[i].getConfigurator().defaultTimeoutSeconds = 10;
            swerveDriveMotors[i].getConfigurator().defaultTimeoutSeconds = 10;
            swerveCanCoders[i].getConfigurator().defaultTimeoutSeconds = 10;

            // Sets swerveMotors PID
            Slot0Configs pIDConfigsSwerve = new Slot0Configs();
            pIDConfigsSwerve.kP = SWERVE_DRIVE_P;
            pIDConfigsSwerve.kI = SWERVE_DRIVE_I;
            pIDConfigsSwerve.kD = SWERVE_DRIVE_D;
            pIDConfigsSwerve.kS = 0;
            pIDConfigsSwerve.kV = 0;
            swerveMotors[i].getConfigurator().apply(pIDConfigsSwerve, 1000);

            Slot0Configs pIDConfigsDrive = new Slot0Configs();
            pIDConfigsDrive.kP = 0;
            pIDConfigsDrive.kI = 0;
            pIDConfigsDrive.kD = 0;
            pIDConfigsDrive.kS = DRIVE_FEEDFORWARD[i].ks;
            pIDConfigsDrive.kV = DRIVE_FEEDFORWARD[i].kv;
            swerveDriveMotors[i].getConfigurator().apply(pIDConfigsDrive);

            // Sets current limits for motors
            var swerveCurrentLimitsConfigs = new CurrentLimitsConfigs();
            swerveCurrentLimitsConfigs.SupplyCurrentLimit = SWERVE_MOTOR_CURRENT_LIMIT;
            swerveCurrentLimitsConfigs.SupplyCurrentLimitEnable = true;
            swerveCurrentLimitsConfigs.StatorCurrentLimitEnable = false;

            swerveMotors[i].getConfigurator().apply(swerveCurrentLimitsConfigs);

            var driveCurrentLimitsConfigs = new CurrentLimitsConfigs();
            driveCurrentLimitsConfigs.SupplyCurrentLimit = SWERVE_DRIVE_MOTOR_CURRENT_LIMIT;
            driveCurrentLimitsConfigs.SupplyCurrentLimitEnable = true;
            driveCurrentLimitsConfigs.StatorCurrentLimitEnable = false;
            swerveDriveMotors[i].getConfigurator().apply(driveCurrentLimitsConfigs);

            var torqueCurrentConfigsSwerve = new TorqueCurrentConfigs();
            torqueCurrentConfigsSwerve.PeakForwardTorqueCurrent = SWERVE_MOTOR_CURRENT_LIMIT;
            torqueCurrentConfigsSwerve.PeakReverseTorqueCurrent = -SWERVE_MOTOR_CURRENT_LIMIT;
            torqueCurrentConfigsSwerve.TorqueNeutralDeadband = 1;
            swerveMotors[i].getConfigurator().apply(torqueCurrentConfigsSwerve);

            var torqueCurrentConfigsDrive = new TorqueCurrentConfigs();
            torqueCurrentConfigsDrive.PeakForwardTorqueCurrent = SWERVE_DRIVE_MOTOR_CURRENT_LIMIT;
            torqueCurrentConfigsDrive.PeakReverseTorqueCurrent = -SWERVE_DRIVE_MOTOR_CURRENT_LIMIT;
            torqueCurrentConfigsDrive.TorqueNeutralDeadband = 1;
            swerveDriveMotors[i].getConfigurator().apply(torqueCurrentConfigsDrive);


            var swerveFeedbackConfigs = new FeedbackConfigs();
            swerveFeedbackConfigs.FeedbackRemoteSensorID = 19 + i;
            swerveFeedbackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
            swerveFeedbackConfigs.SensorToMechanismRatio = 1;
            swerveFeedbackConfigs.RotorToSensorRatio = 1 / SWERVE_MOTOR_POSITION_CONVERSION_FACTOR;
            swerveMotors[i].getConfigurator().apply(swerveFeedbackConfigs);


            var driveFeedbackConfigs = new FeedbackConfigs();
            driveFeedbackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
            driveFeedbackConfigs.SensorToMechanismRatio = 1 / (SWERVE_DRIVE_MOTOR_REDUCTION * SWERVE_METER_PER_ROTATION);
            driveFeedbackConfigs.RotorToSensorRatio = 1;
            swerveDriveMotors[i].getConfigurator().apply(driveFeedbackConfigs);


            var absolutePosition = swerveCanCoders[i].getAbsolutePosition();
            absolutePosition.waitForUpdate(1);
            swerveMotors[i].setRotorPosition(absolutePosition.getValue(), 1);
        }

        isBraking = false; // ensure it actually sets the values
        setBrakeMode(true);
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
            inputs.driveMotorPositions[i] = swerveDriveMotors[i].getPosition().getValue();
            inputs.driveMotorVelocities[i] = swerveDriveMotors[i].getVelocity().getValue();
            inputs.driveMotorCurrents[i] = swerveDriveMotors[i].getSupplyCurrent().getValue();
            inputs.driveMotorTemps[i] = swerveDriveMotors[i].getDeviceTemp().getValue();
            inputs.driveMotorVoltages[i] = swerveDriveMotors[i].getSupplyVoltage().getValue();
            inputs.supplyDriveMotorVoltages[i] = swerveDriveMotors[i].getSupplyVoltage().getValue();
            inputs.swerveMotorAbsolutePositions[i] =
                    swerveCanCoders[i].getAbsolutePosition().getValue() * 360; //conv rotations to degrees

            inputs.swerveMotorCurrents[i] = swerveMotors[i].getSupplyCurrent().getValue();
            inputs.swerveMotorTemps[i] = swerveMotors[i].getDeviceTemp().getValue();
            inputs.swerveMotorVoltages[i] = swerveMotors[i].getSupplyVoltage().getValue();
            inputs.supplySwerveMotorVoltages[i] = swerveMotors[i].getSupplyVoltage().getValue();
            inputs.swerveMotorRelativePositions[i] = swerveMotors[i].getPosition().getValue() * 360; //conv rotations to degrees
            inputs.driveMotorFaults[i] = swerveDriveMotors[i].getStickyFaultField().getValue(); //Which faults do we need here
            inputs.swerveMotorFaults[i] = swerveMotors[i].getStickyFaultField().getValue(); //Which faults do we need here
        }
    }

    private boolean isBraking = false;

    private final MotorOutputConfigs coastModeInverted = new MotorOutputConfigs();
    private final MotorOutputConfigs brakeModeInverted = new MotorOutputConfigs();
    private final MotorOutputConfigs coastMode = new MotorOutputConfigs();
    private final MotorOutputConfigs brakeMode = new MotorOutputConfigs();

    {
        coastMode.NeutralMode = NeutralModeValue.Coast;
        brakeMode.NeutralMode = NeutralModeValue.Brake;

        coastModeInverted.NeutralMode = NeutralModeValue.Coast;
        brakeModeInverted.NeutralMode = NeutralModeValue.Brake;

        coastModeInverted.Inverted = InvertedValue.Clockwise_Positive;
        brakeModeInverted.Inverted = InvertedValue.Clockwise_Positive;
    }

    @Override
    protected void setBrakeMode(boolean enable) {
        if (isBraking != enable) {
            for (TalonFX swerveMotor : swerveMotors) {
                swerveMotor.getConfigurator().apply(enable ? brakeModeInverted : coastModeInverted);
            }
            for (TalonFX swerveDriveMotor : swerveDriveMotors) {
                swerveDriveMotor.getConfigurator().apply(enable ? brakeMode : coastMode);
            }
            isBraking = enable;
        }
    }

    private final PositionVoltage positionVoltage = new PositionVoltage(0, true, 0, 0, true);
    /**
     * Set the target position of the selected swerve motor
     *
     * @param motorNum the selected swerve motor
     * @param position the target position in degrees (0-360)
     */
    @Override
    protected void setSwerveMotorPosition(int motorNum, double position) {
        positionVoltage.Position = position / 360;
        swerveMotors[motorNum].setControl(positionVoltage);
    }

    private final VoltageOut voltageOut = new VoltageOut(0, true, false);
    private final TorqueCurrentFOC currentOut
            = new TorqueCurrentFOC(SWERVE_DRIVE_MOTOR_CURRENT_LIMIT, 0, 1, false);


    /**
     * Set the target position of the selected swerve motor
     *
     * @param motorNum the selected swerve motor
     * @param voltage  the wanted voltage
     */
    @Override
    protected void setDriveMotorVoltage(int motorNum, double voltage, boolean voltageControl) {
        if (voltageControl) {
            voltageOut.Output = voltage;
            swerveDriveMotors[motorNum].setControl(voltageOut);
        } else {
            currentOut.Output = voltage / SWERVE_DRIVE_VOLTAGE_LIMIT_AUTO;
            swerveDriveMotors[motorNum].setControl(currentOut);
        }
    }


    @Override
    protected void setSwerveMotorVoltage(int motorNum, double voltage) {
        swerveDriveMotors[motorNum].set(voltage);
    }

    public void resetAbsoluteZeros() {
        System.out.println("Beginning to Reset Zeros");
        for (int i = 0; i < swerveCanCoders.length; i++) {
            CANcoder swerveCanCoder = swerveCanCoders[i];
            var oldVal = swerveCanCoder.getAbsolutePosition().getValue();
            swerveCanCoder.getConfigurator().apply(new MagnetSensorConfigs(), 1); // reset magnet offset
            swerveCanCoder.getAbsolutePosition().waitForUpdate(1, true);

            var magneticSensorConfigs = new MagnetSensorConfigs();
            magneticSensorConfigs.MagnetOffset = -(swerveCanCoder.getAbsolutePosition().getValue());
            swerveCanCoder.getConfigurator().apply(magneticSensorConfigs, 1);
            swerveCanCoder.getAbsolutePosition().waitForUpdate(1, true);

            System.out.println(i + " Setting Zero " + oldVal + " -> " + swerveCanCoder.getAbsolutePosition().getValue());
        }
    }

    public void setDriveVoltageCompLevel(double voltage) {
        VoltageConfigs voltageConfigs = new VoltageConfigs();
        if (voltage > 0) {
            voltageConfigs.PeakForwardVoltage = voltage;
            voltageConfigs.PeakReverseVoltage = -voltage;
        } // else leave at default (16V)
        for (TalonFX driveMotor : swerveDriveMotors) {
            driveMotor.getConfigurator().apply(voltageConfigs);
        }
    }

    @Override
    public void resetPeriodicFrames() {
        // We use the default periodic frames, we don't need to do anything here
    }
}
