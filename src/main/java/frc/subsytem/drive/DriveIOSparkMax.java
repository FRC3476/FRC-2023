package frc.subsytem.drive;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import frc.robot.Constants;
import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

import java.util.concurrent.locks.ReentrantLock;

import static edu.wpi.first.wpilibj.RobotBase.isReal;
import static frc.robot.Constants.*;

public class DriveIOSparkMax extends DriveIO {
    /**
     * Motors that turn the wheels around. Uses Falcon500s
     */
    private final @NotNull CANSparkMax[] swerveMotors = new CANSparkMax[4];
    /**
     * Motors that are driving the robot around and causing it to move
     */
    private final @NotNull CANSparkMax[] swerveDriveMotors = new CANSparkMax[4];
    /**
     * Absolute Encoders for the motors that turn the wheel
     */

    private final @NotNull CANCoder[] swerveCanCoders;
    private final ReentrantLock swerveAutoControllerLock = new ReentrantLock();
    /**
     * Absolute Encoders for the motors that turn the wheel
     */

    private final @Nullable SparkMaxAbsoluteEncoder[] swerveSparkAbsoluteEncoders = new SparkMaxAbsoluteEncoder[4];


    public DriveIOSparkMax() {
        final @NotNull CANSparkMax leftFrontSpark, leftBackSpark, rightFrontSpark, rightBackSpark;
        final @NotNull CANSparkMax leftFrontSparkSwerve, leftBackSparkSwerve, rightFrontSparkSwerve, rightBackSparkSwerve;
        // Swerve Drive Motors
        leftFrontSpark = new CANSparkMax(Constants.DRIVE_LEFT_FRONT_ID, MotorType.kBrushless);
        leftBackSpark = new CANSparkMax(Constants.DRIVE_LEFT_BACK_ID, MotorType.kBrushless);
        rightFrontSpark = new CANSparkMax(Constants.DRIVE_RIGHT_FRONT_ID, MotorType.kBrushless);
        rightBackSpark = new CANSparkMax(Constants.DRIVE_RIGHT_BACK_ID, MotorType.kBrushless);

        leftFrontSpark.setInverted(false);
        rightFrontSpark.setInverted(false);
        leftBackSpark.setInverted(false);
        rightBackSpark.setInverted(false);

        leftFrontSparkSwerve = new CANSparkMax(Constants.DRIVE_LEFT_FRONT_SWERVE_ID, MotorType.kBrushless);
        leftBackSparkSwerve = new CANSparkMax(Constants.DRIVE_LEFT_BACK_SWERVE_ID, MotorType.kBrushless);
        rightFrontSparkSwerve = new CANSparkMax(Constants.DRIVE_RIGHT_FRONT_SWERVE_ID, MotorType.kBrushless);
        rightBackSparkSwerve = new CANSparkMax(Constants.DRIVE_RIGHT_BACK_SWERVE_ID, MotorType.kBrushless);


        swerveMotors[0] = leftFrontSparkSwerve;
        swerveMotors[1] = leftBackSparkSwerve;
        swerveMotors[2] = rightFrontSparkSwerve;
        swerveMotors[3] = rightBackSparkSwerve;

        swerveDriveMotors[0] = leftFrontSpark;
        swerveDriveMotors[1] = leftBackSpark;
        swerveDriveMotors[2] = rightFrontSpark;
        swerveDriveMotors[3] = rightBackSpark;

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
            swerveMotors[i].getPIDController().setP(Constants.SWERVE_DRIVE_P);
            swerveMotors[i].getPIDController().setD(Constants.SWERVE_DRIVE_D);
            swerveMotors[i].getPIDController().setI(Constants.SWERVE_DRIVE_I);
            swerveMotors[i].getPIDController().setFF(Constants.SWERVE_DRIVE_F);
            swerveMotors[i].getPIDController().setIZone(Constants.SWERVE_DRIVE_INTEGRAL_ZONE);
            swerveMotors[i].getPIDController().setPositionPIDWrappingMaxInput(360);
            swerveMotors[i].getPIDController().setPositionPIDWrappingMinInput(0);
            swerveMotors[i].getPIDController().setPositionPIDWrappingEnabled(true);

            // Sets current limits for motors
            swerveMotors[i].setSmartCurrentLimit(SWERVE_MOTOR_CURRENT_LIMIT);
            swerveMotors[i].enableVoltageCompensation(Constants.SWERVE_DRIVE_VOLTAGE_LIMIT);
            swerveMotors[i].getEncoder().setPositionConversionFactor(
                    Constants.SWERVE_MOTOR_POSITION_CONVERSION_FACTOR * 360);
            swerveMotors[i].getEncoder().setVelocityConversionFactor(
                    Constants.SWERVE_MOTOR_POSITION_CONVERSION_FACTOR * 360 / SECONDS_PER_MINUTE);

            swerveDriveMotors[i].setSmartCurrentLimit(SWERVE_DRIVE_MOTOR_CURRENT_LIMIT);
            swerveDriveMotors[i].enableVoltageCompensation(Constants.SWERVE_DRIVE_VOLTAGE_LIMIT);
            swerveDriveMotors[i].getEncoder().setPositionConversionFactor(
                    SWERVE_DRIVE_MOTOR_REDUCTION * SWERVE_METER_PER_ROTATION);
            swerveDriveMotors[i].getEncoder().setVelocityConversionFactor(
                    SWERVE_DRIVE_MOTOR_REDUCTION * SWERVE_METER_PER_ROTATION / SECONDS_PER_MINUTE);
            swerveDriveMotors[i].setIdleMode(IdleMode.kCoast);
            swerveMotors[i].setIdleMode(IdleMode.kCoast);

            swerveMotors[i].setInverted(true);

            if (USE_CANCODERS) {
                swerveCanCoders[i].setStatusFramePeriod(CANCoderStatusFrame.VbatAndFaults, 200);
                swerveCanCoders[i].setStatusFramePeriod(CANCoderStatusFrame.SensorData, 20);
                swerveCanCoders[i].configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
            } else {
                swerveSparkAbsoluteEncoders[i] = swerveMotors[i].getAbsoluteEncoder(Type.kDutyCycle);
                swerveMotors[i].getPIDController().setFeedbackDevice(swerveSparkAbsoluteEncoders[i]);
                swerveSparkAbsoluteEncoders[i].setPositionConversionFactor(360);
                swerveSparkAbsoluteEncoders[i].setVelocityConversionFactor(360 / 60.0);
                swerveMotors[i].getPIDController().setPositionPIDWrappingEnabled(true);
                swerveMotors[i].getPIDController().setOutputRange(-1, 1);
            }
            if (isReal()) {
                swerveMotors[i].burnFlash();
                swerveDriveMotors[i].burnFlash();
            }
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

        for (int i = 0; i < 4; i++) {
            inputs.driveMotorPositions[i] = swerveDriveMotors[i].getEncoder().getPosition();
            inputs.driveMotorVelocities[i] = swerveDriveMotors[i].getEncoder().getVelocity();
            inputs.driveMotorCurrents[i] = swerveDriveMotors[i].getOutputCurrent();
            inputs.driveMotorTemps[i] = swerveDriveMotors[i].getMotorTemperature();
            inputs.driveMotorVoltages[i] = swerveDriveMotors[i].getBusVoltage();

            if (USE_CANCODERS) {
                inputs.swerveMotorAbsolutePositions[i] = swerveCanCoders[i].getAbsolutePosition();
            } else {
                assert swerveSparkAbsoluteEncoders[i] != null;
                inputs.swerveMotorAbsolutePositions[i] = swerveSparkAbsoluteEncoders[i].getPosition();
            }

            inputs.swerveMotorCurrents[i] = swerveMotors[i].getOutputCurrent();
            inputs.swerveMotorTemps[i] = swerveMotors[i].getMotorTemperature();
            inputs.swerveMotorVoltages[i] = swerveMotors[i].getBusVoltage();
            inputs.swerveMotorRelativePositions[i] = swerveMotors[i].getEncoder().getPosition();
        }
    }

    private boolean isBreaking = false;

    @Override
    protected void setBrakeMode(boolean enable) {
        if (isBreaking != enable) {
            for (CANSparkMax swerveMotor : swerveMotors) {
                swerveMotor.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
            }
            for (CANSparkMax swerveDriveMotor : swerveDriveMotors) {
                swerveDriveMotor.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
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
        swerveMotors[motorNum].getPIDController().setReference(position, ControlType.kPosition);
    }

    /**
     * Set the target position of the selected swerve motor
     *
     * @param motorNum the selected swerve motor
     * @param voltage  the wanted voltage
     */
    @Override
    protected void setSwerveMotorVoltage(int motorNum, double voltage) {
        swerveMotors[motorNum].getPIDController().setReference(voltage, ControlType.kVoltage);
    }


    @Override
    protected void setDriveMotorVoltage(int motorNum, double voltage) {
        swerveDriveMotors[motorNum].getPIDController().setReference(voltage, ControlType.kVoltage);
    }

    public void resetAbsoluteZeros() {
        if (USE_CANCODERS) {
            for (int i = 0; i < swerveCanCoders.length; i++) {
                CANCoder swerveCanCoder = swerveCanCoders[i];
                System.out.println(i + " Setting Zero " + swerveCanCoder.configGetMagnetOffset() + " -> 0");
                swerveCanCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
                swerveCanCoder.configMagnetOffset(
                        -(swerveCanCoder.getAbsolutePosition() - swerveCanCoder.configGetMagnetOffset())
                );
            }
        } else {
            for (int i = 0; i < swerveSparkAbsoluteEncoders.length; i++) {
                AbsoluteEncoder swerveSparkAbsoluteEncoder = swerveSparkAbsoluteEncoders[i];
                System.out.println(i + " Setting Zero " + swerveSparkAbsoluteEncoder.getZeroOffset() + " -> 0");
                swerveSparkAbsoluteEncoder.setZeroOffset(
                        -(swerveSparkAbsoluteEncoder.getPosition() - swerveSparkAbsoluteEncoder.getZeroOffset())
                );
                swerveMotors[i].burnFlash();
            }
        }
    }
}
