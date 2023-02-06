package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.utility.swerve.SwerveSetpointGenerator.KinematicLimit;
import frc.utility.swerve.SwerveSetpointGenerator.SwerveSetpoint;
import org.jetbrains.annotations.NotNull;

import java.io.File;
import java.nio.file.Files;

public final class Constants {
    public static final boolean IS_PRACTICE = Files.exists(new File("/home/lvuser/practice").toPath());

    // 2048 sensor units per revolution
    public static final double FALCON_ENCODER_TICKS_PER_ROTATIONS = 2048;
    public static final double FALCON_ENCODER_TICKS_PER_100_MS_TO_RPM = 600 / 2048.0d;
    public static final double SWERVE_MOTOR_POSITION_CONVERSION_FACTOR = 1 / 12.8;

    // Logging Period
    /**
     * Every subsystem will log one for every 20 update periods
     */
    public static final int DEFAULT_PERIODS_PER_LOG = 20;

    //Drive Constants
    public static final int DRIVE_PERIOD = 20;

    public static final int DRIVE_LEFT_FRONT_ID = 11;
    public static final int DRIVE_LEFT_BACK_ID = 12;
    public static final int DRIVE_RIGHT_FRONT_ID = 13;
    public static final int DRIVE_RIGHT_BACK_ID = 14;

    public static final int DRIVE_LEFT_FRONT_SWERVE_ID = 15;
    public static final int DRIVE_LEFT_BACK_SWERVE_ID = 16;
    public static final int DRIVE_RIGHT_FRONT_SWERVE_ID = 17;
    public static final int DRIVE_RIGHT_BACK_SWERVE_ID = 18;

    public static final int CAN_LEFT_FRONT_ID = 19;
    public static final int CAN_LEFT_BACK_ID = 20;
    public static final int CAN_RIGHT_FRONT_ID = 21;
    public static final int CAN_RIGHT_BACK_ID = 22;

    public static final double SWERVE_INCHES_PER_ROTATION = 12.5 * 0.976;
    public static final double SWERVE_METER_PER_ROTATION = Units.inchesToMeters(SWERVE_INCHES_PER_ROTATION);
    public static final double SWERVE_DRIVE_P = 0.1;
    public static final double SWERVE_DRIVE_D = 0.00;
    public static final double SWERVE_DRIVE_I = 0.00;
    public static final double SWERVE_DRIVE_F = 0.00;
    public static final double SWERVE_DRIVE_INTEGRAL_ZONE = 0.00;

    /**
     * Feed forward constants for the drivetrain.
     * <p>
     * 0 -> Left Front
     * <p>
     * 1 -> Left Back
     * <p>
     * 2 -> Right Front
     * <p>
     * 3 -> Right Back
     */
    public static final SimpleMotorFeedforward[] DRIVE_FEEDFORWARD = {
            new SimpleMotorFeedforward(0.17763, 2.7731, 0.5),
            new SimpleMotorFeedforward(0.17763, 2.7731, 0.5),
            new SimpleMotorFeedforward(0.17763, 2.7731, 0.5),
            new SimpleMotorFeedforward(0.17763, 2.7731, 0.5)};


    /**
     * What the module states should be in hold mode. The wheels will be put in an X pattern to prevent the robot from moving.
     * <p>
     * 0 -> Left Front
     * <p>
     * 1 -> Left Back
     * <p>
     * 2 -> Right Front
     * <p>
     * 3 -> Right Back
     */
    public static final SwerveSetpoint HOLD_MODULE_STATES = new SwerveSetpoint(new ChassisSpeeds(), new SwerveModuleState[]{
            new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(45))
    }, new double[]{0, 0, 0, 0});

    // 0.307975 is 12.125 in inches
    public static final @NotNull Translation2d SWERVE_LEFT_FRONT_LOCATION = new Translation2d(0.307975, 0.307975);
    public static final @NotNull Translation2d SWERVE_LEFT_BACK_LOCATION = new Translation2d(-0.307975, 0.307975);
    public static final @NotNull Translation2d SWERVE_RIGHT_FRONT_LOCATION = new Translation2d(0.307975, -0.307975);
    public static final @NotNull Translation2d SWERVE_RIGHT_BACK_LOCATION = new Translation2d(-0.307975, -0.307975);
    public static final @NotNull Translation2d @NotNull [] SWERVE_MODULE_LOCATIONS = {
            SWERVE_LEFT_FRONT_LOCATION,
            SWERVE_LEFT_BACK_LOCATION,
            SWERVE_RIGHT_FRONT_LOCATION,
            SWERVE_RIGHT_BACK_LOCATION
    };

    @NotNull
    public static final SwerveDriveKinematics SWERVE_DRIVE_KINEMATICS = new SwerveDriveKinematics(
            SWERVE_MODULE_LOCATIONS
    );

    public static final double DRIVE_HIGH_SPEED_M = 4.2;
    /**
     * Allowed Turn Error in degrees.
     */
    public static final double MAX_TURN_ERROR = 30;

    public static final int SWERVE_MOTOR_CURRENT_LIMIT = 15;
    public static final int SWERVE_DRIVE_MOTOR_CURRENT_LIMIT = 15;
    public static final int SWERVE_DRIVE_VOLTAGE_LIMIT = 12;

    public static final double SWERVE_DRIVE_MOTOR_REDUCTION = 1 / 8.14;
    // TurnPID

    public static final double DEFAULT_TURN_P = 10.0;
    public static final double DEFAULT_TURN_I = 0;
    public static final double DEFAULT_TURN_D = 0.4;
    public static final double TURN_SPEED_LIMIT_WHILE_AIMING = 4.0;

    public static final double EXPECTED_TELEOP_DRIVE_DT = 0.2;

    public static final double EXPECTED_AUTO_DRIVE_DT = DRIVE_PERIOD / 1000.0;
    public static final int MAX_TELEOP_TURN_SPEED = 7;
    //Robot Tracker
    public static final int ROBOT_TRACKER_PERIOD = 20;
    /**
     * Acceleration due to gravity in meters per second squared. (9.80665 m/s^2)
     */
    public static final double GRAVITY = 9.80665;
    public static final int PIGEON_CAN_ID = 30;
    // Vision constants
    public static final int VISION_HANDLER_PERIOD = -1;
    public static final double COAST_AFTER_DISABLE_TIME = 0.5;
    public static final double FIELD_HEIGHT_METERS = 8.0137;
    public static final double FIELD_WIDTH_METERS = 16.54175;
    //Old Constants
    public static final int SWERVE_MOTOR_PID_TIMEOUT_MS = 50;
    public static final double SWERVE_ACCELERATION =
            ((360 * 10 * FALCON_ENCODER_TICKS_PER_ROTATIONS) / SWERVE_MOTOR_POSITION_CONVERSION_FACTOR) / 360; //5 rot/s^2
    public static final double SWERVE_CRUISE_VELOCITY =
            ((360 * 7 * FALCON_ENCODER_TICKS_PER_ROTATIONS) / SWERVE_MOTOR_POSITION_CONVERSION_FACTOR) / 360; //6.5 rot/s
    public static final double MAX_ANGULAR_ACCELERATION = Math.toRadians(360 * 22);
    /**
     * How many amps the intake can use while picking up
     */
    public static final int INTAKE_CURRENT_LIMIT = 20;
    /**
     * Percent output for intaking
     */
    public static final double INTAKE_OUTPUT_POWER = 1.0;
    /**
     * Percent output for holding
     */
    public static final double INTAKE_HOLD_POWER = 0.07;
    public static final int INTAKE_PERIOD = 20;
    public static final int INTAKE_CAN_ID = 40;
    public static final int ARM_CURRENT_LIMIT = 20;
    public static final int ARM_MOTOR_ID = 41;
    public static final int ARM_PERIOD = 20;
    public static final double ARM_P = 0.0001;
    public static final double ARM_I = 0;
    public static final double ARM_D = 0;
    public static final double ARM_TUCKED_ROTATION = 0;
    public static final double ARM_ROTATION_EXTENDED = -1469;
    public static final double ARM_GEAR_RATIO = 75;

    public enum KinematicLimits {
        /**
         * Normal acceleration limit while driving. This ensures that the driver can't tip the robot.
         */
        NORMAL_DRIVING(new KinematicLimit(4.2, 5, Math.PI * 2 * 6));
        public final KinematicLimit kinematicLimit;

        KinematicLimits(KinematicLimit kinematicLimit) {
            this.kinematicLimit = kinematicLimit;
        }
    }

    public static final double MOTOR_STARTING_TIME = 0.5;
    public static final double MOTOR_SPEED_DECREASING_RATE = -0.01;
    public static final double STALLING_CURRENT = -12;
}
