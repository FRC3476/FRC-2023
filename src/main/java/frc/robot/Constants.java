package frc.robot;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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
    public static final double AUTO_BALANCING_VELOCITY = 10;
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

    public static final int MAX_TELEOP_TURN_SPEED = 7;
    public static final double COAST_AFTER_DISABLE_TIME = 0.5;


    //Robot Tracker
    public static final int ROBOT_TRACKER_PERIOD = 20;
    /**
     * Acceleration due to gravity in meters per second squared. (9.80665 m/s^2)
     */
    public static final double GRAVITY = 9.80665;

    public static final int PIGEON_CAN_ID = 30;

    // Elevator
    public static final int ELEVATOR_PERIOD = 20;
    public static final TrapezoidProfile.Constraints ELEVATOR_CONSTRAINTS
            = new TrapezoidProfile.Constraints(3, 3);
    public static final ElevatorFeedforward ELEVATOR_FEEDFORWARD = new ElevatorFeedforward(0, 0, 0, 0);
    public static final int ELEVATOR_P = 5;
    public static final int ELEVATOR_I = 5;
    public static final int ELEVATOR_D = 5;
    public static final int ELEVATOR_POSITION_MULTIPLIER = 5;
    public static final int ELEVATOR_NOMINAL_VOLTAGE = 9;
    public static final int ELEVATOR_SMART_CURRENT_LIMIT = 40;

    // Telescoping Arm
    public static final int TELESCOPING_ARM_PERIOD = 20;
    public static final TrapezoidProfile.Constraints TELESCOPING_ARM_CONSTRAINTS
            = new TrapezoidProfile.Constraints(3, 3);
    public static final ElevatorFeedforward TELESCOPING_ARM_FEEDFORWARD = new ElevatorFeedforward(0, 0, 0, 0);
    public static final int TELESCOPING_ARM_P = 5;
    public static final int TELESCOPING_ARM_I = 5;
    public static final int TELESCOPING_ARM_D = 5;
    public static final int TELESCOPING_ARM_POSITION_MULTIPLIER = 5;
    public static final int TELESCOPING_ARM_NOMINAL_VOLTAGE = 9;
    public static final int TELESCOPING_ARM_SMART_CURRENT_LIMIT = 40;

    // Grabber
    public static final int GRABBER_PERIOD = 20;
    public static final ArmFeedforward GRABBER_FEEDFORWARD = new ArmFeedforward(0, 0, 0, 0);
    public static final TrapezoidProfile.Constraints GRABBER_CONSTRAINTS
            = new TrapezoidProfile.Constraints(3, 3);
    public static final int GRABBER_P = 5;
    public static final int GRABBER_I = 5;
    public static final int GRABBER_D = 5;
    public static final int GRABBER_POSITION_MULTIPLIER = 5;
    public static final int GRABBER_NOMINAL_VOLTAGE = 9;
    public static final int GRABBER_SMART_CURRENT_LIMIT = 40;
    public static final int PIVOT_SMART_CURRENT_LIMIT = 40;

    // Vision constants
    public static final int VISION_HANDLER_PERIOD = -1;

    public static final double FIELD_HEIGHT_METERS = 8.0137;
    public static final double FIELD_WIDTH_METERS = 16.54175;
}
