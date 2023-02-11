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

    public static final double SECONDS_PER_MINUTE = 60;

    public static final boolean IS_PRACTICE = Files.exists(new File("/home/lvuser/practice").toPath());
    public static final boolean USE_CANCODERS = false;

    // 2048 sensor units per revolution
    public static final double FALCON_ENCODER_TICKS_PER_ROTATIONS = 2048;
    public static final double FALCON_ENCODER_TICKS_PER_100_MS_TO_RPM = 600 / 2048.0d;
    public static final double SWERVE_MOTOR_POSITION_CONVERSION_FACTOR = 1 / 12.8;

    // Logging Period
    /**
     * Every subsystem will log one for every 20 update periods
     */
    public static final int DEFAULT_PERIODS_PER_LOG = 20;

    private static final int NOMINAL_DT_MS = 20;

    //Drive Constants
    public static final double AUTO_BALANCING_VELOCITY = 0.5;
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
    public static final double SWERVE_DRIVE_P = .01;
    public static final double SWERVE_DRIVE_D = 0.00;
    public static final double SWERVE_DRIVE_I = 0.00;
    public static final double SWERVE_DRIVE_F = 0.00;
    public static final double SWERVE_DRIVE_INTEGRAL_ZONE = 0.00;
    public static final double AUTO_BALANCE_COMPLETE_THRESHOLD = 9;
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
            new SimpleMotorFeedforward(0.17763, 2.7731, 0.0),
            new SimpleMotorFeedforward(0.17763, 2.7731, 0.0),
            new SimpleMotorFeedforward(0.17763, 2.7731, 0.0),
            new SimpleMotorFeedforward(0.17763, 2.7731, 0.0)};


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
    public static final @NotNull Translation2d SWERVE_LEFT_FRONT_LOCATION = new Translation2d(0.26352, 0.26352);
    public static final @NotNull Translation2d SWERVE_LEFT_BACK_LOCATION = new Translation2d(-0.26352, 0.26352);
    public static final @NotNull Translation2d SWERVE_RIGHT_FRONT_LOCATION = new Translation2d(0.26352, -0.26352);
    public static final @NotNull Translation2d SWERVE_RIGHT_BACK_LOCATION = new Translation2d(-0.26352, -0.26352);
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

    public static final double SWERVE_DRIVE_MOTOR_REDUCTION = 1 / 6.75; // L2 gear ratio
    // TurnPID

    public static final double DEFAULT_TURN_P = 10.0;
    public static final double DEFAULT_TURN_I = 0;
    public static final double DEFAULT_TURN_D = 0.4;
    public static final double TURN_SPEED_LIMIT_WHILE_AIMING = 4.0;

    public static final double EXPECTED_TELEOP_DRIVE_DT = 0.02;

    public static final double NOMINAL_DT = NOMINAL_DT_MS / 1000.0;

    public static final int MAX_TELEOP_TURN_SPEED = 7;
    /**
     * Acceleration due to gravity in meters per second squared. (9.80665 m/s^2)
     */
    public static final double GRAVITY = 9.80665;
    public static final int PIGEON_CAN_ID = 30;
    public static final double COAST_AFTER_DISABLE_TIME = 0.5;
    public static final double FIELD_HEIGHT_METERS = 8.0137;
    public static final double FIELD_WIDTH_METERS = 16.54175;

    public static final double ALLOWED_SWERVE_ANGLE_ERROR = 2;
    public static final TrapezoidProfile.Constraints ELEVATOR_CONSTRAINTS
            = new TrapezoidProfile.Constraints(3, 3);
    public static final ElevatorFeedforward ELEVATOR_FEEDFORWARD = new ElevatorFeedforward(0, 0, 0, 0);
    public static final double ELEVATOR_P = 1.3;
    public static final double ELEVATOR_I = .0128;
    public static final int ELEVATOR_D = 0;
    public static final double ELEVATOR_IZONE = .05;
    // 1 Meter divided by the circumference of the sprocket in meters
    public static final double ELEVATOR_ROTATIONS_PER_METER = 1 / ((2 * Math.PI) / 39.37);
    public static final double ELEVATOR_REDUCTION = 1.0 / 5.0;

    public static final double ELEVATOR_LOWER_LIMIT = .01;
    public static final double ELEVATOR_UPPER_LIMIT = 1.20;
    public static final double ELEVATOR_NOMINAL_VOLTAGE = 9;
    public static final int ELEVATOR_SMART_CURRENT_LIMIT = 20;

    // TODO: Figure out how much the elevator is angled at
    public static final double ELEVATOR_TILT_RADIANS = Math.toRadians(64.24203436);
    public static final double ELEVATOR_STALLING_CURRENT = 12;
    public static final double ELEVATOR_MIN_HOME_TIME = 0.5;
    public static final double MOTOR_SPEED_DECREASING_RATE = -0.1;

    public static final int ELEVATOR_MAIN_CAN_ID = 40;
    public static final int ELEVATOR_FOLLOWER_CAN_ID = 42;
    public static final TrapezoidProfile.Constraints TELESCOPING_ARM_CONSTRAINTS
            = new TrapezoidProfile.Constraints(3, 3);
    public static final ElevatorFeedforward TELESCOPING_ARM_FEEDFORWARD = new ElevatorFeedforward(0, 0, 0, 0);
    public static final double TELESCOPING_ARM_P = 0.1;
    public static final double TELESCOPING_ARM_I = 0.0;
    public static final double TELESCOPING_ARM_D = 0.0;
    public static final double TELESCOPING_ARM_ROTATIONS_PER_METER = 66.66646561;
    public static final double TELESCOPING_ARM_NOMINAL_VOLTAGE = 9;
    public static final int TELESCOPING_ARM_SMART_CURRENT_LIMIT = 15;
    public static final int TELESCOPING_ARM_CAN_ID = 60;
    public static final ArmFeedforward GRABBER_FEEDFORWARD = new ArmFeedforward(0, 0, 0, 0);
    public static final TrapezoidProfile.Constraints GRABBER_PIVOT_CONSTRAINTS
            = new TrapezoidProfile.Constraints(3, 3);
    public static final double PIVOT_P = 0.1;
    public static final double PIVOT_I = 0.05;
    public static final double PIVOT_D = 0;
    public static final double PIVOT_ROTATIONS_PER_DEGREE = 0.185185185185185;
    public static final double PIVOT_IZONE = 10;
    public static final double GRABBER_NOMINAL_VOLTAGE = 9;
    public static final int GRABBER_SMART_CURRENT_LIMIT = 5;
    public static final int PIVOT_SMART_CURRENT_LIMIT = 15;
    public static final int GRABBER_ROLLER_SMART_CURRENT_LIMIT = 5;

    public static final int GRABBER_PIVOT_CAN_ID = 50;
    public static final int GRABBER_CAN_ID = 51;
    public static final int GRABBER_ROLLER_MAIN_CAN_ID = 52;
    public static final int GRABBER_ROLLER_FOLLOWER_CAN_ID = 53;

    public static final double GRABBER_ROLLER_VOLTAGE = -6;
    public static final double GRABBER_ROLLER_IDLE = -1;


    public enum KinematicLimits {
        /**
         * Normal acceleration limit while driving. This ensures that the driver can't tip the robot.
         */
        NORMAL_DRIVING(new KinematicLimit(4, 5000, Math.PI * 2 * 10));
        public final KinematicLimit kinematicLimit;

        KinematicLimits(KinematicLimit kinematicLimit) {
            this.kinematicLimit = kinematicLimit;
        }
    }


    // Realtime path generation
    public static final double START_POS_PREDICT_AHEAD = 0.1;
    public static final double END_VECTOR_LEN = 0.5;
    public static final int VELOCITY_VECTOR_LEN_SCALE = 1;
    public static final double MAX_VELOCITY_ERROR_NEW_PATH = 0.05;

    public static final float GRIDS_RED_X = 1.373f;
    public static final float GRIDS_BLUE_X = 15.168f;
    public static final float GRIDS_START_Y = -1.5f;

    public static final float COMMUNITY_BORDER_LENGTH = 1.5f;
    public static final float COMMUNITY_BORDER_HEIGHT = 0.025f;

    public static final float FIELD_WALL_RIGHT_X = (float) Constants.FIELD_WIDTH_METERS;
    public static final float FIELD_WALL_LEFT_X = 0;
    public static final float FIELD_WALL_TOP_Y = 4.012f;
    public static final float FIELD_WALL_BOTTOM_Y = -FIELD_WALL_TOP_Y;

    public static final float CHARGING_STATION_WIDTH =
            (float) Units.inchesToMeters(76.125);
    public static final float CHARGING_STATION_HEIGHT =
            (float) Units.inchesToMeters(97.25);

    public static final float CHARGING_STATION_RED_LOWER_LEFT_X = 2.911f;
    public static final float CHARGING_STATION_BLUE_LOWER_LEFT_X = 11.689f;
    public static final float CHARGING_STATION_LOWER_LEFT_Y = -0.047f;

    public static final float ROBOT_WIDTH = 0.8128f;
    public static final float HALF_ROBOT_WIDTH = ROBOT_WIDTH / 2;
    public static final float ROBOT_LENGTH = 0.8128f;
    public static final float HALF_ROBOT_LENGTH = ROBOT_LENGTH / 2;


    /**
     * How far away from the edge of the grids do we want to create a line for the intersection test to choose the best scoring
     * location?
     */
    public static final double INTERSECTION_TEST_LINE_X_OFFSET = 0.5;


    public static final Rotation2d SCORING_ANGLE_RED = Rotation2d.fromDegrees(-180);
    public static final Rotation2d SCORING_ANGLE_BLUE = Rotation2d.fromDegrees(0);

    public static final Rotation2d PICKUP_ANGLE_RED = Rotation2d.fromDegrees(0);
    public static final Rotation2d PICKUP_ANGLE_BLUE = Rotation2d.fromDegrees(180);

    public static final double PICKUP_POSITION_Y = -3.4199;
    public static final double PICKUP_POSITION_X_OFFSET_FROM_WALL = 0.7635;
}
