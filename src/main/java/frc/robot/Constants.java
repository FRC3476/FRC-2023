package frc.robot;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.utility.swerve.SwerveSetpointGenerator.KinematicLimit;
import org.jetbrains.annotations.NotNull;

import java.io.File;
import java.nio.file.Files;

public final class Constants {
    public static final double ANGULAR_VELOCITY_BALANCE_THRESHHOLD = 10;
    public static final double BALANCE_REVERSE_SPEED = 0.1;
    public static final String LOG_DIRECTORY = "/home/lvuser/logs";

    public static final double SECONDS_PER_MINUTE = 60;

    public static final boolean IS_PRACTICE = Files.exists(new File("/home/lvuser/practice").toPath());
    public static final boolean USE_CANCODERS = true;
    public static final boolean USE_RELATIVE_ENCODER_POSITION = true; // The relative position is equivalent to the absolute
    // position on the falcon 500 encoder
    public static final double SECONDS_PER_MICROSECOND = 1e-6;
    public static final long MIN_FREE_SPACE = IS_PRACTICE ?
            100000000 : // 100 MB
            1000000000; // 1 GB

    public static final double GYRO_IS_FLAT_FOR_PICKUP_THRESHOLD_DEGREES = 5;
    public static final double PRE_SCORE_TIME_S = 1.3;
    public static final double PRE_SCORE_TIME_2_S = 0.4;
    public static final double SCORE_TIME_S = 0.0;
    public static final double SCORE_POSITION_ERROR_SQUARED = 0.08 * 0.8; // 4 cm
    // Drive Constants
    public static final double SWERVE_MOTOR_POSITION_CONVERSION_FACTOR = 1 / 12.8;
    public static final int DEFAULT_PERIODS_PER_LOG = 0;
    private static final int NOMINAL_DT_MS = 20;

    public static final double BALANCE_P = .040;
    public static final double BALANCE_I = 0;
    public static final double BALANCE_D = 0;
    public static final int AUTO_BALANCE_VELOCITY_THRESHOLD_DEGREES_PER_SECOND = 8;

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

    public static final double SWERVE_INCHES_PER_ROTATION = 12.5 * 0.976 * 0.96488764044943820224719101123596;
    public static final double SWERVE_METER_PER_ROTATION = Units.inchesToMeters(SWERVE_INCHES_PER_ROTATION);
    public static final double SWERVE_DRIVE_P = 100;
    public static final double SWERVE_DRIVE_D = 0.00;
    public static final double SWERVE_DRIVE_I = 0.00;
    public static final double SWERVE_DRIVE_F = 0.00;
    public static final double SWERVE_DRIVE_INTEGRAL_ZONE = 0.00;
    public static final double AUTO_BALANCE_COMPLETE_THRESHOLD_DEGREES = 7;
    public static final double MAX_ERROR_PRINT_TIME = 0.5;
    public static final boolean PERIODIC_DRIVE_PRINT = false;
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
            //ka = 0.55
            new SimpleMotorFeedforward(0.281485, 2.3016, 0.45),
            new SimpleMotorFeedforward(0.281485, 2.3016, 0.45),
            new SimpleMotorFeedforward(0.281485, 2.3016, 0.45),
            new SimpleMotorFeedforward(0.281485, 2.3016, 0.45)};


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
    public static final SwerveModuleState[] HOLD_MODULE_STATES = new SwerveModuleState[]{
            new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
            new SwerveModuleState(0, Rotation2d.fromDegrees(45))
    };

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

    public static final double DRIVE_HIGH_SPEED_M = DRIVE_FEEDFORWARD[0].maxAchievableVelocity(12, 0);
    /**
     * Allowed Turn Error in degrees.
     */
    public static final double MAX_TURN_ERROR = 30;

    public static final int SWERVE_MOTOR_CURRENT_LIMIT = 20;
    public static final int SWERVE_DRIVE_MOTOR_CURRENT_LIMIT = 40;
    public static final int SWERVE_DRIVE_VOLTAGE_LIMIT_AUTO = 12;
    public static final int SWERVE_DRIVE_VOLTAGE_LIMIT_TELEOP = 15;


    public static final double SWERVE_DRIVE_MOTOR_REDUCTION = 1 / 5.9; // L3.1 gear ratio
    public static final double ALLOWED_XY_ERROR_RAMSETE = 0.04;
    public static final double PID_CONTROL_RANGE_AUTO_DRIVE_METERS = 0.1;
    public static final double AUTO_DRIVE_PATH_EARLY_END_SECONDS = 0.1;
    public static final double ALLOWED_AUTO_DRIVE_POSITION_ERROR_METERS = 0.02;
    public static final double ALLOWED_AUTO_DRIVE_ANGLE_ERROR_RADIANS = Math.toRadians(1);
    public static final double MAX_VELOCITY_END_PATH = 0.02;

    public static final double MAX_ACCEL_DOUBLE_SUBSTATION_PICKUP = 3.2;
    public static final double MAX_ACCEL_AUTO_DRIVE = 2.9;


    // TurnPID

    public static final double DEFAULT_TURN_P = 4.5;
    public static final double DEFAULT_TURN_I = 0;
    public static final double DEFAULT_TURN_D = 0.3;

    public static final double DEFAULT_AUTO_P = 5;
    public static final double DEFAULT_AUTO_I = 0;
    public static final double DEFAULT_AUTO_D = 0;
    public static final double TURN_SPEED_LIMIT_WHILE_AIMING = 4.0;

    public static final double EXPECTED_TELEOP_DRIVE_DT = 0.02;

    public static final double NOMINAL_DT = NOMINAL_DT_MS / 1000.0;

    public static final int MAX_TELEOP_TURN_SPEED = 10;
    /**
     * Acceleration due to gravity in meters per second squared. (9.80665 m/s^2)
     */
    public static final double GRAVITY = 9.80665;
    public static final int PIGEON_CAN_ID = 30;
    public static final double COAST_AFTER_DISABLE_TIME = 7;
    public static final double FIELD_HEIGHT_METERS = 8.0137;
    public static final double FIELD_WIDTH_METERS = 16.54175;

    public static final double ALLOWED_SWERVE_ANGLE_ERROR = 0;
    public static final TrapezoidProfile.Constraints ELEVATOR_CONSTRAINTS
            = new TrapezoidProfile.Constraints(3, 10);
    public static final ElevatorFeedforward ELEVATOR_FEEDFORWARD = new ElevatorFeedforward(0, 0.58, 0, 0);

    // 1 Meter divided by the circumference of the sprocket in meters
    public static final double ELEVATOR_ROTATIONS_PER_METER = 1 / ((2 * Math.PI) / 39.37);
    public static final double ELEVATOR_REDUCTION = 1.0 / 5.0;
    public static final double ELEVATOR_P = 6;
    public static final double ELEVATOR_I = .001;
    public static final int ELEVATOR_D = 20;
    public static final double ELEVATOR_IZONE = .05;

    public static final double ELEVATOR_LOWER_LIMIT = .01;
    public static final double ELEVATOR_UPPER_LIMIT = 1.20;
    public static final double ELEVATOR_NOMINAL_VOLTAGE = 9;
    public static final int ELEVATOR_SMART_CURRENT_LIMIT = 40;

    public static final double ELEVATOR_TILT_RADIANS = Math.toRadians(64.24203436);
    public static final double ELEVATOR_STALLING_CURRENT = 35;
    public static final double ELEVATOR_MIN_HOME_TIME = 0.2;
    public static final double ELEVATOR_HOME_VOLTAGE = -1.5;

    public static final int ELEVATOR_MAIN_CAN_ID = 40;
    public static final int ELEVATOR_FOLLOWER_CAN_ID = 42;
    public static final TrapezoidProfile.Constraints TELESCOPING_ARM_CONSTRAINTS
            = new TrapezoidProfile.Constraints(3, 3);
    public static final ElevatorFeedforward TELESCOPING_ARM_FEEDFORWARD = new ElevatorFeedforward(0, 0, 0, 0);
    public static final double TELESCOPING_ARM_P = 10;
    public static final double TELESCOPING_ARM_I = 0.0;
    public static final double TELESCOPING_ARM_D = 60.0;
    public static final double TELESCOPING_ARM_MIN_HOME_TIME = 0.2;
    public static final double TELESCOPING_ARM_STALLING_CURRENT = 35;
    public static final double TELESCOPING_ARM_ROTATIONS_PER_METER = 96.664 / (Math.PI);
    public static final double TELESCOPING_ARM_NOMINAL_VOLTAGE = 9;
    public static final int TELESCOPING_ARM_SMART_CURRENT_LIMIT = 40;
    public static final double TELESCOPING_ARM_HOME_VOLTAGE = -1.5;
    public static final int TELESCOPING_ARM_CAN_ID = 60;
    public static final double TELESCOPING_ARM_ALLOWED_ERROR = 0.0000;
    public static final ArmFeedforward GRABBER_FEEDFORWARD = new ArmFeedforward(0.32, 0.34, 0, 0);
    public static final TrapezoidProfile.Constraints GRABBER_PIVOT_CONSTRAINTS
            = new TrapezoidProfile.Constraints(10, 10);
    public static final double PIVOT_P = 0.1;
    public static final double PIVOT_I = 0.00;
    public static final double PIVOT_D = 2;
    public static final double PIVOT_ROTATIONS_PER_DEGREE = 1 / 5.4;
    public static final double PIVOT_IZONE = 10;
    public static final double GRABBER_NOMINAL_VOLTAGE = 9;
    public static final int GRABBER_SMART_CURRENT_LIMIT = 35;
    public static final int GRABBED_CURRENT_THRESHOLD = GRABBER_SMART_CURRENT_LIMIT - 10;
    public static final int PIVOT_SMART_CURRENT_LIMIT = 40;
    public static final int GRABBER_ROLLER_SMART_CURRENT_LIMIT = 30;
    public static final double DEGREES_PER_ROTATION = 360;
    public static final int GRABBER_PIVOT_CAN_ID = 50;
    public static final int GRABBER_CAN_ID = 51;
    public static final int GRABBER_ROLLER_MAIN_CAN_ID = 52;
    public static final int GRABBER_ROLLER_FOLLOWER_CAN_ID = 53;

    public static final double GRABBER_ROLLER_VOLTAGE = -6;
    public static final double GRABBER_ROLLER_IDLE = -0;
    public static final double GRABBER_CLOSE_THRESHOLD_DEGREES = 48;
    public static final double GRABBER_OPEN_THRESHOLD_DEGREES = 55;
    public static final boolean USE_GRABBER_ENCODER = !IS_PRACTICE;
    public static final boolean GRABBER_WHEELS_USED = false;

    public static final double GRABBER_LENGTH = .308;

    public static final boolean IS_AUTO_GRAB_ENABLED = true;
    public static final boolean USE_PIVOT_ABSOLUTE_ENCODER = !IS_PRACTICE;

    public enum KinematicLimits {
        /**
         * Normal acceleration limit while driving. This ensures that the driver can't tip the robot.
         */
        NORMAL_DRIVING(new KinematicLimit(7, 5000, Math.PI * 2 * 2));
        public final KinematicLimit kinematicLimit;

        KinematicLimits(KinematicLimit kinematicLimit) {
            this.kinematicLimit = kinematicLimit;
        }
    }


    // Realtime path generation
    public static final double START_POS_PREDICT_AHEAD = 0.05;
    /**
     * The length of the end vector of the path. This has the effect of requiring the robot to move in a straight line before the
     * end of the path.
     */
    public static final double END_VECTOR_LEN = 0.5;

    /**
     * The length of the end vector of the path. This has the effect of requiring the robot to move in a straight line before the
     * end of the path. (Longer to require more straightening out for the double substation pickups)
     */
    public static final double LONGER_END_VECTOR_LEN = 2;

    public static final double VELOCITY_VECTOR_LEN_SCALE = 0.3;
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

    public static final float ROBOT_WIDTH = 0.84f;
    public static final float HALF_ROBOT_WIDTH = ROBOT_WIDTH / 2;
    public static final float ROBOT_LENGTH = 0.84f;
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

    public static final double LOWER_PICKUP_POSITION_Y = -3.56 + Units.inchesToMeters(2);
    public static final double UPPER_PICKUP_POSITION_Y = -2.05892 + Units.inchesToMeters(2);
    public static final double PICKUP_POSITION_X_OFFSET_FROM_WALL = FIELD_WIDTH_METERS - 15.2 + Units.inchesToMeters(-1.5);

    public static final double SCORING_POSITION_OFFSET_CONE_FROM_WALL = 0.05 + Units.inchesToMeters(1);
    public static final double SCORING_POSITION_OFFSET_CUBE_FROM_WALL = 0.05 + Units.inchesToMeters(3.5);

    public static final double SINGLE_STATION_RED_X = 14.39;
    public static final double SINGLE_STATION_BLUE_X = FIELD_WALL_RIGHT_X - SINGLE_STATION_RED_X;
    public static final double SINGLE_STATION_Y = -3.21;
    public static final Rotation2d SINGLE_STATION_ANGLE = Rotation2d.fromDegrees(-90);

    public static final double SINGLE_SUBSTATION_PICKUP_ANGLE_CUTOFF_DEGREES = -45;


    // Constraints
    public static final double MAX_WRIST_ANGLE = 126;
    public static final double MIN_WRIST_ANGLE = -90;
    public static final double BASE_MIN_X = Units.inchesToMeters(1);
    public static final double BASE_MAX_X = .43;
    public static final double MIN_Y = -0.03;
    public static final double MAX_Y = 1.05 + Units.inchesToMeters(2.1);


    //RobotTracker

    public static final double MISMATCH_LOOK_BACK_TIME = 0.15;

    /**
     * The maximum velocity error at which we consider the velocity to be matched.
     */
    public static final double MISMATCH_VELOCITY_ERROR_THRESHOLD = Double.MAX_VALUE;

    /**
     * The maximum accumulated error before we consider the velocity to be mismatched.
     */
    public static final double MISMATCH_ACCUMULATED_VELOCITY_ERROR_THRESHOLD = Double.MAX_VALUE;

    public static final double MAX_VELOCITY_MISMATCH_TIME = 5;
    public static final double ACCUMULATED_ERROR_DECAY = 0.95;


    // Controller Settings
    /**
     * Max speed of the wrist in degrees/s
     */
    public static final double ARCADE_WRIST_ANGLE_SPEED = 360;
    public static final double ARCADE_MODE_TRANSLATION_SPEED = 1;

    //Led Constants
    public static final int LED_LENGTH = 5;
}
