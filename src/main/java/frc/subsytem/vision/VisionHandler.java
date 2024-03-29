package frc.subsytem.vision;

import com.dacubeking.AutoBuilder.robot.sender.pathpreview.RobotPositionSender;
import com.dacubeking.AutoBuilder.robot.sender.pathpreview.RobotState;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent.Kind;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
import frc.robot.ScoringPositionManager;
import frc.robot.ScoringPositionManager.PositionType;
import frc.subsytem.AbstractSubsystem;
import frc.subsytem.MechanismStateManager.MechanismStates;
import frc.subsytem.robottracker.RobotTracker;
import frc.utility.LimelightHelpers;
import org.jetbrains.annotations.NotNull;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean;

import java.io.IOException;
import java.util.ArrayList;
import java.util.EnumSet;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import static frc.robot.Constants.*;
import static frc.utility.LimelightHelpers.getLimelightNTTableEntry;
import static frc.utility.OrangeUtility.fixCoords;
import static frc.utility.geometry.GeometryUtils.dist2;
import static org.joml.Math.tan;

/**
 * Gives pose estimates based on april tag vision
 */
public class VisionHandler extends AbstractSubsystem {

    public static final double NO_VISION_UPDATES_TIME_THRESHOLD = 0.15;
    public static final double REALSENSE_THROWOUT_THRESHOLD_METERS = 0.0;
    public static final double LIMELIGHT_LED_ON_DISTANCE_THRESHOLD_SQUARED = 2 * 2;
    public static final double USE_LIMELIGHT_THRESHOLD_METERS_SQUARED = 4;

    private final LoggedDashboardBoolean isRealsenseConnected
            = new LoggedDashboardBoolean("RealSense Connected", false);
    private final LoggedDashboardBoolean isLimelightLeftConnected
            = new LoggedDashboardBoolean("Limelight Left Connected", false);
    private final LoggedDashboardBoolean isLimelightRightConnected
            = new LoggedDashboardBoolean("Limelight Right Connected", false);
    private final LoggedDashboardBoolean forceDisableLimelightLEDs = new LoggedDashboardBoolean("Disable Limelight LEDs");

    {
        Logger.getInstance().registerDashboardInput(isRealsenseConnected);
        Logger.getInstance().registerDashboardInput(isLimelightLeftConnected);
        Logger.getInstance().registerDashboardInput(isLimelightRightConnected);
        Logger.getInstance().registerDashboardInput(forceDisableLimelightLEDs);
    }

    private final ExecutorService asyncVisionUpdateSaver = Executors.newSingleThreadExecutor();

    static final Vector<N3> POSITIVE_X = VecBuilder.fill(1, 0, 0);
    static final Vector<N3> POSITIVE_Y = VecBuilder.fill(0, 1, 0);
    static final Vector<N3> POSITIVE_Z = VecBuilder.fill(0, 0, 1);

    private static final Rotation3d POSITIVE_Y_90 = new Rotation3d(POSITIVE_Y, Math.toRadians(90.0));
    private static final Rotation3d POSITIVE_X_NEGATIVE_90 = new Rotation3d(POSITIVE_X, Math.toRadians(-90.0));
    private static final Rotation3d POSITIVE_Z_180 = new Rotation3d(POSITIVE_Z, Math.toRadians(180));

    private static String[] limelightNames = new String[]{"limelight-left", "limelight-right"};

    private static final @NotNull Pose3d[] fieldTags;

    private static final List<Integer> redTags = List.of(1, 2, 3, 4);

    static {
        try {
            @NotNull AprilTagFieldLayout fieldLayout;
            var wpiFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);

            var adjustedAprilTags = new ArrayList<AprilTag>();

            for (AprilTag tag : wpiFieldLayout.getTags()) {
                final var tagPos =
                        new Pose3d(
                                tag.pose
                                        .getTranslation().rotateBy(POSITIVE_Z_180)
                                        .plus(new Translation3d(FIELD_WIDTH_METERS, FIELD_HEIGHT_METERS / 2, 0)),
                                tag.pose.getRotation()
                                        .rotateBy(POSITIVE_Z_180)
                        );
                adjustedAprilTags.add(new AprilTag(tag.ID, tagPos));
            }

            fieldLayout = new AprilTagFieldLayout(adjustedAprilTags, FIELD_HEIGHT_METERS, FIELD_WIDTH_METERS);

            // Initialize Cache
            fieldTags = new Pose3d[fieldLayout.getTags().size()];

            for (AprilTag tag : fieldLayout.getTags()) {
                fieldTags[tag.ID - 1] = tag.pose;
            }

//            System.out.println("AprilTag Positions: ");
//            for (AprilTag tag : fieldLayout.getTags()) {
//                System.out.println(
//                        "ID: " + tag.ID + " Pos: " + tag.pose.getTranslation() + " angle: x:" + tag.pose.getRotation().getX()
//                                + ", y:" + tag.pose.getRotation().getY() + ", z:" + tag.pose.getRotation().getZ());
//            }
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
    }

    private final Pose3d cameraPose = new Pose3d(new Translation3d(Units.inchesToMeters(3.44 + 9),
            Units.inchesToMeters(-3.44),
            Units.inchesToMeters(52.425)),
            new Rotation3d(VecBuilder.fill(0, 1, 0), Math.toRadians(-28.6)));

    private final Pose3d negativeCameraPose = new Pose3d(
            cameraPose.getTranslation().unaryMinus(),
            cameraPose.getRotation().unaryMinus()
    );


    VisionInputs visionInputs = new VisionInputs();

    private boolean isVisionEnabled = true;

    public synchronized void setVisionEnabled(boolean visionEnabled) {
        isVisionEnabled = visionEnabled;
    }

    public VisionHandler() {
        super();
        // Network table init
        // Network tables
        @NotNull NetworkTableInstance networkTableInstance = NetworkTableInstance.getDefault();
        @NotNull NetworkTable visionTable = networkTableInstance.getTable("Vision");
        @NotNull var visionMiscTable = networkTableInstance.getTable("Vision Misc");
        @NotNull var configTable = networkTableInstance.getTable("Vision Config");

        // Send out connection flag to april tags processor
        visionMiscTable.getEntry("Connection Flag").setBoolean(true);

        configTable.getEntry("Exposure").setDouble(60);
        configTable.getEntry("Camera Type").setDouble(0);
        configTable.getEntry("X Resolution").setDouble(1280);
        configTable.getEntry("Y Resolution").setDouble(800);
        configTable.getEntry("Framerate").setDouble(30);
        configTable.getEntry("Threads").setDouble(4);
        configTable.getEntry("Do Stream").setBoolean(false);
        configTable.getEntry("Stream Port").setDouble(5810);
        configTable.getEntry("Stream Ip").setString("10.34.76.225");
        configTable.getEntry("Decision Margin").setDouble(10);
        configTable.getEntry("Encode Quality").setDouble(50);
        configTable.getEntry("Record Video").setBoolean(false);

        NetworkTableInstance.getDefault().addListener(visionMiscTable.getEntry("Vision Looptime").getTopic(),
                EnumSet.of(Kind.kValueRemote),
                (event) -> {
                    synchronized (this) {
                        visionInputs.lastVisionUpdateRealsense = Timer.getFPGATimestamp();
                    }
                });


        for (int i = 1; i <= 8; i++) {
            var table = visionTable.getEntry(String.valueOf(i));
            int finalI = i;
            NetworkTableInstance.getDefault().addListener(table.getTopic(),
                    EnumSet.of(Kind.kValueRemote),
                    (event) -> asyncVisionUpdateSaver.submit(() -> {
                        var visionUpdate = new VisionUpdate(event.valueData.value.getDoubleArray(), finalI);
                        synchronized (this) {
                            visionInputs.visionUpdates.add(visionUpdate);
                        }
                    }));
        }

        for (int i = 0; i < 2; i++) {
            var limelightName = limelightNames[i];

            int finalI = i;
            NetworkTableInstance.getDefault().addListener(
                    getLimelightNTTableEntry(limelightName, "botpose_wpired").getTopic(),
                    EnumSet.of(Kind.kValueRemote),
                    (event) -> asyncVisionUpdateSaver.submit(() -> {
                        // https://docs.limelightvision.io/en/latest/apriltags_in_3d.html#using-wpilib-s-pose-estimator
                        double[] botpose = event.valueData.value.getDoubleArray();
                        Pose3d llPose = LimelightHelpers.toPose3D(botpose);
                        if (dist2(llPose.getTranslation()) < 0.1) {
                            return;
                        }
                        Pose3d adjustedPose = new Pose3d(
                                llPose.getTranslation().plus(new Translation3d(0, -FIELD_HEIGHT_METERS / 2, 0)),
                                llPose.getRotation()
                        );
                        synchronized (this) {
                            visionInputs.limelightUpdates.add(new LimelightUpdate(
                                    adjustedPose,
                                    Timer.getFPGATimestamp() - (botpose[6] / 1000.0),
                                    finalI
                            ));
                        }
                    }));
        }

        NetworkTableInstance.getDefault().addListener(getLimelightNTTableEntry("limelight-left", "tl").getTopic(),
                EnumSet.of(Kind.kValueRemote),
                (event) -> asyncVisionUpdateSaver.submit(() -> {
                    synchronized (this) {
                        visionInputs.lastVisionUpdateLimelightLeft = Timer.getFPGATimestamp();
                    }
                }));

        NetworkTableInstance.getDefault().addListener(getLimelightNTTableEntry("limelight-right", "tl").getTopic(),
                EnumSet.of(Kind.kValueRemote),
                (event) -> asyncVisionUpdateSaver.submit(() -> {
                    synchronized (this) {
                        visionInputs.lastVisionUpdateLimelightRight = Timer.getFPGATimestamp();
                    }
                }));

        asyncVisionUpdateSaver.submit(() -> {System.out.println("Warming Up the vision update thread pool");});
    }

    private final MatBuilder<N4, N1> visionStdMatBuilder = new MatBuilder<>(Nat.N4(), Nat.N1());

    private void processNewTagPosition(VisionUpdate data) {
        final var expectedTagPosition = fieldTags[data.tagId - 1]; // We should never get an unknown tag

        final var tagTranslation = new Translation3d(data.posZ, -data.posX, -data.posY); //camera to tag
        var distanceToTag = tagTranslation.getNorm();
        //negative camera pose  -> camera to robot
        //camera pose
        final var tagTranslationRobotCentric = tagTranslation
                .rotateBy(negativeCameraPose.getRotation())
                .plus(cameraPose.getTranslation()); //vision robot to tag
        final var tagRotationRobotCentric = new Rotation3d(new Quaternion(data.rotW, data.rotZ, -data.rotX, -data.rotY))
                .rotateBy(negativeCameraPose.getRotation()); // vision tag to robot


        Rotation3d gyroAngle = Robot.getRobotTracker().getGyroAngleAtTime(data.timestamp); //vison_field_to_robot
        // hypothetically ?????

        var rotationFromTag = expectedTagPosition.getRotation()// field to tag
                .minus(tagRotationRobotCentric)
                .rotateBy(POSITIVE_Z_180); //vison_field_to_robot hypothetically

        var calculatedTranslationFromGyro =
                expectedTagPosition.getTranslation() //field to tag in field system
                        .plus(
                                tagTranslationRobotCentric//robot to tag in robot system
                                        // Rotate the translation by the gyro angle (to make it relative to the field)
                                        .rotateBy(gyroAngle)//robot to tag in field system
                                        // Make the vector from the tag to the robot (instead of the robot to the tag)
                                        .unaryMinus()// tag to robot in field system
                        ); //field to robot in field system


        // Use position we calculated from the gyro, but use the rotation we calculated from the tag
        // the position we calculated from the gyro is more accurate,
        // but still pass the rotation we calculated from the tag, so we can use it to compensate for the gyro drift
        var poseToFeedToRobotTracker = new Pose3d(
                calculatedTranslationFromGyro, // 3d pos on the field
                distanceToTag > 4 ? gyroAngle : rotationFromTag
        );


        var calculatedTranslationFromTagOrientation =
                expectedTagPosition.getTranslation()// field to tag in field system
                        .plus(
                                tagTranslationRobotCentric//robot to tag in robot system
                                        // Rotate the translation by the tag orientation (to make it relative to the field)
                                        .rotateBy(rotationFromTag)
                                        // Make the vector from the tag to the robot (instead of the robot to the tag)
                                        .unaryMinus() //tag to robot in field system
                        ); //field to robot in field system

        var visionOnlyPose = new Pose3d(
                calculatedTranslationFromTagOrientation, // 3d pos on the field
                rotationFromTag
        );


        Logger.getInstance().recordOutput("VisionHandler/TagPose/" + data.tagId, fixCoords(expectedTagPosition));
        Logger.getInstance().recordOutput("VisionHandler/VisionOnlyPose/" + data.tagId, fixCoords(visionOnlyPose));


        RobotPositionSender.addRobotPosition(
                new RobotState(poseToFeedToRobotTracker.toPose2d(), data.timestamp, "Fed Vision Pose Tag: " + data.tagId));
        RobotPositionSender.addRobotPosition(
                new RobotState(visionOnlyPose.toPose2d(), data.timestamp, "Vision Only Pose Tag: " + data.tagId));
        var defaultDevs = RobotTracker.REALSENSE_DEFAULT_VISION_DEVIATIONS;
        if (distanceToTag == 0) return;
        var distanceToTag2 = distanceToTag * distanceToTag;
        var devs = visionStdMatBuilder.fill(
                defaultDevs.get(0, 0) * distanceToTag2,
                defaultDevs.get(1, 0) * distanceToTag2,
                defaultDevs.get(2, 0) * distanceToTag2,
                Math.atan(tan(defaultDevs.get(3, 0)) * distanceToTag2 * distanceToTag));
        Robot.getRobotTracker().addVisionMeasurement(poseToFeedToRobotTracker, data.timestamp, devs);

        Logger.getInstance().recordOutput("VisionHandler/VisionOnlyPoseAngles/" + data.tagId + "/X",
                Math.toDegrees(tagRotationRobotCentric.getX()));
        Logger.getInstance().recordOutput("VisionHandler/VisionOnlyPoseAngles/" + data.tagId + "/Y",
                Math.toDegrees(tagRotationRobotCentric.getY()));
        Logger.getInstance().recordOutput("VisionHandler/VisionOnlyPoseAngles/" + data.tagId + "/Z",
                Math.toDegrees(tagRotationRobotCentric.getZ()));

        Logger.getInstance().recordOutput("VisionHandler/VisionOnlyPosePosition/" + data.tagId + "/X",
                tagTranslationRobotCentric.getX());
        Logger.getInstance().recordOutput("VisionHandler/VisionOnlyPosePosition/" + data.tagId + "/Y",
                tagTranslationRobotCentric.getY());
        Logger.getInstance().recordOutput("VisionHandler/VisionOnlyPosePosition/" + data.tagId + "/Z",
                tagTranslationRobotCentric.getZ());

        Logger.getInstance().recordOutput("VisionHandler/FedPoses/" + data.tagId,
                fixCoords(new Pose3d(poseToFeedToRobotTracker.getTranslation(), gyroAngle)));


        // Draw the tag on the field
//        var drawables = new Drawable[2];
//        int j = 0;
//        drawables[j++] = new Line((float) expectedTagPosition.getX(),
//                (float) (expectedTagPosition.getY() - 0.5),
//                (float) expectedTagPosition.getX(),
//                (float) (expectedTagPosition.getY() + 0.5), new Color8Bit(255, 255, 0));
//        drawables[j++] = new Line((float) expectedTagPosition.getX(),
//                (float) expectedTagPosition.getY(),
//                (float) (expectedTagPosition.getX() + cos(expectedTagPosition.getRotation().getZ()) * 0.1),
//                (float) (expectedTagPosition.getY()),
//                new Color8Bit(255, 255, 0));
//        Renderer.render(drawables);
    }


    private boolean recordingWanted = false;

    public void forceRecord(boolean record) {
        if (record != recordingWanted) {
            recordingWanted = record;
            NetworkTableInstance.getDefault().getTable("Vision Config").getEntry("Record Video").setBoolean(record);
        }
    }

    @Override
    public synchronized void update() {
        int llPipelineIndex;
        if (Robot.isRed()) {
            llPipelineIndex = 0;
        } else {
            llPipelineIndex = 1;
        }

        for (String limelightName : limelightNames) {
            LimelightHelpers.setPipelineIndex(limelightName, llPipelineIndex);
        }

        Logger.getInstance().processInputs("VisionHandler", visionInputs);

        isRealsenseConnected.set(
                Timer.getFPGATimestamp() - visionInputs.lastVisionUpdateRealsense < NO_VISION_UPDATES_TIME_THRESHOLD);
        isLimelightLeftConnected.set(
                Timer.getFPGATimestamp() - visionInputs.lastVisionUpdateLimelightLeft < NO_VISION_UPDATES_TIME_THRESHOLD);
        isLimelightRightConnected.set(
                Timer.getFPGATimestamp() - visionInputs.lastVisionUpdateLimelightRight < NO_VISION_UPDATES_TIME_THRESHOLD);


        // Find the distance to the closest tag
        var distanceToTag2 = Double.MAX_VALUE;
        for (var tags : fieldTags) {
            var dist2 = dist2(tags.getTranslation().minus(Robot.getRobotTracker().getLatestPose3d().getTranslation()));
            if (dist2 < distanceToTag2) {
                distanceToTag2 = dist2;
            }
        }

        Logger.getInstance().recordOutput("VisionHandler/Tags Updates", visionInputs.visionUpdates.size());

        // Process vision updates

        if (distanceToTag2 > REALSENSE_THROWOUT_THRESHOLD_METERS * REALSENSE_THROWOUT_THRESHOLD_METERS
                || ScoringPositionManager.getInstance().getSelectedPosition().positionType == PositionType.CUBE) {
            for (var visionUpdate : visionInputs.visionUpdates) {
                if (DriverStation.isAutonomous() && Robot.isRed() != redTags.contains(visionUpdate.tagId)) {
                    continue;
                }
                processNewTagPosition(visionUpdate);
            }
        }
        visionInputs.visionUpdates.clear();

        // Turn on the limelight LEDs if we are close to a tag
        if (distanceToTag2 < LIMELIGHT_LED_ON_DISTANCE_THRESHOLD_SQUARED && !forceDisableLimelightLEDs.get()) {
            for (String limelightName : limelightNames) {
                LimelightHelpers.setLEDMode_PipelineControl(limelightName);
            }
        } else {
            for (String limelightName : limelightNames) {
                LimelightHelpers.setLEDMode_ForceOff(limelightName);
            }
        }

        int limelightUpdatesSent = 0;
        if (distanceToTag2 < USE_LIMELIGHT_THRESHOLD_METERS_SQUARED
                || Robot.getMechanismStateManager().getLastState() != MechanismStates.STOWED) {
            for (var limelightUpdate : visionInputs.limelightUpdates) {
                var defaultDevs = RobotTracker.LIMELIGHT_DEFAULT_VISION_DEVIATIONS;

                var expectedPoseRotation = Robot.getRobotTracker().getGyroAngleAtTime(limelightUpdate.timestamp);
                var pose = limelightUpdate.pose3d();
                var poseRotation = pose.getRotation();

                // Check if the expected angle has a similar rotation to what we got from the limelight
                double maxAllowedLimelightAngleError = Robot.isOnAllianceSide() ? 5 : 20;
                var rotDiff = poseRotation.minus(expectedPoseRotation);
                if ((Math.abs(rotDiff.getX()) > Math.toRadians(maxAllowedLimelightAngleError)
                        || Math.abs(rotDiff.getY()) > Math.toRadians(maxAllowedLimelightAngleError)
                        || Math.abs(rotDiff.getZ()) > Math.toRadians(maxAllowedLimelightAngleError))) {
                    continue;
                }

                // Don't use the orientation from the limelight
                var robotTranslationFromGyro = pose.getTranslation()
                        .rotateBy(pose.getRotation().unaryMinus())
                        .rotateBy(expectedPoseRotation);

                // Throw out rotation from the limelight
                var poseToFeedToRobotTracker = new Pose3d(robotTranslationFromGyro, expectedPoseRotation);

                // Find the closest tag to this pose estimate
                var distanceToTagLimelight2 = Double.MAX_VALUE;
                for (var tags : fieldTags) {
                    var dist2 = dist2(tags.getTranslation().minus(poseToFeedToRobotTracker.getTranslation()));
                    if (dist2 < distanceToTagLimelight2) {
                        distanceToTagLimelight2 = dist2;
                    }
                }

                var devs = visionStdMatBuilder.fill(
                        defaultDevs.get(0, 0) * distanceToTagLimelight2,
                        defaultDevs.get(1, 0) * distanceToTagLimelight2,
                        defaultDevs.get(2, 0) * distanceToTagLimelight2,
                        Math.atan(tan(defaultDevs.get(3, 0)) * distanceToTagLimelight2 * distanceToTagLimelight2));
                Robot.getRobotTracker().addVisionMeasurement(pose, limelightUpdate.timestamp(), devs);
                limelightUpdatesSent++;
                Logger.getInstance().recordOutput("VisionManager/Limelight Pose " + limelightUpdate.limelightIndex,
                        fixCoords(pose));
            }
        }

        Logger.getInstance().recordOutput("VisionManager/Limelight Updates Sent", limelightUpdatesSent);

        visionInputs.limelightUpdates.clear();
    }

    record VisionUpdate(double posX, double posY, double posZ, double rotX, double rotY, double rotZ, double rotW,
                        double timestamp, int tagId) {
        /**
         * This treats the data as a double array in the following order: posX, posY, posZ, rotW, rotX, rotY, rotZ, latency
         *
         * @param data the data
         * @param id   the tagId of the tag
         */
        public VisionUpdate(double[] data, int id) {
            this(data[0], data[1], data[2], data[3], data[4], data[5], data[6],
                    (Logger.getInstance().getRealTimestamp() * SECONDS_PER_MICROSECOND) - (data[7] / 1000.0), id);
        }

        public double[] toArray() {
            return new double[]{posX, posY, posZ, rotW, rotX, rotY, rotZ, timestamp, tagId};
        }

        /**
         * This treats the data as a double array in the following order: posX, posY, posZ, rotW, rotX, rotY, rotZ, timestamp,
         * tagId
         *
         * @return a new VisionUpdate object
         */
        public static VisionUpdate fromArray(double[] array) {
            return new VisionUpdate(array[0], array[1], array[2], array[3], array[4], array[5], array[6], array[7],
                    (int) Math.round(array[8]));
        }
    }

    record LimelightUpdate(Pose3d pose3d, double timestamp, int limelightIndex) {}
}
