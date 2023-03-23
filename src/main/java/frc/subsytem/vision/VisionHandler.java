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
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
import frc.subsytem.AbstractSubsystem;
import frc.subsytem.robottracker.RobotTracker;
import frc.utility.LimelightHelpers;
import org.jetbrains.annotations.NotNull;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean;

import java.io.IOException;
import java.util.ArrayList;
import java.util.EnumSet;

import static frc.robot.Constants.*;
import static frc.utility.OrangeUtility.fixCoords;
import static frc.utility.geometry.GeometryUtils.dist2;
import static org.joml.Math.tan;

/**
 * Gives pose estimates based on april tag vision
 */
public class VisionHandler extends AbstractSubsystem {

    public static final double NO_VISION_UPDATES_TIME_THRESHOLD = 0.1;
    private final LoggedDashboardBoolean isVisionConnected = new LoggedDashboardBoolean("Vision Connected", false);

    {
        Logger.getInstance().registerDashboardInput(isVisionConnected);
    }

    static final Vector<N3> POSITIVE_X = VecBuilder.fill(1, 0, 0);
    static final Vector<N3> POSITIVE_Y = VecBuilder.fill(0, 1, 0);
    static final Vector<N3> POSITIVE_Z = VecBuilder.fill(0, 0, 1);

    static final Rotation3d POSITIVE_Y_90 = new Rotation3d(POSITIVE_Y, Math.toRadians(90.0));
    static final Rotation3d POSITIVE_X_NEGATIVE_90 = new Rotation3d(POSITIVE_X, Math.toRadians(-90.0));
    static final Rotation3d POSITIVE_Z_180 = new Rotation3d(POSITIVE_Z, Math.toRadians(180));
    private static final @NotNull Pose3d[] fieldTags;

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
                fieldTags[tag.ID] = tag.pose;
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

    private final Pose3d cameraPose = new Pose3d(new Translation3d(Units.inchesToMeters(3.44 + 11.4375),
            Units.inchesToMeters(-3.44 - 0.44),
            Units.inchesToMeters(52.425)),
            new Rotation3d(VecBuilder.fill(0, 1, 0), Math.toRadians(-31)));

    private final Pose3d negativeCameraPose = new Pose3d(
            cameraPose.getTranslation().unaryMinus(),
            cameraPose.getRotation().unaryMinus()
    );


    VisionInputs visionInputs = new VisionInputs();

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

        configTable.getEntry("Exposure").setDouble(50);
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
                        visionInputs.lastVisionUpdate = Timer.getFPGATimestamp();
                    }
                });

        for (int i = 1; i <= 8; i++) {
            var table = visionTable.getEntry(String.valueOf(i));
            int finalI = i;
            NetworkTableInstance.getDefault().addListener(table.getTopic(),
                    EnumSet.of(Kind.kValueRemote),
                    (event) -> {
                        var visionUpdate = new VisionUpdate(event.valueData.value.getDoubleArray(), finalI);
                        synchronized (this) {
                            visionInputs.visionUpdates.add(visionUpdate);
                        }
                    });
        }

        NetworkTableInstance.getDefault().addListener(
                // table name of null gets the default table
                LimelightHelpers.getLimelightNTTableEntry(null, "botpose_wpired").getTopic(),
                EnumSet.of(Kind.kValueRemote),
                (event) -> {
                    synchronized (this) {
                        // https://docs.limelightvision.io/en/latest/apriltags_in_3d.html#using-wpilib-s-pose-estimator
                        double[] botpose = event.valueData.value.getDoubleArray();
                        Pose3d llPose = LimelightHelpers.toPose3D(botpose);
                        Pose3d adjustedPose = new Pose3d(
                                llPose.getTranslation().plus(new Translation3d(0, -FIELD_HEIGHT_METERS / 2, 0)),
                                llPose.getRotation()
                        );
                        visionInputs.limelightUpdates.add(new LimelightUpdate(
                                adjustedPose,
                                Timer.getFPGATimestamp() - (botpose[6] / 1000.0)
                        ));
                    }
                });
    }

    private final MatBuilder<N4, N1> visionStdMatBuilder = new MatBuilder<>(Nat.N4(), Nat.N1());

    private void processNewTagPosition(VisionUpdate data) {
        final var expectedTagPosition = fieldTags[data.tagId]; // We should never get an
        // unknown tag

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
        Logger.getInstance().processInputs("VisionHandler", visionInputs);

        isVisionConnected.set(Timer.getFPGATimestamp() - visionInputs.lastVisionUpdate < NO_VISION_UPDATES_TIME_THRESHOLD);

        Logger.getInstance().recordOutput("VisionHandler/Tags Updates", visionInputs.visionUpdates.size());
        // Process vision updates
        for (var visionUpdate : visionInputs.visionUpdates) {
            processNewTagPosition(visionUpdate);
        }
        visionInputs.visionUpdates.clear();

        for (var limelightUpdate : visionInputs.limelightUpdates) {
            var defaultDevs = RobotTracker.REALSENSE_DEFAULT_VISION_DEVIATIONS;
            var distanceToTag2 = Double.MAX_VALUE;

            var pose = limelightUpdate.pose3d();

            for (var tags : fieldTags) {
                var dist2 = dist2(tags.getTranslation().minus(pose.getTranslation()));
                if (dist2 < distanceToTag2) {
                    distanceToTag2 = dist2;
                }
            }

            var devs = visionStdMatBuilder.fill(
                    defaultDevs.get(0, 0) * distanceToTag2,
                    defaultDevs.get(1, 0) * distanceToTag2,
                    defaultDevs.get(2, 0) * distanceToTag2,
                    Math.atan(tan(defaultDevs.get(3, 0)) * distanceToTag2 * distanceToTag2));

            Robot.getRobotTracker().addVisionMeasurement(pose, limelightUpdate.timestamp(), devs);
        }

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

    record LimelightUpdate(Pose3d pose3d, double timestamp) {}
}
