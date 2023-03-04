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
import frc.robot.Robot;
import frc.subsytem.AbstractSubsystem;
import frc.subsytem.robottracker.RobotTracker;
import org.jetbrains.annotations.NotNull;
import org.littletonrobotics.junction.Logger;

import java.io.IOException;
import java.util.ArrayList;
import java.util.EnumSet;
import java.util.HashMap;

import static frc.robot.Constants.*;
import static org.joml.Math.tan;

/**
 * Gives pose estimates based on april tag vision
 */
public class VisionHandler extends AbstractSubsystem {

    static final Vector<N3> POSITIVE_X = VecBuilder.fill(1, 0, 0);
    static final Vector<N3> POSITIVE_Y = VecBuilder.fill(0, 1, 0);
    static final Vector<N3> POSITIVE_Z = VecBuilder.fill(0, 0, 1);

    static final Rotation3d POSITIVE_Y_90 = new Rotation3d(POSITIVE_Y, Math.toRadians(90.0));
    static final Rotation3d POSITIVE_X_NEGATIVE_90 = new Rotation3d(POSITIVE_X, Math.toRadians(-90.0));
    static final Rotation3d POSITIVE_Z_180 = new Rotation3d(POSITIVE_Z, Math.toRadians(180));
    private static final @NotNull HashMap<Integer, Pose3d> fieldTagCache;

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
            fieldTagCache = new HashMap<>();

            for (AprilTag tag : fieldLayout.getTags()) {
                fieldTagCache.put(tag.ID, tag.pose);
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
            new Rotation3d(VecBuilder.fill(0, 1, 0), Math.toRadians(-28)));

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

        configTable.getEntry("Exposure").setDouble(100);
        configTable.getEntry("Camera Type").setDouble(0);
        configTable.getEntry("X Resolution").setDouble(1280);
        configTable.getEntry("Y Resolution").setDouble(800);
        configTable.getEntry("Framerate").setDouble(30);
        configTable.getEntry("Threads").setDouble(4);
        configTable.getEntry("Do Stream").setBoolean(false);
        configTable.getEntry("Stream Port").setDouble(5810);
        configTable.getEntry("Stream Ip").setString("10.34.76.225");
        configTable.getEntry("Decision Margin").setDouble(15);
        configTable.getEntry("Encode Quality").setDouble(90);

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
    }

    private final MatBuilder<N4, N1> visionStdMatBuilder = new MatBuilder<>(Nat.N4(), Nat.N1());

    private void processNewTagPosition(VisionUpdate data) {
        final var expectedTagPosition = fieldTagCache.get(data.tagId); // We should never get an
        // unknown tag

        final var tagTranslation = new Translation3d(data.posZ, -data.posX, -data.posY);
        var distanceToTag = tagTranslation.getNorm();

        final var tagTranslationRobotCentric = tagTranslation
                .rotateBy(negativeCameraPose.getRotation())
                .plus(cameraPose.getTranslation());
        final var tagRotationRobotCentric = new Rotation3d(new Quaternion(data.rotW, data.rotZ, -data.rotX, -data.rotY))
                .rotateBy(negativeCameraPose.getRotation());


        Rotation3d gyroAngle = Robot.getRobotTracker().getGyroAngleAtTime(data.timestamp);


        var calculatedTranslationFromGyro =
                expectedTagPosition.getTranslation()
                        .plus(
                                tagTranslationRobotCentric
                                        // Rotate the translation by the gyro angle (to make it relative to the field)
                                        .rotateBy(gyroAngle)
                                        // Make the vector from the tag to the robot (instead of the robot to the tag)
                                        .unaryMinus()
                        );

        var rotationFromTag = expectedTagPosition.getRotation()
                .plus(POSITIVE_Z_180)
                .minus(tagRotationRobotCentric);


        var calculatedTranslationFromTagOrientation =
                expectedTagPosition.getTranslation()
                        .plus(
                                tagTranslationRobotCentric
                                        // Rotate the translation by the tag orientation (to make it relative to the field)
                                        .rotateBy(rotationFromTag)
                                        // Make the vector from the tag to the robot (instead of the robot to the tag)
                                        .unaryMinus()
                        );


        // Use position we calculated from the gyro, but use the rotation we calculated from the tag
        // the position we calculated from the gyro is more accurate,
        // but still pass the rotation we calculated from the tag, so we can use it to compensate for the gyro drift
        var poseToFeedToRobotTracker = new Pose3d(
                calculatedTranslationFromGyro, // 3d pos on the field
                distanceToTag > 4 ? gyroAngle : rotationFromTag
        );

        var visionOnlyPose = new Pose3d(
                calculatedTranslationFromTagOrientation, // 3d pos on the field
                rotationFromTag
        );

        RobotPositionSender.addRobotPosition(
                new RobotState(poseToFeedToRobotTracker.toPose2d(), data.timestamp, "Fed Vision Pose Tag: " + data.tagId));
        RobotPositionSender.addRobotPosition(
                new RobotState(visionOnlyPose.toPose2d(), data.timestamp, "Vision Only Pose Tag: " + data.tagId));
        var defaultDevs = RobotTracker.DEFAULT_VISION_DEVIATIONS;
        if (distanceToTag == 0) return;
        var distanceToTag3 = distanceToTag * distanceToTag;
        var devs = visionStdMatBuilder.fill(
                defaultDevs.get(0, 0) * distanceToTag3,
                defaultDevs.get(1, 0) * distanceToTag3,
                defaultDevs.get(2, 0) * distanceToTag3,
                Math.atan(tan(defaultDevs.get(3, 0)) * distanceToTag3 * distanceToTag));
        Robot.getRobotTracker().addVisionMeasurement(poseToFeedToRobotTracker, data.timestamp, devs);

        Logger.getInstance().recordOutput("VisionHandler/VisionOnlyPose/" + data.tagId, visionOnlyPose);
        Logger.getInstance().recordOutput("VisionHandler/VisionOnlyPoseAngles/" + data.tagId + "/X",
                Math.toDegrees(tagRotationRobotCentric.getX()));
        Logger.getInstance().recordOutput("VisionHandler/VisionOnlyPoseAngles/" + data.tagId + "/Y",
                Math.toDegrees(tagRotationRobotCentric.getY()));
        Logger.getInstance().recordOutput("VisionHandler/VisionOnlyPoseAngles/" + data.tagId + "/Z",
                Math.toDegrees(tagRotationRobotCentric.getZ()));

        Logger.getInstance().recordOutput("VisionHandler/VisionOnlyPose/" + data.tagId, visionOnlyPose);
        Logger.getInstance().recordOutput("VisionHandler/VisionOnlyPosePosition/" + data.tagId + "/X",
                tagTranslationRobotCentric.getX());
        Logger.getInstance().recordOutput("VisionHandler/VisionOnlyPosePosition/" + data.tagId + "/Y",
                tagTranslationRobotCentric.getY());
        Logger.getInstance().recordOutput("VisionHandler/VisionOnlyPosePosition/" + data.tagId + "/Z",
                tagTranslationRobotCentric.getZ());
        Logger.getInstance().recordOutput("VisionHandler/FedPoses/" + data.tagId, poseToFeedToRobotTracker);


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

    @Override
    public synchronized void update() {
        Logger.getInstance().processInputs("VisionHandler", visionInputs);

        Logger.getInstance().recordOutput("Vision Handler/Tags Updates", visionInputs.visionUpdates.size());
        // Process vision updates
        for (var visionUpdate : visionInputs.visionUpdates) {
            processNewTagPosition(visionUpdate);
        }
        visionInputs.visionUpdates.clear();
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
}
