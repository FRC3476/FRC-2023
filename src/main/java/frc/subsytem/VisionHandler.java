package frc.subsytem;

import com.dacubeking.AutoBuilder.robot.drawable.Drawable;
import com.dacubeking.AutoBuilder.robot.drawable.Line;
import com.dacubeking.AutoBuilder.robot.drawable.Renderer;
import com.dacubeking.AutoBuilder.robot.sender.pathpreview.RobotPositionSender;
import com.dacubeking.AutoBuilder.robot.sender.pathpreview.RobotState;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent.Kind;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants;
import org.jetbrains.annotations.NotNull;

import java.io.IOException;
import java.util.ArrayList;
import java.util.EnumSet;

import static frc.robot.Constants.FIELD_HEIGHT_METERS;
import static frc.robot.Constants.FIELD_WIDTH_METERS;
import static java.lang.Math.cos;

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
    private static final @NotNull AprilTagFieldLayout fieldLayout;

    static {
        try {
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

    private final Translation3d cameraOffset = new Translation3d(0, 0, 0);

    // Network tables
    private final @NotNull NetworkTableInstance networkTableInstance;
    private final @NotNull NetworkTable visionTable;
    private final @NotNull NetworkTable visionMiscTable;
    private final @NotNull NetworkTable configTable;

    private static VisionHandler instance = new VisionHandler(Constants.VISION_HANDLER_PERIOD, 5);

    public static VisionHandler getInstance() {

        return instance;
    }

    public VisionHandler(int period, int loggingInterval) {
        super(period, loggingInterval);

        // Network table init
        networkTableInstance = NetworkTableInstance.getDefault();
        visionTable = networkTableInstance.getTable("Vision");
        visionMiscTable = networkTableInstance.getTable("Vision Misc");
        configTable = networkTableInstance.getTable("Vision Config");

        // Send out connection flag to april tags processor
        visionMiscTable.getEntry("Connection Flag").setBoolean(true);

        configTable.getEntry("Exposure").setDouble(100);
        configTable.getEntry("Camera Type").setDouble(0);
        configTable.getEntry("X Resolution").setDouble(1280);
        configTable.getEntry("Y Resolution").setDouble(800);
        configTable.getEntry("Framerate").setDouble(30);
        configTable.getEntry("Threads").setDouble(4);
        configTable.getEntry("Do Stream").setBoolean(false);

        for (int i = 1; i <= 8; i++) {
            var table = visionTable.getEntry(String.valueOf(i));
            int finalI = i;
            NetworkTableInstance.getDefault().addListener(table.getTopic(),
                    EnumSet.of(Kind.kValueRemote),
                    (event) -> processNewTagPosition(event.valueData.value, finalI));
        }
    }

    private void processNewTagPosition(NetworkTableValue value, int tagId) {
        final var expectedTagPosition = fieldLayout.getTagPose(tagId).orElseThrow(); // We should never get an unknown tag

        // pos:
        // x,y,z
        // rot:
        // w, x, y, z
        // latency
        final var data = value.getDoubleArray();
        final var posX = data[0];
        final var posY = data[1];
        final var posZ = data[2];

        final var rotX = data[3];
        final var rotY = data[4];
        final var rotZ = data[5];
        final var rotW = data[6];
        final var latency = data[7];


        final var translation = new Translation3d(posX, posY, posZ)
                .rotateBy(POSITIVE_Y_90)
                .rotateBy(POSITIVE_X_NEGATIVE_90);
        final var rotation = new Rotation3d(new Quaternion(rotW, -rotZ, -rotX, rotY));
        final var timestamp = Timer.getFPGATimestamp() - (latency / 1000.0);


        //The translation is now from the center of the robot to the center of the tag relative to the rotation of the robot
        final var robotToTagRobotRelative = translation.plus(cameraOffset);


        Rotation3d gyroAngle = RobotTracker.getInstance().getGyroAngleAtTime(timestamp);

        var calculatedTranslationFromGyro =
                expectedTagPosition.getTranslation()
                        .plus(
                                robotToTagRobotRelative
                                        // Rotate the translation by the gyro angle (to make it relative to the field)
                                        .rotateBy(gyroAngle)
                                        // Make the vector from the tag to the robot (instead of the robot to the tag)
                                        .unaryMinus()
                        );

        var rotationFromTag = expectedTagPosition.getRotation()
                .plus(POSITIVE_Z_180)
                .plus(rotation);


        var calculatedTranslationFromTagOrientation =
                expectedTagPosition.getTranslation()
                        .plus(
                                robotToTagRobotRelative
                                        // Rotate the translation by the tag orientation (to make it relative to the field)
                                        .rotateBy(rotationFromTag)
                                        // Make the vector from the tag to the robot (instead of the robot to the tag)
                                        .unaryMinus()
                        );


        // Use position we calculated from the gyro, but use the rotation we calculated from the tag
        // the position we calculated from the gyro is more accurate,
        // but still pass the rotation we calculated from the tag, so we can use it to compensate for the gyro drift
        var poseToFeedToRobotTracker = new Pose2d(
                calculatedTranslationFromGyro.toTranslation2d(), // 2d pos on the field
                rotationFromTag.toRotation2d() //Returns the rotation of the robot around the z axis
        );

        var visionOnlyPose = new Pose2d(
                calculatedTranslationFromTagOrientation.toTranslation2d(), // 2d pos on the field
                rotationFromTag.toRotation2d() //Returns the rotation of the robot around the z axis
        );

        RobotPositionSender.addRobotPosition(
                new RobotState(poseToFeedToRobotTracker, timestamp, "Fed Vision Pose Tag: " + tagId));
        RobotPositionSender.addRobotPosition(
                new RobotState(visionOnlyPose, timestamp, "Vision Only Pose Tag: " + tagId));
        RobotTracker.getInstance().addVisionMeasurement(poseToFeedToRobotTracker, timestamp);


        // Draw the tag on the field
        var drawables = new Drawable[2];
        int j = 0;
        drawables[j++] = new Line((float) expectedTagPosition.getX(),
                (float) (expectedTagPosition.getY() - 0.5),
                (float) expectedTagPosition.getX(),
                (float) (expectedTagPosition.getY() + 0.5), new Color8Bit(255, 255, 0));
        drawables[j++] = new Line((float) expectedTagPosition.getX(),
                (float) expectedTagPosition.getY(),
                (float) (expectedTagPosition.getX() + cos(expectedTagPosition.getRotation().getZ()) * 0.1),
                (float) (expectedTagPosition.getY()),
                new Color8Bit(255, 255, 0));
        Renderer.render(drawables);
    }

    @Override
    public void selfTest() {

    }

    @Override
    public void logData() {

    }
}
