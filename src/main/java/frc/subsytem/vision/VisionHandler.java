package frc.subsytem.vision;

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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.Constants;
import frc.subsytem.AbstractSubsystem;
import frc.subsytem.robottracker.RobotTracker;
import org.jetbrains.annotations.NotNull;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardBoolean;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

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

    private static final VisionHandler INSTANCE = new VisionHandler();

    public static VisionHandler getInstance() {
        return INSTANCE;
    }


    VisionInputs visionInputs = new VisionInputs();

    private VisionHandler() {
        super(Constants.VISION_HANDLER_PERIOD);
        // Network table init
        // Network tables
        @NotNull NetworkTableInstance networkTableInstance = NetworkTableInstance.getDefault();
        @NotNull NetworkTable visionTable = networkTableInstance.getTable("Vision");

        new LoggedDashboardNumber("Vision Config/Exposure", 100);
        new LoggedDashboardNumber("Vision Config/Camera Type", 0);
        new LoggedDashboardNumber("Vision Config/X Resolution", 1280);
        new LoggedDashboardNumber("Vision Config/Y Resolution", 800);
        new LoggedDashboardNumber("Vision Config/Framerate", 30);
        new LoggedDashboardNumber("Vision Config/Threads", 4);
        new LoggedDashboardBoolean("Vision Config/Do Stream", false);
        new LoggedDashboardBoolean("Vision Config/Connection Flag", true);

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

    private void processNewTagPosition(VisionUpdate data) {
        final var expectedTagPosition = fieldLayout.getTagPose(data.tagId).orElseThrow(); // We should never get an
        // unknown tag

        final var translation = new Translation3d(data.posX, data.posY, data.posZ)
                .rotateBy(POSITIVE_Y_90)
                .rotateBy(POSITIVE_X_NEGATIVE_90);
        final var rotation = new Rotation3d(new Quaternion(data.rotW, -data.rotZ, -data.rotX, data.rotY));


        //The translation is now from the center of the robot to the center of the tag relative to the rotation of the robot
        final var robotToTagRobotRelative = translation.plus(cameraOffset);


        Rotation3d gyroAngle = RobotTracker.getInstance().getGyroAngleAtTime(data.timestamp);

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
                new RobotState(poseToFeedToRobotTracker, data.timestamp, "Fed Vision Pose Tag: " + data.tagId));
        RobotPositionSender.addRobotPosition(
                new RobotState(visionOnlyPose, data.timestamp, "Vision Only Pose Tag: " + data.tagId));
        RobotTracker.getInstance().addVisionMeasurement(poseToFeedToRobotTracker, data.timestamp);

        Logger.getInstance().recordOutput("VisionHandler/VisionOnlyPose/" + data.tagId, visionOnlyPose);
        Logger.getInstance().recordOutput("VisionHandler/FedPoses/" + data.tagId, poseToFeedToRobotTracker);


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
    public synchronized void update() {
        Logger.getInstance().processInputs("VisionHandler", visionInputs);

        // Process vision updates
        for (var visionUpdate : visionInputs.visionUpdates) {
            processNewTagPosition(visionUpdate);
        }
        visionInputs.visionUpdates.clear();
    }

    record VisionUpdate(double posX, double posY, double posZ, double rotW, double rotX, double rotY, double rotZ,
                        double timestamp, int tagId) {
        /**
         * This treats the data as a double array in the following order: posX, posY, posZ, rotW, rotX, rotY, rotZ, latency
         *
         * @param data the data
         * @param id   the tagId of the tag
         */
        public VisionUpdate(double[] data, int id) {
            this(data[0], data[1], data[2], data[3], data[4], data[5], data[6],
                    Timer.getFPGATimestamp() - (data[7] / 1000.0), id);
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
