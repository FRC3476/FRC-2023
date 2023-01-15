package frc.subsytem;

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
import frc.robot.Constants;
import org.jetbrains.annotations.NotNull;

import java.io.IOException;
import java.util.EnumSet;

import static frc.robot.Constants.FIELD_HEIGHT;
import static java.lang.Math.cos;
import static java.lang.Math.sin;

/**
 * Gives pose estimates based on april tag vision
 */
public class VisionHandler extends AbstractSubsystem {

    private final @NotNull AprilTagFieldLayout fieldLayout;

    {
        try {
            fieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
            fieldLayout.setOrigin(new Pose3d(new Translation3d(0, FIELD_HEIGHT / 2, 0), new Rotation3d()));
            for (AprilTag tag : fieldLayout.getTags()) {
                System.out.println("Pos: " + tag.pose.getTranslation() + " angle: x:" + tag.pose.getRotation().getX()
                        + ", y:" + tag.pose.getRotation().getY() + ", z:" + tag.pose.getRotation().getZ());
            }
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

    Vector<N3> POSITIVE_X = VecBuilder.fill(1, 0, 0);
    Vector<N3> POSITIVE_Y = VecBuilder.fill(0, 1, 0);
    Vector<N3> POSITIVE_Z = VecBuilder.fill(0, 0, 1);

    private void processNewTagPosition(NetworkTableValue value, int tagId) {
        final var expectedTagPosition = fieldLayout.getTagPose(tagId).orElseThrow();
        // pos:
        // x,y,z
        // rot:
        // w, x, y, z
        // time
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
                .rotateBy(new Rotation3d(POSITIVE_Y, Math.toRadians(90.0)))
                .rotateBy(new Rotation3d(POSITIVE_X, Math.toRadians(-90.0)));
        final var rotation = new Rotation3d(new Quaternion(rotW, rotX, rotY, rotZ))
                .rotateBy(new Rotation3d(POSITIVE_Y, Math.toRadians(90.0)))
                .rotateBy(new Rotation3d(POSITIVE_X, Math.toRadians(-90.0)));


        System.out.println("Tag " + tagId + " pos: " + translation + " rot: " + toEulerAngles(rotation.getQuaternion()));
        final var timestamp = Timer.getFPGATimestamp() - latency;


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

        // TODO: Check if we should add/subtract the rotation of the orientation here
        var rotationFromTag = expectedTagPosition.getRotation().unaryMinus().rotateBy(rotation);

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

        RobotPositionSender.addRobotPosition(new RobotState(poseToFeedToRobotTracker, timestamp, "Fed Vision Pose"));
        RobotPositionSender.addRobotPosition(new RobotState(visionOnlyPose, timestamp, "Vision Only Pose"));
        RobotTracker.getInstance().addVisionMeasurement(poseToFeedToRobotTracker, timestamp);
    }


    private EulerAngles toEulerAngles(Quaternion q) {
        EulerAngles angles = new EulerAngles();

        double w = q.getW();
        double x = q.getX();
        double y = q.getY();
        double z = q.getZ();

        // roll (x-axis rotation)
        double sinr_cosp = 2 * (w * x + y * z);
        double cosr_cosp = 1 - 2 * (x * x + y * y);
        angles.roll = Math.atan2(sinr_cosp, cosr_cosp);

        // pitch (y-axis rotation)
        double sinp = Math.sqrt(1 + 2 * (w * y - x * z));
        double cosp = Math.sqrt(1 - 2 * (w * y - x * z));
        angles.pitch = 2 * Math.atan2(sinp, cosp) - Math.PI / 2;

        // yaw (z-axis rotation)
        double siny_cosp = 2 * (w * z + x * y);
        double cosy_cosp = 1 - 2 * (y * y + z * z);
        angles.yaw = Math.atan2(siny_cosp, cosy_cosp);

        return angles;
    }

    Quaternion toQuaternion(double roll, double pitch, double yaw) // roll (x), pitch (Y), yaw (z)
    {
        // Abbreviations for the various angular functions

        double cr = cos(roll * 0.5);
        double sr = sin(roll * 0.5);
        double cp = cos(pitch * 0.5);
        double sp = sin(pitch * 0.5);
        double cy = cos(yaw * 0.5);
        double sy = sin(yaw * 0.5);


        double w = cr * cp * cy + sr * sp * sy;
        double x = sr * cp * cy - cr * sp * sy;
        double y = cr * sp * cy + sr * cp * sy;
        double z = cr * cp * sy - sr * sp * cy;
        return new Quaternion(w, x, y, z);
    }

    public static class EulerAngles {
        public double roll;
        public double pitch;
        public double yaw;

        @Override
        public String toString() {
            return "x: " + Math.toDegrees(roll) + ", y: " + Math.toDegrees(pitch) + ", z: " + Math.toDegrees(yaw);
        }
    }

    @Override
    public void update() {

    }

    @Override
    public void selfTest() {

    }

    @Override
    public void logData() {

    }
}
