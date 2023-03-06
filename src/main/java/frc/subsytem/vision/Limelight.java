package frc.subsytem.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;
import frc.subsytem.AbstractSubsystem;
import org.littletonrobotics.junction.Logger;

public class Limelight extends AbstractSubsystem {
    double[] botPoseWPIRed;

    public void getPose() {
        Rotation3d robotRotation = new Rotation3d(botPoseWPIRed[3], botPoseWPIRed[4], botPoseWPIRed[5]);
        Pose3d robotPose = new Pose3d(botPoseWPIRed[0], botPoseWPIRed[1] - Constants.FIELD_HEIGHT_METERS / 2,
                botPoseWPIRed[2], robotRotation);
    }

    @Override
    public void update() {
        botPoseWPIRed = NetworkTableInstance.getDefault().getTable("limelight").
                getEntry("botpose_wpired").getDoubleArray(new double[6]);
        getPose();
    }

    @Override
    public void logData() {
        Logger.getInstance().recordOutput("Limelight/X", botPoseWPIRed[0]);
        Logger.getInstance().recordOutput("Limelight/Y", botPoseWPIRed[1] - Constants.FIELD_HEIGHT_METERS / 2);
        Logger.getInstance().recordOutput("Limelight/Z", botPoseWPIRed[2]);
        Logger.getInstance().recordOutput("Limelight/Roll", botPoseWPIRed[3]);
        Logger.getInstance().recordOutput("Limelight/Pitch", botPoseWPIRed[4]);
        Logger.getInstance().recordOutput("Limelight/Yaw", botPoseWPIRed[5]);
    }
}
