package frc.subsytem.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;
import frc.subsytem.vision.VisionHandler.LimelightUpdate;
import frc.subsytem.vision.VisionHandler.VisionUpdate;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import java.util.ArrayList;

public class VisionInputs implements LoggableInputs {

    public final ArrayList<VisionUpdate> visionUpdates = new ArrayList<>();

    public final ArrayList<LimelightUpdate> limelightUpdates = new ArrayList<>();

    public double lastVisionUpdate = -10000;

    @Override
    public void toLog(LogTable table) {
        double[] realsenseData = new double[visionUpdates.size() * 9];
        for (int i = 0; i < visionUpdates.size(); i++) {
            double[] update = visionUpdates.get(i).toArray();
            System.arraycopy(update, 0, realsenseData, i * 9, update.length);
        }

        table.put("VisionUpdates", realsenseData);

        double[] limelightData = new double[limelightUpdates.size() * 8];
        for (int i = 0; i < limelightUpdates.size(); i++) {
            double[] update = new double[8];

            update[0] = limelightUpdates.get(i).pose3d().getX();
            update[1] = limelightUpdates.get(i).pose3d().getY();
            update[2] = limelightUpdates.get(i).pose3d().getZ();

            update[3] = limelightUpdates.get(i).pose3d().getRotation().getQuaternion().getW();
            update[4] = limelightUpdates.get(i).pose3d().getRotation().getQuaternion().getX();
            update[5] = limelightUpdates.get(i).pose3d().getRotation().getQuaternion().getY();
            update[6] = limelightUpdates.get(i).pose3d().getRotation().getQuaternion().getZ();

            update[7] = limelightUpdates.get(i).timestamp();
            System.arraycopy(update, 0, limelightData, i * 8, update.length);
        }

        table.put("LimelightUpdates", limelightData);
    }

    private final double[] update = new double[9];

    @Override
    public void fromLog(LogTable table) {
        double[] realsenseData = table.getDoubleArray("VisionUpdates", new double[0]);
        visionUpdates.clear();
        for (int i = 0; i < realsenseData.length; i += 9) {
            System.arraycopy(realsenseData, i, update, 0, update.length);
            visionUpdates.add(VisionUpdate.fromArray(update));
        }

        double[] limelightData = table.getDoubleArray("LimelightUpdates", new double[0]);
        limelightUpdates.clear();
        for (int i = 0; i < limelightData.length; i += 8) {
            System.arraycopy(limelightData, i, update, 0, update.length);
            limelightUpdates.add(
                    new LimelightUpdate(
                            new Pose3d(
                                    update[0],
                                    update[1],
                                    update[2],
                                    new Rotation3d(new Quaternion(update[3], update[4], update[5], update[6]))
                            ),
                            update[7]
                    )
            );
        }
    }
}
