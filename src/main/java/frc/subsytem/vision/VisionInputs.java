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

    public double lastVisionUpdateRealsense = -10000;
    public double lastVisionUpdateLimelightLeft = -10000;
    public double lastVisionUpdateLimelightRight = -10000;
    public String lastVisionStatus = "";


    @Override
    public void toLog(LogTable table) {
        double[] realsenseData = new double[visionUpdates.size() * 9];
        for (int i = 0; i < visionUpdates.size(); i++) {
            double[] update = visionUpdates.get(i).toArray();
            System.arraycopy(update, 0, realsenseData, i * 9, update.length);
        }

        table.put("VisionUpdates", realsenseData);

        double[] limelightData = new double[limelightUpdates.size() * 9];
        for (int i = 0; i < limelightUpdates.size(); i++) {
            double[] update = new double[9];

            update[0] = limelightUpdates.get(i).pose3d().getX();
            update[1] = limelightUpdates.get(i).pose3d().getY();
            update[2] = limelightUpdates.get(i).pose3d().getZ();

            update[3] = limelightUpdates.get(i).pose3d().getRotation().getQuaternion().getW();
            update[4] = limelightUpdates.get(i).pose3d().getRotation().getQuaternion().getX();
            update[5] = limelightUpdates.get(i).pose3d().getRotation().getQuaternion().getY();
            update[6] = limelightUpdates.get(i).pose3d().getRotation().getQuaternion().getZ();

            update[7] = limelightUpdates.get(i).timestamp();
            update[8] = limelightUpdates.get(i).limelightIndex();
            System.arraycopy(update, 0, limelightData, i * 9, update.length);
        }

        table.put("LimelightUpdates", limelightData);
        table.put("LastVisionUpdateRealsense", lastVisionUpdateRealsense);
        table.put("LastVisionUpdateLimelightLeft", lastVisionUpdateLimelightLeft);
        table.put("LastVisionUpdateLimelightRight", lastVisionUpdateLimelightRight);
        table.put("LastVisionStatus", lastVisionStatus);
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
        final double[] update2 = new double[9];
        for (int i = 0; i < limelightData.length; i += 9) {
            System.arraycopy(limelightData, i, update2, 0, update2.length);
            limelightUpdates.add(
                    new LimelightUpdate(
                            new Pose3d(
                                    update2[0],
                                    update2[1],
                                    update2[2],
                                    new Rotation3d(new Quaternion(update2[3], update2[4], update2[5], update2[6]))
                            ),
                            update2[7],
                            (int) update2[8]
                    )
            );
        }

        lastVisionUpdateRealsense = table.getDouble("LastVisionUpdateRealsense", -10000);
        lastVisionUpdateLimelightLeft = table.getDouble("LastVisionUpdateLimelightLeft", -10000);
        lastVisionUpdateLimelightRight = table.getDouble("LastVisionUpdateLimelightRight", -10000);
        lastVisionStatus = table.getString("LastVisionStatus", "");
    }
}
