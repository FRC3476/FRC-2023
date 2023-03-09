package frc.subsytem.vision;

import frc.subsytem.vision.VisionHandler.VisionUpdate;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import java.util.ArrayList;

public class VisionInputs implements LoggableInputs {

    public final ArrayList<VisionUpdate> visionUpdates = new ArrayList<>();
    public double lastVisionUpdate = -10000;

    @Override
    public void toLog(LogTable table) {
        double[] data = new double[visionUpdates.size() * 9];
        for (int i = 0; i < visionUpdates.size(); i++) {
            double[] update = visionUpdates.get(i).toArray();
            System.arraycopy(update, 0, data, i * 9, update.length);
        }

        table.put("VisionUpdates", data);
    }

    private final double[] update = new double[9];

    @Override
    public void fromLog(LogTable table) {
        double[] data = table.getDoubleArray("VisionUpdates", new double[0]);
        visionUpdates.clear();
        for (int i = 0; i < data.length; i += 9) {
            System.arraycopy(data, i, update, 0, update.length);
            visionUpdates.add(VisionUpdate.fromArray(update));
        }
    }
}
