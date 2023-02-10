package frc.subsytem.robottracker;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import java.util.ArrayList;
import java.util.List;

public class GyroInputs implements LoggableInputs {
    record Entry<U>(double timestamp, U value) {}

    protected List<Entry<Rotation3d>> rotations = new ArrayList<>();
    protected List<Entry<Translation3d>> accelerations = new ArrayList<>();

    protected double gyroYawVelocity = 0.0;
    protected double gyroRollVelocity = 0.0;
    protected double gyroPitchVelocity = 0.0;

    protected Rotation2d rotation2d = new Rotation2d();

    // Created here to avoid creating a new array every time
    private final double[] xyz_dps = new double[3];

    public void updateInputs(WPI_Pigeon2 pigeon2) {
        pigeon2.getRawGyro(xyz_dps);
        gyroYawVelocity = xyz_dps[2];
        gyroRollVelocity = xyz_dps[0];
        gyroPitchVelocity = xyz_dps[1];

        rotation2d = pigeon2.getRotation2d();
    }

    @Override
    public void toLog(LogTable table) {
        double[] rotationData = new double[rotations.size() * 5];
        for (int i = 0; i < rotations.size(); i++) {
            var entry = rotations.get(i);
            var quaternion = entry.value.getQuaternion();
            rotationData[i * 5] = entry.timestamp;
            rotationData[i * 5 + 1] = quaternion.getW();
            rotationData[i * 5 + 2] = quaternion.getX();
            rotationData[i * 5 + 3] = quaternion.getY();
            rotationData[i * 5 + 4] = quaternion.getZ();
        }

        table.put("Rotations", rotationData);

        double[] accelerationData = new double[accelerations.size() * 4];

        for (int i = 0; i < accelerations.size(); i++) {
            var entry = accelerations.get(i);
            var vector = entry.value;
            accelerationData[i * 4] = entry.timestamp;
            accelerationData[i * 4 + 1] = vector.getX();
            accelerationData[i * 4 + 2] = vector.getY();
            accelerationData[i * 4 + 3] = vector.getZ();
        }

        table.put("Accelerations", accelerationData);

        table.put("GyroYawVelocity", gyroYawVelocity);
        table.put("GyroRollVelocity", gyroRollVelocity);
        table.put("GyroPitchVelocity", gyroPitchVelocity);
        table.put("GyroYaw", rotation2d.getDegrees());
    }

    @Override
    public void fromLog(LogTable table) {
        double[] rotationData = table.getDoubleArray("Rotations", new double[0]);
        rotations.clear();
        for (int i = 0; i < rotationData.length; i += 5) {
            rotations.add(new Entry<>(rotationData[i], new Rotation3d(
                    new Quaternion(rotationData[i + 1], rotationData[i + 2], rotationData[i + 3], rotationData[i + 4]))));
        }

        double[] accelerationData = table.getDoubleArray("Accelerations", new double[0]);
        accelerations.clear();
        for (int i = 0; i < accelerationData.length; i += 4) {
            accelerations.add(new Entry<>(accelerationData[i],
                    new Translation3d(accelerationData[i + 1], accelerationData[i + 2], accelerationData[i + 3])));
        }

        gyroYawVelocity = table.getDouble("GyroYawVelocity", 0.0);
        gyroRollVelocity = table.getDouble("GyroRollVelocity", 0.0);
        gyroPitchVelocity = table.getDouble("GyroPitchVelocity", 0.0);
        rotation2d = Rotation2d.fromDegrees(table.getDouble("GyroYaw", 0.0));
    }
}
