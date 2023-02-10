package frc.subsytem.telescopingarm;

import org.littletonrobotics.junction.AutoLog;

public class TelescopingArmIO {
    @AutoLog
    public static class TelescopingArmInputs {
        double position = 0.0;
        double velocity = 0.0;
        double current = 0.0;
        double temp = 0.0;
        double voltage = 0.0;
    }

    public void updateInputs(TelescopingArmInputs inputs) {}

    public void setTelescopingArmVoltage(double voltage) {}

    public void setTelescopingArmPosition(double position, double arbFFVoltage) {}
}
