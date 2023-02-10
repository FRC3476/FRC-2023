package frc.subsytem.Elevator;

import org.littletonrobotics.junction.AutoLog;

public abstract class ElevatorIO {
    @AutoLog
    public static class ElevatorInputs {
        double elevatorPosition = 0.0;
        double elevatorVelocity = 0.0;
        double[] elevatorCurrent = {0.0, 0.0};
        double[] elevatorTemp = {0.0, 0.0};
        double[] elevatorVoltage = {0.0, 0.0};
    }

    public void updateInputs(ElevatorInputs inputs) {}

    public void setElevatorVoltage(double voltage) {}

    public void setElevatorPosition(double position, double arbFFVoltage) {}
}
