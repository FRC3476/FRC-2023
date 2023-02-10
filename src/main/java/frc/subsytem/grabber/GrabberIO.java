package frc.subsytem.grabber;

import org.littletonrobotics.junction.AutoLog;

public class GrabberIO {
    @AutoLog
    public static class GrabberInputs {
        double pivotPosition = 0.0;
        double pivotVelocity = 0.0;
        double pivotCurrent = 0.0;
        double pivotTemp = 0.0;
        double pivotVoltage = 0.0;

        double grabberPosition = 0.0;
        double grabberVelocity = 0.0;
        double grabberCurrent = 0.0;
        double grabberTemp = 0.0;
        double grabberVoltage = 0.0;
    }

    public void updateInputs(GrabberInputsAutoLogged inputs) {}

    public void setPivotVoltage(double voltage) {}

    public void setPivotPosition(double position, double arbFFVoltage) {}

    public void setGrabberVoltage(double voltage) {}
}
