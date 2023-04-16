package frc.subsytem.grabber;

import org.littletonrobotics.junction.AutoLog;

public abstract class GrabberIO {
    @AutoLog
    public static class GrabberInputs {
        double pivotPosition = 0.0;
        double pivotVelocity = 0.0;
        double pivotRelativePosition = 0.0;
        double pivotRelativeVelocity = 0.0;
        double pivotCurrent = 0.0;
        double pivotTemp = 0.0;
        double pivotVoltage = 0.0;

        double grabberPosition = 0.0;
        double grabberAbsolutePosition = 0.0;
        double grabberVelocity = 0.0;
        double grabberCurrent = 0.0;
        double grabberTemp = 0.0;
        double grabberVoltage = 0.0;
        double grabberAppliedOutput = 0.0;
        double grabberBusVoltage = 0.0;


        double rollerMainPosition = 0.0;
        double rollerMainVelocity = 0.0;
        double rollerMainCurrent = 0.0;
        double rollerMainTemp = 0.0;
        double rollerMainVoltage = 0.0;

        double rollerFollowerPosition = 0.0;
        double rollerFollowerVelocity = 0.0;
        double rollerFollowerCurrent = 0.0;
        double rollerFollowerTemp = 0.0;
        double rollerFollowerVoltage = 0.0;
        boolean isLimitSwitchTriggered = false;
    }

    public synchronized void updateInputs(GrabberInputsAutoLogged inputs) {}

    public void setPivotVoltage(double voltage) {}

    public void setPivotPosition(double position, double arbFFVoltage) {}

    public void setGrabberVoltage(double current) {}

    public void resetPivotPosition(double position) {}

    public void resetGrabberPosition(double position) {}

    public void setRollerVoltage(double voltage) {}

    public void setAutoGrab(boolean enabled) {}
}
