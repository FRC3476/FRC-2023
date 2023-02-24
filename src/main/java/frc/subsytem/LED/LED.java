package frc.subsytem.LED;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.robot.Constants;
import frc.subsytem.AbstractSubsystem;
import org.littletonrobotics.junction.Logger;

public class LED extends AbstractSubsystem {
    AddressableLED ledStrip;
    AddressableLEDBuffer addressableLEDBuffer;

    LED(int loggingInterval) {
        ledStrip = new AddressableLED(0);
        ledStrip.setLength(Constants.LED_LENGTH);
        addressableLEDBuffer = new AddressableLEDBuffer(Constants.LED_LENGTH);
        ledStrip.start();
    }

    public enum LedState {
        YELLOW(255, 255, 0),
        BLUE(0, 0, 255);

        final int r;
        final int g;
        final int b;

        LedState(int r, int g, int b) {
            this.r = r;
            this.g = g;
            this.b = b;
        }
    }

    public void setColor(LedState ledState) {
        for(int i = 0; i < addressableLEDBuffer.getLength(); i++) {
            addressableLEDBuffer.setRGB(i, ledState.r, ledState.g, ledState.b);
            ledStrip.setData(addressableLEDBuffer);
        }
    }

    public void off() {
        ledStrip.close();
    }

    public void logData() {
        Logger.getInstance().recordOutput("LED Color", String.valueOf(addressableLEDBuffer.getLED(0)));
    }
}
