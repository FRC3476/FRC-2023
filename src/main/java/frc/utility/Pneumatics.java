package frc.utility;

import edu.wpi.first.wpilibj.PneumaticHub;

public final class Pneumatics {
    private static final PneumaticHub pneumaticHub = new PneumaticHub(3);

    public synchronized static PneumaticHub getPneumaticsHub() {
        return pneumaticHub;
    }
}
