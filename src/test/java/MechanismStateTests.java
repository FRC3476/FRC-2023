import frc.subsytem.MechanismStateManager;
import org.junit.jupiter.api.Test;

public class MechanismStateTests {

    @Test
    public void convertTest() {
        MechanismStateManager.MechanismStateCoordinates coordinates =
                new MechanismStateManager.MechanismStateCoordinates(.2, .2, 0);

        MechanismStateManager.MechanismStateSubsystemPositions subsystemPositions = MechanismStateManager.coordinatesToSubsystemPositions(coordinates);
        System.out.println(subsystemPositions.elevatorPositionMeters() + " "  + subsystemPositions.telescopingArmPositionMeters()
                + " " + subsystemPositions.grabberAngleDegrees());
    }

    @Test
    public void limitTest() {
        MechanismStateManager.MechanismStateCoordinates coordinates =
                new MechanismStateManager.MechanismStateCoordinates(0, 1, 126);

        MechanismStateManager.MechanismStateCoordinates limitedCoordinates = MechanismStateManager.limitCoordinates(coordinates);
        System.out.println(limitedCoordinates.xMeters() + " "  + limitedCoordinates.yMeters()
                + " " + limitedCoordinates.grabberAngleDegrees());
    }
}
