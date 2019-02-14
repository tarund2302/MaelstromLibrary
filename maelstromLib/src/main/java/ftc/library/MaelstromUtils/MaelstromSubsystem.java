package ftc.library.MaelstromUtils;

import ftc.library.MaelstromSubsystems.MaelstromCollectors.TwoMotorCollector;
import ftc.library.MaelstromWrappers.MaelstromController;

public interface MaelstromSubsystem {
    void DriverControl(MaelstromController controller, TwoMotorCollector.Toggler toggler);
}
