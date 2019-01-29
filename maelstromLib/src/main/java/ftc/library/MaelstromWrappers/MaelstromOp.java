package ftc.library.MaelstromWrappers;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import ftc.library.MaelstromRobot;

public abstract class MaelstromOp extends OpMode {
    public MaelstromRobot robot;
    public MaelstromTelemetry feed = new MaelstromTelemetry(super.telemetry);
    protected MaelstromController controller1 = new MaelstromController(super.gamepad1,"controller1");
    protected MaelstromController controller2 = new MaelstromController(super.gamepad2,"controller2");
}
