package ftc.library.MaelstromWrappers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import ftc.library.MaelstromUtils.MaelstromUtils;

public abstract class MaelstromLinearOp extends LinearOpMode {
    protected MaelstromTelemetry feed;
    protected MaelstromController controller1, controller2;
    public final void runOpMode() throws InterruptedException {
        try{
            feed = new MaelstromTelemetry(super.telemetry);
            feed.setNewFirst();
            MaelstromUtils.setLinearOpMode(this);
            controller1 = new MaelstromController(super.gamepad1,"controller1");
            controller2 = new MaelstromController(super.gamepad2,"controller2");
            runLinearOpMode();
        }
        finally {
            stopLinearOpMode();
        }
    }
    public abstract void runLinearOpMode() throws InterruptedException;
    public void stopLinearOpMode() {}
    public void runSimultaneously(Runnable r1, Runnable r2){
        Thread t1 = new Thread(r1);
        Thread t2 = new Thread(r2);
        t1.start();
        t2.start();
        while(opModeIsActive() && (t1.isAlive() || t2.isAlive())){
            idle();
        }
    }
    public void sleep(int timeSeconds){
        try {
            Thread.sleep((long) timeSeconds * 1000);
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        }
    }
    public void sleep(double timeSeconds) {
        try {
            Thread.sleep((long) timeSeconds * 1000);
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        }
    }
    public void sleep(float timeMilli) {
        try {
            Thread.sleep((long) timeMilli * 1000);
        } catch (InterruptedException ex) {
            Thread.currentThread().interrupt();
        }
    }
}
