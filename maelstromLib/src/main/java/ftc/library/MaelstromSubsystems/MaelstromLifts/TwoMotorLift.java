package ftc.library.MaelstromSubsystems.MaelstromLifts;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import ftc.library.MaelstromControl.PIDController;
import ftc.library.MaelstromMotions.MaelstromMotors.MaelstromMotorSystem;
import ftc.library.MaelstromMotions.MaelstromMotors.MotorModel;
import ftc.library.MaelstromSubsystems.MaelstromCollectors.TwoMotorCollector;
import ftc.library.MaelstromUtils.MaelstromSubsystem;
import ftc.library.MaelstromUtils.TimeConstants;
import ftc.library.MaelstromWrappers.MaelstromController;

public class TwoMotorLift implements TimeConstants {
    public MaelstromMotorSystem lift;
    public MotorModel model;
    public double extendPower,retractPower;
    public double kp, ki, kd;
    public double trigger1, trigger2;
    public boolean button1, button2;
    public double targetPosition, currPosition;
    public PIDController liftPid = new PIDController(kp,ki,kd);

    public TwoMotorLift(String name1, String name2, MotorModel model, HardwareMap hwMap){
        lift = new MaelstromMotorSystem(name1, name2, DcMotorSimple.Direction.FORWARD, hwMap, model);
        this.model = model;
        lift.stopAndReset();
        lift.runWithoutEncoders();
    }

    public TwoMotorLift(String name1, String name2, MotorModel model, DcMotorSimple.Direction direction1, DcMotorSimple.Direction direction2, HardwareMap hwMap){
        lift = new MaelstromMotorSystem(name1, name2, direction1, direction2, hwMap, model);
        this.model = model;
        lift.stopAndReset();
        lift.runWithoutEncoders();
    }

    public void DriverControl(MaelstromController controller) {

        kp = (5e-7 * -getCounts()) + 0.001;
        if(controller.leftTrigger() > 0) extend();
        else if(controller.rightTrigger() > 0) retract();
        else stop();

        if(controller.leftTrigger() > 0 || controller.rightTrigger() > 0) targetPosition = getCounts();
        else {
            currPosition = getCounts();
            double pidPower = liftPid.power(currPosition, targetPosition);
            lift.setPower(pidPower);
        }
        liftPid.setKP(kp);
    }

    public void extend(){
        setPower(extendPower);
    }

    public void retract(){
        setPower(retractPower);
    }

    public void stop(){
        setPower(0);
    }

    public void setLiftPowers(double extendPower, double retractPower){
        this.extendPower = extendPower;
        this.retractPower = retractPower;
    }

    public void setPower(double power){
        lift.setPower(power);
    }

    public double getRawPower(){
        return lift.motor1.getPower();
    }

    public double getTotalPower(){
        return (lift.motor1.getPower() + lift.motor2.getPower()) / 2;
    }

    public int getRawCounts(){
        return lift.motor1.getCounts();
    }

    public int getCounts(){
        return (lift.motor1.getCounts() + lift.motor2.getCounts()) / 2;
    }

    public void setPID(double ki, double kd){
        this.ki = ki;
        this.kd = kd;
    }

/*    public void setButtons(double trigger1, double trigger2, boolean button1, boolean button2){
        if(trigger1 != 0){
            this.trigger1 = trigger1;
        }
        else if(trigger2 != 0){
            this.trigger2 = trigger2;
        }
    }*/

}
