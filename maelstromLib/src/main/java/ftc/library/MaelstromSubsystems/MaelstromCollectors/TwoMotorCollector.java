package ftc.library.MaelstromSubsystems.MaelstromCollectors;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import ftc.library.MaelstromMotions.MaelstromMotors.MaelstromMotorSystem;
import ftc.library.MaelstromMotions.MaelstromMotors.MotorModel;
import ftc.library.MaelstromUtils.MaelstromSubsystem;
import ftc.library.MaelstromUtils.TimeConstants;
import ftc.library.MaelstromWrappers.MaelstromController;

public class TwoMotorCollector implements TimeConstants {

    public MaelstromMotorSystem collector;
    public double intakePower, outtakePower;
    public MotorModel model;

    public enum Toggler{
        ON,
        OFF
    }

    public TwoMotorCollector(String name1, String name2, MotorModel model, HardwareMap hwMap){
        collector = new MaelstromMotorSystem(name1, name2, DcMotorSimple.Direction.FORWARD, DcMotorSimple.Direction.REVERSE, hwMap, model);
        this.model = model;
    }

    public void DriverControl(MaelstromController controller, Toggler toggler) {
        if(toggler == Toggler.OFF){
            if(controller.y()) intake();
            else if(controller.x()) outtake();
            else stop();
        }
        else if(toggler == Toggler.ON){
            if(controller.yToggle()) intake();
            else if(controller.xToggle()) outtake();
            else stop();
        }
    }

    public void reverse(double power, int time){
        if(collector.isStalled(power, time)){
            collector.motor1.setPower(intakePower);
            collector.motor2.setPower(outtakePower);
        }
        else stop();
    }

    public void intake(){
        setPower(intakePower);
    }

    public void outtake(){
        setPower(outtakePower);
    }

    public void stop(){
        setPower(0);
    }

    public void setPower(double power){
        collector.setPower(power);
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior){
        collector.setZeroPowerBehavior(behavior);
    }

    public void setCollectorPowers(double intakePower, double outtakePower){
        this.intakePower = intakePower;
        this.outtakePower = outtakePower;
    }



}
