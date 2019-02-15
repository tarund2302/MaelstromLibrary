package ftc.library.MaelstromSubsystems.MaelstromCollectors.MotorCollectors;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import ftc.library.MaelstromMotions.MaelstromMotors.MaelstromMotor;
import ftc.library.MaelstromMotions.MaelstromMotors.MaelstromMotorSystem;
import ftc.library.MaelstromMotions.MaelstromMotors.MotorModel;
import ftc.library.MaelstromUtils.MaelstromSubsystem;
import ftc.library.MaelstromUtils.TimeConstants;
import ftc.library.MaelstromWrappers.MaelstromController;

public class OneMotorCollector implements TimeConstants {

    //public MaelstromMotorSystem collector;
    public MaelstromMotor collector;
    public double intakePower, outtakePower;
    public MotorModel model;


    public OneMotorCollector (String name1, MotorModel model, HardwareMap hwMap){
        collector = new MaelstromMotor(name1,model,hwMap);
        this.model = model;
    }

    public void DriverControl(MaelstromController controller, MaelstromController.Toggler toggler) {
        if(toggler == MaelstromController.Toggler.OFF){
            if(controller.y()) intake();
            else if(controller.x()) outtake();
            else stop();
        }
        else if(toggler == MaelstromController.Toggler.ON){
            if(controller.yToggle()) intake();
            else if(controller.xToggle()) outtake();
            else stop();
        }
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

