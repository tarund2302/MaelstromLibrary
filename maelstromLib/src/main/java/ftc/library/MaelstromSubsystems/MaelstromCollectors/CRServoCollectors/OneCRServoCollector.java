package ftc.library.MaelstromSubsystems.MaelstromCollectors.CRServoCollectors;

import com.qualcomm.robotcore.hardware.HardwareMap;

import ftc.library.MaelstromMotions.MaelstromServos.CRServo.MaelstromCRServo;
import ftc.library.MaelstromUtils.TimeConstants;
import ftc.library.MaelstromWrappers.MaelstromController;

public class OneCRServoCollector implements TimeConstants {

    //public MaelstromMotorSystem collector;
    public MaelstromCRServo collector;
    public double intakePower, outtakePower;

    public OneCRServoCollector (String name1, HardwareMap hwMap){
        collector = new MaelstromCRServo(name1,hwMap);
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

    public void setCollectorPowers(double intakePower, double outtakePower){
        this.intakePower = intakePower;
        this.outtakePower = outtakePower;
    }
}

