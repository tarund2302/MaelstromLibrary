package ftc.library.MaelstromMotions.MaelstromServos.CRServo;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

/*custom class for cr servos*/
public class MaelstromCRServo {
    private CRServo servo;

    public MaelstromCRServo(String name, HardwareMap hwMap){
        servo = hwMap.crservo.get(name);
    }
    public MaelstromCRServo(String name, CRServo.Direction direction, HardwareMap hwMap){
        servo = hwMap.crservo.get(name);
        servo.setDirection(direction);
    }

    public void setDirection(CRServo.Direction direction){servo.setDirection(direction);}

    public void setPower(double power){
        servo.setPower(power);
    }

    public double getPower(){
        return servo.getPower();
    }

    public void sleep(int time) throws InterruptedException {servo.wait(time);}


}
