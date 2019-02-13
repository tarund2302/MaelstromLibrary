package ftc.library.MaelstromMotions.MaelstromServos.CRServo;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Arrays;
import java.util.List;

/*custom class for setting up  cr servo systems*/
public class MaelstromCRServoSystem {
    private MaelstromCRServo servo1, servo2, servo3, servo4;
    private List<MaelstromCRServo> servos;

    public MaelstromCRServoSystem(String name1, String name2, HardwareMap hwMap){
        this( new MaelstromCRServo(name1,hwMap), new MaelstromCRServo(name2, hwMap));
    }
    public MaelstromCRServoSystem(String name1, String name2, String name3, HardwareMap hwMap){
        this ( new MaelstromCRServo(name1,hwMap), new MaelstromCRServo(name2, hwMap), new MaelstromCRServo(name3, hwMap));
    }
    public MaelstromCRServoSystem(String name1, String name2, String name3, String name4, HardwareMap hwMap){
        this ( new MaelstromCRServo(name1,hwMap), new MaelstromCRServo(name2, hwMap), new MaelstromCRServo(name3, hwMap), new MaelstromCRServo(name4,hwMap));
    }

    public MaelstromCRServoSystem(MaelstromCRServo one, MaelstromCRServo two){
        servo1 = one; servo2 = two; servo3 = null; servo4 =null;
        servos = Arrays.asList(servo1, servo2);
    }
    public MaelstromCRServoSystem(MaelstromCRServo one, MaelstromCRServo two, MaelstromCRServo three){
        servo1 = one; servo2 = two; servo3 = three; servo4 =null;
        servos = Arrays.asList(servo1,servo2,servo3);
    }
    public MaelstromCRServoSystem(MaelstromCRServo one, MaelstromCRServo two, MaelstromCRServo three, MaelstromCRServo four){
        servo1 = one; servo2 = two; servo3 = three; servo4 = four;
        servos = Arrays.asList(servo1,servo2,servo3,servo4);
    }

    public void setPower(double power){for (MaelstromCRServo servo : servos) servo.setPower(power);}

    public double getPower(){return servo1.getPower();}

    public void sleep(int time) throws InterruptedException {for (MaelstromCRServo servo : servos) servo.wait(time);}

    public void setPower(double p1, double p2){servo1.setPower(p1); servo2.setPower(p2);}



}
