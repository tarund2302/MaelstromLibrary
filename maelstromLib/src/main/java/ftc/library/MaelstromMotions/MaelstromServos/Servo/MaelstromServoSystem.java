package ftc.library.MaelstromMotions.MaelstromServos.Servo;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Arrays;
import java.util.List;

import ftc.library.MaelstromMotions.MaelstromServos.Servo.MaelstromServo;

/*custom class for creating set-position servo systems*/
public class MaelstromServoSystem {
    private MaelstromServo servo1;
    private  MaelstromServo servo2;
    private MaelstromServo servo3;
    private  MaelstromServo servo4;
    private int offset;
    private double position;
    private int time;
    private List<MaelstromServo> servos;

    public MaelstromServoSystem(String name1, String name2, HardwareMap hwMap) {
        this(new MaelstromServo(name1,hwMap),new MaelstromServo(name2, hwMap));
    }
    public MaelstromServoSystem(String name1, String name2, String name3, String name4,HardwareMap hwMap){
        this(new MaelstromServo(name1,hwMap),new MaelstromServo(name2, hwMap), new MaelstromServo(name3,hwMap), new MaelstromServo(name4,hwMap));
    }
    public MaelstromServoSystem(String name1, String name2, String name3, HardwareMap hwMap){
        this(new MaelstromServo(name1,hwMap),new MaelstromServo(name2, hwMap), new MaelstromServo(name3,hwMap));
    }
    public MaelstromServoSystem(String name1, String name2, String name3, String name4,
                                Servo.Direction direction1, Servo.Direction direction2, Servo.Direction direction3, Servo.Direction direction4,
                                HardwareMap hwMap){
        this(new MaelstromServo(name1,direction1, hwMap),new MaelstromServo(name2,direction2, hwMap),new MaelstromServo(name3,direction3, hwMap), new MaelstromServo(name4,direction4, hwMap));
    }
    public MaelstromServoSystem(MaelstromServo one, MaelstromServo two){
        servo1 = one; servo2 = two; servo3 = null; servo4 = null;
        servos = Arrays.asList(servo1,servo2);
    }
    public MaelstromServoSystem(MaelstromServo one, MaelstromServo two,MaelstromServo three){
        servo1 = one; servo2 = two; servo3 = three; servo4 = null;
        servos = Arrays.asList(servo1,servo2,servo3);
    }
    public MaelstromServoSystem(MaelstromServo one, MaelstromServo two, MaelstromServo three, MaelstromServo four){
        servo1 = one; servo2 = two; servo3 = three; servo4 = four;
        servos = Arrays.asList(servo1, servo2, servo3, servo4);
    }

    public double getPos(){return position;}

    public void setPos(double position){
        this.position = position;
        for(MaelstromServo servo : servos){
            servo.setPos(position);
        }
    }

    public void setPos(double position, int time){
        this.position = position;
        this.time = time;
        for(MaelstromServo servo : servos){
            servo.setPos(position,time);
        }
    }

    public void sleep(int time) throws InterruptedException{
        for (MaelstromServo servo : servos){
            servo.sleep(time);
        }
    }






}
