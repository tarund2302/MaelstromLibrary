package ftc.library.MaelstromMotions.MaelstromServos.Servo;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Hardware;

import ftc.library.MaelstromRobot;
import ftc.library.MaelstromSensors.MaelstromTimer;
import ftc.library.MaelstromUtils.MaelstromUtils;

/*custom class for set-position servos*/
public class MaelstromServo {
    private Servo servo;
    private String nameServo;
    MaelstromTimer timer = new MaelstromTimer();
    private double targetPos;
    private double max = 1, min = 0;
    private MaelstromUtils.AutonomousOpMode auto;

    public MaelstromServo(String name, HardwareMap hwMap){
        this.nameServo = name;
        servo = hwMap.servo.get(name);
    }
    public MaelstromServo(String name, Servo.Direction direction, HardwareMap hwMap){
        this.nameServo = name;
        servo = hwMap.servo.get(name);
        servo.setDirection(direction);
    }

    public void setPos(double position){
        targetPos = position;
        position = ((max - min)*position) + min;
        servo.setPosition(position);
    }
    public double getPos(){
        return servo.getPosition();
    }

    public void setPos(double position, int time){
        for (double i = servo.getPosition(); i < position; i += 0.01){
            if (!opModeActive())break;

            else setPos(i);
            try {Thread.sleep(time);}
            catch (InterruptedException e){}
        }
    }

    public boolean isStalled( int time){
        boolean isStalled = false;
        double prePos = getPos();
        if (getPos() == prePos
                && !timer.elapsedTime(time, MaelstromTimer.Time.SECS)) isStalled = true;
        return isStalled;
    }

    public void sleep(int time) throws InterruptedException{servo.wait(time);}

    public boolean opModeActive(){
        return auto.getOpModeIsActive();
    }

}
