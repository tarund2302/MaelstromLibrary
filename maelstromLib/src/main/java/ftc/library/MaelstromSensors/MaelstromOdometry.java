package ftc.library.MaelstromSensors;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import ftc.library.MaelstromMotions.MaelstromMotors.MaelstromMotor;
import ftc.library.MaelstromMotions.MaelstromMotors.MotorModel;
import ftc.library.MaelstromUtils.TimeConstants;

public class MaelstromOdometry implements TimeConstants {
    MaelstromMotor motor;
    private int previousPos = 0;
    private long previousTime = 0;
    private double rpm = 0;
    private double power;

    public MaelstromOdometry(String name, MotorModel model, HardwareMap hwMap){
        this.motor = new MaelstromMotor(name,model,hwMap);
    }

    public int getPosition(){
        return motor.getCounts();
    }

    public int getTargetCounts(int pos){
        pos = (int)((pos/(Math.PI*2))*motor.getEncoder().getCPR());
        return pos;
    }
    public int trackPosition(int pos){
        pos = (int) ((pos*motor.getEncoder().getCPR())*(2*Math.PI));
        return pos;
    }
    public void reset(){
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

}
