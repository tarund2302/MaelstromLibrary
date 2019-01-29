package ftc.library.MaelstromMotions.MaelstromMotors;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Arrays;
import java.util.List;

public class MaelstromMotorSystem {
    public MaelstromMotor motor1,motor2,motor3,motor4;
    private int numMotors;
    private List<MaelstromMotor> motors;
    private double currPower = 0;
    private double slowPower = 0;
    private String systemName;
    private MotorModel model;
    public MaelstromMotorSystem(String name1, String name2, String name3, String name4, double Kp, double Ki, double Kd, String systemName, HardwareMap hwMap, MotorModel encoder){
        this.systemName = systemName;
        motor1 = new MaelstromMotor(name1,encoder, DcMotorSimple.Direction.FORWARD,hwMap);
        motor2 = new MaelstromMotor(name2,encoder, DcMotorSimple.Direction.FORWARD,hwMap);
        motor3 = new MaelstromMotor(name3,encoder, DcMotorSimple.Direction.FORWARD,hwMap);
        motor4 = new MaelstromMotor(name4,encoder, DcMotorSimple.Direction.FORWARD,hwMap);
        motors = Arrays.asList(motor1,motor2,motor3,motor4);
        for(MaelstromMotor motor : motors) {
            motor.setKP(Kp);
            motor.setKI(Ki);
            motor.setKD(Kd);
        }
        numMotors = 4;
        model = encoder;
    }
    public MaelstromMotorSystem(String name1, String name2, String name3, String name4, double Kp, double Ki, double Kd, String systemName,DcMotorSimple.Direction direction, HardwareMap hwMap, MotorModel encoder){
        this.systemName = systemName;
        motor1 = new MaelstromMotor(name1,encoder, direction,hwMap);
        motor2 = new MaelstromMotor(name2,encoder, direction,hwMap);
        motor3 = new MaelstromMotor(name3,encoder, direction,hwMap);
        motor4 = new MaelstromMotor(name4,encoder, direction,hwMap);
        motors = Arrays.asList(motor1,motor2,motor3,motor4);
        for(MaelstromMotor motor : motors) {
            motor.setKP(Kp);
            motor.setKI(Ki);
            motor.setKD(Kd);
        }
        numMotors = 4;
        model = encoder;
    }
    public MaelstromMotorSystem(String name1, String name2, String systemName,DcMotorSimple.Direction direction, HardwareMap hwMap, MotorModel encoder){
        this.systemName = systemName;
        motor1 = new MaelstromMotor(name1,encoder, direction,hwMap);
        motor2 = new MaelstromMotor(name2,encoder, direction,hwMap);
        motors = Arrays.asList(motor1,motor2);
        numMotors = 4;
        model = encoder;
    }

    public MaelstromMotorSystem(String name1, String name2,  double Kp, double Ki, double Kd, String systemName,DcMotorSimple.Direction direction, HardwareMap hwMap, MotorModel encoder){
        this.systemName = systemName;
        motor1 = new MaelstromMotor(name1,encoder, direction,hwMap);
        motor2 = new MaelstromMotor(name2,encoder, direction,hwMap);
        motors = Arrays.asList(motor1,motor2);
        for(MaelstromMotor motor : motors) {
            motor.setKP(Kp);
            motor.setKI(Ki);
            motor.setKD(Kd);
        }
        numMotors = 4;
        model = encoder;
    }

    public MaelstromMotorSystem setKp(double kp){
        for (MaelstromMotor motor: motors) motor.setKP(kp);
        return this;
    }
    public MaelstromMotorSystem setKi(double ki){
        for (MaelstromMotor motor: motors) motor.setKP(ki);
        return this;
    }
    public MaelstromMotorSystem setKd(double kd){
        for (MaelstromMotor motor: motors) motor.setKP(kd);
        return this;
    }

    public MaelstromMotorSystem setGearRatio(double gearRatio){
        for(MaelstromMotor motor : motors) motor.getEncoder().setGearRatio(gearRatio);
        return this;
    }

    public double getInches(){
        double inches = motor1.getEncoder().getInches();
        return inches;
    }

    public MaelstromMotorSystem runWithoutEncoders(){
        for (MaelstromMotor motor : motors){
            motor.runWithoutEncoders();
        }
        return this;
    }

    public MaelstromMotorSystem stopAndReset(){
        for(MaelstromMotor motor : motors){
            motor.stopAndReset();
        }
        return this;
    }

    public double getAngle(){
        double angle = motor1.getAngle();
        return angle;
    }

    public double getCounts(){
        double counts = motor1.getCounts();
        return counts;
    }

    public double getGearRatio(){
        double gearRatio = motor1.getEncoder().getGearRatio();
        return gearRatio;
    }

    public double getWheelCircumference(){
        double circumference = motor1.getEncoder().getWheelCircumference();
        return circumference;
    }

    public double getPower(){
        double power = motor1.getPower();
        return power;
    }

    public MotorModel getModel(){
        return model;
    }

    public void setPower(double power){
        for (MaelstromMotor motor : motors){
            motor.setPower(power);
        }
    }

    public void setVelocity(double velocity){
        for (MaelstromMotor motor : motors){
            motor.setVelocity(velocity);
        }
    }



}
