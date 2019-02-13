package ftc.library.MaelstromMotions.MaelstromMotors;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Arrays;
import java.util.List;

import ftc.library.MaelstromSensors.MaelstromLimitSwitch;

/*custom class for creating motor systems*/
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
            motor.setPID(Kp,Ki,Kd);
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
            motor.setPID(Kp,Ki,Kd);
        }
        numMotors = 4;
        model = encoder;
    }
    public MaelstromMotorSystem(String name1, String name2, String systemName,DcMotorSimple.Direction direction, HardwareMap hwMap, MotorModel encoder){
        this.systemName = systemName;
        motor1 = new MaelstromMotor(name1,encoder, direction,hwMap);
        motor2 = new MaelstromMotor(name2,encoder, direction,hwMap);
        motors = Arrays.asList(motor1,motor2);
        numMotors = 2;
        model = encoder;
    }
    public MaelstromMotorSystem(String name1, String name2, String name3, double Kp, double Ki, double Kd, String systemName,DcMotorSimple.Direction direction, HardwareMap hwMap, MotorModel encoder){
        this.systemName = systemName;
        motor1 = new MaelstromMotor(name1,encoder, direction,hwMap);
        motor2 = new MaelstromMotor(name2,encoder, direction,hwMap);
        motor3 = new MaelstromMotor(name3,encoder, direction,hwMap);
        motors = Arrays.asList(motor1,motor2,motor3);
        for(MaelstromMotor motor : motors) {
            motor.setPID(Kp,Ki,Kd);
        }
        numMotors = 3;
        model = encoder;
    }

    public MaelstromMotorSystem(String name1, String name2,  double Kp, double Ki, double Kd, String systemName,DcMotorSimple.Direction direction, HardwareMap hwMap, MotorModel encoder){
        this.systemName = systemName;
        motor1 = new MaelstromMotor(name1,encoder, direction,hwMap);
        motor2 = new MaelstromMotor(name2,encoder, direction,hwMap);
        motors = Arrays.asList(motor1,motor2);
        for(MaelstromMotor motor : motors) {
            motor.setPID(Kp,Ki,Kd);
        }
        numMotors = 2;
        model = encoder;
    }
    public MaelstromMotorSystem(String name1, String name2,  double Kp, double Ki, double Kd,DcMotorSimple.Direction direction1, DcMotorSimple.Direction direction2, HardwareMap hwMap, MotorModel encoder){
        this.systemName = systemName;
        motor1 = new MaelstromMotor(name1,encoder, direction1,hwMap);
        motor2 = new MaelstromMotor(name2,encoder, direction2,hwMap);
        motors = Arrays.asList(motor1,motor2);
        for(MaelstromMotor motor : motors) {
            motor.setPID(Kp,Ki,Kd);
        }
        numMotors = 2;
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
    public MaelstromMotorSystem setPID(double kp, double ki, double kd){
        for (MaelstromMotor motor: motors) motor.setPID(kp,ki,kd);
        return this;
    }

    public MaelstromMotorSystem setGearRatio(double gearRatio){
        for(MaelstromMotor motor : motors) motor.getEncoder().setGearRatio(gearRatio);
        return this;
    }

    public double getInches(){
        double total = 0;
        for (MaelstromMotor motor : motors) total += motor.getInches();
        return total / numMotors;
    }

    public void setLimitrs(MaelstromLimitSwitch min, MaelstromLimitSwitch max){
        for(MaelstromMotor motor : motors){
            motor.setLimits(min,max);
        }
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
        double total = 0;
        for (MaelstromMotor motor : motors) total += motor.getAngle();
        return total / numMotors;
    }

    public void setAngle(double angle){
        for (MaelstromMotor motor : motors){
            motor.setAngle(angle);
        }
    }

    public double getCounts(){
        double total = 0;
        for (MaelstromMotor motor : motors) total += motor.getCounts();
        return total / numMotors;
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
        double total = 0;
        for (MaelstromMotor motor : motors) total += motor.getPower();
        return total / numMotors;
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
