package ftc.library.MaelstromMotions.MaelstromMotors;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import ftc.library.MaelstromControl.PIDController;
import ftc.library.MaelstromControl.PIDPackage;
import ftc.library.MaelstromSensors.MaelstromEncoder;
import ftc.library.MaelstromUtils.TimeConstants;

public class MaelstromMotor implements TimeConstants {
  private DcMotor motor;
  private MaelstromEncoder encoder;
  private PIDPackage pidPackage;
  private int previousPos = 0;
  private double previousRate;
  private long previousTime = 0;
  private double rpm = 0;
  private double power;
  private double motorPower;
  private double KP,KI,KD;
  private double minPower;
  private double targetPower;
  private double minPosition, maxPosition;
  //private double motorCounts = NEVEREST_40_COUNTS_PER_REV;
  private double motorCounts = encoder.getCPR();
  private boolean closedLoop;

  private PIDController PID;

    public MaelstromMotor(String name, MotorModel type,DcMotor.Direction direction, HardwareMap hwMap){
        motor = hwMap.dcMotor.get(name);
        setDirection(direction);
        encoder = new MaelstromEncoder(this,type);
    }
    public MaelstromMotor(String name, MotorModel type,double Kp, double Ki, double Kd, DcMotor.Direction direction, HardwareMap hwMap){
        motor = hwMap.dcMotor.get(name);
        setDirection(direction);
        encoder = new MaelstromEncoder(this,type);
        this.PID = new PIDController(Kp,Ki,Kd,1);
    }


    public void init(HardwareMap hardwareMap, String name){
        motor = hardwareMap.dcMotor.get(name);
    }

    public void setPower(double power){
        this.power = power;
        motor.setPower(power);
    }

    public void setMode(DcMotor.RunMode runMode){
        motor.setMode(runMode);
    }

    public double getRPM(){
        int deltaPos = motor.getCurrentPosition() - previousPos;
        double deltaTime = (System.nanoTime() - previousTime)/NANOSECS_PER_MIN;
        if (deltaTime*6e4 > 10) {
            rpm = (deltaPos/ motorCounts)/(deltaTime);
            previousPos = motor.getCurrentPosition();
            previousTime = System.nanoTime();
        }
        return rpm;
    }

    public void setRPM(double rpm){
        power = PID.power(rpm,getRPM());
        motor.setPower((power > 0 && getRPM() < 0) || (power < 0 && getRPM() > 0) ? 0: power);
    }

    public double getVelocity(){
        int deltaPos = getCounts() - previousPos;
        double deltaTime = (System.nanoTime() - previousTime)/NANOSECS_PER_MIN;
        previousPos = getCounts();
        double rate = deltaPos / deltaTime;
        rate = (rate * SECONDS_PER_MIN) / motorCounts;
        if(rate != 0) return rate;
        else{
            previousRate = rate;
            return previousRate;
        }
    }

    public double calculateVelocityCorrection(){
        double deltaTime = (System.nanoTime() - previousTime)/NANOSECS_PER_MIN;
        double setRPM;
        setRPM = getRPM() * targetPower;
        double power = PID.power(setRPM,getVelocity());
        return power;
    }

    public void setVelocity(double velocity){
        targetPower = velocity;
        motorPower = calculateVelocityCorrection();
        if(!closedLoop) motorPower = targetPower;
        if(Math.abs(motorPower) < minPower && minPosition != 0) motorPower = 0;
        setPower(motorPower);
    }

    public void setSpeed(double speed){
        double rpm = motorCounts * speed;
        power = PID.power(rpm, getRPM());
        motor.setPower((power > 0 && getRPM() < 0) || (power < 0 && getRPM() > 0) ? 0: power);
    }

    public void setKP(double KP){
        this.KP = KP;
    }
    public void setKI(double KI){
        this.KI = KI;
    }
    public void setKD(double KD){
        this.KD = KD;
    }


    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior){
        motor.setZeroPowerBehavior(behavior);
    }

    public double getAngle (){
        double angle = (360*(motor.getCurrentPosition()) % getEncoder().getCountsPerInch())/ getEncoder().getCPR();
        return angle;
    }

    public void setAngle(double angle, double countsPerInch, double countsPerRev){
        double power = PID.power(angle, getAngle());
        motor.setPower(/*(power > 0 && getRPM() < 0) || (power < 0 && getRPM() > 0) ? 0:*/ power);
    }

    public void runWithoutEncoders(){
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void stopAndReset(){
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public int getCounts(){return motor.getCurrentPosition();}

    public double getPower(){return motor.getPower();}

    public void setDirection(DcMotor.Direction direction){motor.setDirection(direction);}

    public MaelstromEncoder getEncoder(){
        return encoder;
    }

}
