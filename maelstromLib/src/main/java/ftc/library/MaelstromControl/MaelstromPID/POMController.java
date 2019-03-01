package ftc.library.MaelstromControl.MaelstromPID;

import ftc.library.MaelstromUtils.TimeConstants;

/*class for Proportional-On-Measurement control system*/
public class POMController implements TimeConstants {
    private double i=0;
    private double d;
    private double measurement;
    private double initialPoint;
    private double power;
    private double KP;
    private double KI;
    private double KD;
    private double maxI;
    private double previousTime = 0;
    private double error;
    private double previousError = 0;

    public POMController(double KP, double KI, double KD, double maxI){
        this.KP = KP;
        this.KI = KI;
        this.KD = KD;
        this.maxI = maxI;
    }

    public double power(double target, double currentLoc, double initialPoint){
        this.initialPoint = initialPoint;
        double measurement = currentLoc - initialPoint;
        this.measurement = measurement;
        double error = target - currentLoc;
        this.error = error;
        double deltaTime = (System.nanoTime() - previousTime)/NANOSECS_PER_MIN;
        i += error*deltaTime;
        d = (error - previousError)/deltaTime;
        power = (-KP*measurement) + (KI*i) + (KD*d);
        previousTime = System.nanoTime();
        previousError = error;
        return power;
    }

    public double getP(){return -KP*measurement;}
    public double getI(){return KI*i;}
    public double getD(){return KD*d;}

    public double getIntegral() {
        return i;
    }

    public double getKP() {
        return KP;
    }
    public double getKI() {
        return KI;
    }
    public double getKD() {
        return KD;
    }

    public double getMeasurement(){return measurement;}
    public double getError(){return error;}

    public void setInititalPoint(double initialPoint){
        this.initialPoint = initialPoint;
    }

    public void setKP(double KP){this.KP = KP;}
    public void setKI(double KI){this.KI = KI;}
    public void setKD(double KD){this.KD = KD;}

    public void reset(){
        i = 0;
        previousError = 0;
    }


}
