package ftc.library.MaelstromSensors;

import ftc.library.MaelstromMotions.MaelstromMotors.MaelstromMotor;
import ftc.library.MaelstromMotions.MaelstromMotors.MotorModel;

public class MaelstromEncoder {
    private MotorModel model;
    private MaelstromMotor motor;
    private double wheelDiameter, gearRatio;
    private double currCounts, zeroPos;
    private double countsPerInch;

    public MaelstromEncoder(MaelstromMotor motor, MotorModel model){
        countsPerInch = (model.CPR() / (wheelDiameter * Math.PI)) * gearRatio;
        this.model = model;
        this.motor = motor;
    }

    public double getCountsPerInch(){
        return countsPerInch;
    }

    public double getRelativePosition(){
        currCounts = (int) (getPosition() - zeroPos);
        return currCounts;
    }

    public double getPosition(){
        return motor.getCounts();
    }

    public double getInches(){
        return ((Math.PI * wheelDiameter * getRelativePosition()) / getCPR()) * gearRatio;
    }

    public double getCPR(){ return model.CPR(); }
    public double getRPM(){return model.RPM();}

    public void resetEncoder(){
        zeroPos = (int) getPosition();
        currCounts = 0;
    }

    public double getWheelCircumference(){
        double circumference = getWheelDiameter() * Math.PI;
        return circumference;
    }

    public void setWheelDiameter(double wheelDiameter){this.wheelDiameter = wheelDiameter;}
    public void setGearRatio(double gearRatio){this.gearRatio = gearRatio;}
    public void setType(MotorModel model){ this.model = model;}
    public double getWheelDiameter(){return wheelDiameter;}
    public double getGearRatio(){return gearRatio;}

}
