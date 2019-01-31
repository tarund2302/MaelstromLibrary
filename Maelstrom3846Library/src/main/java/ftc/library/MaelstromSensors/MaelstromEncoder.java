package ftc.library.MaelstromSensors;

import ftc.library.MaelstromMotors.MaelstromMotor;

public class MaelstromEncoder {
    private MotorType type;
    private MaelstromMotor motor;
    private double wheelDiameter, gearRatio;
    private double currCounts, zeroPos;
    private double countsPerInch;

    public MaelstromEncoder(MaelstromMotor motor, MotorType type){
        countsPerInch = (type.CPR() / (wheelDiameter * Math.PI)) * gearRatio;
        this.type = type;
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

    public double getCPR(){ return type.CPR(); }
    public double getRPM(){return type.RPM();}

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
    public void setType(MotorType type){ this.type = type;}
    public double getWheelDiameter(){return wheelDiameter;}
    public double getGearRatio(){return gearRatio;}

}
