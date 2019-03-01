package ftc.library.MaelstromSubsystems;


import com.qualcomm.robotcore.hardware.HardwareMap;

import ftc.library.MaelstromControl.MaelstromPID.PIDController;
import ftc.library.MaelstromMotions.MaelstromMotors.MaelstromMotor;
import ftc.library.MaelstromMotions.MaelstromMotors.MaelstromMotorSystem;
import ftc.library.MaelstromMotions.MaelstromMotors.MotorModel;
import ftc.library.MaelstromMotions.MaelstromServos.CRServo.MaelstromCRServo;
import ftc.library.MaelstromMotions.MaelstromServos.CRServo.MaelstromCRServoSystem;
import ftc.library.MaelstromUtils.SubsystemModels;
import ftc.library.MaelstromUtils.TimeConstants;
import ftc.library.MaelstromWrappers.MaelstromController;

public class MaelstromElevator implements TimeConstants {
    public MaelstromMotor oneMotorLift;
    public MaelstromMotorSystem twoMotorLift;
    public MaelstromCRServo oneCrLift;
    public MaelstromCRServoSystem twoCrLift;
    public MotorModel model;
    public MaelstromController controller;
    public double numOfMotors;
    public double numOfCr;
    public SubsystemModels subModel;
    public double extendPower,retractPower;
    public double kp = 0.01, ki = 0, kd = 0;
    public double counts, power;
    public double targetPosition, currPosition;
    public PIDController liftPid = new PIDController(kp,ki,kd);

    public MaelstromElevator(String name1, MotorModel model, SubsystemModels subsystem, HardwareMap hwMap){
        if(subsystem == SubsystemModels.MOTOR) {
            oneMotorLift = new MaelstromMotor(name1, model, hwMap);
            this.model = model;
            this.subModel = subsystem;
            numOfMotors = 1;
            reset();
        }
        else if(subsystem == SubsystemModels.CR){
            oneCrLift = new MaelstromCRServo(name1,hwMap);
            this.model = model;
            this.subModel = subsystem;
            numOfCr = 1;
        }
    }

    public MaelstromElevator(String name1, String name2, MotorModel model, SubsystemModels subsystem,  HardwareMap hwMap){
        if(subsystem == SubsystemModels.MOTOR){
            twoMotorLift = new MaelstromMotorSystem(name1,name2,model,hwMap);
            this.model = model;
            this.subModel = subsystem;
            numOfMotors = 2;
            reset();
        }
        else if(subsystem == SubsystemModels.CR){
            twoCrLift = new MaelstromCRServoSystem(name1,name2,hwMap);
            this.model = model;
            this.subModel = subsystem;
            numOfCr = 2;
        }
    }

    public void DriverControl(MaelstromController controller) {
        this.controller = controller;
        if(controller.leftTriggerPressed()) extend();
        else if(controller.rightTriggerPressed()) retract();
        else stop();

        if(subModel == SubsystemModels.MOTOR) liftPid();
    }

    public void liftPid(){
        if(controller.leftTriggerPressed() || controller.rightTriggerPressed()) targetPosition = getCounts();
        else {
            currPosition = getCounts();
            double pidPower = liftPid.power(currPosition, targetPosition);
            setPower(pidPower);
        }
        liftPid.setPID(kp,ki,kd);
    }

    public void extend(){
        setPower(extendPower);
    }

    public void retract(){
        setPower(retractPower);
    }

    public void stop(){
        setPower(0);
    }

    public void setLiftPowers(double extendPower, double retractPower){
        this.extendPower = extendPower;
        this.retractPower = retractPower;
    }

    public void setPower(double power){
        if(numOfMotors == 1) oneMotorLift.setPower(power);
        else if(numOfMotors == 2) twoMotorLift.setPower(power);

        if(numOfCr == 1) oneCrLift.setPower(power);
        else if(numOfCr == 2) twoCrLift.setPower(power);
    }

    public double getPower(){
        if(numOfMotors == 1) power = oneMotorLift.getPower();
        else if(numOfMotors == 2) power = twoMotorLift.getPower();

        if(numOfCr == 1) power = oneCrLift.getPower();
        else if(numOfCr == 2) power = twoCrLift.getPower();
        return power;
    }

    public double getCounts(){
        if(numOfMotors == 1) counts = oneMotorLift.getCounts();
        else if(numOfMotors == 2) counts = twoMotorLift.getCounts();
        return counts;
    }

    public void reset(){
        if(numOfMotors == 1){
            oneMotorLift.stopAndReset();
            oneMotorLift.runWithoutEncoders();
        }
        else if(numOfMotors == 2){
            twoMotorLift.stopAndReset();
            twoMotorLift.runWithoutEncoders();
        }
    }

    public void setPID(double kp, double ki, double kd){
        this.kp = kp;
        this.ki = ki;
        this.kd = kd;
    }

/*    public void setButtons(double trigger1, double trigger2, boolean button1, boolean button2){
        if(trigger1 != 0){
            this.trigger1 = trigger1;
        }
        else if(trigger2 != 0){
            this.trigger2 = trigger2;
        }
    }*/

}
