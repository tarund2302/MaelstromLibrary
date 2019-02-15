package ftc.library.MaelstromSubsystems.MaelstromElevators;


import com.qualcomm.robotcore.hardware.HardwareMap;

import ftc.library.MaelstromControl.PIDController;
import ftc.library.MaelstromMotions.MaelstromMotors.MaelstromMotor;
import ftc.library.MaelstromMotions.MaelstromMotors.MotorModel;
import ftc.library.MaelstromUtils.TimeConstants;
import ftc.library.MaelstromWrappers.MaelstromController;

public class MaelstromOneMotorElevator implements TimeConstants {
    public MaelstromMotor lift;
    public MotorModel model;
    public double extendPower,retractPower;
    public double kp, ki, kd;
/*    public double trigger1, trigger2;
    public boolean button1, button2;*/
    public double targetPosition, currPosition;
    public PIDController liftPid = new PIDController(kp,ki,kd);

    public MaelstromOneMotorElevator(String name1, MotorModel model, HardwareMap hwMap){
        lift = new MaelstromMotor(name1,model,hwMap);
        this.model = model;
        lift.stopAndReset();
        lift.runWithoutEncoders();
    }

    public void DriverControl(MaelstromController controller) {
        //kp = (5e-7 * -getCounts()) + 0.001;
        if(controller.leftTriggerPressed()) extend();
        else if(controller.rightTriggerPressed()) retract();
        else stop();

        if(controller.leftTriggerPressed() || controller.rightTriggerPressed()) targetPosition = getCounts();
        else {
            currPosition = getCounts();
            double pidPower = liftPid.power(currPosition, targetPosition);
            lift.setPower(pidPower);
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
        lift.setPower(power);
    }

    public double getPower(){
        return lift.getPower();
    }

    public int getCounts(){
        return lift.getCounts();
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
