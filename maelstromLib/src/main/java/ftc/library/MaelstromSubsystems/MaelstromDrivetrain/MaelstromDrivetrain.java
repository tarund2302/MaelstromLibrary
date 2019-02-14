package ftc.library.MaelstromSubsystems.MaelstromDrivetrain;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import ftc.library.MaelstromMotions.MaelstromMotors.MaelstromMotorSystem;
import ftc.library.MaelstromMotions.MaelstromMotors.MotorModel;
import ftc.library.MaelstromSensors.MaelstromEncoder;

public class MaelstromDrivetrain {

    public MaelstromMotorSystem leftDrive, rightDrive = null;
    public double driveGearReduction;
    public double drivenGearReduction;
    public DrivetrainModels model;
    public MaelstromEncoder encoderSensor;

    public MaelstromDrivetrain(String name1, String name2, String name3, String name4, HardwareMap hwMap, MotorModel encoder) {
        leftDrive = new MaelstromMotorSystem(name1, name2, DcMotorSimple.Direction.REVERSE, hwMap, encoder);
        rightDrive = new MaelstromMotorSystem(name3, name4, DcMotorSimple.Direction.FORWARD, hwMap, encoder);
    }

    public MaelstromDrivetrain(DrivetrainModels model, double gearRatio, double Kp, double Ki, double Kd, HardwareMap hwMap, MotorModel type) {
        //leftDrive = new MotorSystem("leftFront", "leftBack", "Left Drive",DcMotor.Direction.REVERSE, "LEFTDRIVE", hwMap, type);
        leftDrive = new MaelstromMotorSystem("leftFront", "leftBack", Kp, Ki, Kd, DcMotor.Direction.REVERSE, hwMap, type);
        rightDrive = new MaelstromMotorSystem("rightFront", "rightBack", Kp, Ki, Kd, DcMotor.Direction.FORWARD, hwMap, type);
        leftDrive.setGearRatio(gearRatio);
        rightDrive.setGearRatio(gearRatio);
        driveGearReduction = (1 / gearRatio);
        drivenGearReduction = gearRatio;
        this.model = model;
    }

    public void eReset() {
        leftDrive.stopAndReset();
        leftDrive.runWithoutEncoders();
        rightDrive.stopAndReset();
        rightDrive.runWithoutEncoders();
    }

    public void setPower(double power) {
        leftDrive.setPower(power);
        rightDrive.setPower(power);
    }

    public void setPower(double left, double right) {
        leftDrive.setPower(left);
        rightDrive.setPower(right);
    }

    public void setVelocity(double velocity) {
        leftDrive.setVelocity(velocity);
        rightDrive.setVelocity(velocity);
    }

    public void setVelocity(double left, double right) {
        leftDrive.setVelocity(left);
        rightDrive.setVelocity(right);
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior){
        leftDrive.setZeroPowerBehavior(behavior);
        rightDrive.setZeroPowerBehavior(behavior);
    }

    public double getPower() {
        return (leftDrive.getPower() + rightDrive.getPower()) / 2;
    }

    public double getCounts() {
        return leftDrive.getCounts();
    }

    public double getDriveGearReduction() {
        return driveGearReduction;
    }

    public double getDrivenGearReduction() {
        return drivenGearReduction;
    }

    public double getRawInches() {
        return leftDrive.getInches();
    }

    public double getTotalInches() {
        return (leftDrive.getInches() + rightDrive.getInches()) / 2;
    }

    public double getInches() {
        return (leftDrive.getInches() + rightDrive.getInches()) / 2;
    }

    public double getWheelDiameter() {
        return encoderSensor.getWheelDiameter();
    }

    public double setWheelDiameter() {
        return encoderSensor.getWheelDiameter();
    }

    public DrivetrainModels getModel() {
        return model;
    }

    public void setPID(double kp, double ki, double kd) {
        leftDrive.setPID(kp, ki, kd);
        rightDrive.setPID(kp, ki, kd);
    }
}
