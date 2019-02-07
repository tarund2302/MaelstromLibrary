package ftc.library.MaelstromSensors;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;

import ftc.library.MaelstromUtils.MaelstromUtils;

/*Custon class for gyro imu*/
public class MaelstromIMU implements Runnable {

    private double relativeYaw = 0;
    private double lastAngle = 0;
    private final BNO055IMU imu;
    private MaelstromUtils.AutonomousOpMode auto;

    public MaelstromIMU(String name, HardwareMap hwMap){
        imu = hwMap.get(BNO055IMU.class,name);
        setParameters();
        // auto = hardware.auto;
        Thread updateYaw = new Thread(this);
        updateYaw.start();
    }

    private void setParameters(){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.useExternalCrystal = true;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.pitchMode = BNO055IMU.PitchMode.WINDOWS;
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        imu.initialize(parameters);
    }

    public void updateRelativeYaw(){
        if (lastAngle > 90 && getAngles()[0] < 0) {
            relativeYaw = 180 * Math.round(relativeYaw/180) + (180 + getAngles()[0] );
        }
        else if (lastAngle < -90 && getAngles()[0]  > 0) {
            relativeYaw = 180 * Math.round(relativeYaw/180) - (180 - getAngles()[0] );
        }
        else if (Math.abs(relativeYaw) <= 180) {
            relativeYaw = getAngles()[0];
        }
        else {
            relativeYaw += getAngles()[0]  - lastAngle;
        }
        lastAngle = getAngles()[0];
    }

    public double getRelativeYaw()
    {
        return relativeYaw;
    }

    public double[] getAngles(){
        Quaternion quatAngles = imu.getQuaternionOrientation();

        double w = quatAngles.w;
        double x = quatAngles.x;
        double y = quatAngles.y;
        double z = quatAngles.z;

        double roll = Math.atan2( 2*(w*x + y*z) , 1 - (2*(x*x + y*y)) ) * 180.0 / Math.PI;
        double pitch = Math.asin( 2*(w*y - x*z) ) * 180.0 / Math.PI;
        double yaw = Math.atan2( 2*(w*z + x*y), 1 - (2*(y*y + z*z)) ) * 180.0 / Math.PI;

        return new double[]{yaw,pitch,roll};
    }

    public void resetAngle(){
        relativeYaw = 0;
    }


    public double adjustAngle(double angle){
        while (angle > 180) angle -= 360;
        while (angle <= -180) angle += 360;
        return angle;
    }

    public double getYaw() {return getAngles()[0];}

    public double getRoll(){return getAngles()[1];}

    public double getPitch(){return getAngles()[2];}

    public void run(){
        if(auto == null){
            return;
        }
        while (!auto.getOpModeIsActive()) {
            try {
                Thread.sleep(50);
            }
            catch (InterruptedException e) {

            }
        }

        while (auto.getOpModeIsActive()) {
            updateRelativeYaw();
            try {
                Thread.sleep(10);
            }
            catch (InterruptedException e) {
            }
        }
    }

    public String dataOutput() {
        return String.format("Yaw: %.3f  Pitch: %.3f  Roll: %.3f", getAngles()[0], getAngles()[1], getAngles()[2]);
    }

}
