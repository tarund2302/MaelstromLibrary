package ftc.library.MaelstromSensors;


import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

/*Class for Maxbotix ultrasonic sensor*/
public class MaxbotixUltrasonicSensor implements DistanceSensor  {

    private final AnalogInput rangeSensor;

    public MaxbotixUltrasonicSensor(AnalogInput rangeSensor){
        this.rangeSensor = rangeSensor;
    }

    public double getMaxVoltage(){
        final double sensorMaxVoltage = 3.3;
        return Math.min(sensorMaxVoltage, rangeSensor.getMaxVoltage());
    }

    public double mmDistance() {
        double reading =  Range.clip(readRawVoltage(), 0, getMaxVoltage());
        return (reading/(getMaxVoltage()/1023))*5;
    }

    public double readRawVoltage(){
        return rangeSensor.getVoltage();
    }


    @Override
    public double getDistance(DistanceUnit unit) {
        double mm = mmDistance();
        return unit.fromUnit(DistanceUnit.MM, mm);
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Adafruit;
    }

    @Override
    public String getDeviceName() {
        return AppUtil.getDefContext().getString(com.qualcomm.robotcore.R.string.configTypeOpticalDistanceSensor);
    }

    @Override
    public String getConnectionInfo() {
        return rangeSensor.getConnectionInfo();
    }

    @Override
    public int getVersion() {
        return 0;
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {

    }

    @Override
    public void close() {

    }
}
