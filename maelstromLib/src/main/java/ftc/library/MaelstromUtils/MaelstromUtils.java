package ftc.library.MaelstromUtils;

import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import ftc.library.MaelstromWrappers.MaelstromLinearOp;

/*utils class containing clipping and normalizing values*/
public class MaelstromUtils {

    private static MaelstromLinearOp linearOpMode;

    public static double clipValue (double input, double clipRange) {

        double clippedValue = Math.min(clipRange, Math.max(-clipRange, input));

        return clippedValue;
    }

    public static void clipValue (double[] input, double clipRange) {

        for (int i = 0; i < input.length; i++) {
            input[i] = Math.min(clipRange, Math.max(-clipRange, input[i]));
        }

    }

    public static double clipValueToRange (double input, double lowerRange, double upperRange) {

        double clippedValue = Math.min(upperRange, Math.max(lowerRange, input));

        return clippedValue;
    }

    public static double clipValueToRange(double input, double upperRange){
        double clippedValue = Math.min(input, upperRange);
        return clippedValue;
    }

    public static void  clipValueToRange (double[] input, double lowerRange, double upperRange) {

        for (int i = 0; i < input.length; i++) {
            input[i] = Math.min(upperRange, Math.max(lowerRange, input[i]));
        }
    }

    public static void normalizeValues (double[] values) {

        double maxValue = 0;

        for (int i = 0; i < values.length; i++) {
            maxValue = Math.max(maxValue, Math.abs(values[i]));
        }

        for (int i = 0; i < values.length; i++) {
            values[i] /= maxValue;
        }
    }

    public static void normalizeSpeedsToMax (double[] speeds, double maxSpeed) {
        double maxValue = 0;

        for (int i = 0; i < speeds.length; i++) {
            maxValue = Math.max(maxValue, Math.abs(speeds[i]));
        }

        if (maxValue > maxSpeed) {
            for (int i = 0; i < speeds.length; i++) {
                speeds[i] /= maxValue;
                speeds[i] *= maxSpeed;
            }
        }
    }

    public static void setLinearOpMode(MaelstromLinearOp pLinearOpMode) {
        linearOpMode = pLinearOpMode;
    }

    public static boolean opModeActive(){
        return linearOpMode.opModeIsActive();
    }

    public class KP {
        public static final double DT = +0.001;
        public static final double DISTANCE = +0.01;
        public static final double TURN = +0.02;
        public static final double RANGE = +0.02;
        public static final double SIDE = +0.02;
        public static final double POS = +0.02;
    }
    public class KI {
        public static final double DT = +0.003;
        public static final double DISTANCE = +0.0;
        public static final double TURN = +0.0;
        public static final double RANGE = +0.2;
        public static final double SIDE = +0.0;
        public static final double POS = +0.0;
    }
    public class KD {
        public static final double DT = +0.0;
        public static final double DISTANCE = +0.0;
        public static final double TURN = +0.0;
        public static final double RANGE = +0.0;
        public static final double SIDE = +0.0;
        public static final double POS = +0.0;
    }

    public enum AutoColor {
        RED, BLUE
    }

    public interface AutonomousOpMode {
        boolean getOpModeIsActive();

        Telemetry getTelemetry();
    }
}
