package ftc.library.MaelstromMotions.MaelstromMotors;

/*class for motor types and its rpm and cpr*/
public enum  MotorModel {

    ORBITAL20, NEVEREST_3_7, E4T_ENCODER, NEVEREST40, NEVEREST60, USDIGITAL_E4T, REVHDHEX,NEVEREST_NAKED, NONE;
    public static double DEFAULT_CPR = 2240;
    public double CPR(MotorModel motorModel) {
        switch (motorModel){
            case ORBITAL20:
                return 537.6;
            case NEVEREST_3_7:
                return 103;
            case E4T_ENCODER:
                return  537.6;
            case NEVEREST40:
                return 1120;
            case NEVEREST60:
                return 1680;
            case USDIGITAL_E4T:
                return 1440;
            case REVHDHEX:
                return 1120;
            case NEVEREST_NAKED:
                return 28;
        }
        return DEFAULT_CPR;
    }
    public double CPR () {
        return CPR(this);
    }
    public static int DEFAULT_RPM = 150;
    public static int RPM(MotorModel motorModel) {
        switch (motorModel) {
            case ORBITAL20:
                return 340;
            case NEVEREST_3_7:
                return 1784;
            case NEVEREST40:
                return 160;
            case NEVEREST60:
                return 105;
            case REVHDHEX:
                return 150;
        }
        return DEFAULT_RPM;
    }
    public int RPM () {
        return RPM(this);
    }
}
