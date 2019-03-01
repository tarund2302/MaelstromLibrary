package ftc.library.MaelstromMotions.MaelstromMotors;

/*custom class for direction of motors*/
public enum Direction {
    FORWARD (+1.0),
    BACKWARD(-1.0),
    LEFT(+1.0),
    RIGHT(-1.0),
    UNKNOWN(+0.0),
    IN(-1.0),
    OUT(+1.0),
    DEFAULT(+1.0),
    REVERSE(-1.0);
    public final double value;
    Direction (double value){this.value = value;}
}
