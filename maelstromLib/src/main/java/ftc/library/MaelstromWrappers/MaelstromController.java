package ftc.library.MaelstromWrappers;

import com.qualcomm.robotcore.hardware.Gamepad;

/*custom gamepad class which inclues toggles*/
public class MaelstromController {
    private String name;
    private Gamepad gamepad;

    boolean currState = false;
    boolean prevState = false;
    boolean taskState = true;

    public MaelstromController(Gamepad g, String name){
        this.name = name;
        this.gamepad = g;
    }

    public enum Toggler{
        ON,
        OFF
    }

    private double currLeftStickX = 0;
    private double currLeftStickY = 0;
    private double currRightStickX = 0;
    private double currRightStickY = 0;
    private double STEP_AMOUNT = 0.16;

    public boolean toggle(boolean boolState){

        if(boolState) currState = true;
        else{
            currState = false;
            if(prevState) taskState = !taskState;
        }
        prevState = currState;

        return taskState;
    }

    public boolean a() {return gamepad.a;}
    public boolean x() {return gamepad.x;}
    public boolean y() {return gamepad.y;}
    public boolean b() {return gamepad.b;}

    public double leftStickX(){return gamepad.left_stick_x;}
    public double leftStickY() {
        return gamepad.left_stick_y;
    }
    public double rightStickX() {
        return gamepad.right_stick_x;
    }
    public double rightStickY() {
        return gamepad.right_stick_y;
    }

    public boolean leftStickXMoved(){return leftStickX() > 0;}
    public boolean leftStickYMoved(){return leftStickY() > 0;}
    public boolean rightStickXMoved(){return rightStickX() > 0;}
    public boolean rightStickYMoved(){return rightStickY() > 0;}

    public double getTan(double x, double y){
        double tan = -y/x;
        if (tan == Double.NEGATIVE_INFINITY || tan == Double.POSITIVE_INFINITY || tan != tan) tan = 0;
        return tan;
    }

    public double lazyLeftStickX() {
        if (leftStickX() < currLeftStickX) currLeftStickX -= STEP_AMOUNT;

        else if (leftStickX() > currLeftStickX) currLeftStickX += STEP_AMOUNT;

        if (Math.abs(leftStickX() - currLeftStickX) < STEP_AMOUNT) currLeftStickX = leftStickX();

        return currLeftStickX;
    }

    public double lazyLeftStickY() {
        if (leftStickY() < currLeftStickY)  currLeftStickY -= STEP_AMOUNT;
         else if (leftStickY() > currLeftStickY) currLeftStickY += STEP_AMOUNT;

        if (Math.abs(leftStickY() - currLeftStickY) < STEP_AMOUNT) currLeftStickY = leftStickY();

        return currLeftStickY;
    }

    public double lazyRightStickY(){
        if (rightStickY() < currRightStickY) currRightStickY -= STEP_AMOUNT;
        else if (rightStickY() > currRightStickY) currRightStickY += STEP_AMOUNT;

        if (Math.abs(rightStickY() - currRightStickY) < STEP_AMOUNT) currRightStickY = rightStickY();

        return currRightStickY;
    }

    public double lazyRighStickX() {
        if (rightStickX() < currRightStickX) currRightStickX -= STEP_AMOUNT;
         else if (rightStickX() > currRightStickX) currRightStickX += STEP_AMOUNT;

        if (Math.abs(rightStickX() - currRightStickX) < STEP_AMOUNT) currRightStickX = rightStickX();

        return currRightStickX;
    }

    public boolean rightStickButton(){return gamepad.right_stick_button;}
    public boolean leftStickButton(){return gamepad.left_stick_button;}

    public boolean dPadUp() {return gamepad.dpad_up;}
    public boolean dPadDown() {
        return gamepad.dpad_down;
    }
    public boolean dPadLeft() {
        return gamepad.dpad_left;
    }
    public boolean dPadRight() {
        return gamepad.dpad_right;
    }

    public boolean leftBumper() {
        return gamepad.left_bumper;
    }
    public boolean rightBumper() {
        return gamepad.right_bumper;
    }
    public double leftTrigger() {return gamepad.left_trigger;}
    public double rightTrigger() {return gamepad.right_trigger;}

    public boolean leftTriggerPressed(){return leftTrigger() > 0;}
    public boolean rightTriggerPressed(){return rightTrigger() > 0;}

    public boolean aToggle(){return toggle( a());}
    public boolean bToggle(){return toggle( b());}
    public boolean xToggle(){return toggle( x());}
    public boolean yToggle(){return toggle( y());}
    public boolean leftBumperToggle(){return toggle(leftBumper());}
    public boolean rightBumperToggle(){return toggle(rightBumper());}
    public boolean upDpadToggle(){return toggle(dPadUp());}
    public boolean downpDpadToggle(){return toggle(dPadDown());}
    public boolean leftDpadToggle(){return toggle(dPadLeft());}
    public boolean rightDpadToggle(){return toggle(dPadRight());}
    public boolean leftJoystickToggle(){return toggle(leftStickButton());}
    public boolean rightJoystickToggle(){return toggle(rightStickButton());}


}
