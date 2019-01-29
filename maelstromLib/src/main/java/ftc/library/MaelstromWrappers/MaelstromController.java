package ftc.library.MaelstromWrappers;

import com.qualcomm.robotcore.hardware.Gamepad;

public class MaelstromController extends Gamepad {
    private String name;
    private Gamepad gamepad;

    boolean currState = false;
    boolean prevState = false;
    boolean taskState = true;

    public MaelstromController(Gamepad g, String name){
        this.name = name;
        this.gamepad = g;
    }


    public boolean toggle(boolean boolState){

        if(boolState){
            currState = true;
        }
        else{
            currState = false;
            if(prevState){
                taskState = !taskState;
            }
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
