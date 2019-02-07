package ftc.library.MaelstromSensors;

import com.qualcomm.robotcore.util.ElapsedTime;

import ftc.library.MaelstromUtils.TimeConstants;

/*Custom class for Timer*/
public class MaelstromTimer implements TimeConstants {

    private long startTime;
    private long stopState;
    private boolean isPaused = false;
    private long pauseStart;
    private long pauseLength;
    private String name = "TIMER";
   private ElapsedTime timer = new ElapsedTime();

    public MaelstromTimer(){this.reset();}

    public void reset(){
        isPaused = false;
        pauseStart = 0L;
        pauseLength = 0L;
        startTime = System.nanoTime();
    }

    public long nanoSecs() {return System.nanoTime() - startTime - pauseLength;}
    public double milliSecs() {return nanoSecs() * NANOSECS_PER_MILISEC;}
    public double secs(){return nanoSecs() * NANOSECS_PER_SEC;}

    public enum Time{
        NANOSECS(1),
        MILLISECS(1E6),
        SECS(1E9);
        private final double multiplier;
        Time (double multiplier){this.multiplier = multiplier;}
    }

    public boolean elapsedTime(double time, Time type){
        return nanoSecs() > (long)(time*type.multiplier);
    }

    public long startTime(){
        startTime = System.nanoTime();
        //this.startTime = startTime;
        return startTime;
    }

    public long stopState(){
        stopState = (System.nanoTime() - startTime) / NANOSECS_PER_MILISEC;
        return stopState;
    }

    public void pause(){
        if (!isPaused) pauseStart = System.nanoTime();
        isPaused = true;
    }

    public void resume(){
        if (isPaused) pauseLength += (System.nanoTime() - pauseStart);
        isPaused = false;
    }

    public boolean isPause(){return isPaused;}
    public String getName(){return name;}


}
