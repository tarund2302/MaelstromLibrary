package ftc.library.MaelstromControl;

public class PIDPackage {
    private double dtKp, dtKi, dtKd,
            distanceKp, distanceKi,distanceKd,
            turnKp,turnKi,turnKd,
            rangeKp,rangeKi,rangeKd,
            sideKp,sideKi,sideKd ;

    public PIDPackage(double dtKp,double  dtKi,double  dtKd,
                      double distanceKp,double  distanceKi,double distanceKd,
                      double turnKp,double turnKi,double turnKd,
                      double rangeKp,double rangeKi,double rangeKd,
                      double sideKp,double sideKi,double sideKd){
        this.dtKp = dtKp;
        this.dtKi = dtKi;
        this.dtKd = dtKd;
        this.distanceKp = distanceKp;
        this.distanceKi = distanceKi;
        this.distanceKd = distanceKd;
        this.turnKp = turnKp;
        this.turnKi = turnKi;
        this.turnKd = turnKd;
        this.rangeKp = rangeKp;
        this.rangeKi = rangeKi;
        this.rangeKd = rangeKd;
        this.sideKp = sideKp;
        this.sideKi = sideKi;
        this.sideKd = sideKd;
    }
}
