package ftc.library.MaelstromControl.MaelstromPID;

import ftc.library.MaelstromUtils.MaelstromUtils;

/*custom class for pid constants*/
public class PIDPackage {
    private double dtKp, dtKi, dtKd,
            distanceKp, distanceKi,distanceKd,
            turnKp,turnKi,turnKd,
            rangeKp,rangeKi,rangeKd,
            sideKp,sideKi,sideKd,
            drivePosKp, drivePosKi, drivePosKd ;

    public PIDPackage(double dtKp,double  dtKi,double  dtKd,
                      double distanceKp,double  distanceKi,double distanceKd,
                      double turnKp,double turnKi,double turnKd,
                      double rangeKp,double rangeKi,double rangeKd,
                      double sideKp,double sideKi,double sideKd,
                      double drivePosKp, double drivePosKi, double drivePosKd){
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
        this.drivePosKp = drivePosKp;
        this.drivePosKi = drivePosKi;
        this.drivePosKd = drivePosKd;
    }

    public PIDPackage(){
        this.dtKp = MaelstromUtils.KP.DT;
        this.dtKi = MaelstromUtils.KI.DT;
        this.dtKd = MaelstromUtils.KD.DT;
        this.distanceKp = MaelstromUtils.KP.DISTANCE;
        this.distanceKi = MaelstromUtils.KI.DISTANCE;
        this.distanceKd = MaelstromUtils.KD.DISTANCE;
        this.turnKp = MaelstromUtils.KP.TURN;
        this.turnKi = MaelstromUtils.KI.TURN;
        this.turnKd = MaelstromUtils.KD.TURN;
        this.rangeKp = MaelstromUtils.KP.RANGE;
        this.rangeKi = MaelstromUtils.KI.RANGE;
        this.rangeKd = MaelstromUtils.KD.RANGE;
        this.sideKp = MaelstromUtils.KP.SIDE;
        this.sideKi = MaelstromUtils.KI.SIDE;
        this.sideKd = MaelstromUtils.KD.SIDE;
        this.drivePosKp = MaelstromUtils.KP.POS;
        this.drivePosKi = MaelstromUtils.KI.POS;
        this.drivePosKd = MaelstromUtils.KD.POS;
    }

    public double getDtKp(){
        return dtKp;
    }
    public double getDtKi(){
        return dtKi;
    }
    public double getDtKd(){
        return dtKd;
    }

    public void setDtKp(double dtKp) {
        this.dtKp = dtKp;
    }
    public void setDtKi(double dtKi) {
        this.dtKi = dtKi;
    }
    public void setDtKd(double dtKd) {
        this.dtKd = dtKd;
    }

    public double getDistanceKp(){
        return distanceKp;
    }
    public double getDistanceKi(){
        return distanceKi;
    }
    public double getDistanceKd(){
        return distanceKd;
    }

    public void setDistanceKp(double distanceKp) {
        this.distanceKp = distanceKp;
    }
    public void setDistanceKi(double distanceKi) {
        this.distanceKi = distanceKi;
    }
    public void setDistanceKd(double distanceKd) {
        this.distanceKd = distanceKd;
    }

    public double getTurnKp(){return turnKp;}
    public double getTurnKi(){return turnKi;}
    public double getTurnKd(){return turnKd;}

    public void setTurnKp(double turnKp) {
        this.turnKp = turnKp;
    }
    public void setTurnKi(double turnKi) {
        this.turnKi = turnKi;
    }
    public void setTurnKd(double turnKd) {
        this.turnKd = turnKd;
    }

    public double getRangeKp() {
        return rangeKp;
    }
    public double getRangeKi() {
        return rangeKi;
    }
    public double getRangeKd() {
        return rangeKd;
    }

    public void setRangeKp(double rangeKp) {
        this.rangeKp = rangeKp;
    }
    public void setRangeKi(double rangeKi) {
        this.rangeKi = rangeKi;
    }
    public void setRangeKd(double rangeKd) {
        this.rangeKd = rangeKd;
    }

    public double getSideKp() {
        return sideKp;
    }
    public double getSideKi() {
        return sideKi;
    }
    public double getSideKd() {
        return sideKd;
    }
    public void setSideKp(double sideKp) {
        this.sideKp = sideKp;
    }
    public void setSideKi(double sideKi) {
        this.sideKi = sideKi;
    }
    public void setSideKd(double sideKd) {
        this.sideKd = sideKd;
    }
    public double getDrivePosKp(){
        return drivePosKp;
    }
    public double getDrivePosKi(){
        return drivePosKi;
    }
    public double getDrivePosKd(){
        return drivePosKd;
    }
    public void setDrivePosKp(double drivePosKp){
        this.drivePosKp = drivePosKp;
    }
    public void setDrivePosKi(double drivePosKi){
        this.drivePosKp = drivePosKi;
    }
    public void setDrivePosKd(double drivePosKd){
        this.drivePosKp = drivePosKd;
    }

    public PIDPackage getPIDPackage(){
        return this;
    }

}
