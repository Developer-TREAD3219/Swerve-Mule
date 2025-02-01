package frc.robot.subsystems;

public class TargetLock {
    public double fiducialID;
    public double tx;
    public double ty;
    public double ts;
    public double distance;

    public TargetLock(double fiducialID, double tx, double ty, double ts, double distance) {
        this.fiducialID = fiducialID;
        this.tx = tx;
        this.ty = ty;
        this.ts = ts;
        this.distance = distance;
    }
}