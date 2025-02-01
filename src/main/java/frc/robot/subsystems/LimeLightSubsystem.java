package frc.robot.subsystems;

import frc.robot.utils.LimelightHelpers;

import java.util.Arrays;
import java.util.Optional;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.TargetLock;

public class LimeLightSubsystem extends SubsystemBase {
    // Limelight for reading AprilTags
    private final String limelightCam = "Limelight";
    private LimelightHelpers.LimelightResults result;
    private TargetLock currentLock;

    // List of Reef Tags
    private final int[] reefTags = {6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22};
    
    // Target lock buffer duration: how long we're ok with keeping an old result
    private final double bufferTime = 1.0;
    private double lastUpdateTime = 0.0;

    // Target lock on maxDistance: the farthest away in meters that we want to lock on from
    private final double maxLockOnDistance = 10;

    private double currentLockDistance = Double.MAX_VALUE;

    // NetworkTables instance and entries
    private final NetworkTable limelightTable;
    private final NetworkTableEntry apriltagIDEntry;
    private final NetworkTableEntry skewEntry;
    private final NetworkTableEntry yawEntry;
    private final NetworkTableEntry pitchEntry;
    private final NetworkTableEntry distanceEntry;

    public LimeLightSubsystem() {
        limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
        apriltagIDEntry = limelightTable.getEntry("Apriltag ID");
        skewEntry = limelightTable.getEntry("Skew");
        yawEntry = limelightTable.getEntry("Yaw");
        pitchEntry = limelightTable.getEntry("Pitch");
        distanceEntry = limelightTable.getEntry("Distance");
    }

    public double getApriltagID() {
        return currentLock != null ? currentLock.fiducialID : -1;
    }

    public double getSkew() {
        return currentLock != null ? currentLock.ts : 0.0;
    }

    public double getYaw() {
        return currentLock != null ? currentLock.tx : 0.0;
    }

    public double getPitch() {
        return currentLock != null ? currentLock.ty : 0.0;
    }

    public double get2dDistance(TargetLock target) {
        return target.distance;
    }

    public TargetLock getCurrentLock(){
        return currentLock;
    }

    @Override
    public void periodic() {
        update();
        if (currentLock != null) {
            apriltagIDEntry.setDouble(getApriltagID());
            skewEntry.setDouble(getSkew());
            yawEntry.setDouble(getYaw());
            pitchEntry.setDouble(getPitch());
            distanceEntry.setDouble(get2dDistance(currentLock));
        }
    }

    public void update() {
        double currentTime = Timer.getFPGATimestamp();
        result = LimelightHelpers.getLatestResults(limelightCam);

        // Clear our lock on if it's been too long
        if (currentLock != null && (currentTime - lastUpdateTime > bufferTime)) {
            currentLock = null;
            currentLockDistance = Double.MAX_VALUE;
        }

        // Look for the last result that has a valid target
        if (result != null && result.targets_Fiducials.length > 0) {
            for (LimelightHelpers.LimelightTarget_Fiducial target : result.targets_Fiducials) {
                if (Arrays.stream(reefTags).anyMatch(id -> id == target.fiducialID)) {
                    double distance = get2dDistance(new TargetLock(target.fiducialID, target.tx, target.ty, target.ts, target.getTargetPose_RobotSpace().getTranslation().toTranslation2d().getNorm()));
                    if (distance < maxLockOnDistance && distance < currentLockDistance) {
                        currentLock = new TargetLock(target.fiducialID, target.tx, target.ty, target.ts, distance);
                        lastUpdateTime = currentTime;
                        break;
                    }
                }
            }
        }
    }
}