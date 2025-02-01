package frc.robot.subsystems;

import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.LimelightHelpers.LimelightTarget_Fiducial;

import java.util.Arrays;
import java.util.Optional;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Constants.VisionConstants;

public class LimeLightSubsystem extends SubsystemBase {
    // Limelight for reading AprilTags
    private final String limelightCam = "Limelight";
    private LimelightHelpers.LimelightResults result;
    private LimelightHelpers.LimelightTarget_Fiducial currentLock;

    // List of Reef Tags
    private final int[] reefTags = {6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22};
    
    // Target lock buffer duration: how long we're ok with keeping an old result
    private final double bufferTime = 1.0;
    private double lastUpdateTime = 0.0;

    // Target lock on maxDistance: the farthest away in meters that we want to lock on from
    private final double maxLockOnDistance = 10;

    private double currentLockDistance = Double.MAX_VALUE;

    public LimeLightSubsystem() {
        // Initialize Limelight settings if needed
    Shuffleboard.getTab("Vision").addBoolean("hasReefTarget", () -> false);
    }

    public void driverMode() {
        LimelightHelpers.setLEDMode_ForceOff(limelightCam);
        LimelightHelpers.setStreamMode_Standard(limelightCam);
    }

    public void detectAprilTags() {
        LimelightHelpers.setLEDMode_ForceOff(limelightCam);
        LimelightHelpers.setPipelineIndex(limelightCam, 0);
    }

    public Optional<LimelightTarget_Fiducial> getCurrentLock() {
        return Optional.ofNullable(currentLock);
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
                    currentLock = target;
                    lastUpdateTime = currentTime;
                    break;
                }
            }
        }
    }

    public void attemptReefLockon() {
        if (result != null && result.targets_Fiducials.length > 0) {
            for (LimelightHelpers.LimelightTarget_Fiducial target : result.targets_Fiducials) {
                if (Arrays.stream(reefTags).anyMatch(id -> id == target.fiducialID)) {
                    double distance = get2dDistance(target);
                    if (distance < maxLockOnDistance && distance < currentLockDistance) {
                        currentLock = target;
                        currentLockDistance = distance;
                    }
                }
            }
        }
    }

    public int getApriltagID() {
        return currentLock != null ? (int) currentLock.fiducialID : -1;
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

    // public boolean isAmbiguousPose() {
    //     return currentLock != null && currentLock.ambiguity > 0.1;
    // }

    public double get2dDistance(LimelightTarget_Fiducial target) {
        return target.getTargetPose_RobotSpace().getTranslation().toTranslation2d().getNorm();
    }

    @Override
    public void periodic() {
        update();
        if (currentLock != null) {
            Shuffleboard.getTab("Vision").add("Apriltag ID", getApriltagID());
            Shuffleboard.getTab("Vision").add("Skew", getSkew());
            Shuffleboard.getTab("Vision").add("Yaw", getYaw());
            Shuffleboard.getTab("Vision").add("Pitch", getPitch());
            // Shuffleboard.getTab("Vision").add("Ambiguous Pose", isAmbiguousPose());
            Shuffleboard.getTab("Vision").add("Distance", get2dDistance(currentLock));
        }
        // This method will be called once per scheduler run
    }
}