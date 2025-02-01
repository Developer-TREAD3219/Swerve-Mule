package frc.robot.commands;

import java.util.Optional;
import frc.robot.utils.LimelightHelpers.LimelightTarget_Fiducial;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.TargetLock;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class FollowTag extends Command {
    private final SwerveSubsystem drive;
    private final LimeLightSubsystem limeLight;

    public FollowTag(SwerveSubsystem drive, LimeLightSubsystem limeLight) {
        this.drive = drive;
        this.limeLight = limeLight;
        addRequirements(drive, limeLight);
    }

    @Override
    public void initialize() {
        SmartDashboard.putBoolean("FollowTag Running", true);
    }

    @Override
    public void execute() {
        NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry apriltagIDEntry = limelightTable.getEntry("Apriltag ID");
        NetworkTableEntry txEntry = limelightTable.getEntry("tx");
        NetworkTableEntry tyEntry = limelightTable.getEntry("ty");
        NetworkTableEntry tsEntry = limelightTable.getEntry("ts");
        NetworkTableEntry distanceEntry = limelightTable.getEntry("Distance");

        double apriltagID = apriltagIDEntry.getDouble(-1);
        double tx = txEntry.getDouble(0.0);
        double ty = tyEntry.getDouble(0.0);
        double ts = tsEntry.getDouble(0.0);
        double distance = distanceEntry.getDouble(0.0);

        Pose2d targetPose;
        if (apriltagID != -1) {
            // Assuming getCameraPose_TargetSpace() and toPose2d() are methods to convert the data to Pose2d
            targetPose = new Pose2d(tx, ty, new Rotation2d(ts))
                .transformBy(new Transform2d(new Translation2d(1.0, 0.0), new Rotation2d(0.0)));
        } else {
            targetPose = drive.getPose();
        }
        drive.driveToPose(targetPose).schedule();
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putBoolean("FollowTag Running", false);
    }

    @Override
    public boolean isFinished() {
        return false; // Run until explicitly interrupted
    }
}