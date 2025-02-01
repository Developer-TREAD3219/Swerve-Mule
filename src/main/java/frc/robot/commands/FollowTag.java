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
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

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
        Optional<LimelightTarget_Fiducial> currentLock = limeLight.getCurrentLock();
        Pose2d targetPose;
        if (currentLock.isPresent()) {
            targetPose = currentLock.get()
                .getCameraPose_TargetSpace()
                .toPose2d()
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