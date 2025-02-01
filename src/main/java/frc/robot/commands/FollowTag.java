// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import java.util.Optional;
import frc.robot.utils.LimelightHelpers.LimelightTarget_Fiducial;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.LimeLightSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.utils.LimelightHelpers;

public class FollowTag extends SequentialCommandGroup {

  public FollowTag(SwerveSubsystem drive, LimeLightSubsystem limeLight) {
      Optional<LimelightTarget_Fiducial> currentLock = limeLight.getCurrentLock();
      Pose2d targetPose;
      if (currentLock.isPresent()) {
          targetPose = currentLock.get()
              .getCameraPose_TargetSpace()
              .toPose2d() //TODO: This is probably wrong (problem for Mr Berry)
              .transformBy(new Transform2d(new Translation2d(1.0, 0.0), new Rotation2d(0.0)));
      } else {
          targetPose = drive.getPose();
      }
      addCommands(drive.driveToPose(targetPose));
  }
}