// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package main.java.frc.commands;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import java.util.Optional;
import java.subsystems.LimeLightSubsystem;
import java.subsystems.swervedrive.SwerveSubsystem;


public class FollowDemoTag extends SequentialCommandGroup {

    
  public FollowDemoTag(Drive drive, LimeLightSubsystem limeLight) {
    addCommands(
            new DriveToPose(
                    drive,
                    () -> {
                        LimelightTarget_Fiducial currentLock = limeLight.getCurrentLock();
                        if (currentLock.isEmpty()) {
                          return drive.getPose();
                        }
      
                        // Calculate robot pose with a fixed rotation
                        Pose2d targetPose = tagPose
                            .get()
                            .toPose2d()
                            .transformBy(
                                new Transform2d(
                                    new Translation2d(1.0, 0.0), 0.0));
                        return drive.driveToPose(targetPose);
                      }));
        }
      }