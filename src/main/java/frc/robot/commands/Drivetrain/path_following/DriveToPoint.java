// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain.path_following;

import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.DrivetrainConfig;
import frc.robot.subsystems.Drivetrain;

public class DriveToPoint extends CommandBase {
    private final Drivetrain drivetrain;
    private final Pose2d targetPose;
    private FollowPath pathCommand;

    public DriveToPoint(Drivetrain drivetrain, Pose2d targetPose) {
        this.drivetrain = drivetrain;
        this.targetPose = targetPose;
    }

    @Override
    public void initialize() {
        pathCommand = new FollowPath(drivetrain, generateTrajectory(drivetrain, targetPose));
        CommandScheduler.getInstance().schedule(pathCommand);
    }

    @Override
    public void end(boolean interrupted) {
        if (pathCommand != null) {
            pathCommand.cancel();
        }
    }

    @Override
    public boolean isFinished() {
        return pathCommand.isFinished();
    }

    public static PathPlannerTrajectory generateTrajectory(Drivetrain drivetrain, Pose2d targetPose) {
        Pose2d currentPose = drivetrain.getPose();

        Rotation2d angleToEnd = Rotation2d.fromRadians(Math.atan2(targetPose.getY() - currentPose.getY(),
            targetPose.getX() - currentPose.getX()));
        Rotation2d angleToStart = Rotation2d.fromDegrees(angleToEnd.getDegrees() + 180);
        
        PathPoint currentPoint = new PathPoint(currentPose.getTranslation(), angleToEnd, currentPose.getRotation());
        PathPoint endPoint = new PathPoint(targetPose.getTranslation(),
            angleToStart, targetPose.getRotation());
        
        return PathPlanner.generatePath(DrivetrainConfig.pathConstraints, List.of(currentPoint, endPoint));
  }
}
