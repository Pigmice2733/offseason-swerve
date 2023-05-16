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
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.DrivetrainConfig;
import frc.robot.commands.RumbleController;
import frc.robot.subsystems.Drivetrain;

public class DriveToPoint extends CommandBase {
    private final Drivetrain drivetrain;
    private final Pose2d targetPose;
    private final XboxController controllerToRumble;

    private FollowPath pathCommand;

    public DriveToPoint(Drivetrain drivetrain, Pose2d targetPose, XboxController controllerToRumble) {
        this.drivetrain = drivetrain;
        this.targetPose = targetPose;
        this.controllerToRumble = controllerToRumble;
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
        CommandScheduler.getInstance().schedule(new RumbleController(controllerToRumble, RumbleType.kBothRumble, 1, 1));
    }

    @Override
    public boolean isFinished() {
        return pathCommand.isFinished();
    }

    public static PathPlannerTrajectory generateTrajectory(Drivetrain drivetrain, Pose2d targetPose) {
        Pose2d currentPose = drivetrain.getPose();

        // Angle facing the end point when at the current point
        Rotation2d angleToEnd = Rotation2d.fromRadians(Math.atan2(targetPose.getY() - currentPose.getY(),
            targetPose.getX() - currentPose.getX()));

        // Angle facing the current point when at the end point
        Rotation2d angleToStart = Rotation2d.fromDegrees(angleToEnd.getDegrees() + 180);
        
        PathPoint currentPoint = new PathPoint(currentPose.getTranslation(), angleToEnd, currentPose.getRotation());
        PathPoint endPoint = new PathPoint(targetPose.getTranslation(), angleToStart, targetPose.getRotation());
        
        return PathPlanner.generatePath(DrivetrainConfig.PATH_CONSTRAINTS, List.of(currentPoint, endPoint));
  }
}
