// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain.path_following;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Timer;
import java.util.TimerTask;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.DrivetrainConfig;
import frc.robot.pathfinder.Pathfinder;
import frc.robot.pathfinder.PathfinderResult;
import frc.robot.subsystems.Drivetrain;

public class PathfindToPoint extends CommandBase {
    private final Drivetrain drivetrain;
    private final Pathfinder pathfinder;
    private final Pose2d goalPose;

    private FollowPath pathCommed;

    public PathfindToPoint(Drivetrain drivetrain, Pathfinder pathfinder, Pose2d goalPose) {
        this.drivetrain = drivetrain;
        this.pathfinder = pathfinder;
        this.goalPose = goalPose;
    }


    @Override
    public void initialize() {
        PathfinderResult result = pathfinder.FindPath(drivetrain.getPose().getTranslation(), goalPose.getTranslation());
        
        if (!result.pathFound) {
            end(true);
            return;
        }

        pathCommed = new FollowPath(drivetrain, result.getAsTrajectory());
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {

        
    }

    @Override
    public boolean isFinished() {
        return pathCommed.isFinished();
    }
}
