// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain.path_following;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.pathfinder.Pathfinder;
import frc.robot.pathfinder.PathfinderResult;
import frc.robot.subsystems.Drivetrain;

public class PathfindToPoint extends CommandBase {
    private final Drivetrain drivetrain;
    private final Pathfinder pathfinder;
    private final Pose2d goalPose;

    private FollowPath pathCommand;

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
        pathCommand = new FollowPath(drivetrain, result.getAsTrajectory());
        pathCommand.schedule();
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) { 
        if (pathCommand != null) {
           pathCommand.cancel();
        }
    }

    @Override
    public boolean isFinished() {
        return pathCommand == null || pathCommand.isFinished();
    }
}
