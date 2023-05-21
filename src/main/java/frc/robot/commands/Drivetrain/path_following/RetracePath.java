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

import org.opencv.core.Point;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.DrivetrainConfig;
import frc.robot.subsystems.Drivetrain;

public class RetracePath extends CommandBase {
    private final Drivetrain drivetrain;

    public RetracePath(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
    }

    Timer timer;
    TimerTask task;
    ArrayList<Pose2d> positions;

    @Override
    public void initialize() {
        positions = new ArrayList<Pose2d>();

        // timer.cancel();
        // timer.purge();

        task = new TimerTask() {
            @Override
            public void run() {
                recordPosition();
            }
        };

        timer = new Timer();
        timer.scheduleAtFixedRate(task, 0, 500);
    }

    public void recordPosition() {
        positions.add(drivetrain.getPose());
        System.out.println("Timer Task Called");
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
        timer.cancel();
        timer.purge();
        task.cancel();

        if (positions.size() < 2) return;

        Collections.reverse(positions);

        ArrayList<PathPoint> points = new ArrayList<PathPoint>();
        for (int i = 0; i < positions.size()-1; i++) {
            Pose2d current = positions.get(i);
            Pose2d next = positions.get(i+1);
            Rotation2d angleToNext = Rotation2d.fromRadians(Math.atan2(
                next.getY() - current.getY(),
                next.getX() - current.getX()));
        
            points.add(new PathPoint(current.getTranslation(), angleToNext, current.getRotation()));
        }

        points.add(new PathPoint(
            positions.get(positions.size()-1).getTranslation(), 
            points.get(points.size()-1).heading,
            positions.get(positions.size()-1).getRotation()));

        PathPlannerTrajectory trajectory = PathPlanner.generatePath(DrivetrainConfig.PATH_CONSTRAINTS, points);
        CommandScheduler.getInstance().schedule(new FollowPath(drivetrain, trajectory));
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
