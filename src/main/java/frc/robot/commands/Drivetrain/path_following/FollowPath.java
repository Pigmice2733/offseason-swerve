package frc.robot.commands.drivetrain.path_following;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import java.util.HashMap;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.DrivetrainConfig;
import frc.robot.subsystems.Drivetrain;

public class FollowPath extends SequentialCommandGroup {

    /**
     * Use a SwerveController to follow a specified path.
     * 
     * @param drivetrain a drivetrain subsystem
     * @param trajectory a path-following trajectory
     */
    public FollowPath(Drivetrain drivetrain, PathPlannerTrajectory trajectory) {
        addCommands(
            drivetrain.resetOdometryCommand(trajectory.getInitialHolonomicPose()),
            new PPSwerveControllerCommand(
                trajectory,
                drivetrain::getPose,
                DrivetrainConfig.kinematics,
                new PIDController(0, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                new PIDController(0, 0, 0), // Y controller (usually the same values as X controller)
                new PIDController(0, 0, 0), 
                (output) -> drivetrain.driveModuleStates(output), 
                drivetrain
            )
        );
        addRequirements(drivetrain);
    }

    /**
     * Use a SwerveController to follow a specified path.
     * @param drivetrain a drivetrain subsystem
     * @param trajectory a path-following trajectory
     * @param eventMap commands to execute at certain events along the path (configure events in Path Planner)
     */
    public FollowPath(Drivetrain drivetrain, PathPlannerTrajectory trajectory, HashMap<String, Command> eventMap) {
        addCommands(
            drivetrain.resetOdometryCommand(trajectory.getInitialHolonomicPose()),
            new FollowPathWithEvents(
                new PPSwerveControllerCommand(
                    trajectory,
                    drivetrain::getPose,
                    DrivetrainConfig.kinematics,
                    new PIDController(0, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                    new PIDController(0, 0, 0), // Y controller (usually the same values as X controller)
                    new PIDController(0, 0, 0), 
                    (output) -> drivetrain.driveModuleStates(output), 
                    drivetrain
                ),
                trajectory.getMarkers(),
                eventMap
            )
        );
        addRequirements(drivetrain);
    }

    /**
     * Use a SwerveController to follow a specified path.
     * 
     * @param drivetrain a drivetrain subsystem
     * @param pathName the name of a premade path to follow
     * @param reversed reverse the robots direction
     */
    public FollowPath(Drivetrain drivetrain, String pathName, boolean reversed) {
        this(
                drivetrain,
                PathPlanner.loadPath(
                        pathName,
                        DrivetrainConfig.PATH_CONSTRAINTS,
                        reversed));
    }

    /**
     * Use a SwerveController to follow a specified path.
     * 
     * @param drivetrain a drivetrain subsystem
     * @param pathName the name of a premade path to follow
     * @param eventMap commands to execute at certain events along the path (configure events in Path Planner)
     * @param reversed reverse the robots direction
     */
    public FollowPath(Drivetrain drivetrain, String pathName, HashMap<String, Command> eventMap, boolean reversed) {
        this(
                drivetrain,
                PathPlanner.loadPath(
                        pathName,
                        DrivetrainConfig.PATH_CONSTRAINTS,
                        reversed),
                eventMap);
    }
}