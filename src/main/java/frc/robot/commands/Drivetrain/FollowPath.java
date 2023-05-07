package frc.robot.commands.drivetrain;

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
     * Use a RamseteController to follow a specified path.
     * Call only from autonomous path-following commands that define a trajectory
     * and a trajectory configuration.
     * 
     * @param drivetrain a drivetrain subsystem
     * @param trajectory a path-following trajectory
     */
    public FollowPath(Drivetrain drivetrain, PathPlannerTrajectory trajectory) {
        addCommands(
            drivetrain.resetOdometryCommand(trajectory.getInitialPose()),
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

    public FollowPath(Drivetrain drivetrain, PathPlannerTrajectory trajectory, HashMap<String, Command> eventMap) {
        addCommands(
            drivetrain.resetOdometryCommand(trajectory.getInitialPose()),
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
     * Use a RamseteController to follow a specified path.
     * Call only from autonomous path-following commands that define a trajectory
     * and a trajectory configuration.
     * 
     * @param drivetrain a drivetrain subsystem
     * @param pathName   the name of a premade path to follow
     */
    public FollowPath(Drivetrain drivetrain, String pathName, boolean reversed) {
        this(
                drivetrain,
                PathPlanner.loadPath(
                        pathName,
                        DrivetrainConfig.pathConstraints,
                        reversed));
    }

    public FollowPath(Drivetrain drivetrain, String pathName, HashMap<String, Command> eventMap, boolean reversed) {
        this(
                drivetrain,
                PathPlanner.loadPath(
                        pathName,
                        DrivetrainConfig.pathConstraints,
                        reversed),
                eventMap);
    }
}