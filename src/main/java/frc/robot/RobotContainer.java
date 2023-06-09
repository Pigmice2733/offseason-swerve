// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.drivetrain.DriveWithJoysticks;
import frc.robot.commands.drivetrain.path_following.DriveToPoint;
import frc.robot.commands.drivetrain.path_following.FollowPath;
import frc.robot.commands.drivetrain.path_following.PathfindToPoint;
import frc.robot.commands.drivetrain.path_following.RetracePath;
import frc.robot.pathfinder.Pathfinder;
import frc.robot.subsystems.Drivetrain;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    private Drivetrain drivetrain = new Drivetrain();
    private XboxController driver = new XboxController(0);
    private XboxController operator = new XboxController(1);
    private Controls controls = new Controls(driver, operator);
    private Pathfinder pathfinder = new Pathfinder(Units.inchesToMeters(30), "testing2");

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Configure the button bindings
        configureButtonBindings();
        drivetrain.setDefaultCommand(new DriveWithJoysticks(drivetrain, controls::getDriveSpeedX,
                controls::getDriveSpeedY, controls::getTurnSpeed, () -> true));
        // drivetrain.setDefaultCommand(new DriveWithBoundaries(drivetrain, controls::getDriveSpeedX,
        //         controls::getDriveSpeedY, controls::getTurnSpeed, new Translation2d(-1.5, -1.5), new Translation2d(1.5, 1.5)));

        SmartDashboard.putNumber("Goal X", 0);
        SmartDashboard.putNumber("Goal Y", 0);
        ShuffleboardHelper.addOutput("GoalDriveable", ShuffleboardHelper.drivetrainTab, () -> pathfinder.grid.FindCloseNode(new Translation2d(SmartDashboard.getNumber("Goal X", 0), SmartDashboard.getNumber("Goal Y", 0))).driveable);
        ShuffleboardHelper.addOutput("CurrentDriveable", ShuffleboardHelper.drivetrainTab, () -> pathfinder.grid.FindCloseNode(drivetrain.getPose().getTranslation()).driveable);
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private PathfindToPoint pathfindCommand;
    private void configureButtonBindings() {
        new JoystickButton(driver, Button.kX.value).onTrue(new InstantCommand(() -> drivetrain.resetOdometry()));
        // new JoystickButton(driver, Button.kA.value).whileTrue(new DriveFacingPosition(drivetrain,
        //         controls::getDriveSpeedX, controls::getDriveSpeedY, new Translation2d(0, 0)));
        new JoystickButton(driver, Button.kY.value).whileTrue(new DriveToPoint(drivetrain, new Pose2d(), driver));

        new JoystickButton(driver, Button.kB.value).whileTrue(new RetracePath(drivetrain));

        new JoystickButton(driver, Button.kA.value).onTrue(new InstantCommand(
            () -> {
                pathfindCommand = new PathfindToPoint(drivetrain, pathfinder, new Pose2d(SmartDashboard.getNumber("Goal X", 0), SmartDashboard.getNumber("Goal Y", 0), new Rotation2d()));
                pathfindCommand.schedule();
            }
            )).onFalse(new InstantCommand(() -> pathfindCommand.end(true)));
    }

    public void stopControllerRumble() {
        driver.setRumble(RumbleType.kBothRumble, 0);
        operator.setRumble(RumbleType.kBothRumble, 0);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return new FollowPath(drivetrain, "New New Path", false);
    }
}