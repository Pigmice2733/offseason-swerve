// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.DrivetrainConfig;
import frc.robot.commands.Drivetrain.DriveFacingPosition;
import frc.robot.commands.Drivetrain.DriveWithJoysticks;
import frc.robot.subsystems.Drivetrain;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private Drivetrain drivetrain = new Drivetrain();
  private XboxController driver = new XboxController(0);
  private XboxController operator = new XboxController(1);
  private Controls controls = new Controls(driver, operator);
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    drivetrain.setDefaultCommand(new DriveWithJoysticks(drivetrain, controls::getDriveSpeedX, controls::getDriveSpeedY, controls::getTurnSpeed, () -> true));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(driver, Button.kX.value).onTrue(new InstantCommand(() -> drivetrain.resetOdometry()));
    new JoystickButton(driver, Button.kA.value).whileTrue(new DriveFacingPosition(drivetrain, controls::getDriveSpeedX, controls::getDriveSpeedY, new Translation2d(2, 0)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    drivetrain.resetOdometry();
    PathPlannerTrajectory traj = PathPlanner.loadPath(
      "New Path",
      new PathConstraints(1.0, 1.5));

      drivetrain.resetOdometry(traj.getInitialHolonomicPose());

    return new PPSwerveControllerCommand(
        traj,
        drivetrain::getPose, DrivetrainConfig.kinematics, 
        new PIDController(0, 0, 0), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
        new PIDController(0, 0, 0), // Y controller (usually the same values as X controller)
        new PIDController(0, 0, 0), (output) -> drivetrain.driveModuleStates(output), drivetrain);
  }
}