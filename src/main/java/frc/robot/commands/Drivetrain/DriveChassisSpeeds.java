// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.drivetrain;


import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriveChassisSpeeds extends CommandBase {
  private final Drivetrain drivetrain;
  private ChassisSpeeds speeds;

  /** Drive a steady ChassisSpeed for testing */
  public DriveChassisSpeeds(Drivetrain drivetrain, ChassisSpeeds speeds) {
    this.drivetrain = drivetrain;
    this.speeds = speeds;
  }

  @Override
  public void initialize() {
    drivetrain.resetOdometry();
  }

  @Override
  public void execute() {
    drivetrain.driveChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, drivetrain.getHeading()));
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.driveChassisSpeeds(new ChassisSpeeds());
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
