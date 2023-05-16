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

  public DriveChassisSpeeds(Drivetrain drivetrain, ChassisSpeeds speeds) {
    this.drivetrain = drivetrain;
    this.speeds = speeds;
  }

  @Override
  public void initialize() {
    drivetrain.resetOdometry();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.driveChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(speeds, drivetrain.getHeading()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.driveChassisSpeeds(new ChassisSpeeds());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
