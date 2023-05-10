
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.commands.drivetrain;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriveFacingPosition extends CommandBase {
  private final Drivetrain drivetrain;
  private final Supplier<Double> driveSpeedX, driveSpeedY;
  private final Translation2d targetPos;
  private PIDController controller = new PIDController(0.1, 0, 0);

  public DriveFacingPosition(Drivetrain drivetrain, Supplier<Double> driveSpeedX, Supplier<Double> driveSpeedY, Translation2d targetPos) {
    this.drivetrain = drivetrain;
    this.driveSpeedX = driveSpeedX;
    this.driveSpeedY = driveSpeedY;
    this.targetPos = targetPos;

    controller.enableContinuousInput(-180, 180);
    addRequirements(drivetrain);
  }

  @Override
  public void execute() {
    Translation2d robotPose = drivetrain.getPose().getTranslation();

    double angle = Math.toDegrees(Math.atan2(targetPos.getY()-robotPose.getY(), targetPos.getX()-robotPose.getX()));
    double radPerSec = controller.calculate(drivetrain.getPose().getRotation().getDegrees(), angle);
    
    drivetrain.driveChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(driveSpeedY.get(), driveSpeedX.get(), radPerSec, drivetrain.getHeading()));
  }
}