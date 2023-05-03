// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriveWithJoysticks extends CommandBase {
  private final Drivetrain drivetrain;
  private final Supplier<Double> driveSpeedX, driveSpeedY, turnSpeed;
  private PIDController controller = new PIDController(0.05, 0, 0);

  public DriveWithJoysticks(Drivetrain drivetrain, Supplier<Double> driveSpeedX, Supplier<Double> driveSpeedY, Supplier<Double> turnSpeed) {
    this.drivetrain = drivetrain;
    this.driveSpeedX = driveSpeedX;
    this.driveSpeedY = driveSpeedY;
    this.turnSpeed = turnSpeed;

    controller.enableContinuousInput(-180, 180);
    controller.setTolerance(5);

    addRequirements(drivetrain);
  }

  @Override
  public void execute() {
    // Translation2d robotPose = drivetrain.getPose().getTranslation();
    // Translation2d targetPose = new Translation2d(4, 0);

    // double angle = Math.toDegrees(Math.atan2(targetPose.getY()-robotPose.getY(), targetPose.getX()-robotPose.getX()));
    // angle = Math.IEEEremainder(angle, 180);
    // SmartDashboard.putNumber("Target Angle", angle);
    // SmartDashboard.putNumber("Current Rotation", drivetrain.getPose().getRotation().getDegrees());

    drivetrain.driveChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(driveSpeedY.get(), driveSpeedX.get(), -turnSpeed.get(), drivetrain.getHeading()));
    //drivetrain.driveChassisSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(driveSpeedY.get(), driveSpeedX.get(), controller.calculate(drivetrain.getPose().getRotation().getDegrees(), angle) - turnSpeed.get(), drivetrain.getHeading()));
  }
}
