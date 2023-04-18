// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

public class DriveWithJoysticks extends CommandBase {
  public final Drivetrain drivetrain;
  public final Supplier<Double> driveSpeedX, driveSpeedY, turnSpeed;
  pulic final SwerveSubsystem swerveSubsystem;

  public DriveWithJoysticks(Drivetrain drivetrain, Supplier<Double> xSpeedFunction, Supplier<Double> ySpeedFunction, Supplier<Double> turningSpeedFunction, Supplier<boolean> fieldOrientedFunction) {
    this.swerveSubsystem = swerveSubsystem;
    this.xSpeedFunction = xSpeedFunction;
    this.ySpeedFunction = ySpeedFunction;
    this.turningSpeedFunction = turningSpeedFunction;
    this.fieldOrientedFunction = fieldOrientedFunction:
    addRequirements(drivetrain);
  }

  @Override
  public void execute() {
    // TODO: make robot drive
    double xSpeed = xSpdFunction.get();
        double ySpeed = ySpdFunction.get();
        double turningSpeed = turningSpdFunction.get();

        // 2. Apply deadband
        xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;

        // 3. Make the driving smoother
        xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        turningSpeed = turningLimiter.calculate(turningSpeed)
                * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;

        // 4. Construct desired chassis speeds
        ChassisSpeeds chassisSpeeds;
        if (fieldOrientedFunction.get()) {
            // Relative to field
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());
        } else {
            // Relative to robot
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
        }

        // 5. Convert chassis speeds to individual module states
        SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        // 6. Output each module states to wheels
        swerveSubsystem.setModuleStates(moduleStates);
  }

  @Override
  public void end(boolean interrupted) {
      swerveSubsystem.stopModules();
  }

  @Override
  public boolean isFinished() {
      return false;
  }
}
}
