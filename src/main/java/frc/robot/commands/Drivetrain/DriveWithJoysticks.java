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

  public DriveWithJoysticks(Drivetrain drivetrain, Supplier<Double> driveSpeedX, Supplier<Double> driveSpeedY, Supplier<Double> turnSpeed) {
    this.drivetrain = drivetrain;
    this.driveSpeedX = driveSpeedX;
    this.driveSpeedY = driveSpeedY;
    this.turnSpeed = turnSpeed;

    addRequirements(drivetrain);
  }

  @Override
  public void execute() {
    // TODO: make robot drive
  }
}
