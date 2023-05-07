// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConfig;

public class Drivetrain extends SubsystemBase {
  public final SwerveModule frontLeftModule = DrivetrainConfig.FRONT_LEFT_MODULE.build();
  public final SwerveModule frontRightModule = DrivetrainConfig.FRONT_RIGHT_MODULE.build();
  public final SwerveModule backLeftModule = DrivetrainConfig.BACK_LEFT_MODULE.build();
  public final SwerveModule backRightModule = DrivetrainConfig.BACK_RIGHT_MODULE.build();

  private final AHRS gyro = new AHRS();
  private final SwerveDriveOdometry odometry;

  private Pose2d pose;
  private SwerveModuleState[] states = new SwerveModuleState[4];
  
  public Drivetrain() {
    states = getModuleStates();
    odometry = new SwerveDriveOdometry(DrivetrainConfig.kinematics, new Rotation2d(), getModulePositions());

    resetOdometry();
  }

  @Override
  public void periodic() {
    applyModuleStates();
    updateShuffleboard();
    pose = odometry.update(getHeading(), getModulePositions());
  }

  private void updateShuffleboard() {
    SmartDashboard.putNumber("Drivetrain Yaw", gyro.getAngle());
    SmartDashboard.putNumber("Angle", getHeading().getDegrees());
    SmartDashboard.putNumber("Pose X", pose.getX());
    SmartDashboard.putNumber("Pose Y", pose.getY());
    SmartDashboard.putNumber("Pose Rotation", pose.getRotation().getDegrees());
  }

  /** Apply the current target swerve module states*/
  private void applyModuleStates() {
    if (states == null)
      System.out.println("Module states are NULL");

    SwerveDriveKinematics.desaturateWheelSpeeds(states, DrivetrainConfig.MAX_VELOCITY_METERS_PER_SECOND);

    frontLeftModule.set(states[0].speedMetersPerSecond / DrivetrainConfig.MAX_VELOCITY_METERS_PER_SECOND * DrivetrainConfig.MAX_VOLTAGE, states[0].angle.getRadians());
    frontRightModule.set(states[1].speedMetersPerSecond / DrivetrainConfig.MAX_VELOCITY_METERS_PER_SECOND * DrivetrainConfig.MAX_VOLTAGE, states[1].angle.getRadians());
    backLeftModule.set(states[2].speedMetersPerSecond / DrivetrainConfig.MAX_VELOCITY_METERS_PER_SECOND * DrivetrainConfig.MAX_VOLTAGE, states[2].angle.getRadians());
    backRightModule.set(states[3].speedMetersPerSecond / DrivetrainConfig.MAX_VELOCITY_METERS_PER_SECOND * DrivetrainConfig.MAX_VOLTAGE, states[3].angle.getRadians());
  }

  /** @param speeds set target swerve module states based on a ChassisSpeed */
  public void driveChassisSpeeds(ChassisSpeeds speeds) {
    SmartDashboard.putNumber("Target X (m/s)", speeds.vxMetersPerSecond);
    SmartDashboard.putNumber("Target Y m/s", speeds.vyMetersPerSecond);
    SmartDashboard.putNumber("Target Rotation (rad/sec)", speeds.omegaRadiansPerSecond);
    driveModuleStates(DrivetrainConfig.kinematics.toSwerveModuleStates(speeds));
  }

  /** @param states set target swerve module states */
  public void driveModuleStates(SwerveModuleState[] states) {
    this.states = states;
  }

  /** @return module positions representing the drivetrains swerve modules */
  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = {
      new SwerveModulePosition(frontLeftModule.getDriveDistance(), new Rotation2d(frontLeftModule.getSteerAngle())),
      new SwerveModulePosition(frontRightModule.getDriveDistance(), new Rotation2d(frontRightModule.getSteerAngle())),
      new SwerveModulePosition(backLeftModule.getDriveDistance(), new Rotation2d(backLeftModule.getSteerAngle())),
      new SwerveModulePosition(backRightModule.getDriveDistance(), new Rotation2d(backRightModule.getSteerAngle()))
    };
    return positions;
  }

  /** @return module states representing the drivetrains swerve modules */
  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = {
      new SwerveModuleState(frontLeftModule.getDriveVelocity(), new Rotation2d(frontLeftModule.getSteerAngle())),
      new SwerveModuleState(frontRightModule.getDriveVelocity(), new Rotation2d(frontRightModule.getSteerAngle())),
      new SwerveModuleState(backLeftModule.getDriveVelocity(), new Rotation2d(backLeftModule.getSteerAngle())),
      new SwerveModuleState(backRightModule.getDriveVelocity(), new Rotation2d(backRightModule.getSteerAngle()))
    };
    return states;
  }

  /** @param pose a pose to set the odometry and gyro to */
  public void resetOdometry(Pose2d pose) {
    gyro.reset();
    gyro.setAngleAdjustment(pose.getRotation().getDegrees());
    odometry.resetPosition(getHeading(), getModulePositions(), pose);
  }

  /** Reset odometry & gyro to pose (0, 0) with 0 rotation */
  public void resetOdometry() {
    resetOdometry(new Pose2d());
  }

  public Command resetOdometryCommand(Pose2d pose) { return new InstantCommand(() -> resetOdometry(pose)); }

  /** @return the current yaw of the robot */
  public Rotation2d getHeading() {
    return new Rotation2d(Math.toRadians(-gyro.getAngle()));
  }

  /** @return the current estimated pose of the robot */
  public Pose2d getPose() {
    if (pose == null) System.out.println("Robot pose is NULL");
    return pose;
  }
}
