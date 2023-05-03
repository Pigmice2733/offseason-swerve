// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.kauailabs.navx.frc.AHRS;
import com.swervedrivespecialties.swervelib.MkSwerveModuleBuilder;
import com.swervedrivespecialties.swervelib.MotorType;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SwerveModuleConfig;
import frc.robot.Constants.DrivetrainConfig;

public class Drivetrain extends SubsystemBase {
  // SwerveModule frontLeftModule = SwerveModuleConfig.FRONT_LEFT_CONFIG.createModule();
  // SwerveModule frontRightModule = SwerveModuleConfig.FRONT_RIGHT_CONFIG.createModule();
  // SwerveModule backLeftModule = SwerveModuleConfig.BACK_LEFT_CONFIG.createModule();
  // SwerveModule backRightModule = SwerveModuleConfig.BACK_RIGHT_CONFIG.createModule();
  SwerveModule frontLeftModule;
  SwerveModule frontRightModule;
  SwerveModule backLeftModule;
  SwerveModule backRightModule;
  

  private final AHRS gyro = new AHRS();
  private final SwerveDriveOdometry odometry;

  private Pose2d pose;
  public Pose2d getPose() {
    return pose;
  }

  private SwerveModuleState[] states = new SwerveModuleState[4];
  
  public Drivetrain() {
  frontLeftModule = 
  new MkSwerveModuleBuilder().withGearRatio(SdsModuleConfigurations.MK4I_L2)
                .withDriveMotor(MotorType.NEO, 10)
                .withSteerMotor(MotorType.NEO, 11)
                .withSteerEncoderPort(20)
                .withSteerOffset(Math.toRadians(290))
                .build();
  frontRightModule = 
  new MkSwerveModuleBuilder().withGearRatio(SdsModuleConfigurations.MK4I_L2)
                .withDriveMotor(MotorType.NEO, 13)
                .withSteerMotor(MotorType.NEO, 12)
                .withSteerEncoderPort(22)
                .withSteerOffset(Math.toRadians(319))
                .build();
  backLeftModule =
  new MkSwerveModuleBuilder().withGearRatio(SdsModuleConfigurations.MK4I_L2)
                .withDriveMotor(MotorType.NEO, 17)
                .withSteerMotor(MotorType.NEO, 16)
                .withSteerEncoderPort(26)
                .withSteerOffset(Math.toRadians(131+180))
                .build();
  backRightModule =
  new MkSwerveModuleBuilder().withGearRatio(SdsModuleConfigurations.MK4I_L2)
                .withDriveMotor(MotorType.NEO, 14)
                .withSteerMotor(MotorType.NEO, 15)
                .withSteerEncoderPort(24)
                .withSteerOffset(Math.toRadians(252))
                .build();

    states = DrivetrainConfig.kinematics.toSwerveModuleStates(new ChassisSpeeds());
    SwerveModulePosition[] positions = {new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition() };
    odometry = new SwerveDriveOdometry(DrivetrainConfig.kinematics, new Rotation2d(), positions);

    resetOdometry();
  }

  @Override
  public void periodic() {
    SwerveDriveKinematics.desaturateWheelSpeeds(states, DrivetrainConfig.MAX_VELOCITY_METERS_PER_SECOND);

    frontLeftModule.set(states[0].speedMetersPerSecond / DrivetrainConfig.MAX_VELOCITY_METERS_PER_SECOND * DrivetrainConfig.MAX_VOLTAGE, states[0].angle.getRadians());
    frontRightModule.set(states[1].speedMetersPerSecond / DrivetrainConfig.MAX_VELOCITY_METERS_PER_SECOND * DrivetrainConfig.MAX_VOLTAGE, states[1].angle.getRadians());
    backLeftModule.set(states[2].speedMetersPerSecond / DrivetrainConfig.MAX_VELOCITY_METERS_PER_SECOND * DrivetrainConfig.MAX_VOLTAGE, states[2].angle.getRadians());
    backRightModule.set(states[3].speedMetersPerSecond / DrivetrainConfig.MAX_VELOCITY_METERS_PER_SECOND * DrivetrainConfig.MAX_VOLTAGE, states[3].angle.getRadians());
    SmartDashboard.putNumber("Drivetrain Yaw", gyro.getAngle());

    SmartDashboard.putNumber("Angle", getHeading().getDegrees());

    pose = odometry.update(getHeading(), getModulePositions());
    SmartDashboard.putNumber("Pose X", pose.getX());
    SmartDashboard.putNumber("Pose Y", pose.getY());
    SmartDashboard.putNumber("Pose Rotation", pose.getRotation().getDegrees());
  }

  public void driveChassisSpeeds(ChassisSpeeds speeds) {
    SmartDashboard.putNumber("Target X (m/s)", speeds.vxMetersPerSecond);
    SmartDashboard.putNumber("Target Y m/s", speeds.vyMetersPerSecond);
    SmartDashboard.putNumber("Target Rotation (rad/sec)", speeds.omegaRadiansPerSecond);
    states = DrivetrainConfig.kinematics.toSwerveModuleStates(speeds);
  }

  public void driveModuleStates(SwerveModuleState[] states) {
    this.states = states;
  }

  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = {
      new SwerveModulePosition(-frontLeftModule.getDriveDistance(), new Rotation2d(frontLeftModule.getSteerAngle())),
      new SwerveModulePosition(-frontRightModule.getDriveDistance(), new Rotation2d(frontRightModule.getSteerAngle())),
      new SwerveModulePosition(backLeftModule.getDriveDistance(), new Rotation2d(backLeftModule.getSteerAngle())),
      new SwerveModulePosition(backRightModule.getDriveDistance(), new Rotation2d(backRightModule.getSteerAngle()))
    };
    return positions;
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] positions = {
      new SwerveModuleState(frontLeftModule.getDriveVelocity(), new Rotation2d(frontLeftModule.getSteerAngle())),
      new SwerveModuleState(frontRightModule.getDriveVelocity(), new Rotation2d(frontRightModule.getSteerAngle())),
      new SwerveModuleState(backLeftModule.getDriveVelocity(), new Rotation2d(backLeftModule.getSteerAngle())),
      new SwerveModuleState(backRightModule.getDriveVelocity(), new Rotation2d(backRightModule.getSteerAngle()))
    };
    return positions;
  }

  public void resetOdometry() {
    resetOdometry(new Pose2d());
  }

  public void resetOdometry(Pose2d pose) {
    gyro.reset();
    gyro.setAngleAdjustment(pose.getRotation().getDegrees());
    odometry.resetPosition(getHeading(), getModulePositions(), pose);
  }

  public Rotation2d getHeading() {
    return new Rotation2d(Math.toRadians(-gyro.getAngle()));
  }

  public SwerveDriveKinematics getKinematics() {
    return DrivetrainConfig.kinematics;
  }
}
