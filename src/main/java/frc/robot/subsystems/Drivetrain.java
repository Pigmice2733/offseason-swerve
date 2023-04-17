// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  ShuffleboardTab drivetrainTab = Shuffleboard.getTab("Drivetrain");

  public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
    SdsModuleConfigurations.MK4I_L2.getDriveReduction() *
    SdsModuleConfigurations.MK4I_L2.getWheelDiameter() * Math.PI;
  
  public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
    Math.hypot(0.2921, 0.2921);

  public static final double MAX_VOLTAGE = 12.0;

  SwerveModule frontLeftModule = Mk4iSwerveModuleHelper.createNeo(
    // drivetrainTab.getLayout("Front Left Module", BuiltInLayouts.kList)
    // .withSize(2, 4)
    // .withPosition(0, 0),
    Mk4iSwerveModuleHelper.GearRatio.L2,
    1, 2, 0, 0);

  SwerveModule frontRightModule = Mk4iSwerveModuleHelper.createNeo(
    // drivetrainTab.getLayout("Front Left Module", BuiltInLayouts.kList)
    // .withSize(2, 4)
    // .withPosition(0, 0),
    Mk4iSwerveModuleHelper.GearRatio.L2,
    3, 4, 1, 0);

  SwerveModule backLeftModule = Mk4iSwerveModuleHelper.createNeo(
    // drivetrainTab.getLayout("Front Left Module", BuiltInLayouts.kList)
    // .withSize(2, 4)
    // .withPosition(0, 0),
    Mk4iSwerveModuleHelper.GearRatio.L2,
    5, 6, 2, 0);
    
  SwerveModule backRightModule = Mk4iSwerveModuleHelper.createNeo(
    // drivetrainTab.getLayout("Front Left Module", BuiltInLayouts.kList)
    // .withSize(2, 4)
    // .withPosition(0, 0),
    Mk4iSwerveModuleHelper.GearRatio.L2,
    7, 8, 3, 0);

  private final AHRS gyro = new AHRS();


  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
    // Front left
    new Translation2d(0.2921, 0.2921),
    // Front right
    new Translation2d(0.2921, -0.2921),
    // Back left
    new Translation2d(-0.2921, 0.2921),
    // Back right
    new Translation2d(-0.2921, -0.2921)
  );

  private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, 0);
  
  public Drivetrain() {
    resetHeading();
  }

  @Override
  public void periodic() {
    SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);

    frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[0].angle.getRadians());
    frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[1].angle.getRadians());
    backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[2].angle.getRadians());
    backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[3].angle.getRadians());
  
  }

  public void driveChassisSpeeds(ChassisSpeeds speeds) {
    this.chassisSpeeds = speeds;
  }

  public void resetHeading() {
    gyro.reset();
  }
}
