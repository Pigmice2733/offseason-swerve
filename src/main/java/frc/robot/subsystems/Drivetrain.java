// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    drivetrainTab.getLayout("Front Left Module", BuiltInLayouts.kList)
    .withSize(2, 4)
    .withPosition(0, 0),
    Mk4iSwerveModuleHelper.GearRatio.L2,
    10, 11, 20, -Math.toRadians(290));

  SwerveModule frontRightModule = Mk4iSwerveModuleHelper.createNeo(
    drivetrainTab.getLayout("Front Right Module", BuiltInLayouts.kList)
    .withSize(2, 4)
    .withPosition(0, 0),
    Mk4iSwerveModuleHelper.GearRatio.L2,
    13, 12, 22, -Math.toRadians(319));

  SwerveModule backLeftModule = Mk4iSwerveModuleHelper.createNeo(
    drivetrainTab.getLayout("Back Left Module", BuiltInLayouts.kList)
    .withSize(2, 4)
    .withPosition(0, 0),
    Mk4iSwerveModuleHelper.GearRatio.L2,
    17, 16, 26, -Math.toRadians(131+180));
    
  SwerveModule backRightModule = Mk4iSwerveModuleHelper.createNeo(
    drivetrainTab.getLayout("Back Right Module", BuiltInLayouts.kList)
    .withSize(2, 4)
    .withPosition(0, 0),
    Mk4iSwerveModuleHelper.GearRatio.L2,
    14, 15, 24, -Math.toRadians(252));

  private final AHRS gyro = new AHRS();
  private final SwerveDriveOdometry odometry;

  private Pose2d pose;
  public Pose2d getPose() {
    return pose;
  }

  private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
    // Front left
    new Translation2d(0.2921, 0.2921),
    // Front right
    new Translation2d(0.2921, -0.2921),
    // Back left
    new Translation2d(-0.2921, 0.2921),
    // Back right
    new Translation2d(-0.2921, -0.2921)
  );

  private SwerveModuleState[] states = new SwerveModuleState[4];
  
  public Drivetrain() {
    states = kinematics.toSwerveModuleStates(new ChassisSpeeds());
    SwerveModulePosition[] positions = {new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition() };
    odometry = new SwerveDriveOdometry(kinematics, new Rotation2d(), positions);

    resetOdometry();
  }

  @Override
  public void periodic() {
    SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);

    frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[0].angle.getRadians());
    frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[1].angle.getRadians());
    backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[2].angle.getRadians());
    backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[3].angle.getRadians());
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
    states = kinematics.toSwerveModuleStates(speeds);
  }

  public void driveModuleStates(SwerveModuleState[] states) {
    this.states = states;
  }

  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = {
      new SwerveModulePosition(frontLeftModule.getDriveVelocity(), new Rotation2d(frontLeftModule.getSteerAngle())),
      new SwerveModulePosition(frontRightModule.getDriveVelocity(), new Rotation2d(frontRightModule.getSteerAngle())),
      new SwerveModulePosition(backLeftModule.getDriveVelocity(), new Rotation2d(backLeftModule.getSteerAngle())),
      new SwerveModulePosition(backRightModule.getDriveVelocity(), new Rotation2d(backRightModule.getSteerAngle()))
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
    return kinematics;
  }

  public void driveModuleStates() {

  }
}
