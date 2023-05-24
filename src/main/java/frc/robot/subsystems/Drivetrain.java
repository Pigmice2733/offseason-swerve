// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ShuffleboardHelper;
import frc.robot.Constants.DrivetrainConfig;

public class Drivetrain extends SubsystemBase {
    private final SwerveModule frontLeftModule = DrivetrainConfig.FRONT_LEFT_MODULE.build();
    private final SwerveModule frontRightModule = DrivetrainConfig.FRONT_RIGHT_MODULE.build();
    private final SwerveModule backLeftModule = DrivetrainConfig.BACK_LEFT_MODULE.build();
    private final SwerveModule backRightModule = DrivetrainConfig.BACK_RIGHT_MODULE.build();

    private final AHRS gyro = new AHRS();
    private final SwerveDriveOdometry odometry;

    private Pose2d pose = new Pose2d();

    private ChassisSpeeds targetSpeeds = new ChassisSpeeds();
    private SwerveModuleState[] states = new SwerveModuleState[4];

    public Drivetrain() {
        states = getModuleStates();
        odometry = new SwerveDriveOdometry(DrivetrainConfig.KINEMATICS, new Rotation2d(), getModulePositions());

        resetOdometry();

        ShuffleboardHelper.drivetrainTab.add("Heading", gyro).withPosition(4, 0);
        ShuffleboardHelper.addOutput("Pose X", ShuffleboardHelper.drivetrainTab, () -> pose.getX()).withPosition(4, 2);
        ShuffleboardHelper.addOutput("Pose Y", ShuffleboardHelper.drivetrainTab, () -> pose.getY()).withPosition(5, 2);

        ShuffleboardHelper
                .addOutput("Target X vel", ShuffleboardHelper.drivetrainTab, () -> targetSpeeds.vxMetersPerSecond)
                .withPosition(0, 3);
        ShuffleboardHelper
                .addOutput("Target Y vel", ShuffleboardHelper.drivetrainTab, () -> targetSpeeds.vyMetersPerSecond)
                .withPosition(1, 3);
        ShuffleboardHelper.addOutput("Target Rot. vel", ShuffleboardHelper.drivetrainTab,
                () -> targetSpeeds.omegaRadiansPerSecond).withPosition(2, 3);

        ShuffleboardHelper.addOutput("Velocity X", ShuffleboardHelper.drivetrainTab, () -> gyro.getVelocityX());
        ShuffleboardHelper.addOutput("Velocity Y", ShuffleboardHelper.drivetrainTab, () -> gyro.getVelocityY());
        ShuffleboardHelper.addOutput("Speed", ShuffleboardHelper.drivetrainTab,
                () -> Math.sqrt(Math.pow(gyro.getVelocityX(), 2) + Math.pow(gyro.getVelocityY(), 2)));
    }

    @Override
    public void periodic() {
        applyModuleStates();
        pose = odometry.update(getHeading(), getModulePositions());
    }

    /** Apply the current target swerve module states */
    private void applyModuleStates() {
        if (states == null)
            System.out.println("Module states are NULL");

        SwerveDriveKinematics.desaturateWheelSpeeds(states, DrivetrainConfig.MAX_ATTAINABLE_SPEED);

        frontLeftModule.set(calculateFeedForward(states[0].speedMetersPerSecond),
                states[0].angle.getRadians());
        frontRightModule.set(calculateFeedForward(states[1].speedMetersPerSecond),
                states[1].angle.getRadians());
        backLeftModule.set(calculateFeedForward(states[2].speedMetersPerSecond),
                states[2].angle.getRadians());
        backRightModule.set(calculateFeedForward(states[3].speedMetersPerSecond),
                states[3].angle.getRadians());
    }

    private double calculateFeedForward(double velocity) {
        velocity = MathUtil.applyDeadband(velocity, 0.001);
        return DrivetrainConfig.DRIVE_FEED_FORWARD.calculate(velocity);
    }

    /** @param speeds set target swerve module states based on a ChassisSpeed */
    public void driveChassisSpeeds(ChassisSpeeds speeds) {
        targetSpeeds = speeds;
        driveModuleStates(DrivetrainConfig.KINEMATICS.toSwerveModuleStates(speeds));
    }

    /** @param states set target swerve module states */
    public void driveModuleStates(SwerveModuleState[] states) {
        targetSpeeds = DrivetrainConfig.KINEMATICS.toChassisSpeeds(states);
        this.states = states;
    }

    /** @return module positions representing the drivetrains swerve modules */
    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = {
                new SwerveModulePosition(frontLeftModule.getDriveDistance(),
                        new Rotation2d(frontLeftModule.getSteerAngle())),
                new SwerveModulePosition(frontRightModule.getDriveDistance(),
                        new Rotation2d(frontRightModule.getSteerAngle())),
                new SwerveModulePosition(backLeftModule.getDriveDistance(),
                        new Rotation2d(backLeftModule.getSteerAngle())),
                new SwerveModulePosition(backRightModule.getDriveDistance(),
                        new Rotation2d(backRightModule.getSteerAngle()))
        };
        return positions;
    }

    /** @return module states representing the drivetrains swerve modules */
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = {
                new SwerveModuleState(frontLeftModule.getDriveVelocity(),
                        new Rotation2d(frontLeftModule.getSteerAngle())),
                new SwerveModuleState(frontRightModule.getDriveVelocity(),
                        new Rotation2d(frontRightModule.getSteerAngle())),
                new SwerveModuleState(backLeftModule.getDriveVelocity(),
                        new Rotation2d(backLeftModule.getSteerAngle())),
                new SwerveModuleState(backRightModule.getDriveVelocity(),
                        new Rotation2d(backRightModule.getSteerAngle()))
        };
        return states;
    }

    /** @param pose a pose to set the odometry and gyro to */
    public void resetOdometry(Pose2d pose) {
        System.out.println("Reset Odometry to: " + pose);
        gyro.reset();
        gyro.setAngleAdjustment(-pose.getRotation().getDegrees());
        odometry.resetPosition(getHeading(), getModulePositions(), pose);
    }

    /** Reset odometry & gyro to pose (0, 0) with 0 rotation */
    public void resetOdometry() {
        resetOdometry(new Pose2d());
    }

    public Command resetOdometryCommand(Pose2d pose) {
        return new InstantCommand(() -> resetOdometry(pose));
    }

    /** @return the current yaw of the robot */
    public Rotation2d getHeading() {
        return new Rotation2d(Math.toRadians(-gyro.getAngle()));
    }

    /** @return the current estimated pose of the robot */
    public Pose2d getPose() {
        if (pose == null)
            System.out.println("Robot pose is NULL");
        return pose;
    }
}
