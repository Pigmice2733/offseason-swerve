// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathConstraints;
import com.pigmice.frc.lib.swerve.SwerveConfig;
import com.swervedrivespecialties.swervelib.MkSwerveModuleBuilder;
import com.swervedrivespecialties.swervelib.MotorType;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final double AXIS_THRESHOLD = 0.2;

    public final static class ShuffleboardTabs {
        public static final ShuffleboardTab DRIVETRAIN = Shuffleboard.getTab("Drivetrain");
    }

    /** Put all can ID's here */
    public final static class CANConfig {
        public static final int FRONT_LEFT_DRIVE = 10;
        public static final int FRONT_LEFT_STEER = 11;
        public static final int FRONT_RIGHT_DRIVE = 13;
        public static final int FRONT_RIGHT_STEER = 12;
        public static final int BACK_LEFT_DRIVE = 17;
        public static final int BACK_LEFT_STEER = 16;
        public static final int BACK_RIGHT_DRIVE = 14;
        public static final int BACK_RIGHT_STEER = 15;

        public static final int FRONT_LEFT_ABS_ENCODER = 20;
        public static final int FRONT_RIGHT_ABS_ENCODER = 22;
        public static final int BACK_LEFT_ABS_ENCODER = 26;
        public static final int BACK_RIGHT_ABS_ENCODER = 24;
    }

    public final static class DrivetrainConfig {
        public static final double MAX_DRIVE_SPEED = 2.5; // max meters / second
        public static final double MAX_TURN_SPEED = 4; // max radians / second

        private static final double TRACK_WIDTH_METERS = 0.5842; // distance from the center of one wheel to
                                                                 // another
                                                                 // (meters)
        private final static SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(
                new Translation2d(TRACK_WIDTH_METERS / 2, TRACK_WIDTH_METERS / 2), // Front left
                new Translation2d(TRACK_WIDTH_METERS / 2, -TRACK_WIDTH_METERS / 2), // Front right
                new Translation2d(-TRACK_WIDTH_METERS / 2, TRACK_WIDTH_METERS / 2), // Back left
                new Translation2d(-TRACK_WIDTH_METERS / 2, -TRACK_WIDTH_METERS / 2) // Back right
        );

        // Constants found in Sysid (volts)
        private static final SimpleMotorFeedforward DRIVE_FEED_FORWARD = new SimpleMotorFeedforward(
                0.35493, 2.3014, 0.12872);

        // From what I have seen, it is common to only use a P value in pathfollowing
        private static final PathConstraints PATH_CONSTRAINTS = new PathConstraints(3, 2.5);
        private static final PIDController PATH_DRIVE_PID = new PIDController(0.3, 0, 0);
        private static final PIDController PATH_TURN_PID = new PIDController(0.31, 0, 0);

        private static final MkSwerveModuleBuilder FRONT_LEFT_MODULE = new MkSwerveModuleBuilder()
                .withLayout(ShuffleboardTabs.DRIVETRAIN
                        .getLayout("Front Left", BuiltInLayouts.kList)
                        .withSize(1, 3)
                        .withPosition(0, 0))
                .withGearRatio(SdsModuleConfigurations.MK4I_L2)
                .withDriveMotor(MotorType.NEO, CANConfig.FRONT_LEFT_DRIVE)
                .withSteerMotor(MotorType.NEO, CANConfig.FRONT_LEFT_STEER)
                .withSteerEncoderPort(CANConfig.FRONT_LEFT_ABS_ENCODER)
                .withSteerOffset(-Math.toRadians(290));

        private static final MkSwerveModuleBuilder FRONT_RIGHT_MODULE = new MkSwerveModuleBuilder()
                .withLayout(ShuffleboardTabs.DRIVETRAIN
                        .getLayout("Front Right", BuiltInLayouts.kList)
                        .withSize(1, 3)
                        .withPosition(1, 0))
                .withGearRatio(SdsModuleConfigurations.MK4I_L2)
                .withDriveMotor(MotorType.NEO, CANConfig.FRONT_RIGHT_DRIVE)
                .withSteerMotor(MotorType.NEO, CANConfig.FRONT_RIGHT_STEER)
                .withSteerEncoderPort(CANConfig.FRONT_RIGHT_ABS_ENCODER)
                .withSteerOffset(-Math.toRadians(319));

        private static final MkSwerveModuleBuilder BACK_LEFT_MODULE = new MkSwerveModuleBuilder()
                .withLayout(ShuffleboardTabs.DRIVETRAIN
                        .getLayout("Back Left", BuiltInLayouts.kList)
                        .withSize(1, 3)
                        .withPosition(2, 0))
                .withGearRatio(SdsModuleConfigurations.MK4I_L2)
                .withDriveMotor(MotorType.NEO, CANConfig.BACK_LEFT_DRIVE)
                .withSteerMotor(MotorType.NEO, CANConfig.BACK_LEFT_STEER)
                .withSteerEncoderPort(CANConfig.BACK_LEFT_ABS_ENCODER)
                .withSteerOffset(-Math.toRadians(312));

        private static final MkSwerveModuleBuilder BACK_RIGHT_MODULE = new MkSwerveModuleBuilder()
                .withLayout(ShuffleboardTabs.DRIVETRAIN
                        .getLayout("Back Right", BuiltInLayouts.kList)
                        .withSize(1, 3)
                        .withPosition(3, 0))
                .withGearRatio(SdsModuleConfigurations.MK4I_L2)
                .withDriveMotor(MotorType.NEO, CANConfig.BACK_RIGHT_DRIVE)
                .withSteerMotor(MotorType.NEO, CANConfig.BACK_RIGHT_STEER)
                .withSteerEncoderPort(CANConfig.BACK_RIGHT_ABS_ENCODER)
                .withSteerOffset(-Math.toRadians(252));

        public static final SwerveConfig SWERVE_CONFIG = new SwerveConfig(FRONT_LEFT_MODULE, FRONT_RIGHT_MODULE,
                BACK_LEFT_MODULE, BACK_RIGHT_MODULE,
                PATH_CONSTRAINTS, PATH_DRIVE_PID, PATH_TURN_PID,
                MAX_DRIVE_SPEED, MAX_TURN_SPEED, KINEMATICS,
                DRIVE_FEED_FORWARD, ShuffleboardTabs.DRIVETRAIN);
    }
}
