// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.swervedrivespecialties.swervelib.MkModuleConfiguration;
import com.swervedrivespecialties.swervelib.MkSwerveModuleBuilder;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final double AXIS_THRESHOLD = 0.1;

    public final static class CANConfig {
        public static final int FRONT_LEFT_DRIVE = 0;
        public static final int FRONT_LEFT_TURN = 0;
        public static final int FRONT_RIGHT_DRIVE = 0;
        public static final int FRONT_RIGHT_TURN = 0;
        public static final int BACK_LEFT_DRIVE = 0;
        public static final int BACK_LEFT_TURN = 0;
        public static final int BACK_RIGHT_DRIVE = 0;
        public static final int BACK_RIGHT_TURN = 0;
    }

    public final static class DrivetrainConfig {
        public static final double DRIVE_SPEED = 2;
        public static final double TURN_SPEED = 2.5;

    MkSwerveModuleBuilder FRONT_LEFT_MODULE = new MkSwerveModuleBuilder()
        .withLayout(drivetrainTab.getLayout("Front Left Module", BuiltInLayouts.kList)
        .withSize(2, 4)
        .withPosition(0, 0))
        .withGearRatio(SdsModuleConfigurations.MK4I_L2)
        .withDriveMotor(MotorType.NEO, 10)
        .withSteerMotor(MotorType.NEO, 11)
        .withSteerEncoderPort(20)
        .withSteerOffset(-Math.toRadians(290))
        .build();

    frontRightModule = new MkSwerveModuleBuilder()
      .withLayout(drivetrainTab.getLayout("Front Right Module", BuiltInLayouts.kList)
        .withSize(2, 4)
        .withPosition(0, 0))
      .withGearRatio(SdsModuleConfigurations.MK4I_L2)
      .withDriveMotor(MotorType.NEO, 13)
      .withSteerMotor(MotorType.NEO, 12)
      .withSteerEncoderPort(22)
      .withSteerOffset(-Math.toRadians(319))
      .build();

    backLeftModule = new MkSwerveModuleBuilder()
      .withLayout(drivetrainTab.getLayout("Back Left Module", BuiltInLayouts.kList)
        .withSize(2, 4)
        .withPosition(0, 0))
      .withGearRatio(SdsModuleConfigurations.MK4I_L2)
      .withDriveMotor(MotorType.NEO, 17)
      .withSteerMotor(MotorType.NEO, 16)
      .withSteerEncoderPort(26)
      .withSteerOffset(-Math.toRadians(131+180))
      .build();

    backRightModule = new MkSwerveModuleBuilder()
      .withLayout(drivetrainTab.getLayout("Back Right Module", BuiltInLayouts.kList)
        .withSize(2, 4)
        .withPosition(0, 0))
      .withGearRatio(SdsModuleConfigurations.MK4I_L2)
      .withDriveMotor(MotorType.NEO, 14)
      .withSteerMotor(MotorType.NEO, 15)
      .withSteerEncoderPort(24)
      .withSteerOffset(-Math.toRadians(252))
      .build();
    }
}
