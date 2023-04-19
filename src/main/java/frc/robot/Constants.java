// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

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
        public static final int FRONT_LEFT_DRIVE = 1;
        public static final int FRONT_LEFT_TURN = 2;
        public static final int FRONT_RIGHT_DRIVE = 3;
        public static final int FRONT_RIGHT_TURN = 4;
        public static final int BACK_LEFT_DRIVE = 5;
        public static final int BACK_LEFT_TURN = 6;
        public static final int BACK_RIGHT_DRIVE = 7;
        public static final int BACK_RIGHT_TURN = 8;
    }

    public final static class DrivetrainConfig {
        public static final double DRIVE_SPEED = 0.2;
        public static final double TURN_SPEED = 0.2;

        public static final boolean DRIVE_MOTOR_INVERTED = false;
        public static final boolean TURN_MOTOR_INVERTED = false;
        public static final boolean ABS_ENCODER_INVERTED = false;
        public static final double ABS_ENCODER_OFFSET = 0;
        public static final int ABS_ENCODER_PORT = 0;
        public static final double MAX_PHYSICAL_SPEED = 5;

        // Constants from video for path following
        // public static final double kWheelDiametersMeter = Units.inchesToMeters(4);
        // public static final double kDriveMotorGearRatio = 1 / 5.8426;
        // public static final double kTurningMotorGearRatio = 1 / 18.0;
        // public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiametersMeter;
        // public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
        // public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        // public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
    
        public static final SwerveModuleConfig F_L_MODULE_CONFIG = 
            new SwerveModuleConfig(0, 0, 0, false, false, false, 0);
        public static final SwerveModuleConfig F_R_MODULE_CONFIG = 
            new SwerveModuleConfig(0, 0, 0, false, false, false, 0);
        public static final SwerveModuleConfig B_L_MODULE_CONFIG = 
            new SwerveModuleConfig(0, 0, 0, false, false, false, 0);
        public static final SwerveModuleConfig B_R_MODULE_CONFIG = 
            new SwerveModuleConfig(0, 0, 0, false, false, false, 0);

        public static final double TURN_P = 0.001;
    }
}
