// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

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
        public static final double MAX_DRIVE_SPEED = 2; // max meters / second
        public static final double MAX_TURN_SPEED = 2.5; // max radians / second

        public static final double TRACK_WIDTH_METERS = 0.5842; // distance from the center of one wheel to another (meters)
        public final static SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            new Translation2d(TRACK_WIDTH_METERS/2, TRACK_WIDTH_METERS/2), // Front left
            new Translation2d(TRACK_WIDTH_METERS/2, -TRACK_WIDTH_METERS/2), // Front right
            new Translation2d(-TRACK_WIDTH_METERS/2, TRACK_WIDTH_METERS/2), // Back left
            new Translation2d(-TRACK_WIDTH_METERS/2, -TRACK_WIDTH_METERS/2) // Back right
        );

        // Default values from example project
        public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
            SdsModuleConfigurations.MK4I_L2.getDriveReduction() *
            SdsModuleConfigurations.MK4I_L2.getWheelDiameter() * Math.PI;
        public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
            Math.hypot(TRACK_WIDTH_METERS/2, TRACK_WIDTH_METERS/2);
        public static final double MAX_VOLTAGE = 12.0;
    }
}
