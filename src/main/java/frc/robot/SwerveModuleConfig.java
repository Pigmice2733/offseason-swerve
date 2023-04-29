package frc.robot;

import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;
import com.swervedrivespecialties.swervelib.Mk4iSwerveModuleHelper.GearRatio;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;

public class SwerveModuleConfig {
    public static SwerveModuleConfig FRONT_LEFT_CONFIG = new SwerveModuleConfig(
        Shuffleboard.getTab("Drivetrain").getLayout("Front Left Module", BuiltInLayouts.kList)
        .withSize(1, 3)
        .withPosition(0, 0),
        10, 11, 20, -Math.toRadians(290));

    public static SwerveModuleConfig FRONT_RIGHT_CONFIG = new SwerveModuleConfig(
        Shuffleboard.getTab("Drivetrain").getLayout("Front Right Module", BuiltInLayouts.kList)
        .withSize(1, 3)
        .withPosition(1, 0),
        13, 12, 22, -Math.toRadians(319));

    public static SwerveModuleConfig BACK_LEFT_CONFIG = new SwerveModuleConfig(
        Shuffleboard.getTab("Drivetrain").getLayout("Back Left Module", BuiltInLayouts.kList)
        .withSize(1, 3)
        .withPosition(2, 0),
        17, 16, 26, -Math.toRadians(131+180));
    
    public static SwerveModuleConfig BACK_RIGHT_CONFIG = new SwerveModuleConfig(
        Shuffleboard.getTab("Drivetrain").getLayout("Back Right Module", BuiltInLayouts.kList)
        .withSize(1, 3)
        .withPosition(3, 0),
        14, 15, 24, -Math.toRadians(252));
        
    private final ShuffleboardLayout debugLayout;
    private final GearRatio gearRatio = GearRatio.L2;
    private final int driveMotorPort, steerMotorPort, absEncoderPort;
    private final double absEncoderOffset;

    public SwerveModuleConfig(ShuffleboardLayout debugLayout, int driveMotorPort, int steerMotorPort, int absEncoderPort, double absEncoderOffset) 
    {
        this.debugLayout = debugLayout;
        this.driveMotorPort = driveMotorPort;
        this.steerMotorPort = steerMotorPort;
        this.absEncoderPort = absEncoderPort;
        this.absEncoderOffset = absEncoderOffset;
    }
    
    public SwerveModule createModule() {
        return Mk4iSwerveModuleHelper.createNeo(debugLayout, gearRatio, 
            driveMotorPort, steerMotorPort, absEncoderPort, absEncoderOffset);
    }
}