package frc.robot;

public class SwerveModuleConfig {
    public final int driveMotorID, turnMotorID, absEncoderID;
    public final boolean driveInverted, turnInverted, absEncoderInverted;
    public final double absEncoderOffsetRad;

    public SwerveModuleConfig(int driveMotorID, int turnMotorID, int absEncoderID, boolean driveInverted, boolean turnInverted, boolean absEncoderInverted, double absEncoderOffsetRad) 
    {
        this.driveMotorID = driveMotorID;
        this.turnMotorID = turnMotorID;
        this.absEncoderID = absEncoderID;

        this.driveInverted = driveInverted;
        this.turnInverted = turnInverted;
        this.absEncoderInverted = absEncoderInverted;

        this.absEncoderOffsetRad = absEncoderOffsetRad;
    }
}
