 import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;

public class SwerveModule {
    private final CANSparkMax driveMotor;
    private final CANSparkMax turningMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turningEncoder;

    private final PIDController turningPidController;

    private final AnalogInput absEncoder;
    private final boolean absEncoderInverted;
    private final double absEncoderOffsetRad;

    public SwerveModule(int driveMotorID, int turningMotorID, boolean driveInverted, boolean turningInverted, int absEncoderID, double absEncoderOffsetRad, boolean absEncoderInverted) {
        driveMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
        turningMotor = new CANSparkMax(turningMotor, MotorType.kBrushless);

        driveMotor.setInverted(driveInverted);
        turningEncoder.setInverted(turningInverted);

        driveEncoder = driveMotor.getEncoder();
        turningEncoder = turningMotor.getEncoder();

        absEncoder = new AnalogInput(absEncoderID);

        this.absEncoderInverted = absEncoderInverted;
        this.absEncoderOffsetRad = absEncoderOffsetRad;

        turningPIDController = new PIDController(0.001, 0, 0);
        turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
        
        resetEncoders():

    }
    

    public double getDrivePosition() {
        return driveEncoder.getPostion();
    }
    
    public double getTurningPosition() {
        return turningEncoder.getPostion();
    }
    
    public double getDriveVelocity(){
        return driveEncoder.getVelocity
    }
    

    public double getTurningVelocity(){
        return turningEncoder.getVelocity();
    }
    
    public double getAbsoluteEncoderRad(){
        double angle = absoluteEncoder.getVoltage() / RobotController.getVoltage5V();
        angle *= 2.0 * Math.PI;
        angle -= absoluteEncoderOffsetRad;
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0)
    }
        
    Public void resetEncoders(){
        driveEncoder.setPosition(0);
        turningEncoder.serPosition(getAbsoluteEncoderRad());
    }

    Public SwerveModuleState getState() {
        return new swerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state){
        if(Math.abs(state.speedMetersPerSecond) < 0.001){
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSec);
        turningMotor.set(turningPIDController.calculate(getTurningPosition(), state.angle.getRadians()));
        SmartDashboard.putString("Sweve[" + absoluteEncoder.getChannel() + "] state", state.ToString());        
    }
    
    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }

    
}