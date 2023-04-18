 import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
        turningMotor = new CANSparkMax(turningMotorID, MotorType.kBrushless);

        driveMotor.setInverted(driveInverted);
        turningEncoder.setInverted(turningInverted);

        driveEncoder = driveMotor.getEncoder();
        turningEncoder = turningMotor.getEncoder();

        absEncoder = new AnalogInput(absEncoderID);

        this.absEncoderInverted = absEncoderInverted;
        this.absEncoderOffsetRad = absEncoderOffsetRad;

        turningPidController = new PIDController(0.001, 0, 0);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);
        
        resetEncoders();

    }
    

    public double getDrivePosition() {
        return driveEncoder.getPosition();
    }
    
    public double getTurningPosition() {
        return turningEncoder.getPosition();
    }
    
    public double getDriveVelocity(){
        return driveEncoder.getVelocity();
    }
    

    public double getTurningVelocity(){
        return turningEncoder.getVelocity();
    }
    
    public double getAbsoluteEncoderRad(){
        double angle = absoluteEncoder.getVoltage() / RobotController.getVoltage5V();
        angle *= 2.0 * Math.PI;
        angle -= absEncoderOffsetRad;
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0)
    }
        
    public void resetEncoders(){
        driveEncoder.setPosition(0);
        turningEncoder.setPosition(getAbsoluteEncoderRad());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state){
        if(Math.abs(state.speedMetersPerSecond) < 0.001){
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(state.speedMetersPerSecond / DriveConstant.kPhysicalMaxSpeedMetersPerSec);
        turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
        SmartDashboard.putString("Sweve[" + absoluteEncoder.getChannel() + "] state", state.toString());        
    }
    
    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }

    
}