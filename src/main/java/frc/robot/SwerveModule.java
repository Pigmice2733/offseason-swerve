package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DrivetrainConfig;
import frc.robot.commands.Drivetrain.DriveWithJoysticks;

public class SwerveModule {
    private final CANSparkMax driveMotor;
    private final CANSparkMax turnMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turnEncoder;

    private final PIDController turnPidController;

    private final AnalogInput absEncoder;
    private final SwerveModuleConfig config;

    public SwerveModule(SwerveModuleConfig config) {
        driveMotor = new CANSparkMax(config.driveMotorID, MotorType.kBrushless);
        turnMotor = new CANSparkMax(config.turnMotorID, MotorType.kBrushless);

        driveMotor.setInverted(config.driveInverted);
        turnMotor.setInverted(config.turnInverted);

        driveEncoder = driveMotor.getEncoder();
        turnEncoder = turnMotor.getEncoder();

        absEncoder = new AnalogInput(config.absEncoderID);

        this.config = config;

        turnPidController = new PIDController(DrivetrainConfig.TURN_P, 0, 0);
        turnPidController.enableContinuousInput(-Math.PI, Math.PI);
        
        resetEncoders();

    }
    
    public double getAbsEncoderRad(){
        double angle = absEncoder.getVoltage() / RobotController.getVoltage5V();
        angle *= 2.0 * Math.PI;
        angle -= config.absEncoderOffsetRad;
        return angle * (config.absEncoderInverted ? -1.0 : 1.0);
    }
        
    public void resetEncoders(){
        driveEncoder.setPosition(0);
        turnEncoder.setPosition(getAbsEncoderRad());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(driveEncoder.getVelocity(), new Rotation2d(driveEncoder.getPosition()));
    }

    public void setDesiredState(SwerveModuleState state){
        if(Math.abs(state.speedMetersPerSecond) < 0.001){
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);

        driveMotor.set(state.speedMetersPerSecond / DrivetrainConfig.MAX_PHYSICAL_SPEED);
        turnMotor.set(turnPidController.calculate(turnEncoder.getPosition(), state.angle.getRadians()));
    }
    
    public void stop() {
        driveMotor.set(0);
        turnMotor.set(0);
    }
}