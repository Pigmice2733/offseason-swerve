package frc.robot.subsystems;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SwerveModule;
import frc.robot.Constants.CANConfig;
import frc.robot.Constants.DrivetrainConfig;
import frc.robot.Constants.SwerveModuleConfig;


public class Drivetrain extends SubsystemBase{
    private final SwerveModule frontLeft = new SwerveModule(
        CANConfig.FRONT_LEFT_DRIVE,
        CANConfig.FRONT_LEFT_TURN,
        DrivetrainConfig.DRIVE_MOTOR_INVERTED,
        DrivetrainConfig.TURN_MOTOR_INVERTED,
        DrivetrainConfig.ABS_ENCODER_PORT,
        DrivetrainConfig.ABS_ENCODER_OFFSET,
        DrivetrainConfig.ABS_ENCODER_INVERTED);
    
    private final SwerveModule frontRight = new SwerveModule(
        CANConfig.FRONT_RIGHT_DRIVE,
        CANConfig.FRONT_RIGHT_TURN,
        DrivetrainConfig.DRIVE_MOTOR_INVERTED,
        DrivetrainConfig.TURN_MOTOR_INVERTED,
        DrivetrainConfig.ABS_ENCODER_PORT,
        DrivetrainConfig.ABS_ENCODER_OFFSET,
        DrivetrainConfig.ABS_ENCODER_INVERTED);

    private final SwerveModule backLeft = new SwerveModule(
        CANConfig.BACK_LEFT_DRIVE,
        CANConfig.BACK_LEFT_TURN,
        DrivetrainConfig.DRIVE_MOTOR_INVERTED,
        DrivetrainConfig.TURN_MOTOR_INVERTED,
        DrivetrainConfig.ABS_ENCODER_PORT,
        DrivetrainConfig.ABS_ENCODER_OFFSET,
        DrivetrainConfig.ABS_ENCODER_INVERTED);

    private final SwerveModule backRight = new SwerveModule(
        CANConfig.BACK_LEFT_DRIVE,
        CANConfig.BACK_LEFT_TURN,
        DrivetrainConfig.DRIVE_MOTOR_INVERTED,
        DrivetrainConfig.TURN_MOTOR_INVERTED,
        DrivetrainConfig.ABS_ENCODER_PORT,
        DrivetrainConfig.ABS_ENCODER_OFFSET,
        DrivetrainConfig.ABS_ENCODER_INVERTED);

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(new Translation2d(-1, 1), new Translation2d(1, 1), new Translation2d(-1, -1), new Translation2d(-1, -1));
    private final AHRS gyro = new AHRS(); 

    public Drivetrain() {
        new Thread(() -> {
            try {
              Thread.sleep(1000);
              zeroHeading();
            } catch (Exception e) { }
          }).start();
    }

        
    public void zeroHeading() {
        gyro.reset();
    }

    public double getHeading() {
        return Math.IEEEremainder(gyro.getAngle(), 360);
    }

    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    @Override
    public void periodic(){
    }
        
    public void stopModules() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }
        
    public void setModuleStates(SwerveModuleState[] desiredStates){
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DrivetrainConfig.MAX_PHYSICAL_SPEED);
        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);
    }

    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }
}