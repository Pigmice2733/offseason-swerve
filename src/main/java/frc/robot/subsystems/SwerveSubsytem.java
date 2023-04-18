
public class Drivetrain extends SubsystemBase{
    private final SwerveModule frontLeft = new SwerveModule(
        DriveConstants.kFrontLeftDriveMotorPort,
        DriveConstants.kFrontLeftTurningMotorPort,
        DriveConstants.kFrontLeftDriveEncoderReversed,
        DriveConstants.kFrontLeftTurningEncoderReversed,
        DriveConstants.kFrontLeftDriveAbsoluteEncoderPort
        DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad,
        DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed);
    
    private final SwerveModule frontRight = new SwerveModule(
        DriveConstants.kFrontRightDriveMotorPort,
        DriveConstants.kFrontRightTurningMotorPort,
        DriveConstants.kFrontRightDriveEncoderReversed,
        DriveConstants.kFrontRightTurningEncoderReversed,
        DriveConstants.kFrontRightDriveAbsoluteEncoderPort
        DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad,
        DriveConstants.kFrontRightDriveAbsoluteEncoderReversed);

    private final SwerveModule BackLeft = new SwerveModule(
        DriveConstants.kBackLeftDriveMotorPort,
        DriveConstants.kBackLeftTurningMotorPort,
        DriveConstants.kBackLeftDriveEncoderReversed,
        DriveConstants.kBackLeftTurningEncoderReversed,
        DriveConstants.kBackLeftDriveAbsoluteEncoderPort
        DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad,
        DriveConstants.kBackLeftDriveAbsoluteEncoderReversed);
        
    )

    private final SwerveModule BackRight = new SwerveModule(
        DriveConstants.kBackRightDriveMotorPort,
        DriveConstants.kBackRightTurningMotorPort,
        DriveConstants.kBackRightDriveEncoderReversed,
        DriveConstants.kBackRightTurningEncoderReversed,
        DriveConstants.kBackRightDriveAbsoluteEncoderPort
        DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad,
        DriveConstants.kBackRightDriveAbsoluteEncoderReversed);

    )

    private final AHRS gyro = new AHRS(SPI.Port.kMXP);

    public SwerveSubsystem() {
        new Thread(()-> {
            Try {
                Thread.sleep(1000);
                zeroHeading();
            } catch (Exception e) {
            }
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

        @Overide
        public void periodic(){
            SmartDashboard.putNumber("Robot Heading", getHeading());
        }
         
        public void stopModules() {
            fronLeft.stop();
            frontRight.stop();
            backLeft.stop();
            backRight.stop();
        }
         
        public void setModuleStates(SwerveModuleState[] desiredStates){
            SwerveDriveKinematics.normalizeWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
            frontLeft.setDesiredState(desiredStates[0]);
            frontRight.setDesiredState(desiredStates[1]);
            backLeft.setDesiredState(desiredStates[2]);
            backRight.setDesiredState(desiredStates[3]);
        }
    }
}