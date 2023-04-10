package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DrivetrainConfig;
import frc.robot.subsystems.Drivetrain;

public class Controls {
    XboxController driver;
    XboxController operator;

    private double threshold = Constants.AXIS_THRESHOLD;

    // Create a new Controls
    public Controls(XboxController driver, XboxController operator) {
        this.driver = driver;
        this.operator = operator;
    }

    LinearFilter driveSpeedYFilter = LinearFilter.singlePoleIIR(0.1, 0.02);
    /** @return The Left Y Axis multiplied by the drive speed. */
    public double getDriveSpeedY() {
        double joystickValue = driver.getLeftY();
        joystickValue = MathUtil.applyDeadband(-joystickValue, threshold); // deals with stick drag
        joystickValue = driveSpeedYFilter.calculate(joystickValue); // input smoothing

        return joystickValue * DrivetrainConfig.DRIVE_SPEED;
    }

    LinearFilter driveSpeedXFilter = LinearFilter.singlePoleIIR(0.1, 0.02);
    /** @return The Left X Axis multiplied by the drive speed. */
    public double getDriveSpeedX() {
        double joystickValue = driver.getLeftX();
        joystickValue = MathUtil.applyDeadband(-joystickValue, threshold); // deals with stick drag
        joystickValue = driveSpeedXFilter.calculate(joystickValue); // input smoothing

        return joystickValue * DrivetrainConfig.DRIVE_SPEED;
    }

    LinearFilter turnSpeedFilter = LinearFilter.singlePoleIIR(0.1, 0.02);

    /** @return The Right X Axis multiplied by the turn speed. */
    public double getTurnSpeed() {
        double joystickValue = driver.getRightX();
        joystickValue = MathUtil.applyDeadband(joystickValue, threshold); // deals with stick drag
        joystickValue = turnSpeedFilter.calculate(joystickValue); // input smoothing

        return joystickValue * DrivetrainConfig.TURN_SPEED;
    }
}