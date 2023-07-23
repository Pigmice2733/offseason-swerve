package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class RumbleController extends SequentialCommandGroup {

    /** Rumble the controller with a strength of 0.5 for 1 second */
    public RumbleController(XboxController controller, RumbleType type) {
        this(controller, type, 1.0, 0.5);
    }

    /** Rumble the controller for 1 second at a given strength */
    public RumbleController(XboxController controller, RumbleType type, double strength) {
        this(controller, type, 1.0, strength);
    }

    /** Rumble the controller with a given time and strength */
    public RumbleController(XboxController controller, RumbleType type, double seconds, double strength) {
        addCommands(
                new InstantCommand(() -> controller.setRumble(type, strength)),
                new WaitCommand(seconds),
                new InstantCommand(() -> controller.setRumble(type, 0)));
    }
}