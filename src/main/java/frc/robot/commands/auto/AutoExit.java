package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.SwerveJoystickDriveCommand;

/** Exits during auto. */
public class AutoExit extends SequentialCommandGroup {
    /** Creates a command group to exit during auto. */
    public AutoExit() {
        addCommands(
            new SwerveJoystickDriveCommand(() -> 0.0, () -> 0.3,   () -> 0.0, () -> true).withTimeout(3) // Exit the community
        );
    }
}
