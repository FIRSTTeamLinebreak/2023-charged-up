package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.SwerveJoystickDriveCommand;

/** Scores and exits during auto. */
public class AutoScoreAndExit extends SequentialCommandGroup {
    /** Creates a command group to score and exit during auto. */
    public AutoScoreAndExit() {
        addCommands(
            new SwerveJoystickDriveCommand(() -> 0.0, () -> 0.50,  () -> 0.0, () -> true).withTimeout(0.10), // Drop the pre-load cube via a jerk forward
            new SwerveJoystickDriveCommand(() -> 0.0, () -> 0.0,   () -> 0.0, () -> true).withTimeout(0.25), // Wait
            new SwerveJoystickDriveCommand(() -> 0.0, () -> -0.25, () -> 0.0, () -> true).withTimeout(1.0), // Score the cube via moving back
            new SwerveJoystickDriveCommand(() -> 0.0, () -> 0.0,   () -> 0.0, () -> true).withTimeout(0.25), // Wait
            new SwerveJoystickDriveCommand(() -> 0.0, () -> 0.3,   () -> 0.0, () -> true).withTimeout(3) // Exit the community
        );
    }
}
