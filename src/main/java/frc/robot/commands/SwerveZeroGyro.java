package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;

/** Zeros the gyroscope of the roboRio. */
public class SwerveZeroGyro extends CommandBase {
    private final SwerveDrive swerveDrive;

    /** Creates a new command that literally does nothing. */
    public SwerveZeroGyro() {
        this.swerveDrive = SwerveDrive.getInstance();
    }

    /** Called once when the command is initially scheduled. */
    @Override
    public void initialize() {}

    /** Called repeatedly while the command is scheduled. */
    @Override
    public void execute() {
        swerveDrive.zeroGyro();
    }

    /** Called when either the command finishes normally, or when it interrupted/canceled. Do not schedule commands here that share requirements with this command. Use andThen(Command) instead.
     *
     * @param interrupted Weather this command was interrupted
     */
    @Override
    public void end(boolean interrupted) {}

    /** Whether the command has finished. If true, calls end().
     *
     * @return Weather this command is done.
     */
    @Override
    public boolean isFinished() {
        return true;
    }
}
