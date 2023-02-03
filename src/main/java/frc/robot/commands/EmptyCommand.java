package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;

/** A command that does literally nothing. */
public class EmptyCommand extends CommandBase {

    /** Creates a new command that literally does nothing. */
    public EmptyCommand() {

    }

    /** Called once when the command is initially scheduled. */
    @Override
    public void initialize() {

    }

    /** Called repeatedly while the command is scheduled. */
    @Override
    public void execute() {

    }

    /** Called when either the command finishes normally, or when it interrupted/canceled. Do not schedule commands here that share requirements with this command. Use andThen(Command) instead.
     *
     * @param interrupted Weather this command was interrupted
     */
    @Override
    public void end(boolean interrupted) {

    }

    /** Whether the command has finished. If true, calls end().
     *
     * @return Weather this command is done.
     */
    @Override
    public boolean isFinished() {
        return false;
    }
}
