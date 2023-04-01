package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Crane;

/** Automatically zeros the pivot and arm encoders. */
public class CraneAutoZero extends CommandBase {
    private Crane craneSub;
    private boolean finished = false;

    /** Creates a command that automatically zeros the pivot and arm encoders. */
    public CraneAutoZero() {
        craneSub = Crane.getInstance();
    }

    /** Called once when the command is initially scheduled. */
    @Override
    public void initialize() {
        craneSub.setArmSpeed(-0.1);
    }

    /** Called repeatedly while the command is scheduled. */
    @Override
    public void execute() {
        if (craneSub.getArmSwitch()) {
            craneSub.setArmSpeed(0);
            craneSub.zeroArmEncoder();
            finished = true;
        }
    }

    /** Called when either the command finishes normally, or when it interrupted/canceled. Do not schedule commands here that share requirements with this command. Use andThen(Command) instead.
     *
     * @param interrupted Weather this command was interrupted
     */
    @Override
    public void end(boolean interrupted) {}

    /** Whether the command has finished. If true, calls end().
     *
     * @return boolean
     */
    @Override
    public boolean isFinished() {
        return finished;
    }
}
