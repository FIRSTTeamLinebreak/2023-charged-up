package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** Identifies which controller is which. */
public class IdentifyControllers extends CommandBase {
    private boolean isTurningController;
    private boolean showTrue;

    /** Creates a command that identifies which controller is which.
     *
     * @param isTurningController Weather to identify the controller calling this command as the turning controller
     * @param showTrue Weather to identify this controller by putting true to the Smart Dashboard
     */
    public IdentifyControllers(boolean isTurningController, boolean showTrue) {
        this.isTurningController = isTurningController;
        this.showTrue = showTrue;
    }

    /** Called once when the command is initially scheduled. */
    @Override
    public void initialize() {
        SmartDashboard.putBoolean(isTurningController ? "Turning Controller" : "Drive Controller", showTrue);
    }

    /** Called repeatedly while the command is scheduled. */
    @Override
    public void execute() {}

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
        return true;
    }
}
