package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Lights;

public class LEDControlCommand extends CommandBase {
    private Lights lightSub;
    private int r, g, b;

    /** Creates a command that automatically zeros the pivot and arm encoders. */
    public LEDControlCommand(int r, int g, int b) {
        lightSub = Lights.getInstance();

        this.r = r;
        this.g = g;
        this.b = b;

        addRequirements(lightSub);
    }

    /** Called once when the command is initially scheduled. */
    @Override
    public void initialize() {
        lightSub.setRedVal(r);
        lightSub.setGreenVal(g);
        lightSub.setBlueVal(b);
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
