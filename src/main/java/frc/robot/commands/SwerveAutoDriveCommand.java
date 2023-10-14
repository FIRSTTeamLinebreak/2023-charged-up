package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;

/** Controls the swerve subsystem via joysticks. */
public class SwerveAutoDriveCommand extends CommandBase {
    private final SwerveDrive swerveSub;

    private double xSpeed;
    private double ySpeed;
    private double turningSpeed;

    private final boolean fieldOrientedDrivingSupplier;

    /** Creates a new command to control the swerve subsystem via joysticks.
     *
     * @param xSpeed A supplier of the joystick X speed
     * @param ySpeed A supplier of the joystick Y speed
     * @param turningSpeed A supplier of the turning speed
     * @param fieldOrientedDrivingSupplier A supplier to say weather to drive relative to the field
     */
    public SwerveAutoDriveCommand(double xSpeed, double ySpeed, double turningSpeed, boolean fieldOrientedDrivingSupplier) {
        this.swerveSub = SwerveDrive.getInstance();

        this.xSpeed = xSpeed;
        this.ySpeed = ySpeed;
        this.turningSpeed = turningSpeed;

        this.fieldOrientedDrivingSupplier = fieldOrientedDrivingSupplier;

        addRequirements(swerveSub);
    }

    /** Called once when the command is initially scheduled. */
    @Override
    public void initialize() {
        swerveSub.setDirection(
            xSpeed, 
            ySpeed, 
            turningSpeed, 
            fieldOrientedDrivingSupplier);
    }

    /** Called repeatedly while the command is scheduled. */
    @Override
    public void execute() {}

    /** Called when either the command finishes normally, or when it interrupted/canceled. Do not schedule commands here that share requirements with this command. Use andThen(Command) instead.
     *
     * @param interrupted Weather this command was interrupted
     */
    @Override
    public void end(boolean interrupted) {
        swerveSub.stop();
    }

    /** Whether the command has finished. If true, calls end() and stops the command from executing
     *
     * @return boolean
     */
    @Override
    public boolean isFinished() {
        return false;
    }
}
