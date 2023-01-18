package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drive;

/** Used in auto to control the robot. */
public class DriveCommand extends CommandBase {
    private Drive driveSub;
    private double xSpeed;
    private double ySpeed;
    private double rotation;

    /** The main command used to direct the robot.
     *
     * @param xSpeed target speed in the left/right direction (-1, 1)
     * @param ySpeed target speed in the forward/backward direction (-1, 1)
     * @param rotation the target speed to rotate the robot in (-1, 1)
     */
    public DriveCommand(double xSpeed, double ySpeed, double rotation) {
        driveSub = Drive.getInstance();
        addRequirements(driveSub);
        this.xSpeed = xSpeed;
        this.ySpeed = ySpeed;
        this.rotation = rotation;
    }

    /** Called every time the scheduler runs while the command is scheduled. */
    @Override
    public void execute() {
        driveSub.setxSpeed(xSpeed);
        driveSub.setySpeed(ySpeed);
        driveSub.setRotation(rotation);
    }

    /** Called once the command ends or is interrupted.
     *
     * @param interrupted is true if the command is interrupted during execution
     */
    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            driveSub.setxSpeed(0);
            driveSub.setySpeed(0);
            driveSub.setRotation(0);
        }
    }

    /** Returns true as the command is done. */
    @Override
    public boolean isFinished() {
        return false;
    }
}
