package frc.robot.commands;

import java.util.function.Supplier;

import static frc.robot.Util.applyCircularDeadzone;
import static frc.robot.Util.applyLinearDeadzone;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.OiConstants;
import frc.robot.subsystems.SwerveDrive;

/** Controls the swerve subsystem via joysticks. */
public class SwerveJoystickDriveCommand extends CommandBase {
    private final SwerveDrive swerveSub;

    private final Supplier<Double> xSpeedSupplier;
    private final Supplier<Double> ySpeedSupplier;
    private final Supplier<Double> turningSpeedSupplier;

    private final Supplier<Boolean> fieldOrientedDrivingSupplier;

    /** Creates a new command to control the swerve subsystem via joysticks.
     *
     * @param xSpeedSupplier A supplier of the joystick X speed
     * @param ySpeedSupplier A supplier of the joystick Y speed
     * @param turningSpeedSupplier A supplier of the turning speed
     * @param fieldOrientedDrivingSupplier A supplier to say weather to drive relative to the field
     */
    public SwerveJoystickDriveCommand(Supplier<Double> xSpeedSupplier, Supplier<Double> ySpeedSupplier, Supplier<Double> turningSpeedSupplier, Supplier<Boolean> fieldOrientedDrivingSupplier) {
        this.swerveSub = SwerveDrive.getInstance();

        this.xSpeedSupplier = xSpeedSupplier;
        this.ySpeedSupplier = ySpeedSupplier;
        this.turningSpeedSupplier = turningSpeedSupplier;

        this.fieldOrientedDrivingSupplier = fieldOrientedDrivingSupplier;

        addRequirements(swerveSub);
    }

    /** Called once when the command is initially scheduled. */
    @Override
    public void initialize() {}

    /** Called repeatedly while the command is scheduled. */
    @Override
    public void execute() {
        double xSpeed = xSpeedSupplier.get();
        double ySpeed = ySpeedSupplier.get();
        double turningSpeed = turningSpeedSupplier.get();

        // Implement a deadzone to prevent joystick drift from becoming a problem
        Pair<Double, Double> speeds = applyCircularDeadzone(OiConstants.joystickDeadzone, xSpeed, ySpeed);
        xSpeed = speeds.getFirst();
        ySpeed = speeds.getSecond();
        turningSpeed = applyLinearDeadzone(OiConstants.joystickDeadzone, turningSpeed);

        swerveSub.setDirection(
                xSpeed, 
                ySpeed, 
                turningSpeed, 
                fieldOrientedDrivingSupplier.get());
    }

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
