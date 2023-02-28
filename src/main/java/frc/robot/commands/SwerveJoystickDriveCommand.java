package frc.robot.commands;

import static frc.robot.Util.applyCircularDeadzone;
import static frc.robot.Util.applyLinearDeadzone;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.OiConstants;
import frc.robot.Constants.SwerveSubsystemConstants;
import frc.robot.subsystems.SwerveDrive;
import java.util.function.Supplier;

/** A command to control the swerve subsystem via joysticks. */
public class SwerveJoystickDriveCommand extends CommandBase {
    private final SwerveDrive swerveSubsystem;

    private final Supplier<Double> xSpeedSupplier;
    private final Supplier<Double> ySpeedSupplier;
    private final Supplier<Double> turningSpeedSupplier;

    private final Supplier<Boolean> fieldOrientedDrivingSupplier;

    /** Creates a new object to control the swerve drive with the joysticks.
     *
     * @param xSpeedSupplier A supplier of the joystick X speed
     * @param ySpeedSupplier A supplier of the joystick Y speed
     * @param turningSpeedSupplier A supplier of the turning speed
     * @param fieldOrientedDrivingSupplier A supplier to say weather to drive relative to the field
     */
    public SwerveJoystickDriveCommand(Supplier<Double> xSpeedSupplier, Supplier<Double> ySpeedSupplier, Supplier<Double> turningSpeedSupplier, Supplier<Boolean> fieldOrientedDrivingSupplier) {
        this.swerveSubsystem = SwerveDrive.getInstance();

        this.xSpeedSupplier = xSpeedSupplier;
        this.ySpeedSupplier = ySpeedSupplier;
        this.turningSpeedSupplier = turningSpeedSupplier;

        this.fieldOrientedDrivingSupplier = fieldOrientedDrivingSupplier;

        addRequirements(swerveSubsystem);
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
        ChassisSpeeds chassisSpeed;

        // Implement a deadzone to prevent joystick drift from becoming a problem
        Double[] deadzones = applyCircularDeadzone(OiConstants.joystickDeadzone, xSpeed, ySpeed);
        xSpeed = deadzones[0];
        ySpeed = deadzones[1];
        turningSpeed = applyLinearDeadzone(OiConstants.joystickDeadzone, turningSpeed);

        // Multiply joystick speeds by their multipliers
        xSpeed *= OiConstants.xySpeedMultiplier;
        ySpeed *= OiConstants.xySpeedMultiplier;
        turningSpeed *= OiConstants.turningSpeedMultiplier;

        // Convert speeds to chassis speeds
        if (fieldOrientedDrivingSupplier.get()) {
            chassisSpeed = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, (-1 * ySpeed), (-1 * turningSpeed), swerveSubsystem.getRotation2d());
        } else {
            chassisSpeed = new ChassisSpeeds((-1 * ySpeed), (-1 * xSpeed), (-1 * turningSpeed)); // Yes, ChassisSpeeds wants x, y, and turning and we give it y, x, turning. This is the fix to rotate the controls 90 degrees
        }

        // Create swerve module states for the desired movement and push to subsystem
        SwerveModuleState[] moduleStates = SwerveSubsystemConstants.driveKinematics.toSwerveModuleStates(chassisSpeed);
        swerveSubsystem.setStates(moduleStates, true);
    }

    /** Called when either the command finishes normally, or when it interrupted/canceled. Do not schedule commands here that share requirements with this command. Use andThen(Command) instead.
     *
     * @param interrupted Weather this command was interrupted
     */
    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stop();
    }

    /** Whether the command has finished. If true, calls end() and stops the command from executing
     *
     * @return Weather this command is done.
     */
    @Override
    public boolean isFinished() {
        return false;
    }
}
