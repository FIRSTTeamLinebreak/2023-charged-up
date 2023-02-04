package frc.robot.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.OiConstants;
import frc.robot.Constants.SwerveMotorConstants;
import frc.robot.subsystems.SwerveDrive;
import java.util.function.Supplier;

/** A command to control the swerve subsystem via joysticks. */
public class SwerveJoystickDriveCommand extends CommandBase {
    private final SwerveDrive swerveSubsystem;

    private final Supplier<Double> xSpeedSupplier;
    private final Supplier<Double> ySpeedSupplier;
    private final Supplier<Double> turningSpeedSupplier;

    private final SlewRateLimiter xSpeedLimiter;
    private final SlewRateLimiter ySpeedLimiter;
    private final SlewRateLimiter turningSpeedLimiter;

    private final Supplier<Boolean> fieldOrientedDrivingSupplier;

    /** Creates a new object to control the swerve drive with the joysticks.
     *
     * @param swerveSubs Swerve drive object
     * @param xSpdSup A supplier of the joystick X speed
     * @param ySpdSup A supplier of the joystick Y speed
     * @param turnSpdSup A supplier of the turning speed
     * @param fieldOrientSup A supplier to say weather to drive relative to the field
     */
    public SwerveJoystickDriveCommand(SwerveDrive swerveSubs, Supplier<Double> xSpdSup, Supplier<Double> ySpdSup, Supplier<Double> turnSpdSup, Supplier<Boolean> fieldOrientSup) {
        swerveSubsystem = swerveSubs;

        xSpeedSupplier = xSpdSup;
        ySpeedSupplier = ySpdSup;
        turningSpeedSupplier = turnSpdSup;

        xSpeedLimiter = new SlewRateLimiter(OiConstants.driveMaxAccel);
        ySpeedLimiter = new SlewRateLimiter(OiConstants.driveMaxAccel);
        turningSpeedLimiter = new SlewRateLimiter(OiConstants.turningMaxAccel);

        fieldOrientedDrivingSupplier = fieldOrientSup;
    }

    /** Called once when the command is initially scheduled. */
    @Override
    public void initialize() {

    }

    /** Called repeatedly while the command is scheduled. */
    @Override
    public void execute() {
        double xSpeed = xSpeedSupplier.get();
        double ySpeed = ySpeedSupplier.get();
        double turningSpeed = turningSpeedSupplier.get();
        ChassisSpeeds chassisSpeed;

        // Implement a deadzone to prevent joystick drift from becoming a problem
        xSpeed = (Math.abs(xSpeed) > OiConstants.joystickDeadzone ? xSpeed : 0.0);
        ySpeed = (Math.abs(ySpeed) > OiConstants.joystickDeadzone ? ySpeed : 0.0);
        turningSpeed = (Math.abs(turningSpeed) > OiConstants.joystickDeadzone ? turningSpeed : 0.0);

        // Make driving smoother (Using a rate limiter and then scale using the teleop max speed)
        xSpeed = xSpeedLimiter.calculate(xSpeed) * OiConstants.driveMaxSpeed;
        ySpeed = ySpeedLimiter.calculate(ySpeed) * OiConstants.driveMaxSpeed;
        turningSpeed = turningSpeedLimiter.calculate(turningSpeed) * OiConstants.turningMaxSpeed;

        // Convert speeds to chassis speeds
        if (fieldOrientedDrivingSupplier.get()) {
            chassisSpeed = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d());
        } else {
            chassisSpeed = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
        }

        // Create swerve module states for the desired movement and push to subsystem
        SwerveModuleState[] moduleStates = SwerveMotorConstants.driveKinematics.toSwerveModuleStates(chassisSpeed);
        swerveSubsystem.setStates(moduleStates);
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
