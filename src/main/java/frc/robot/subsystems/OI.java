package frc.robot.subsystems;

import static frc.robot.Util.applyLinearDeadzone;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OiConstants;
import frc.robot.commands.CraneControlCommand;
import frc.robot.commands.SwerveJoystickDriveCommand;

public class OI extends SubsystemBase {
    // Define the singleton
    private static OI instance;

    /** Get this instance.
     *
     * @return The subsystem
     */
    public static OI getInstance() {
        if (instance == null) {
            instance = new OI();
        }

        return instance;
    }

    private final SwerveDrive swerveSub;
    private final Crane craneSub;

    // Controllers
    private CommandXboxController driveController;
    private CommandXboxController turningController;

    private double swerveTargetTurningSpeed = 0.0;
    private double slowTurningDivisor = 4;

    private final double fastPivotIncrementor = 2.4;
    private final double slowPivotIncrementor = 1.1;

    private final double armIncrementor = 1.5;
    private final double targetClawSpeed = 0.3;

    /** Initializes a new Turntable subsystem object. */
    private OI() {
        swerveSub = SwerveDrive.getInstance();
        craneSub = Crane.getInstance();

        driveController = new CommandXboxController(0);
        turningController = new CommandXboxController(1);

        // PIDController alignPID = new PIDController(0.02, 0, 0);

        // swerveSub.setDefaultCommand(new SwerveJoystickDriveCommand(
        //         () -> 0.0,
        //         () -> 0.0,
        //         () -> {
        //             PhotonPipelineResult result = visionSub.getResult();
        //             if (result.hasTargets()) {
        //                 double val = alignPID.calculate(result.getBestTarget().getYaw(), 0);
        //                 SmartDashboard.putString("Target Data", result.getBestTarget().getYaw() + " " + val);
        //                 return val;
        //             }
        //             return 0.0;
        //         },
        //         () -> false));

        // Swerve control
        swerveSub.setDefaultCommand(new SwerveJoystickDriveCommand(
                () -> driveController.getLeftX() * -1,
                () -> driveController.getLeftY() * -1,
                () -> swerveTargetTurningSpeed * -1,
                () -> !driveController.getHID().getRightBumper()));

        // Crane control
        craneSub.setDefaultCommand(new CraneControlCommand(
                () -> {
                    // Pivot Target
                    double curPos = craneSub.getPivotPosition();

                    // Fast Increment
                    if (applyLinearDeadzone(OiConstants.joystickDeadzone, turningController.getRightY()) > 0 && !craneSub.getFrameSwitch()) {
                        return curPos - fastPivotIncrementor;
                    } else if (applyLinearDeadzone(OiConstants.joystickDeadzone, turningController.getRightY()) < 0) {
                        return curPos + fastPivotIncrementor;
                    }
                    
                    // Slow Increment
                    if (applyLinearDeadzone(OiConstants.joystickDeadzone, turningController.getLeftY()) > 0 && !craneSub.getFrameSwitch()) {
                        return curPos - slowPivotIncrementor;
                    } else if (applyLinearDeadzone(OiConstants.joystickDeadzone, turningController.getLeftY()) < 0) {
                        return curPos -= slowPivotIncrementor;
                    }
                    return curPos;
                },
                () -> {
                    // Arm Target
                    double curPos = craneSub.getArmPosition();
                    if (applyLinearDeadzone(OiConstants.triggerDeadzone, turningController.getLeftTriggerAxis()) > 0) { 
                        return curPos - armIncrementor;
                    } else if (turningController.getHID().getLeftBumper() && !craneSub.getArmSwitch()) {
                        return curPos + armIncrementor;
                    }
                    return curPos;
                },
                () -> {
                    // Claw Speed
                    if (turningController.getHID().getRightBumper()) { // Claw out
                        return targetClawSpeed;
                    } else if (applyLinearDeadzone(OiConstants.triggerDeadzone,
                            turningController.getHID().getRightTriggerAxis()) > 0) { // Claw in
                        return targetClawSpeed * -1;
                    } 
                    return 0.0;
                }));
    }

    @Override
    public void periodic() {
        // Swerve control
        if (applyLinearDeadzone(OiConstants.joystickDeadzone, driveController.getRightX()) != 0) {
            if (driveController.getHID().getLeftBumper()) { // Decrease speed
                swerveTargetTurningSpeed = driveController.getRightX() / slowTurningDivisor;
            } else {
                swerveTargetTurningSpeed = driveController.getRightX();
            }
        } else {
            swerveTargetTurningSpeed = 0;
        }

        if (craneSub.getArmSwitch()) {
            craneSub.zeroArmEncoder();
        }

        if (craneSub.getFrameSwitch()) {
            craneSub.zeroPivotEncoder();
        }
    }
}
