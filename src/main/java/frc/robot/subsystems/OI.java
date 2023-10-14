package frc.robot.subsystems;

import static frc.robot.Util.applyLinearDeadzone;

import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OiConstants;
import frc.robot.commands.CraneControlCommand;
import frc.robot.commands.SwerveJoystickDriveCommand;

public class OI extends SubsystemBase {
    // Define the singleton
    private static OI instance;

    /**
     * Get this instance.
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

    private double slowTurningDivisor = 4;
    private final double pivotIncrementor = 2.4;
    private final double armIncrementor = 0.5;
    private final double targetClawSpeed = 0.3;

    /** Initializes a new Turntable subsystem object. */
    private OI() {
        super();
        swerveSub = SwerveDrive.getInstance();
        craneSub = Crane.getInstance();

        driveController = new CommandXboxController(0);
        turningController = new CommandXboxController(1);

        PIDController visionPid = new PIDController(0.02, 0, 0);

        // Swerve control
        swerveSub.setDefaultCommand(new SwerveJoystickDriveCommand(
                () -> driveController.getLeftX() * -1,
                () -> driveController.getLeftY() * -1,
                () -> {
                    // Rotation
                    PhotonPipelineResult result = Vision.getInstance().getResult();
                    if (driveController.getHID().getAButton()) {
                        if (result.hasTargets()) {
                            double val = visionPid.calculate(result.getBestTarget().getYaw(), 0);
                            return val;
                        }
                    }
                    if (applyLinearDeadzone(OiConstants.joystickDeadzone, driveController.getRightX()) != 0) {
                        if (driveController.getHID().getLeftBumper()) {
                            return driveController.getRightX() / slowTurningDivisor * -1;
                        }
                        return driveController.getRightX() * -1;
                    }
                    return 0.0;
                },
                () -> !driveController.getHID().getRightBumper()));

        // Crane control
        craneSub.setDefaultCommand(new CraneControlCommand(
                () -> {
                    // Pivot Target
                    double curPos = craneSub.getPivotPosition();
                    double pivotMultiplier = applyLinearDeadzone(OiConstants.joystickDeadzone,
                            turningController.getRightY())
                            * -1;

                    if (craneSub.getFrameSwitch() && pivotMultiplier < 0.0) {
                        pivotMultiplier = 0.0;
                    }

                    return curPos + pivotIncrementor * pivotMultiplier;
                },
                () -> {
                    // Pivot Target
                    double curPos = craneSub.getArmPosition();
                    double armMultiplier = applyLinearDeadzone(OiConstants.joystickDeadzone,
                            turningController.getLeftY())
                            * -1;

                    if (craneSub.getArmSwitch() && armMultiplier < 0.0) {
                        armMultiplier = 0.0;
                    }

                    return curPos + armIncrementor * armMultiplier;
                },
                () -> {
                    // Claw Speed
                    double curSpeed = craneSub.getClawSpeed();

                    if (turningController.getHID().getXButtonReleased()) { // Claw out
                        return curSpeed == 0.0 ? targetClawSpeed : 0.0;
                    }
                    if (turningController.getHID().getXButtonReleased()) { // Claw in
                        return curSpeed == 0.0 ? targetClawSpeed * -1 : 0.0;
                    }
                    return curSpeed;
                }));
    }

    @Override
    public void periodic() {
        if (craneSub.getArmSwitch()) {
            craneSub.zeroArmEncoder();
        }

        if (craneSub.getFrameSwitch()) {
            craneSub.zeroPivotEncoder();
        }
    }
}
