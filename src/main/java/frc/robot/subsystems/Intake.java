package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** The subsystem that handles the intake mechanism for game pieces. */
public class Intake extends SubsystemBase {
    // Define the singleton
    private static Intake instance;

    /** Get this instance.
     *
     * @return The subsystem
     */
    public static Intake getInstance() {
        if (instance == null) {
            instance = new Intake();
        }

        return instance;
    }

    // Rollers
    private final CANSparkMax topRoller;
    private final CANSparkMax bottomRoller;

    private final double targetTopRollerSpeed = 0.5; // Speed of top rollers [-1, 1]. @TODO: Tune
    private final double targetBottomRollerSpeed = 0.5; // Speed of bottom rollers [-1, 1]. @TODO: Tune

    private boolean intakeRunning = false;

    // Deployment
    private final CANSparkMax deploymentMotor1;
    private final CANSparkMax deploymentMotor2;

    private final DigitalInput fullDeploymentLimitSwitch;
    private final DigitalInput fullRetractionLimitSwitch;

    private final double targetDeploymentSpeed = 0.5; // Speed of deployment [0, 1]. @TODO: Tune
    private boolean deployed = false;

    /** Initializes a new Intake subsystem object. */
    private Intake() {
        // Init rollers
        topRoller = new CANSparkMax(0, MotorType.kBrushless); // @TODO: Set ID
        bottomRoller = new CANSparkMax(0, MotorType.kBrushless); // @TODO: Set ID

        // Init deployment
        deploymentMotor1 = new CANSparkMax(0, MotorType.kBrushless); // @TODO: Set ID
        deploymentMotor2 = new CANSparkMax(0, MotorType.kBrushless); // @TODO: Set ID

        deploymentMotor2.follow(deploymentMotor1, true);

        fullDeploymentLimitSwitch = new DigitalInput(0); // @TODO: Set ID
        fullRetractionLimitSwitch = new DigitalInput(0); // @TODO: Set ID
    }

    /** Toggles the intake mechanism. */
    public void toggleIntake() {
        if (intakeRunning) {
            topRoller.set(targetTopRollerSpeed);
            bottomRoller.set(targetBottomRollerSpeed);
        } else {
            topRoller.set(0.0);
            bottomRoller.set(0.0);
        }

        intakeRunning = !intakeRunning;
        SmartDashboard.putBoolean("Intake Activated", intakeRunning);
    }

    /** Toggles the deployment mechanism. */
    public void toggleDeployment() {
        if (deployed) {
            while (fullRetractionLimitSwitch.get()) {
                deploymentMotor1.set(targetDeploymentSpeed);
            }
        } else {
            while (fullDeploymentLimitSwitch.get()) {
                deploymentMotor1.set(-1 * targetDeploymentSpeed);
            }
        }

        deploymentMotor1.set(0.0); // Stop the motors that were set in the while loops

        deployed = !deployed;
        SmartDashboard.putBoolean("Intake Deployed", deployed);
    }
}
