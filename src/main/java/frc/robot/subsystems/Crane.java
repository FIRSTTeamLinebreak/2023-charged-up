package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;

/** The subsystem that handles the crane mechanism for placing game pieces. */
public class Crane extends SubsystemBase {
    // Define the singleton
    private static Crane instance;

    /** Get this instance.
     *
     * @return The subsystem
     */
    public static Crane getInstance() {
        if (instance == null) {
            instance = new Crane();
        }

        return instance;
    }

    // Pivot
    private final CANSparkMax pivotMotor;
    private Supplier<Double> pivotMotorSpeedSupplier;

    // Arm
    private final CANSparkMax armMotor;
    private Supplier<Double> armMotorSpeedSupplier;

    // Claw
    private final CANSparkMax clawMotor;
    private Supplier<Double> clawMotorSpeedSupplier;

    /** Initializes a new Crane subsystem object. */
    private Crane() {
        pivotMotor = new CANSparkMax(0, MotorType.kBrushless); // @TODO: Set ID
        pivotMotorSpeedSupplier = () -> 0.0;

        armMotor = new CANSparkMax(0, MotorType.kBrushless); // @TODO: Set ID
        armMotorSpeedSupplier = () -> 0.0;

        clawMotor = new CANSparkMax(0, MotorType.kBrushless); // @TODO: Set ID
        clawMotorSpeedSupplier = () -> 0.0;
    }

    /** Run approx. every 20 ms. */
    @Override
    public void periodic() {
        pivotMotor.set(pivotMotorSpeedSupplier.get());
        armMotor.set(armMotorSpeedSupplier.get());
        clawMotor.set(clawMotorSpeedSupplier.get());
    }

    /** Sets the suppliers for the pivot, arm, and claw motors. */
    public void setSuppliers(Supplier<Double> pivot, Supplier<Double> arm, Supplier<Double> claw) {
        pivotMotorSpeedSupplier = pivot;
        armMotorSpeedSupplier = arm;
        clawMotorSpeedSupplier = claw;
    }
}
