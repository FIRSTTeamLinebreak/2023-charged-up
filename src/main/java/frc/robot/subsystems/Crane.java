package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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
    private final DigitalInput pivotSwitch;
    private double pivotSpeed = 0.0;
    // @TODO: Set all of these
    public static final double pivotFrontTop = 0.0;
    public static final double pivotFrontMid = 6.0;
    public static final double pivotFrontBottom = 0.0;
    public static final double pivotTurntableIntake = 0.0;
    public static final double pivotMax = 0.0;
    public static final double pivotMin = 0.0;

    // Arm
    private final CANSparkMax armMotor;
    private final DigitalInput armSwitch;
    private double armSpeed = 0.0;
    // @TODO: Set all of these
    public static final double armMax = 0.0;
    public static final double armMid = 0.0;
    public static final double armMin = 0.0;
    public static final double armTurntableIntake = 0.0;
    public static final double armGroundIntake = 0.0;
    public static final double armHumanLoad = 0.0;
    public static final double armDropLow = 0.0;
    public static final double armDropMid = 0.0;
    public static final double armDropHigh = 0.0;

    // Claw
    private final CANSparkMax clawMotor;
    private final CANSparkMax clawMotorFollower;
    private double clawSpeed = 0.0;

    /** Initializes a new Crane subsystem object. */
    private Crane() {
        pivotMotor = new CANSparkMax(13, MotorType.kBrushless);
        pivotSwitch = new DigitalInput(0);

        armMotor = new CANSparkMax(14, MotorType.kBrushless);
        armSwitch = new DigitalInput(1);

        clawMotor = new CANSparkMax(15, MotorType.kBrushless);
        clawMotorFollower = new CANSparkMax(16, MotorType.kBrushless);
        clawMotorFollower.follow(clawMotor, true);
    }

    /** Run approx. every 20 ms. */
    @Override
    public void periodic() {
        pivotMotor.set(pivotSpeed);
        armMotor.set(armSpeed);
        clawMotor.set(clawSpeed);
    }

    /** Zeros all motor speeds. */
    public void stop() {
        pivotSpeed = 0.0;
        armSpeed = 0.0;
        clawSpeed = 0.0;
    }

    /** Zeros the pivot encoder. */
    public void zeroPivotEncoder() {
        pivotMotor.getEncoder().setPosition(0);
    }

    /** Zeros the arm encoder. */
    public void zeroArmEncoder() {
        armMotor.getEncoder().setPosition(0);
    }

    /** Weather the pivot limit switch is pressed.
     *
     * @return boolean
     */
    public boolean getPivotSwitch() {
        return pivotSwitch.get();
    }

    /** Weather the arm limit switch is pressed.
     *
     * @return boolean
     */
    public boolean getArmSwitch() {
        return armSwitch.get();
    }

    /** Gets the position of the pivot motor.
     *
     * @return Position in revolutions
     */
    public double getPivotPosition() {
        return pivotMotor.getEncoder().getPosition();
    }

    /** Gets the position of the arm motor.
     *
     * @return Position in revolutions
     */
    public double getArmPosition() {
        return armMotor.getEncoder().getPosition();
    }

    /** Sets the speed of the pivot motor.
     *
     * @param speed Speed from [-1, 1]
     */
    public void setPivotSpeed(double speed) {
        pivotSpeed = speed;
    }

    /** Sets the speed of the arm motor.
     *
     * @param speed Speed from [-1, 1]
     */
    public void setArmSpeed(double speed) {
        armSpeed = speed;
    }

    /** Sets the speed of the claw motor.
     *
     * @param speed Speed from [-1, 1]
     */
    public void setClawSpeed(double speed) {
        clawSpeed = speed;
    }
}
