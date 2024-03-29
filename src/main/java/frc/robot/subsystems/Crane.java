package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
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

    private final CANSparkMax pivotMotor;
    private final CANSparkMax armMotor;

    private final CANSparkMax clawMotor;
    private final CANSparkMax clawMotorFollower;

    private final DigitalInput armSwitch;
    private final DigitalInput frameSwitch;

    private double cranePivotTargetPosition = 0.0;
    private double craneArmTargetPosition = 0.0;

    private final PIDController pivotPidController;
    private final PIDController armPidController;
    private double clawSpeed = 0.0;

    /** Initializes a new Crane subsystem object. */
    private Crane() {
        pivotPidController = new PIDController(0.175, 0.07, 0.0025);
        armPidController = new PIDController(0.12, 0.1, 0.0025);

        pivotPidController.setTolerance(0.25);
        armPidController.setTolerance(0.125);

        pivotMotor = new CANSparkMax(13, MotorType.kBrushless);

        armMotor = new CANSparkMax(14, MotorType.kBrushless);
        armSwitch = new DigitalInput(0);
        frameSwitch = new DigitalInput(1);

        clawMotor = new CANSparkMax(15, MotorType.kBrushless);
        clawMotorFollower = new CANSparkMax(16, MotorType.kBrushless);
        clawMotorFollower.follow(clawMotor, true);
    }

    /** Run approx. every 20 ms. */
    @Override
    public void periodic() {
        pivotMotor.set(pivotPidController.calculate(this.getPivotPosition(), cranePivotTargetPosition));
        armMotor.set(armPidController.calculate(this.getArmPosition(), craneArmTargetPosition));
        clawMotor.set(clawSpeed);
    }

    /** Zeros all motor speeds. */
    public void stop() {
        cranePivotTargetPosition = this.getPivotPosition();
        craneArmTargetPosition = this.getArmPosition();
        clawSpeed = 0.0;
    }

    /** Zeros the arm encoder. */
    public void zeroArmEncoder() {
        armMotor.getEncoder().setPosition(0);
    }

    /** Zeros the arm encoder. */
    public void zeroPivotEncoder() {
        pivotMotor.getEncoder().setPosition(0);
    }

    /** Weather the arm limit switch is pressed.
     *
     * @return boolean
     */
    public boolean getArmSwitch() {
        return !armSwitch.get(); // Invert bc VEX limit switches are normally closed
    }

    /** Weather the frame limit switch is pressed.
     *
     * @return boolean
     */
    public boolean getFrameSwitch() {
        return !frameSwitch.get(); // Invert bc VEX limit switches are normally closed
    }

    /** Gets the position of the pivot motor, from the motor's encoder.
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
    public void setPivotTarget(double target) {
        cranePivotTargetPosition = target;
    }

    /** get the current target of the pivot motor
     *
     * @return double
     */
    public double getPivotTarget() {
        return cranePivotTargetPosition;
    }

    /** Sets the speed of the arm motor.
     *
     * @param speed Speed from [-1, 1]
     */
    public void setArmTarget(double target) {
        craneArmTargetPosition = target;
    }

    /** get the current target of the arm motor
     *
     * @return double
     */
    public double getArmTarget() {
        return craneArmTargetPosition;
    }

    /** Sets the speed of the claw motor.
     *
     * @param speed Speed from [-1, 1]
     */
    public void setClawSpeed(double speed) {
        clawSpeed = speed;
    }

    public double getClawSpeed() {
        return clawSpeed;
    }
}
