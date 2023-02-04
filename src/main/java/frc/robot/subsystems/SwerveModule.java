package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.SwerveMotorConstants;

/** An individual swerve module. */
public class SwerveModule {

    // Drive
    private final WPI_TalonFX driveController;

    // Turning
    private final CANSparkMax turningController;
    private final RelativeEncoder turningEncoder;
    private final PIDController turningPid;

    // CAN Coder
    private final CANCoder canCoder;
    private int canCoderId;
    private final double canCoderOffset; // Offset in radians for the CAN coder

    /** Creates a new swerve module.
     *
     * @param driveId CAN ID for the drive motor (Falcon)
     * @param isDriveReversed If the drive motor is reversed
     * @param turningId CAN ID for the turning motor (NEO)
     * @param isTurningReversed If the turning motor is reversed
     * @param coderId CAN ID for the CAN coder within the swerve module
     * @param coderOffset The offset to get the CAN coder to true position
     */
    public SwerveModule(int driveId, boolean isDriveReversed, int turningId, boolean isTurningReversed, int coderId, double coderOffset) {
        // Drive
        driveController = new WPI_TalonFX(driveId);
        driveController.setInverted(isDriveReversed);

        // Turning
        turningController = new CANSparkMax(turningId, MotorType.kBrushless);
        turningController.setInverted(isTurningReversed);

        turningEncoder = turningController.getEncoder();
        turningEncoder.setPositionConversionFactor(SwerveMotorConstants.turningRotToRadians);
        turningEncoder.setVelocityConversionFactor(SwerveMotorConstants.turningRpsToRps);

        turningPid = new PIDController(SwerveMotorConstants.turningPidP, 0, 0);
        turningPid.enableContinuousInput(-Math.PI, Math.PI); // Tell the PID controller that it can go through -PI and PI because its a circle

        // CAN Coder
        canCoder = new CANCoder(coderId);
        canCoderId = coderId;
        canCoderOffset = coderOffset;

        resetEncoders();
    }

    /** Reset the motor encoder positions. */
    public void resetEncoders() {
        driveController.setSelectedSensorPosition(0.0);
        turningEncoder.setPosition(getAbsoluteEncoderRad());
    }

    /** Creates a SwerveModuleState for this swerve module given its position and velocity.
     * This is needed for WPILib functions.
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    /** Sets the swerve module to a new state.
     *
     * @param state The new desired state
     */
    public void setState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) <= .001) { // Implements a "deadzone" so releasing the joystick won't make the wheels reset to 0
            stop();
            return;
        }

        state = SwerveModuleState.optimize(state, getState().angle); // Optimize movements to not move more than 90 deg for any new state
        driveController.set(state.speedMetersPerSecond / SwerveMotorConstants.drivePhysicalMaxSpeed);
        turningController.set(turningPid.calculate(getTurningPosition(), state.angle.getRadians()));

        SmartDashboard.putString("Swerve " + canCoderId + " state", state.toString());
    }

    /** Stops the swerve module. */
    public void stop() {
        driveController.set(0);
        turningController.set(0);
    }

    /** Gets the drive motor position.
     *
     * @return Drive motor position
     */
    public double getDrivePosition() {
        return driveController.getSelectedSensorPosition();
    }

    /** Gets the velocity of the drive motor.
     *
     * @return Drive motor velocity
     */
    public double getDriveVelocity() {
        return driveController.getSelectedSensorVelocity();
    }

    /** Gets the turning motor position.
     *
     * @return Turning motor position
     */
    public double getTurningPosition() {
        return turningEncoder.getPosition();
    }

    /** Gets the turning motor velocity.
     *
     * @return Turning motor velocity
     */
    public double getTurningVelocity() {
        return turningEncoder.getVelocity();
    }

    /** Gets the absolute position of the CAN coder.
     *
     * @return CAN coder absolute position
     */
    public double getAbsoluteEncoderRad() {
        return canCoder.getAbsolutePosition() + canCoderOffset;
    }
}