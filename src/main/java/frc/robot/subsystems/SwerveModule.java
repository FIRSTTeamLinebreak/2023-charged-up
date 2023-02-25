package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
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
    private final CANCoder turningEncoder;
    private final PIDController turningPid;

    private final double turningEncoderOffset;

    /** Creates a new swerve module.
     *
     * @param driveId CAN ID for the drive motor (Falcon)
     * @param isDriveReversed If the drive motor is reversed
     * @param turningId CAN ID for the turning motor (NEO)
     * @param isTurningReversed If the turning motor is reversed
     * @param coderId CAN ID for the CAN coder within the swerve module
     * @param coderOffset The offset to get the CAN coder to true position (NEED TO BE POSITIVE)
     */
    public SwerveModule(int driveId, boolean isDriveReversed, int turningId, boolean isTurningReversed, int coderId, double turningEncoderOffset) {
        // Drive
        driveController = new WPI_TalonFX(driveId);
        driveController.setInverted(isDriveReversed);

        // Turning
        turningController = new CANSparkMax(turningId, MotorType.kBrushless);
        turningController.setInverted(isTurningReversed);
        turningController.restoreFactoryDefaults(true);
        turningController.setIdleMode(IdleMode.kBrake);

        this.turningEncoderOffset = turningEncoderOffset;

        turningEncoder = new CANCoder(coderId);

        CANCoderConfiguration config = new CANCoderConfiguration();
        config.sensorCoefficient = 2 * Math.PI / 4096.0;
        config.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        config.unitString = "rad";
        config.sensorTimeBase = SensorTimeBase.PerSecond;

        turningEncoder.configAllSettings(config);

        turningPid = new PIDController(SwerveMotorConstants.turningPidP, SwerveMotorConstants.turningPidI, SwerveMotorConstants.turningPidD);
        turningPid.enableContinuousInput(0, 2 * Math.PI); // Tell the PID controller that it can go through -PI and PI because its a circle
        turningPid.setTolerance(SwerveMotorConstants.turningPidTollerance);
    }

    /** Reset the motor encoder positions. */
    public void resetEncoders() {
        driveController.setSelectedSensorPosition(0.0);
        // turningEncoder.setPosition(getTurningPosition());
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
        if (Math.abs(state.speedMetersPerSecond) <= .01) { // Implements a "deadzone" so releasing the joystick won't make the wheels reset to 0
            stop();
            return;
        }

        state = SwerveModuleState.optimize(state, Rotation2d.fromRadians(getTurningPosition())); // Optimize movements to not move more than 90 deg for any new state
        driveController.set(state.speedMetersPerSecond / SwerveMotorConstants.drivePhysicalMaxSpeed);
        turningController.set(turningPid.calculate(getTurningPosition(), state.angle.getRadians()));

        SmartDashboard.putString("Swerve " + Integer.toString(driveController.getDeviceID() - 10).charAt(0) + " state", "Target Speed: " + state.speedMetersPerSecond + ", Target Rotation: " + state.angle.getRadians());
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

    /** Gets the turning motor position in radians.
     *
     * @return Turning motor position
     */
    public double getTurningPosition() {
        // Creates a coninious loop between 0 and PI
        double position = turningEncoder.getPosition();
        String moduleNum = Integer.toString(driveController.getDeviceID() - 10).charAt(0) + "";
        
        SmartDashboard.putNumber(moduleNum + " Raw", position);
        position -= Math.abs(turningEncoderOffset);
        // position = position % Math.PI;
        SmartDashboard.putNumber(moduleNum + " Clean", position % 2 * Math.PI);
        
        return position;
    }

    /** Gets the turning motor velocity.
     *
     * @return Turning motor velocity
     */
    public double getTurningVelocity() {
        return turningEncoder.getVelocity();
    }
}