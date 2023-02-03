package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveCanCoderIds;
import frc.robot.Constants.SwerveDriveCanIds;
import frc.robot.Constants.SwerveMotorConstants;
import frc.robot.Constants.SwerveTurningCanIds;

/** The subsystem that handles swerve drive. */
public class SwerveDrive extends SubsystemBase {

    // Define the singleton
    private static SwerveDrive instance;

    /** Get this instance.
     *
     * @return A swerve drive subsystem
     */
    public static SwerveDrive getInstance() {
        if (instance == null) {
            instance = new SwerveDrive();
        }

        return instance;
    }

    private final SwerveModule frontLeft;
    private final SwerveModule frontRight;
    private final SwerveModule backLeft;
    private final SwerveModule backRight;

    private final ADXRS450_Gyro gyro;

    /** Initializes a new SwerveDrive subsystem object. */
    private SwerveDrive() {
        // Init the swerve modules @TODO: Find offset and implement logic for if the coder is reversed
        frontLeft = new SwerveModule(SwerveDriveCanIds.FL, false, SwerveTurningCanIds.FL, false, SwerveCanCoderIds.FL, 0, false);
        frontRight = new SwerveModule(SwerveDriveCanIds.FR, false, SwerveTurningCanIds.FR, false, SwerveCanCoderIds.FR, 0, false);
        backLeft = new SwerveModule(SwerveDriveCanIds.BL, false, SwerveTurningCanIds.BL, false, SwerveCanCoderIds.BL, 0, false);
        backRight = new SwerveModule(SwerveDriveCanIds.BR, false, SwerveTurningCanIds.BR, false, SwerveCanCoderIds.BR, 0, false);

        gyro = new ADXRS450_Gyro();
        new Thread(() -> { // Can't call gyro.reset() right away because the gyroscope is calibrating, so create a new process and delay
            try {
                Thread.sleep(1000);
                gyro.reset();
            } catch (Exception e) {
                System.out.println("Init failure!");
                System.out.println(e);
            }
        }).start();
    }

    /** Asks the drive train to move the robot. */
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Robot heading", getHeading());
    }

    /** Gets the heading of the robot clamped within 360 degrees.
     *
     * @return Robot heading
     */
    public double getHeading() {
        return Math.IEEEremainder(gyro.getAngle(), 360);
    }

    /** Gets the heading of the robot clamped within 360 degrees.
     *
     * @return Robot heading in Rotation2d
     */
    public Rotation2d getRotation2d() {
        return Rotation2d.fromDegrees(getHeading());
    }

    /** Stops all motors. */
    public void stop() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    /** Sets the swerve modules to a new state.
     *
     * @param desiredStates The new desired states
     */
    public void setStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveMotorConstants.drivePhysicalMaxSpeed); // Normalize the speeds so we don't attempt to overdraw currents

        frontLeft.setState(desiredStates[0]);
        frontRight.setState(desiredStates[1]);
        backLeft.setState(desiredStates[2]);
        backRight.setState(desiredStates[3]);
    }
}
