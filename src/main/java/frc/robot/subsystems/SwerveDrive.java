package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.OiConstants;
import frc.robot.Constants.SwerveConstants;

/** The subsystem that handles swerve drive. */
public class SwerveDrive extends SubsystemBase {
    // Define the singleton
    private static SwerveDrive instance;

    /**
     * Get this instance.
     *
     * @return The subsystem
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

    private final AHRS gyro;
    private final double gyroOffset = 0.0;

    private final SwerveDriveOdometry odometry;
    private Field2d field;

    /** Initializes a new SwerveDrive subsystem object. */
    private SwerveDrive() {
        // Init the swerve modules
        frontLeft = new SwerveModule(21, false, 22, false, 23, 4.516 - Math.PI);
        frontRight = new SwerveModule(31, false, 32, false, 33, 5.604);
        backLeft = new SwerveModule(41, false, 42, false, 43, 5.647 - Math.PI);
        backRight = new SwerveModule(51, false, 52, false, 53, 4.522);

        gyro = new AHRS(SerialPort.Port.kMXP);
        // @TODO: Have a while statement for while not calibrated, calibrate
        new Thread(() -> { // Can't call gyro.reset() right away because the gyroscope is calibrating, so
                           // create a new process and delay
            try {
                Thread.sleep(1000);
                gyro.reset();
            } catch (Exception e) {
                System.out.println("Init failure!");
                System.out.println(e);
            }
        }).start();

        odometry = new SwerveDriveOdometry(SwerveConstants.driveKinematics, new Rotation2d(0),
                new SwerveModulePosition[] {
                        frontLeft.getPosition(),
                        frontRight.getPosition(),
                        backLeft.getPosition(),
                        backRight.getPosition()
                }, new Pose2d(0, 0, Rotation2d.fromDegrees(-gyroOffset)));

        field = new Field2d();
    }

    /** Run approx. every 20 ms. */
    @Override
    public void periodic() {
        odometry.update(getRotation2d(), new SwerveModulePosition[] {
                frontLeft.getPosition(), frontRight.getPosition(),
                backLeft.getPosition(), backRight.getPosition()
        });
        field.setRobotPose(odometry.getPoseMeters());
        SmartDashboard.putData("Field", field);
    }

    /**
     * Gets the heading of the robot clamped within 360 degrees.
     *
     * @return Robot heading in Rotation2d
     */
    public Rotation2d getRotation2d() {
        // if (gyro.isMagnetometerCalibrated()) {
        // return Rotation2d.fromDegrees(gyro.getFusedHeading());
        // }

        // We have to invert the angle of the NavX so that rotating the robot
        // counter-clockwise makes the angle increase.
        // return Rotation2d.fromDegrees((360.0 - gyro.getYaw()));
        return Rotation2d.fromDegrees(360 - gyro.getYaw() - gyroOffset);
    }

    // @TODO implement some form of usage in OI for testing
    public void zeroGyro() {
        gyro.zeroYaw();
    }

    /** Stops all motors. */
    public void stop() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setDirection(double xSpeed, double ySpeed, double turningSpeed, boolean fieldOriented) {
        ChassisSpeeds chassisSpeed;

        // Multiply joystick speeds by their multipliers
        xSpeed *= OiConstants.xySpeedMultiplier;
        ySpeed *= OiConstants.xySpeedMultiplier;
        turningSpeed *= OiConstants.turningSpeedMultiplier;

        // Convert speeds to chassis speeds
        if (fieldOriented) {
            chassisSpeed = ChassisSpeeds.fromFieldRelativeSpeeds(ySpeed, xSpeed, turningSpeed, this.getRotation2d());
        } else {
            chassisSpeed = new ChassisSpeeds(ySpeed, xSpeed, turningSpeed); // Yes, ChassisSpeeds wants x, y, and
                                                                            // turning and we give it y, x, turning.
                                                                            // This is the fix to rotate the controls 90
                                                                            // degrees
        }

        // Create swerve module states for the desired movement and push to subsystem
        SwerveModuleState[] moduleStates = SwerveConstants.driveKinematics.toSwerveModuleStates(chassisSpeed);
        this.setStates(moduleStates, true);
    }

    /**
     * Sets the swerve modules to a new state.
     *
     * @param desiredStates The new desired states
     * @param log           If the Swerve Modules should log there target state and
     *                      current state
     */
    private void setStates(SwerveModuleState[] desiredStates, boolean log) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.drivePhysicalMaxSpeed); // Normalize
                                                                                                           // the speeds
                                                                                                           // so we
                                                                                                           // don't
                                                                                                           // attempt to
                                                                                                           // overdraw
                                                                                                           // currents

        frontLeft.setState(desiredStates[0], log);
        frontRight.setState(desiredStates[1], log);
        backLeft.setState(desiredStates[2], log);
        backRight.setState(desiredStates[3], log);
    }

    /**
     * Put some of the available gyro data to the dashboard.
     */
    public void putGyroData() {
        SmartDashboard.putNumber("IMU_Yaw", gyro.getYaw());
        SmartDashboard.putNumber("IMU_Pitch", gyro.getPitch());
        SmartDashboard.putNumber("IMU_Roll", gyro.getRoll());
    }
}
