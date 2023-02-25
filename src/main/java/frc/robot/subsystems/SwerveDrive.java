package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.SwerveMotorConstants;

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

    private final AHRS gyro;

    /** Initializes a new SwerveDrive subsystem object. */
    private SwerveDrive() {
        // Init the swerve modules @TODO: Find offsets for the CAN coders
        frontLeft  = new SwerveModule(21, false, 22, false, 23, 4.516 - Math.PI); 
        frontRight = new SwerveModule(31, false, 32, false, 33, 5.604);
        backLeft   = new SwerveModule(41, false, 42, false, 43, 5.647 - Math.PI); 
        backRight  = new SwerveModule(51, false, 52, false, 53, 4.522);
 
        gyro = new AHRS(SerialPort.Port.kMXP);
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

    /** Run approx. every 20 ms. */
    @Override
    public void periodic() {
        // SmartDashboard.putNumber("Robot heading", getHeading());
    }

    // /** Gets the heading of the robot clamped within 360 degrees.
    //  *
    //  * @return Robot heading
    //  */
    // public double getHeading() {
    //     return Math.IEEEremainder(gyro.getAngle(), 360);
    // }

    /** Gets the heading of the robot clamped within 360 degrees.
     *
     * @return Robot heading in Rotation2d
     */
    public Rotation2d getRotation2d() {
        if (gyro.isMagnetometerCalibrated()) {
            // We will only get valid fused headings if the magnetometer is calibrated
            return Rotation2d.fromDegrees(gyro.getFusedHeading() + 88);
        }
    
        // We have to invert the angle of the NavX so that rotating the robot counter-clockwise makes the angle increase.
        return Rotation2d.fromDegrees((360.0 - gyro.getYaw()) + 88);
    }

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

    /** A temporary function to get the absolute positions of the CAN coders.
     *
     * @return Absolute CAN coder positions in radians (FL, FR, BL, BR)
     */
    public double[] getTurningEncoderPositions() {
        double[] returnVals = {
                frontLeft.getTurningPosition(),
                frontRight.getTurningPosition(),
                backLeft.getTurningPosition(),
                backRight.getTurningPosition()
        };

        return returnVals;
    }

    public AHRS getGyro() {
        return gyro;
    }

    public void putData() {
        /* Display 6-axis Processed Angle Data */
        // SmartDashboard.putBoolean("IMU_Connected", gyro.isConnected());
        // SmartDashboard.putBoolean("IMU_IsCalibrating", gyro.isCalibrating());
        SmartDashboard.putNumber("IMU_Yaw", gyro.getYaw());
        SmartDashboard.putNumber("IMU_Pitch", gyro.getPitch());
        SmartDashboard.putNumber("IMU_Roll", gyro.getRoll());

        /* Display tilt-corrected, Magnetometer-based heading (requires */
        /* magnetometer calibration to be useful) */

        // SmartDashboard.putNumber("IMU_CompassHeading", gyro.getCompassHeading());

        /* Display 9-axis Heading (requires magnetometer calibration to be useful) */
        // SmartDashboard.putNumber("IMU_FusedHeading", gyro.getFusedHeading());

        /* These functions are compatible w/the WPI Gyro Class, providing a simple */
        /* path for upgrading from the Kit-of-Parts gyro to the navx-MXP */

        SmartDashboard.putNumber("IMU_TotalYaw", gyro.getAngle());
        // SmartDashboard.putNumber("IMU_YawRateDPS", gyro.getRate());

        /* Display Processed Acceleration Data (Linear Acceleration, Motion Detect) */

        // SmartDashboard.putNumber("IMU_Accel_X", gyro.getWorldLinearAccelX());
        // SmartDashboard.putNumber("IMU_Accel_Y", gyro.getWorldLinearAccelY());
        // SmartDashboard.putBoolean("IMU_IsMoving", gyro.isMoving());
        // SmartDashboard.putBoolean("IMU_IsRotating", gyro.isRotating());

        /* Display estimates of velocity/displacement. Note that these values are */
        /* not expected to be accurate enough for estimating robot position on a */
        /* FIRST FRC Robotics Field, due to accelerometer noise and the compounding */
        /* of these errors due to single (velocity) integration and especially */
        /* double (displacement) integration. */

        // SmartDashboard.putNumber("Velocity_X", gyro.getVelocityX());
        // SmartDashboard.putNumber("Velocity_Y", gyro.getVelocityY());
        // SmartDashboard.putNumber("Displacement_X", gyro.getDisplacementX());
        // SmartDashboard.putNumber("Displacement_Y", gyro.getDisplacementY());

        /* Display Raw Gyro/Accelerometer/Magnetometer Values */
        /* NOTE: These values are not normally necessary, but are made available */
        /* for advanced users. Before using this data, please consider whether */
        /* the processed data (see above) will suit your needs. */

        // SmartDashboard.putNumber("RawGyro_X", gyro.getRawGyroX());
        // SmartDashboard.putNumber("RawGyro_Y", gyro.getRawGyroY());
        // SmartDashboard.putNumber("RawGyro_Z", gyro.getRawGyroZ());
        // SmartDashboard.putNumber("RawAccel_X", gyro.getRawAccelX());
        // SmartDashboard.putNumber("RawAccel_Y", gyro.getRawAccelY());
        // SmartDashboard.putNumber("RawAccel_Z", gyro.getRawAccelZ());
        // SmartDashboard.putNumber("RawMag_X", gyro.getRawMagX());
        // SmartDashboard.putNumber("RawMag_Y", gyro.getRawMagY());
        // SmartDashboard.putNumber("RawMag_Z", gyro.getRawMagZ());
        // SmartDashboard.putNumber("IMU_Temp_C", gyro.getTempC());

        // /* Omnimount Yaw Axis Information */
        // /* For more info, see http://navx-mxp.kauailabs.com/installation/omnimount */
        // AHRS.BoardYawAxis yaw_axis = gyro.getBoardYawAxis();
        // SmartDashboard.putString("YawAxisDirection", yaw_axis.up ? "Up" : "Down");
        // SmartDashboard.putNumber("YawAxis", yaw_axis.board_axis.getValue());

        // /* Sensor Board Information */
        // SmartDashboard.putString("FirmwareVersion", gyro.getFirmwareVersion());

        // /* Quaternion Data */
        // /* Quaternions are fascinating, and are the most compact representation of */
        // /* orientation data. All of the Yaw, Pitch and Roll Values can be derived */
        // /* from the Quaternions. If interested in motion processing, knowledge of */
        // /* Quaternions is highly recommended. */
        // SmartDashboard.putNumber("QuaternionW", gyro.getQuaternionW());
        // SmartDashboard.putNumber("QuaternionX", gyro.getQuaternionX());
        // SmartDashboard.putNumber("QuaternionY", gyro.getQuaternionY());
        // SmartDashboard.putNumber("QuaternionZ", gyro.getQuaternionZ());

        // /* Connectivity Debugging Support */
        // SmartDashboard.putNumber("IMU_Byte_Count", gyro.getByteCount());
        // SmartDashboard.putNumber("IMU_Update_Count", gyro.getUpdateCount());
    }
}
