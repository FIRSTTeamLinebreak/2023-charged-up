package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** The subsystem that handles the control of the drive base. */
public class Drive extends SubsystemBase {

    // Define the singleton
    private static Drive instance;

    /** Get this instance.
     *
     * @return This instance
     */
    public static Drive getInstance() {
        if (instance == null) {
            instance = new Drive();
        }

        return instance;
    }

    private final WPI_VictorSPX frontLeftMotor;
    private final WPI_VictorSPX frontRightMotor;
    private final WPI_VictorSPX backLeftMotor;
    private final WPI_VictorSPX backRightMotor;

    private final MecanumDrive drive; // The control class that does all the math required to run a mecanum drive base

    private double acceleration;
    private double xSpeed;
    private double ySpeed;
    private double rotation;

    /** Initializes a new Drive subsystem object. */
    public Drive() {
        // Init the motors
        frontLeftMotor = new WPI_VictorSPX(0);
        frontRightMotor = new WPI_VictorSPX(1);
        backLeftMotor = new WPI_VictorSPX(2);
        backRightMotor = new WPI_VictorSPX(3);
        frontRightMotor.setInverted(true);
        backRightMotor.setInverted(true);

        drive = new MecanumDrive(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor); // Init the drive base
        xSpeed = ySpeed = rotation = 0; // Set speeds to zero
    }

    /** Stores and set the target ramp rate for the motors on the drive train.
     *
     * @param acceleration Seconds to full motor speed
     */
    public void setAcceleration(double acceleration) {
        this.acceleration = acceleration;

        frontLeftMotor.configOpenloopRamp(acceleration);
        frontRightMotor.configOpenloopRamp(acceleration);
        backLeftMotor.configOpenloopRamp(acceleration);
        backRightMotor.configOpenloopRamp(acceleration);

        SmartDashboard.putNumber("Ramp Rate", acceleration);
    }

    /** Set the target x speed.
     *
     * @param xSpeed Target speed in the left/right direction (-1, 1)
     */
    public void setxSpeed(double xSpeed) {
        this.xSpeed = xSpeed;
    }

    /** Set the target y speed.
     *
     * @param ySpeed Target speed in the up/down direction (-1, 1)
     */
    public void setySpeed(double ySpeed) {
        this.ySpeed = ySpeed;
    }

    /** Set the target rotational speed.
     *
     * @param rotation The target speed to rotate the robot in (-1, 1)
     */
    public void setRotation(double rotation) {
        this.rotation = rotation;
    }

    /** Gets the current ramp rate for the motors.
     *
     * @return Time in seconds for the motors to get to full speed
     */
    public double getAcceleration() {
        return acceleration;
    }

    /** Resets this subsystem to default init status. */
    public void init() {
        acceleration = 0;
        xSpeed = 0;
        ySpeed = 0;
        rotation = 0;

        SmartDashboard.putNumber("Ramp Rate", 0);
    }

    /** Asks the drive train to move the robot. */
    @Override
    public void periodic() {
        drive.driveCartesian(ySpeed, xSpeed, rotation);
    }
}
