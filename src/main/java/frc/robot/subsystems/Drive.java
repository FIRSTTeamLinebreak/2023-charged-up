package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * The subsystem that handles the control of the drive base.
 */
public class Drive extends SubsystemBase {

	// Define the signleton 
	private static Drive instance;
	public static Drive getInstance() {
		if (instance == null) {
			instance = new Drive();
		}
		return instance;
	}

	private final WPI_VictorSPX frontLeftMotor, frontRightMotor, backLeftMotor, backRightMotor;
	private final MecanumDrive drive; // The control class that does all the math required to run a mecanum drive base
	private double acceration, xSpeed, ySpeed, rotation;

	public Drive() {
		// Initalize the motors
		frontLeftMotor = new WPI_VictorSPX(0);
		frontRightMotor = new WPI_VictorSPX(1);
		backLeftMotor = new WPI_VictorSPX(2);
		backRightMotor = new WPI_VictorSPX(3);
		frontRightMotor.setInverted(true);
		backRightMotor.setInverted(true);

		// Initalize the drive base
		drive = new MecanumDrive(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor);

		//set speeds to zero
		xSpeed = ySpeed = rotation = 0;
	}

	/**
	 * stores and set the target ramp rate for the motors on the drive train
	 * 
	 * @param acceration seconds to full motor speed
	 */
	public void setAcceration(double acceration) {
		this.acceration = acceration;

		frontLeftMotor.configOpenloopRamp(acceration);
		frontRightMotor.configOpenloopRamp(acceration);
		backLeftMotor.configOpenloopRamp(acceration);
		backRightMotor.configOpenloopRamp(acceration);

		SmartDashboard.putNumber("Ramp Rate", acceration);
	}

	/**
	 * set the target x Speed
	 * 
	 * @param xSpeed target speed in the left/right direction (-1, 1)
	 */
	public void setxSpeed(double xSpeed) {
		this.xSpeed = xSpeed;
	}

	/**
	 * set the target y Speed
	 * 
	 * @param xSpeed target speed in the up/down direction (-1, 1)
	 */
	public void setySpeed(double ySpeed) {
		this.ySpeed = ySpeed;
	}

	/**
	 * set the target roational speed
	 * 
	 * @param rotation the target speed to rotate the robot in (-1, 1)
	 */
	public void setRotation(double rotation) {
		this.rotation = rotation;
	}

	/**
	 * gets the current ramp rate for the motors
	 * 
	 * @return the time in seconds for the motors to get to full speed
	 */
	public double getAcceration() {
		return acceration;
	}

	/**
   * Resets this subsystem to default init status
   */
	public void init() {
		acceration = 0;
		xSpeed = 0;
		ySpeed = 0;
		rotation = 0;

		SmartDashboard.putNumber("Ramp Rate", 0);

	}

	/**
	 * asks the drive train to move the robot
	 */
	@Override
	public void periodic() {
		drive.driveCartesian(ySpeed, xSpeed, rotation);
	}
}
