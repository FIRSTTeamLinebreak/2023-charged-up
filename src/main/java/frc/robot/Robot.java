// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Util.applyLinearDeadzone;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OiConstants;
import frc.robot.commands.CraneControlCommand;
import frc.robot.commands.SwerveJoystickDriveCommand;
import frc.robot.subsystems.Crane;
import frc.robot.subsystems.SwerveDrive;

/** The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as described in the TimedRobot documentation.
 * If you change the name of this class or the package after creating this project, you must also update the build.gradle file in the project.
 */
public class Robot extends TimedRobot {
    // Controllers
    private CommandXboxController driveController;
    private CommandXboxController turningController;

    // Swerve
    private SwerveDrive swerveSub;
    private double swerveTargetTurningSpeed = 0.0;

    private final double slowTurningDivisor = 4; // Joystick input is divided by this amount for slow turning

    private final double fastPivotIncrementor = 1.8;
    private final double slowPivotIncrementor = 1.1;

    private final double armIncrementor = 1.5;

    private final double targetClawSpeed = 0.3;

    // Crane
    private Crane craneSub;

    private double cranePivotTargetPosition = 0.0;
    private double craneArmTargetPosition = 0.0;
    private double craneClawTargetSpeed = 0.0;

    /** This function is run when the robot is first started up. */
    @Override
    public void robotInit() {
        swerveSub = SwerveDrive.getInstance();
        craneSub = Crane.getInstance();

        driveController = new CommandXboxController(0);
        turningController = new CommandXboxController(1);

        NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(9);
    }

    /** This function is called every robot packet, no matter the mode.
     * Use this for items like diagnostics that you want ran during disabled, autonomous, teleoperated and test.
     * This runs after the mode specific periodic functions, but before LiveWindow and SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the scheduler (Polling buttons, adding newly-scheduled commands, running already-scheduled commands, removing finished or interrupted commands, and running subsystem periodic() methods).
        // This must be called from the robot's periodic block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
    }

    /** This function is called at the start of the disabled mode. */
    @Override
    public void disabledInit() {
        swerveSub.stop();
        craneSub.stop();
    }

    /** This function is called periodically when the bot is disabled. For safety use only! */
    @Override
    public void disabledPeriodic() {}

    /** This function runs at the start of the autonomous mode. */
    @Override
    public void autonomousInit() {
        swerveSub.zeroGyro();
        new SwerveJoystickDriveCommand(() -> 0.0, () -> -0.3, () -> 0.0, () -> true).withTimeout(2.5).schedule();
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {}

    /** This function is called at the start of teleop (Driver control). */
    @Override
    public void teleopInit() {
        // Swerve control
        swerveSub.setDefaultCommand(new SwerveJoystickDriveCommand(
            () -> driveController.getLeftX() * -1,
            () -> driveController.getLeftY() * -1,
            () -> swerveTargetTurningSpeed * -1,
            () -> !driveController.getHID().getRightBumper()
        ));

        // Crane control
        craneSub.setDefaultCommand(new CraneControlCommand(
            () -> cranePivotTargetPosition,
            () -> craneArmTargetPosition,
            () -> craneClawTargetSpeed
        ));
    }

    /** This function is called periodically during teleop. */
    @Override
    public void teleopPeriodic() {
        // Swerve control
        if (applyLinearDeadzone(OiConstants.joystickDeadzone, driveController.getRightX()) != 0) {
            if (driveController.getHID().getLeftBumper()) { // Decrease speed
                swerveTargetTurningSpeed = driveController.getRightX() / slowTurningDivisor;
            } else {
                swerveTargetTurningSpeed = driveController.getRightX();
            }
        } else {
            swerveTargetTurningSpeed = 0;
        }

        // Crane control
        if (applyLinearDeadzone(OiConstants.joystickDeadzone, turningController.getRightY()) > 0) { // Pivot up fast
            cranePivotTargetPosition -= fastPivotIncrementor;
        } else if (applyLinearDeadzone(OiConstants.joystickDeadzone, turningController.getRightY()) < 0) { // Pivot down fast
            cranePivotTargetPosition += fastPivotIncrementor;
        } else if (applyLinearDeadzone(OiConstants.joystickDeadzone, turningController.getLeftY()) > 0) { // Pivot up slow
            cranePivotTargetPosition -= slowPivotIncrementor;
        } else if (applyLinearDeadzone(OiConstants.joystickDeadzone, turningController.getLeftY()) < 0) { // Pivot down slow
            cranePivotTargetPosition += slowPivotIncrementor;
        }

        if (applyLinearDeadzone(OiConstants.triggerDeadzone, turningController.getLeftTriggerAxis()) > 0) { // Arm out
            craneArmTargetPosition -= armIncrementor;
        } else if (turningController.getHID().getLeftBumper()) { // Arm in
            craneArmTargetPosition += armIncrementor;
        }

        if (turningController.getHID().getRightBumper()) { // Claw out
            craneClawTargetSpeed = targetClawSpeed * -1;
        } else if (applyLinearDeadzone(OiConstants.triggerDeadzone, turningController.getHID().getRightTriggerAxis()) > 0) { // Claw in
            craneClawTargetSpeed = targetClawSpeed;
        } else { // Stop when no input is given
            craneClawTargetSpeed = 0;
        }


        if (craneSub.getPivotSwitch()) { // Prevent having the target further in when the limit switches are pressed
            cranePivotTargetPosition = craneSub.getPivotPosition();
        }
        // if (craneSub.getArmSwitch()) {
        //     craneArmTargetPosition = craneSub.getArmPosition();
        // }
    }

    /** This function is called at the start of the test mode. */
    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll(); // Cancels all running commands
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {
        SwerveModuleState zero = new SwerveModuleState(0, Rotation2d.fromRadians(0));
        SwerveModuleState[] zeros = {zero, zero, zero, zero};
        swerveSub.setStates(zeros, true);

        SmartDashboard.putNumber("Pivot Position", craneSub.getPivotPosition());
        SmartDashboard.putBoolean("Pivot Switch", craneSub.getPivotSwitch());
        if (turningController.getHID().getAButtonPressed()) {
            craneSub.zeroPivotEncoder();
        }
        craneSub.setPivotSpeed(turningController.getLeftY() * 0.1);
    }
}
