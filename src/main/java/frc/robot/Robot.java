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
    private SwerveDrive swerveSubsystem;
    private double swerveTargetTurningSpeed = 0.0;

    // Crane
    private Crane craneSubsystem;

    private double cranePivotTargetPosition = 0.0;
    private double craneArmTargetPosition = 0.0;
    private double craneClawTargetSpeed = 0.0;

    /** This function is run when the robot is first started up. */
    @Override
    public void robotInit() {
        swerveSubsystem = SwerveDrive.getInstance();
        craneSubsystem = Crane.getInstance();

        driveController = new CommandXboxController(0);
        turningController = new CommandXboxController(1);

        NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0
        );
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
        swerveSubsystem.stop();
        craneSubsystem.stop();
    }

    /** This function is called periodically when the bot is disabled. For safety use only! */
    @Override
    public void disabledPeriodic() {}

    /** This function runs at the start of the autonomous mode. */
    @Override
    public void autonomousInit() {
        swerveSubsystem.zeroGyro();
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {}

    /** This function is called at the start of teleop (Driver control). */
    @Override
    public void teleopInit() {
        // Swerve control
        swerveSubsystem.setDefaultCommand(new SwerveJoystickDriveCommand(
            () -> driveController.getLeftX() * -1,
            () -> driveController.getLeftY(),
            () -> swerveTargetTurningSpeed * -1,
            () -> !driveController.getHID().getRightBumper()
        ));

        // Crane control
        craneSubsystem.setDefaultCommand(new CraneControlCommand(
            () -> cranePivotTargetPosition,
            () -> craneArmTargetPosition,
            () -> craneClawTargetSpeed
        ));
    }

    /** This function is called periodically during teleop. */
    @Override
    public void teleopPeriodic() {
        SmartDashboard.putNumber("Pivot Position", craneSubsystem.getPivotPosition());
        SmartDashboard.putNumber("Arm Position", craneSubsystem.getArmPosition());
        SmartDashboard.putNumber("Pivot Target", cranePivotTargetPosition);
        SmartDashboard.putNumber("Arm Target", craneArmTargetPosition);

        // Swerve control
        if (applyLinearDeadzone(OiConstants.joystickDeadzone, driveController.getRightX()) != 0) {
            if (driveController.getHID().getLeftBumper()) { // Decrease speed
                swerveTargetTurningSpeed = driveController.getRightX() / 4;
            } else {
                swerveTargetTurningSpeed = driveController.getRightX();
            }
        } else {
            swerveTargetTurningSpeed = 0;
        }

        // Crane control
        if (applyLinearDeadzone(OiConstants.joystickDeadzone, turningController.getRightY()) > 0) { // Pivot up fast
            cranePivotTargetPosition -= 0.7;
        } else if (applyLinearDeadzone(OiConstants.joystickDeadzone, turningController.getRightY()) < 0) { // Pivot down fast
            cranePivotTargetPosition += 0.7;
        } else if (applyLinearDeadzone(OiConstants.joystickDeadzone, turningController.getLeftY()) > 0) { // Pivot up slow
            cranePivotTargetPosition -= 0.4;
        } else if (applyLinearDeadzone(OiConstants.joystickDeadzone, turningController.getLeftY()) < 0) { // Pivot down slow
            cranePivotTargetPosition += 0.4;
        }

        if (applyLinearDeadzone(OiConstants.triggerDeadzone, turningController.getLeftTriggerAxis()) > 0) { // Arm out
            craneArmTargetPosition += 1.5;
        } else if (turningController.getHID().getLeftBumper()) { // Arm in
            craneArmTargetPosition -= 1.5;
        }

        if (turningController.getHID().getRightBumper()) { // Claw out
            craneClawTargetSpeed = 0.3;
        } else if (applyLinearDeadzone(OiConstants.triggerDeadzone, turningController.getHID().getRightTriggerAxis()) > 0) { // Claw in
            craneClawTargetSpeed = -0.3;
        } else { // Stop when no input is given
            craneClawTargetSpeed = 0;
        }


        if (craneSubsystem.getPivotSwitch()) { // Prevent having the target further in when the limit switches are pressed
            cranePivotTargetPosition = craneSubsystem.getPivotPosition();
        }
        if (craneSubsystem.getArmSwitch()) {
            craneArmTargetPosition = craneSubsystem.getArmPosition();
        }
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
        swerveSubsystem.setStates(zeros, true);

        SmartDashboard.putNumber("Pivot Position", craneSubsystem.getPivotPosition());
        SmartDashboard.putBoolean("Pivot Switch", craneSubsystem.getPivotSwitch());
        if (turningController.getHID().getAButtonPressed()) {
            craneSubsystem.zeroPivotEncoder();
        }
        craneSubsystem.setPivotSpeed(turningController.getLeftY() * 0.1);
    }
}
