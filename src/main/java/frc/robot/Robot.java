// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static frc.robot.Util.applyLinearDeadzone;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
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
    private SwerveDrive swerveSubsystem;
    private Crane craneSubsystem;
    private CommandXboxController driveController;
    private CommandXboxController turningController;

    private Double cranePivotCurrentTargetState = 0.0;
    private Double craneArmCurrentTargetState = 0.0;

    /** This function is run when the robot is first started up. */
    @Override
    public void robotInit() {
        swerveSubsystem = SwerveDrive.getInstance();
        craneSubsystem = Crane.getInstance();

        driveController = new CommandXboxController(0);
        turningController = new CommandXboxController(1);
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
    public void autonomousInit() {}

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {}

    /** This function is called at the start of teleop (Driver control). */
    @Override
    public void teleopInit() {
        swerveSubsystem.setDefaultCommand(new SwerveJoystickDriveCommand(
            driveController::getLeftX,
            driveController::getLeftY,
            () -> applyLinearDeadzone(OiConstants.joystickDeadzone, turningController.getLeftX()) == 0.0 ? driveController.getRightX() : turningController.getLeftX(), // Allow either driver to turn the robot, but have the turning controller override the drive controller
            () -> !driveController.getHID().getRightBumper()
        ));

        if (turningController.getHID().getYButtonPressed()) {
            cranePivotCurrentTargetState = Crane.PIVOT_TOP;
            craneArmCurrentTargetState = Crane.ARM_TOP;
        } else if (turningController.getHID().getBButtonPressed()) {
            cranePivotCurrentTargetState = Crane.PIVOT_MIDDLE;
            craneArmCurrentTargetState = Crane.ARM_MIDDLE;
        } else if (turningController.getHID().getAButtonPressed()) {
            cranePivotCurrentTargetState = Crane.PIVOT_FRONT;
            craneArmCurrentTargetState = Crane.ARM_FRONT;
        }

        craneSubsystem.setDefaultCommand(new CraneControlCommand(
            () -> cranePivotCurrentTargetState, 
            () -> craneArmCurrentTargetState,
            () -> turningController.getHID().getStartButton() ? 0.3 : turningController.getHID().getBackButton() ? -0.3 : 0
        ));

        // driveController.a().onTrue(new SwerveZeroGyro());
    }

    /** This function is called periodically during teleop. */
    @Override
    public void teleopPeriodic() {}

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
