// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.SwerveJoystickDriveCommand;
import frc.robot.subsystems.SwerveDrive;

/** The VM is configured to automatically run this class, and to call the functions corresponding to each mode, as described in the TimedRobot documentation.
 * If you change the name of this class or the package after creating this project, you must also update the build.gradle file in the project.
 */
public class Robot extends TimedRobot {

    private SwerveDrive swerveSubsystem;
    private XboxController driveController;
    private XboxController turningController;

    /** This function is run when the robot is first started up. */
    @Override
    public void robotInit() {
        swerveSubsystem = SwerveDrive.getInstance();

        driveController = new XboxController(1);
        turningController = new XboxController(2);
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
        swerveSubsystem.setDefaultCommand(new SwerveJoystickDriveCommand(swerveSubsystem,
            () -> driveController.getLeftX(),
            () -> driveController.getLeftY(),
            () -> turningController.getRightX(),
            () -> true
        ));

        // new JoystickButton(turningController, 1).onTrue(swerveSubsystem.zeroHeading()); // Method does not exist. May not be needed because of CAN coders
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
    public void testPeriodic() {}
}
