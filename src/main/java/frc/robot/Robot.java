// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.auto.AutoScoreAndExit;
import frc.robot.subsystems.Crane;
import frc.robot.subsystems.Lights;
import frc.robot.subsystems.OI;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Vision;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation.
 * If you change the name of this class or the package after creating this
 * project, you must also update the build.gradle file in the project.
 */
public class Robot extends TimedRobot {

    /** This function is run when the robot is first started up. */
    @Override
    public void robotInit() {
        SwerveDrive.getInstance();
        Crane.getInstance();
        Vision.getInstance();
        Lights.getInstance();
        OI.getInstance();

        // NetworkTableInstance.getDefault().getTable("limelight").getEntry("camMode").setNumber(0);
        // NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(9);
    }

    /**
     * This function is called every robot packet, no matter the mode.
     * Use this for items like diagnostics that you want ran during disabled,
     * autonomous, teleoperated and test.
     * This runs after the mode specific periodic functions, but before LiveWindow
     * and SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the scheduler (Polling buttons, adding newly-scheduled commands, running
        // already-scheduled commands, removing finished or interrupted commands, and
        // running subsystem periodic() methods).
        // This must be called from the robot's periodic block in order for anything in
        // the Command-based framework to work.
        CommandScheduler.getInstance().run();
    }

    /** This function is called at the start of the disabled mode. */
    @Override
    public void disabledInit() {
        SwerveDrive.getInstance().stop();
        Crane.getInstance().stop();
    }

    /**
     * This function is called periodically when the bot is disabled. For safety use
     * only!
     */
    @Override
    public void disabledPeriodic() {
    }

    /** This function runs at the start of the autonomous mode. */
    @Override
    public void autonomousInit() {
        SwerveDrive.getInstance().zeroGyro();
        new AutoScoreAndExit().schedule();
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {
    }

    /** This function is called at the start of teleop (Driver control). */
    @Override
    public void teleopInit() {
        // Light Control
        // turningController.a().onTrue(new LEDControlCommand(255, 0, 0));
    }

    /** This function is called at the start of the test mode. */
    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll(); // Cancels all running commands
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {
    }
}
