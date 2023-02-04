// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This class should not be used for any other purpose. All constants should be declared globally (i.e. public static).
 * Do not put anything functional in this class. It is advised to statically import this class (or one of its inner classes) wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

    /** CAN IDs for the drive motors of each swerve module. */
    public static final class SwerveDriveCanIds {
        public static final int FL = 31;
        public static final int FR = 41;
        public static final int BL = 51;
        public static final int BR = 61;
    }

    /** CAN IDs for the turning motors of each swerve module. */
    public static final class SwerveTurningCanIds {
        public static final int FL = 32;
        public static final int FR = 42;
        public static final int BL = 52;
        public static final int BR = 62;
    }

    /** CAN IDs for the CAN coders of each swerve module. */
    public static final class SwerveCanCoderIds {
        public static final int FL = 33;
        public static final int FR = 43;
        public static final int BL = 53;
        public static final int BR = 63;
    }

    /** CAN coder offsets. */
    public static final class CanCoderOffsets {
        public static final double FL = 0.0;
        public static final double FR = 0.0;
        public static final double BL = 0.0;
        public static final double BR = 0.0;
    }

    /** Assorted other constants for the swerve subsystem. @TODO: Rename this to SwerveSubsystemConstants */
    public static final class SwerveMotorConstants {
        public static final double wheelDiameter = 0.0; // Meters @TODO: Get this info
        public static final double driveGearRatio = 0.0; // @TODO: Get this info
        public static final double turningGearRatio = 0.0; // @TODO: Get this info

        public static final double driveRotToMeters = driveGearRatio * Math.PI * wheelDiameter; // Drive motor rotations to meters
        public static final double driveRpsToMps = driveRotToMeters / 60; // Drive motor rotations per second to meters per second

        public static final double turningRotToRadians = turningGearRatio * 2 * Math.PI; // Turning motor rotations to radians
        public static final double turningRpsToRps = turningRotToRadians / 60; // Turning motor rotations per second to radians per second
        public static final double turningPidP = 0.5;

        public static final double drivePhysicalMaxSpeed = 0.0; // Physical max speed of the motor in m/s @TODO: Tune

        public static final double trackWidth = 0.0; // @TODO: Document and get
        public static final double wheelBase = 0.0; // Distance between front and back wheels @TODO: Get
        public static final SwerveDriveKinematics driveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2, -trackWidth / 2),
            new Translation2d(wheelBase / 2, trackWidth / 2),
            new Translation2d(-wheelBase / 2, -trackWidth / 2),
            new Translation2d(-wheelBase / 2, trackWidth / 2)
        );
    }

    /** Constants for the operator interface (OI). */
    public static final class OiConstants {
        public static final double joystickDeadzone = 0.01; // The zone around "zero" to ignore. Prevents joystick drift from becoming an issue

        public static final double driveMaxAccel = 10.0; // Max acceleration in teleop mode of the drive motors in units(?) per second @TODO: Tune
        public static final double driveSpdScaler = 10.0; // @TODO: Tune and document

        public static final double turningMaxAccel = 10.0; // Max acceleration in teleop mode of the turning motors in units(?) per second @TODO: Tune
        public static final double turningSpdScaler = 10.0; // @TODO: Tune and document
    }
}
