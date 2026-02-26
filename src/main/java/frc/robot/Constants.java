// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{
    public static class OperatorConstants
    {
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int OPERATOR_CONTROLLER_PORT = 1;
    }

    public static final class DriveConstants {
        public static final int LEFT_DRIVE_LEAD_ID = 10;
        public static final int LEFT_DRIVE_FOLLOW_ID = 11;
        public static final int RIGHT_DRIVE_LEAD_ID = 12;
        public static final int RIGHT_DRIVE_FOLLOW_ID = 13;

        // Current limit for drivetrain motors. 60A is a reasonable maximum to reduce
        // likelihood of tripping breakers or damaging CIM motors
        public static final int DRIVE_MOTOR_CURRENT_LIMIT = 60;

        public static final double DRIVE_FORWARD_MULTIPLIER = 0.7;
        public static final double DRIVE_ROTATION_MULTIPLIER = 0.8;
        public static final double SLOW_DRIVE_FORWARD_MULTIPLIER = 0.2;
        public static final double SLOW_DRIVE_ROTATION_MULTIPLIER = 0.3;
    }

    public static final class FuelConstants {
        // Motor controller IDs for Fuel Mechanism motors
        public static final int LEFT_INTAKE_LAUNCHER_MOTOR_ID = 5;
        public static final int RIGHT_INTAKE_LAUNCHER_MOTOR_ID = 6;
        public static final int INDEXER_MOTOR_ID = 8;

        // Current limit for fuel mechanism motors.
        public static final int INDEXER_MOTOR_CURRENT_LIMIT = 80;
        public static final int LAUNCHER_MOTOR_CURRENT_LIMIT = 80;

        // All values likely need to be tuned based on your robot
        public static final double INDEXER_INTAKE_PERCENT = -.8;
        public static final double INDEXER_LAUNCH_PERCENT = 0.6;
        public static final double INDEXER_SPIN_UP_PRE_LAUNCH_PERCENT = -0.5;

        public static final double INTAKE_INTAKE_PERCENT = 0.6;
        public static final double LAUNCHING_LAUNCHER_PERCENT = .85;
        public static final double INTAKE_EJECT_PERCENT = -0.8;

        public static final double SPIN_UP_SECONDS = 0.75;
    }

    public static final class ClimbConstants {
        // Motor controller IDs for Climb motor
        public static final int CLIMBER_MOTOR_ID = 7;

        // Current limit for climb motor
        public static final int CLIMBER_MOTOR_CURRENT_LIMIT = 80;
        // Percentage to power the motor both up and down
        public static final double CLIMBER_MOTOR_DOWN_PERCENT = -0.8;
        public static final double CLIMBER_MOTOR_UP_PERCENT = 0.8;
    }

    public static final class AutoConstants {
        public static final double LAUNCH_SECONDS = 1.0; // Change after testing
        public static final double CLIMB_UP_L1_SECONDS = 1.0; // Change after testing
        public static final double CLIMB_UP_L2_SECONDS = 1.0; // Change after testing
        public static final double CLIMB_DOWN_SECONDS = 1.0; // Change after testing
        public static final double EJECT_SECONDS = 1.0; // Change after testing
        public static final double INTAKE_SECONDS = 1.0; // Change after testing
    }

    public static final class AutoGyroConstants {
        public static final double trackwidthMeters = 0.69;
        public static final DifferentialDriveKinematics DriveKinematics = new DifferentialDriveKinematics(trackwidthMeters);
        public static final double WHEEL_CIRCUMFERENCE = 0.4787787204;
        public static final double GEAR_RATIO = 10.71;

        public static final boolean gyroReversed = true;

        public static final double maxSpeedMetersPerSecond = 3;
        public static final double maxAccelerationMetersPerSecondSquared = 3;

        // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
        public static final double ramseteB = 2.3;
        public static final double ramseteZeta = 1.5;

        // X-vector Controller
        public static final double PXController = 0.8;
        public static double IXController = 0.0001;
        public static double DXController = 0.00001;

        // Yaw Controller
        public static final double PYawController = 0.8;
        public static final double KIYawController = 0.0001;
        public static double DYawController = 0.00001;

        // Turn Controller
        public static final double PTurnController = 0.04;
        public static final double KITurnController = 0.0009;
        public static double DTurnController = 0.0000000;
    }
}
