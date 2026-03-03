// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.commands.PathfindingCommand;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.enums.Constants.DriveConstants;

public class DriveSubsystem extends SubsystemBase {
    private final SparkMax leftLeader;
    private final SparkMax leftFollower;
    private final SparkMax rightLeader;
    private final SparkMax rightFollower;

    private final DifferentialDrive drive;


    /** Creates a new DriveSubsystem. */
    public DriveSubsystem() {
        // create brushed motors for drive
        leftLeader = new SparkMax(DriveConstants.LEFT_DRIVE_LEAD_ID, MotorType.kBrushed);
        leftFollower = new SparkMax(DriveConstants.LEFT_DRIVE_FOLLOW_ID, MotorType.kBrushed);
        rightLeader = new SparkMax(DriveConstants.RIGHT_DRIVE_LEAD_ID, MotorType.kBrushed);
        rightFollower = new SparkMax(DriveConstants.RIGHT_DRIVE_FOLLOW_ID, MotorType.kBrushed);

        // set up differential drive class
        drive = new DifferentialDrive(leftLeader, rightLeader);

        // Set can time out. Because this project only sets parameters once on
        // construction, the timeout can be long without blocking robot operation. Code
        // which sets or gets parameters during operation may need a shorter timeout.
        leftLeader.setCANTimeout(250);
        rightLeader.setCANTimeout(250);
        leftFollower.setCANTimeout(250);
        rightFollower.setCANTimeout(250);

        // Create the configuration to apply to motors. Voltage compensation
        // helps the robot perform more similarly on different
        // battery voltages (at the cost of a little bit of top speed on a fully charged
        // battery). The current limit helps prevent tripping
        // breakers.
        SparkMaxConfig config = new SparkMaxConfig();
        config.voltageCompensation(12);
        config.smartCurrentLimit(DriveConstants.DRIVE_MOTOR_CURRENT_LIMIT);
        config.idleMode(IdleMode.kBrake);

        // Set configuration to follow each leader and then apply it to corresponding
        // follower. Resetting in case a new controller is swapped
        // in and persisting in case of a controller reset due to breaker trip
        config.follow(leftLeader);
        leftFollower.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        config.follow(rightLeader);
        rightFollower.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // Remove following, then apply config to right leader
        config.disableFollowerMode();
        rightLeader.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        // Set config to inverted and then apply to left leader. Set Left side inverted
        // so that positive values drive both sides forward
        config.inverted(true);
        leftLeader.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        //navx.resetYaw();
        //resetEncoders();

        // Get the encoders from the leaders (followers don't have encoders)
        //leftRelativeEncoder = leftLeader.getEncoder();
        //rightRelativeEncoder = rightLeader.getEncoder();

        //odometry = new DifferentialDriveOdometry(navx.getRotation2d(), getLeftRelativeEncoderDistance(), getRightRelativeEncoderDistance());

        /*try {
            RobotConfig robotConfig = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                    this::getPose, // Robot pose supplier
                    this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
                    this::getRobotVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                    (speeds, feedforwards) -> drive(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
                    new PPLTVController(0.02), // PPLTVController is the built in path following controller for differential drive trains
                    robotConfig, // The robot configuration
                    () -> {
                        // Flip path for red alliance
                        var alliance = DriverStation.getAlliance();
                        return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
                    },
                    this // Reference to this subsystem to set requirements
            );
        } catch (Exception e) {
            DriverStation.reportError("Failed to load PathPlanner config: " + e.getMessage(), e.getStackTrace());
        }*/

        // Warmup pathfinding command
        CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand());
    }

    @Override
    public void periodic() {
        //odometry.update(navx.getRotation2d(), getLeftRelativeEncoderDistance(), getRightRelativeEncoderDistance());
        //field2d.setRobotPose(odometry.getPoseMeters());
    }

    public void driveArcade(double xSpeed, double zRotation) {
        drive.arcadeDrive(xSpeed, zRotation);
    }

    /**
     * Drives the robot using the built in arcade drive method. This is a simple
     * method that does not use any sensors or advanced control, and is useful for
     * teleop control or simple autonomous paths.
     *
     * @param speeds The desired chassis speeds for the robot
     */
    public void drive(ChassisSpeeds speeds){
        drive.feed();

        //xPidController.setSetpoint(speeds.vxMetersPerSecond);
        //yawPidController.setSetpoint(speeds.omegaRadiansPerSecond);

        //drive.arcadeDrive(xPidController.calculate(getRobotVelocity().vxMetersPerSecond), yawPidController.calculate(getRobotVelocity().omegaRadiansPerSecond));
    }

    /**
     * Gets the current pose of the robot from odometry.
     *
     * @return The current Pose2d of the robot
     */
    /*public Pose2d getPose() {
        return odometry.getPoseMeters();
    }*/

    /**
     * Resets the odometry to a given pose.
     *
     * @param pose The pose to reset to
     */
    /*public void resetOdometry(Pose2d pose) {
        resetEncoders();
        odometry.resetPosition(navx.getRotation2d(), getLeftRelativeEncoderDistance(), getRightRelativeEncoderDistance(), pose);
    }*/

    /**
     * Gets the current robot velocity as ChassisSpeeds.
     *
     * @return The current robot-relative ChassisSpeeds
     */
    /*public ChassisSpeeds getRobotVelocity() {
        return kinematics.toChassisSpeeds(getWheelSpeeds());
    }*/

    /**
     * Returns the current wheel speeds of the robot.
     *
     * @return The current wheel speeds.
     */
    /*public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(leftRelativeEncoder.getVelocity(), rightRelativeEncoder.getVelocity());
    }*/

    /** Resets the drive encoders to currently read a position of 0. */
    /*private void resetEncoders() {
        leftRelativeEncoder.setPosition(0);
        rightRelativeEncoder.setPosition(0);
    }*/

    /** Helper method to convert encoder positions to distances. This is needed because the encoders return a position in "rotations" which needs to be converted to a distance in meters. The conversion depends on the wheel circumference and the gear ratio between the motor and the wheels. */
    /*private double getLeftRelativeEncoderDistance() {
        return (leftRelativeEncoder.getPosition() * WHEEL_CIRCUMFERENCE) / GEAR_RATIO;
    }*/

    /** Helper method to convert encoder positions to distances. This is needed because the encoders return a position in "rotations" which needs to be converted to a distance in meters. The conversion depends on the wheel circumference and the gear ratio between the motor and the wheels. */
    /*private double getRightRelativeEncoderDistance() {
        return (rightRelativeEncoder.getPosition() * WHEEL_CIRCUMFERENCE) / GEAR_RATIO;
    }*/

}