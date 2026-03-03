// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.commands.AutoCommand;
import frc.robot.commands.Drive;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FuelSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.util.ShooterCalculator;
import frc.robot.util.dashboard.LoggedNetworkInput;
import frc.robot.util.dashboard.MultiMotorInfoSendable;
import frc.robot.util.enums.PositionCalibrationLocation;

import static frc.robot.util.enums.Constants.ControllerConstants;
import static frc.robot.util.enums.Constants.FuelConstants;
import static frc.robot.util.enums.Constants.ClimberConstants;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // Initializes subsystems
    private final DriveSubsystem driveSubsystem;
    private final ClimberSubsystem climberSubsystem;
    private final FuelSubsystem fuelSubsystem;

    private final ShooterCalculator shooterCalculator;
    private final MultiMotorInfoSendable motorInfo = new MultiMotorInfoSendable();

    // Initializes controllers
    private final CommandXboxController driverController =
            new CommandXboxController(ControllerConstants.DRIVER_CONTROLLER_PORT);
    private final CommandXboxController operatorController = ControllerConstants.OPERATOR_ENABLED
            ? new CommandXboxController(ControllerConstants.OPERATOR_CONTROLLER_PORT)
            : null;

    private final SendableChooser<Command> autoChooser;

    private boolean useOdometry = true;
    private final Trigger useOdometryTrigger = new Trigger(() -> useOdometry);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        driveSubsystem = new DriveSubsystem();
        shooterCalculator = new ShooterCalculator(() -> new Pose2d(0, 0, Rotation2d.kZero));
        fuelSubsystem = FuelConstants.FUEL_SUBSYSTEM_ENABLED ? new FuelSubsystem(shooterCalculator, motorInfo) : null;
        climberSubsystem = ClimberConstants.CLIMBER_ENABLED ? new ClimberSubsystem(motorInfo) : null;

        // autoChooser = AutoBuilder.buildAutoChooser("Epic Auto");
        autoChooser = new SendableChooser<>();
        autoChooser.setDefaultOption("Autonomous", new AutoCommand(driveSubsystem, fuelSubsystem));

        configureBindings();
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be
     * created via the {@link Trigger#Trigger(java.util.function.BooleanSupplier)}
     * constructor with an arbitrary predicate, or via the named factories in
     * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses
     * for {@link CommandXboxController Xbox}/
     * {@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
     * controllers or
     * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
        // for testing
        final boolean fieldOriented = true;
        final boolean forceRobotOrientedRotation = true;

        /*if (fieldOriented) {
            if (forceRobotOrientedRotation && operatorController != null) {
                driveSubsystem.setDefaultCommand(driveSubsystem.driveFieldAndRobotOrientedCommand(
                        () -> -driverController.getLeftY(),
                        () -> -driverController.getLeftX(),
                        () -> -driverController.getRightX(),
                        () -> -operatorController.getLeftX(),
                        () -> -operatorController.getLeftY()));
                driverController.rightStick().whileTrue(driveSubsystem.lockYawTowardsVelocity());
            } else {
                driveSubsystem.setDefaultCommand(driveSubsystem.driveFieldOrientedHeadingCommand(
                        () -> -driverController.getLeftY(),
                        () -> -driverController.getLeftX(),
                        () -> -driverController.getRightX(),
                        () -> -driverController.getRightY()));
            }
        } else {
            driveSubsystem.setDefaultCommand(driveSubsystem.driveRobotOrientedCommand(
                    () -> MathUtil.applyDeadband(-driverController.getLeftY(), 0.1),
                    () -> MathUtil.applyDeadband(-driverController.getLeftX(), 0.1),
                    () -> -driverController.getRightX()));
        }

        driverController.start().onTrue(new InstantCommand(driveSubsystem::zeroGyroWithAlliance));
        driverController.y().whileTrue(driveSubsystem.faceTowardsHubCommand());

        if (operatorController != null) {
            operatorController.start().whileTrue(driveSubsystem.straightenWheelsCommand());
            operatorController
                    .leftTrigger()
                    .whileTrue(driveSubsystem.resetPoseFromCalibrationPosition(
                            PositionCalibrationLocation.LEFT_TRENCH_OUTER));
            operatorController
                    .leftBumper()
                    .whileTrue(driveSubsystem.resetPoseFromCalibrationPosition(
                            PositionCalibrationLocation.LEFT_DEPOT_CORNER));
            operatorController
                    .rightTrigger()
                    .whileTrue(driveSubsystem.resetPoseFromCalibrationPosition(
                            PositionCalibrationLocation.RIGHT_TRENCH_OUTER));
            operatorController
                    .rightBumper()
                    .whileTrue(driveSubsystem.resetPoseFromCalibrationPosition(
                            PositionCalibrationLocation.RIGHT_OUTPOST_CORNER));
            operatorController.x().whileTrue(driveSubsystem.enableManualBumpLock());
        }*/
        driverController.leftBumper().whileTrue(new Drive(driveSubsystem, driverController, true));
        driveSubsystem.setDefaultCommand(new Drive(driveSubsystem, driverController, false));

        if (fuelSubsystem != null && operatorController != null) {
            operatorController.leftBumper().whileTrue(fuelSubsystem.intake());
            operatorController.a().whileTrue(fuelSubsystem.eject());
            operatorController.rightBumper().whileTrue(fuelSubsystem.windUpAndLaunch());
        }

        if (climberSubsystem != null && operatorController != null) {
            operatorController.povUp().whileTrue(climberSubsystem.climb());
            operatorController.povDown().whileTrue(climberSubsystem.lower());
            operatorController.povLeft().whileTrue(climberSubsystem.findLimit());
        }
    }

    public void periodic() {}

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An example command will be run in autonomous
        return autoChooser.getSelected();
    }

    public void initSmartDashboard() {
        SmartDashboard.putData("Auto Chooser", autoChooser);
        SmartDashboard.putData(
                "RobotContainer",
                builder -> builder.addBooleanProperty("Use Odometry", () -> useOdometry, v -> useOdometry = v));
        SmartDashboard.putData("Motor Info", motorInfo);
    }

    public void preSchedulerUpdate() {
        shooterCalculator.clearShotCache();
        LoggedNetworkInput.runAllPeriodic();
    }

    public void postSchedulerUpdate() {
        NetworkTableInstance.getDefault().flush();
    }
}