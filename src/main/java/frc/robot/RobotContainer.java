// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.StopCommand;
import frc.robot.commands.eject.EjectStartCommand;
import frc.robot.commands.intake.IntakeStartCommand;
import frc.robot.commands.launch.LaunchStartCommand;
import frc.robot.subsystems.DriveSubsystem;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

    private final CommandXboxController driverController = new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);
    private final CommandXboxController operatorController = new CommandXboxController(OperatorConstants.OPERATOR_CONTROLLER_PORT);

    private double forwardSpeed = Constants.DriveConstants.DRIVE_FORWARD_MULTIPLIER;
    private double rotationSpeed = Constants.DriveConstants.DRIVE_ROTATION_MULTIPLIER;
    
    
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer()
    {
        // Configure the trigger bindings
        configureBindings();
    }
    
    
    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings()
    {
        if (driverController.rightBumper().getAsBoolean()) {
            forwardSpeed = Constants.DriveConstants.SLOW_DRIVE_FORWARD_MULTIPLIER;
            rotationSpeed = Constants.DriveConstants.SLOW_DRIVE_ROTATION_MULTIPLIER;
        } else if (driverController.leftBumper().getAsBoolean()) {
            forwardSpeed = Constants.DriveConstants.DRIVE_FORWARD_MULTIPLIER;
            rotationSpeed = Constants.DriveConstants.DRIVE_ROTATION_MULTIPLIER;
        }

        // Drive Command
        DriveSubsystem.getInstance().setDefaultCommand(new DriveCommand(-driverController.getLeftY() * forwardSpeed, -driverController.getRightX() * rotationSpeed));

        // Launch Mode
        operatorController.rightBumper().whileTrue(
                new LaunchStartCommand()
            ).whileFalse(
                new StopCommand()
        );
        operatorController.leftBumper().whileTrue(
                new IntakeStartCommand()
            ).whileFalse(
                new StopCommand()
        );
        operatorController.a().whileTrue(
                new EjectStartCommand()
            ).whileFalse(
                new StopCommand()
        );
    }
    
    
    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand()
    {
        return null;
    }
}
