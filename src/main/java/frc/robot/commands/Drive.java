// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.DriveSubsystem;

import static frc.robot.Constants.DriveConstants.*;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Drive extends Command {

    /** Creates a new Drive. */
    private final DriveSubsystem driveSubsystem;
    private final CommandXboxController controller;
    private final boolean slowMode;

    public Drive(DriveSubsystem driveSystem, CommandXboxController driverController, boolean isSlowMode) {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(driveSystem);
        driveSubsystem = driveSystem;
        controller = driverController;
        slowMode = isSlowMode;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    // The Y axis of the controller is inverted so that pushing the
    // stick away from you (a negative value) drives the robot forwards (a positive
    // value). The X axis is scaled down so the rotation is more easily
    // controllable.
    @Override
    public void execute() {
        if (!slowMode) {
            driveSubsystem.driveArcade(-controller.getLeftY() * DRIVE_FORWARD_MULTIPLIER, -controller.getRightX() * DRIVE_ROTATION_MULTIPLIER);
        } else {
            driveSubsystem.driveArcade(-controller.getLeftY() * SLOW_DRIVE_FORWARD_MULTIPLIER, -controller.getRightX() * SLOW_DRIVE_ROTATION_MULTIPLIER);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        driveSubsystem.driveArcade(0, 0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}