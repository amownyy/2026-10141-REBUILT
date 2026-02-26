// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FuelSubsystem;
import edu.wpi.first.wpilibj.Timer;

import static frc.robot.Constants.FuelConstants.*;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeTimed extends Command {

    /** Creates a new Intake. */
    private final FuelSubsystem fuelSubsystem;

    public IntakeTimed(FuelSubsystem fuelSystem, double timeout) {
        addRequirements(fuelSystem);
        this.fuelSubsystem = fuelSystem;
        this.timeout = timeout;
    }

    private Timer timer;
    private final double timeout;

    // Called when the command is initially scheduled. Set the rollers to the
    // appropriate values for intake
    @Override
    public void initialize() {
        fuelSubsystem.setIntakeLauncherRoller(SmartDashboard.getNumber("Intake intake roller value", INTAKE_INTAKE_PERCENT));
        fuelSubsystem.setFeederRoller(SmartDashboard.getNumber("Intake feeder roller value", INDEXER_INTAKE_PERCENT));
        this.timer = new Timer();
        this.timer.reset();
        this.timer.start();
    }

    // Called every time the scheduler runs while the command is scheduled. This
    // command doesn't require updating any values while running
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted. Stop the rollers
    @Override
    public void end(boolean interrupted) {
        fuelSubsystem.setIntakeLauncherRoller(0);
        fuelSubsystem.setFeederRoller(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return this.timer.hasElapsed(this.timeout);
    }
}