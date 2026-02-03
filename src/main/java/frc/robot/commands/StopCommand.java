package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.FuelIntakeAndLauncherSubsystem;


public class StopCommand extends Command {
    private final FuelIntakeAndLauncherSubsystem fuelIntakeAndLauncherSubsystem = FuelIntakeAndLauncherSubsystem.getInstance();
    private final FeederSubsystem feederSubsystem = FeederSubsystem.getInstance();

    public StopCommand() {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.fuelIntakeAndLauncherSubsystem, this.feederSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        fuelIntakeAndLauncherSubsystem.setVoltage(0);
        feederSubsystem.setVoltage(0);
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }

    @Override
    public void end(boolean interrupted) {

    }
}
