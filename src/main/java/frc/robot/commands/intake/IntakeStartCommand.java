package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.FuelIntakeAndLauncherSubsystem;


public class IntakeStartCommand extends Command {
    private final FuelIntakeAndLauncherSubsystem fuelIntakeAndLauncherSubsystem = FuelIntakeAndLauncherSubsystem.getInstance();
    private final FeederSubsystem feederSubsystem = FeederSubsystem.getInstance();

    public IntakeStartCommand() {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.feederSubsystem, this.fuelIntakeAndLauncherSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        fuelIntakeAndLauncherSubsystem.setVoltage(Constants.IntakeLauncherConstants.INTAKE_VOLTAGE);
        feederSubsystem.setVoltage(Constants.FeederConstants.FEEDER_INTAKE_VOLTAGE);
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
