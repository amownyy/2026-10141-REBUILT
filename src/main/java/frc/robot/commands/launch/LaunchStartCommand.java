package frc.robot.commands.launch;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.FuelIntakeAndLauncherSubsystem;


public class LaunchStartCommand extends Command {
    private final FuelIntakeAndLauncherSubsystem fuelIntakeAndLauncherSubsystem = FuelIntakeAndLauncherSubsystem.getInstance();
    private final FeederSubsystem feederSubsystem = FeederSubsystem.getInstance();

    private final Timer spinUpTimer = new Timer();

    public LaunchStartCommand() {
        // each subsystem used by the command must be passed into the
        // addRequirements() method (which takes a vararg of Subsystem)
        addRequirements(this.fuelIntakeAndLauncherSubsystem, this.feederSubsystem);
    }

    @Override
    public void initialize() {
        spinUpTimer.reset();
        spinUpTimer.start();
    }

    @Override
    public void execute() {
        if (spinUpTimer.get() < Constants.SpinConstants.SPIN_UP_SECONDS) {
            fuelIntakeAndLauncherSubsystem.setVoltage(Constants.IntakeLauncherConstants.LAUNCH_VOLTAGE);
            feederSubsystem.setVoltage(Constants.SpinConstants.SPIN_UP_VOLTAGE);
        } else {
            fuelIntakeAndLauncherSubsystem.setVoltage(Constants.IntakeLauncherConstants.LAUNCH_VOLTAGE);
            feederSubsystem.setVoltage(Constants.FeederConstants.FEEDER_LAUNCH_VOLTAGE);
        }
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
