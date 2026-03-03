package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FuelSubsystem;

public class AutoCommand extends SequentialCommandGroup {
    /** Creates a new ExampleAuto. */
    public AutoCommand(DriveSubsystem driveSubsystem, FuelSubsystem ballSubsystem) {
        // Add your commands in the addCommands() call, e.g.
        // addCommands(new FooCommand(), new BarCommand());
        addCommands(
                // Drive backwards for .25 seconds. The driveArcadeAuto command factory
                // intentionally creates a command which does not end which allows us to control
                // the timing using the withTimeout decorator
                new AutoDrive(driveSubsystem,0.5,  0.0).withTimeout(3),
                // Spin up the launcher for 1 second and then launch balls for 9 seconds, for a
                // total of 10 seconds
                ballSubsystem.windUpAndLaunch().withTimeout(10));
    }
}