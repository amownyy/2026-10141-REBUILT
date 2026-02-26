package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.AutoLaunchSequence;
import frc.robot.commands.ClimbDownTimed;
import frc.robot.commands.ClimbUpTimed;
import frc.robot.subsystems.FuelSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.commands.EjectTimed;
import frc.robot.commands.IntakeTimed;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 *
 * @author Team 6045
 */
@SuppressWarnings("unused")
public class Autos {
    @SuppressWarnings("FieldMayBeFinal")
    private SendableChooser<Command> autoChooser;

    public Autos(FuelSubsystem fuelSubsystem, ClimberSubsystem climberSubsystem) {
        // Named Commands //
        NamedCommands.registerCommand("AutoLaunchSequence", new AutoLaunchSequence(fuelSubsystem));
        NamedCommands.registerCommand("ClimbUpL1Timed", new ClimbUpTimed(climberSubsystem, Constants.AutoConstants.CLIMB_UP_L1_SECONDS));
        NamedCommands.registerCommand("ClimbUpL2Timed", new ClimbUpTimed(climberSubsystem, Constants.AutoConstants.CLIMB_UP_L2_SECONDS));
        NamedCommands.registerCommand("ClimbDownTimed", new ClimbDownTimed(climberSubsystem, Constants.AutoConstants.CLIMB_DOWN_SECONDS));
        NamedCommands.registerCommand("EjectTimed", new EjectTimed(fuelSubsystem, Constants.AutoConstants.EJECT_SECONDS));
        NamedCommands.registerCommand("IntakeTimed", new IntakeTimed(fuelSubsystem, Constants.AutoConstants.INTAKE_SECONDS));

        // Autos //
        //noinspection MoveFieldAssignmentToInitializer,Convert2Diamond
        autoChooser = new SendableChooser<Command>();
        createAuto("shootTwiceAndClimb", "Shoot twice and climb", true, true, true, autoChooser);
        createAuto("shootOnceAndClimb", "Shoot once and climb", true, true, true, autoChooser);
        createAuto("kamikaze", "Kamikaze", true, false, false, autoChooser);

        SmartDashboard.putData("Autos", autoChooser);
    }

    @SuppressWarnings("SameParameterValue")
    private void createAuto(String name, String friendlyName, Boolean hasPositionVariants, Boolean hasClimbVariants, Boolean hasClimbLevelVariants, SendableChooser<Command> autoChooser) {
        if (hasPositionVariants) {
            if (hasClimbVariants) {
                if (hasClimbLevelVariants) {
                    autoChooser.addOption("[L] [CL] [L1] " + friendlyName, AutoBuilder.buildAuto(name + "LeftCLeft"));
                    autoChooser.addOption("[L] [CL] [L2] " + friendlyName, AutoBuilder.buildAuto(name + "LeftCLeft2"));
                    autoChooser.addOption("[L] [CR] [L1] " + friendlyName, AutoBuilder.buildAuto(name + "LeftCRight"));
                    autoChooser.addOption("[L] [CR] [L2] " + friendlyName, AutoBuilder.buildAuto(name + "LeftCRight2"));
                    autoChooser.addOption("[M] [CL] [L1] " + friendlyName, AutoBuilder.buildAuto(name + "CLeft"));
                    autoChooser.addOption("[M] [CL] [L2] " + friendlyName, AutoBuilder.buildAuto(name + "CLeft2"));
                    autoChooser.addOption("[M] [CR] [L1] " + friendlyName, AutoBuilder.buildAuto(name + "CRight"));
                    autoChooser.addOption("[M] [CR] [L2] " + friendlyName, AutoBuilder.buildAuto(name + "CRight2"));
                    autoChooser.addOption("[R] [CL] [L1] " + friendlyName, AutoBuilder.buildAuto(name + "RightCLeft"));
                    autoChooser.addOption("[R] [CL] [L2] " + friendlyName, AutoBuilder.buildAuto(name + "RightCLeft2"));
                    autoChooser.addOption("[R] [CR] [L1]" + friendlyName, AutoBuilder.buildAuto(name + "RightCRight"));
                    autoChooser.addOption("[R] [CR] [L2]" + friendlyName, AutoBuilder.buildAuto(name + "RightCRight2"));
                } else {
                    autoChooser.addOption("[L] [CL] " + friendlyName, AutoBuilder.buildAuto(name + "LeftCLeft"));
                    //autoChooser.addOption("[L] [CM] " + friendlyName, AutoBuilder.buildAuto(name+"Left"));
                    autoChooser.addOption("[L] [CR] " + friendlyName, AutoBuilder.buildAuto(name + "LeftCRight"));
                    autoChooser.addOption("[M] [CL] " + friendlyName, AutoBuilder.buildAuto(name + "CLeft"));
                    //autoChooser.addOption("[M] [CM] " + friendlyName, AutoBuilder.buildAuto(name));
                    autoChooser.addOption("[M] [CR] " + friendlyName, AutoBuilder.buildAuto(name + "CRight"));
                    autoChooser.addOption("[R] [CL] " + friendlyName, AutoBuilder.buildAuto(name + "RightCLeft"));
                    //autoChooser.addOption("[R] [CM] " + friendlyName, AutoBuilder.buildAuto(name+"Right"));
                    autoChooser.addOption("[R] [CR] " + friendlyName, AutoBuilder.buildAuto(name + "RightCRight"));
                }
            } else {
                autoChooser.addOption("[L] " + friendlyName, AutoBuilder.buildAuto(name+"Left"));
                autoChooser.addOption("[M] " + friendlyName, AutoBuilder.buildAuto(name));
                autoChooser.addOption("[R] " + friendlyName, AutoBuilder.buildAuto(name+"Right"));
            }
        } else {
            autoChooser.addOption(friendlyName, AutoBuilder.buildAuto(name));
        }
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}