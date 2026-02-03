package frc.robot.subsystems;


import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveSubsystem extends SubsystemBase {

    private final SparkMax leftDriveLead = new SparkMax(Constants.DriveConstants.LEFT_DRIVE_LEAD_ID, SparkMax.MotorType.kBrushed);
    private final SparkMax leftDriveFollow = new SparkMax(Constants.DriveConstants.LEFT_DRIVE_FOLLOW_ID, SparkMax.MotorType.kBrushed);
    private final SparkMax rightDriveLead = new SparkMax(Constants.DriveConstants.RIGHT_DRIVE_LEAD_ID, SparkMax.MotorType.kBrushed);
    private final SparkMax rightDriveFollow = new SparkMax(Constants.DriveConstants.RIGHT_DRIVE_FOLLOW_ID, SparkMax.MotorType.kBrushed);

    private final DifferentialDrive drive = new DifferentialDrive(leftDriveLead, rightDriveLead);

    /**
     * The Singleton instance of this DriveSubsystem. Code should use
     * the {@link #getInstance()} method to get the single instance (rather
     * than trying to construct an instance of this class.)
     */
    private final static DriveSubsystem INSTANCE = new DriveSubsystem();

    /**
     * Returns the Singleton instance of this DriveSubsystem. This static method
     * should be used, rather than the constructor, to get the single instance
     * of this class. For example: {@code DriveSubsystem.getInstance();}
     */
    @SuppressWarnings("WeakerAccess")
    public static DriveSubsystem getInstance() {
        return INSTANCE;
    }

    /**
     * Creates a new instance of this DriveSubsystem. This constructor
     * is private since this class is a Singleton. Code should use
     * the {@link #getInstance()} method to get the singleton instance.
     */
    private DriveSubsystem() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.voltageCompensation(12);
        config.smartCurrentLimit(60);

        config.follow(leftDriveLead);
        leftDriveFollow.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        config.follow(rightDriveFollow);
        rightDriveFollow.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        config.disableFollowerMode();
        config.inverted(false);
        leftDriveLead.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void drive(double forward, double rotation) {
        drive.arcadeDrive(forward, rotation);
    }
}

