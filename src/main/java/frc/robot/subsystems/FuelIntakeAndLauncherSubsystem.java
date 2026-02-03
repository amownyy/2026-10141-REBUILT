package frc.robot.subsystems;


import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FuelIntakeAndLauncherSubsystem extends SubsystemBase {

    private final SparkMax fuelLauncherMotor = new SparkMax(Constants.IntakeLauncherConstants.MOTOR_ID, SparkMax.MotorType.kBrushed);

    /**
     * The Singleton instance of this FuelLauncherSubsystem. Code should use
     * the {@link #getInstance()} method to get the single instance (rather
     * than trying to construct an instance of this class.)
     */
    private final static FuelIntakeAndLauncherSubsystem INSTANCE = new FuelIntakeAndLauncherSubsystem();

    /**
     * Returns the Singleton instance of this FuelLauncherSubsystem. This static method
     * should be used, rather than the constructor, to get the single instance
     * of this class. For example: {@code FuelLauncherSubsystem.getInstance();}
     */
    @SuppressWarnings("WeakerAccess")
    public static FuelIntakeAndLauncherSubsystem getInstance() {
        return INSTANCE;
    }

    /**
     * Creates a new instance of this FuelLauncherSubsystem. This constructor
     * is private since this class is a Singleton. Code should use
     * the {@link #getInstance()} method to get the singleton instance.
     */
    private FuelIntakeAndLauncherSubsystem() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit(60);
        config.inverted(true);
        fuelLauncherMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setVoltage(double voltage) {
        fuelLauncherMotor.setVoltage(voltage);
    }
}

