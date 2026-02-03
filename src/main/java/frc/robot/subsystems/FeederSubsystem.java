package frc.robot.subsystems;


import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FeederSubsystem extends SubsystemBase {

    private final SparkMax feederMotor = new SparkMax(Constants.FeederConstants.MOTOR_ID, SparkMax.MotorType.kBrushed);

    /**
     * The Singleton instance of this FeederSubsystem. Code should use
     * the {@link #getInstance()} method to get the single instance (rather
     * than trying to construct an instance of this class.)
     */
    private final static FeederSubsystem INSTANCE = new FeederSubsystem();

    /**
     * Returns the Singleton instance of this FeederSubsystem. This static method
     * should be used, rather than the constructor, to get the single instance
     * of this class. For example: {@code FeederSubsystem.getInstance();}
     */
    @SuppressWarnings("WeakerAccess")
    public static FeederSubsystem getInstance() {
        return INSTANCE;
    }

    /**
     * Creates a new instance of this FeederSubsystem. This constructor
     * is private since this class is a Singleton. Code should use
     * the {@link #getInstance()} method to get the singleton instance.
     */
    private FeederSubsystem() {
        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit(60);
        feederMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setVoltage(double voltage) {
        feederMotor.setVoltage(voltage);
    }
}

