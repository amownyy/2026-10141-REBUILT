// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.FuelConstants.*;

public class FuelSubsystem extends SubsystemBase {
    private final SparkMax leftIntakeLauncher;
    private final SparkMax rightIntakeLauncher;
    private final SparkMax indexer;

    /** Creates a new FuelSubsystem. */
    public FuelSubsystem() {
        // create brushed motors for each of the motors on the launcher mechanism
        leftIntakeLauncher = new SparkMax(LEFT_INTAKE_LAUNCHER_MOTOR_ID, MotorType.kBrushless);
        rightIntakeLauncher = new SparkMax(RIGHT_INTAKE_LAUNCHER_MOTOR_ID, MotorType.kBrushless);
        indexer = new SparkMax(INDEXER_MOTOR_ID, MotorType.kBrushed);

        // create the configuration for the feeder roller, set a current limit and apply
        // the config to the controller
        SparkMaxConfig feederConfig = new SparkMaxConfig();
        feederConfig.smartCurrentLimit(INDEXER_MOTOR_CURRENT_LIMIT);
        indexer.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // create the configuration for the launcher roller, set a current limit, set
        // the motor to inverted so that positive values are used for both intake and
        // launching, and apply the config to the controller
        SparkMaxConfig launcherConfig = new SparkMaxConfig();

        launcherConfig.smartCurrentLimit(LAUNCHER_MOTOR_CURRENT_LIMIT);
        launcherConfig.voltageCompensation(12);
        launcherConfig.idleMode(IdleMode.kCoast);
        rightIntakeLauncher.configure(launcherConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        launcherConfig.inverted(true);
        leftIntakeLauncher.configure(launcherConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // put default values for various fuel operations onto the dashboard
        // all commands using this subsystem pull values from the dashboard to allow
        // you to tune the values easily, and then replace the values in Constants.java
        // with your new values. For more information, see the Software Guide.
        SmartDashboard.putNumber("Intake feeder roller value", INDEXER_INTAKE_PERCENT);
        SmartDashboard.putNumber("Intake intake roller value", INTAKE_INTAKE_PERCENT);
        SmartDashboard.putNumber("Launching feeder roller value", INDEXER_LAUNCH_PERCENT);
        SmartDashboard.putNumber("Launching launcher roller value", LAUNCHING_LAUNCHER_PERCENT);
        //SmartDashboard.putNumber("Spin-up feeder roller value", SPIN_UP_FEEDER_VOLTAGE);
    }

    // A method to set the voltage of the intake roller
    public void setIntakeLauncherRoller(double power) {
        leftIntakeLauncher.set(power);
        rightIntakeLauncher.set(power); // positive for shooting
    }

    // A method to set the voltage of the intake roller
    public void setFeederRoller(double power) {
        indexer.set(power); // positive for shooting
    }

    // A method to stop the rollers
    public void stop() {
        indexer.set(0);
        leftIntakeLauncher.set(0);
        rightIntakeLauncher.set(0);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
}