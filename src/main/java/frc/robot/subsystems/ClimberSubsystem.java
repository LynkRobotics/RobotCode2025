// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.LoggedCommands;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
    /* Devices */
    private final TalonFX motor;

    /* Control Requests */
    private final VoltageOut deployControl = new VoltageOut(Constants.Climber.deployVoltage).withEnableFOC(true);
    private final VoltageOut retractControl = new VoltageOut(Constants.Climber.retractVoltage).withEnableFOC(true);

    public ClimberSubsystem() {
        /* Devices */
        motor = new TalonFX(Constants.Index.motorID, Constants.Index.canBus);

        applyConfigs();
    }

    public void applyConfigs() {
        /* Configure the motor */
        var motorConfig = new TalonFXConfiguration();
        /* Set motor to brake control */
        motorConfig.MotorOutput.NeutralMode = Constants.Index.motorNeutralValue;
        /* Set the motor direction */
        motorConfig.MotorOutput.Inverted = Constants.Index.motorOutputInverted;
        /* Config the peak outputs */
        motorConfig.Voltage.PeakForwardVoltage = Constants.Index.peakForwardVoltage;
        motorConfig.Voltage.PeakReverseVoltage = Constants.Index.peakReverseVoltage;
        /* Apply Index Motor Configs */
        motor.getConfigurator().apply(motorConfig);
    }

    public Command Deploy() {
        return LoggedCommands.none("Deploy Climber");
    }

    public Command Retract() {
        return LoggedCommands.none("Retract Climber");
    }

    @Override
    public void periodic() {
        DogLog.log("Climber/TorqueCurrent", motor.getTorqueCurrent().getValueAsDouble());
        DogLog.log("Climber/Velocity", motor.getVelocity().getValueAsDouble());
    }
}