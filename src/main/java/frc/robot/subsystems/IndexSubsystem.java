// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.RobotState.CoralState;

public class IndexSubsystem extends SubsystemBase {
    /* Devices */
    private final TalonFX motor;
    /* Control Requests */
    private final VoltageOut intakeControl = new VoltageOut(Constants.Index.intakeVoltage).withEnableFOC(true);
    private final VoltageOut rejectControl = new VoltageOut(Constants.Index.rejectVoltage).withEnableFOC(true);
    private boolean intaking = false;

    public IndexSubsystem() {
        /* Devices */
        motor = new TalonFX(Constants.Index.motorID, Constants.Index.canBus);

        applyConfigs();
    }

    public void applyConfigs() {
        /* Configure the EndAffector Motor */
        var indexMotorConfig = new TalonFXConfiguration();
        /* Set motor to brake control */
        indexMotorConfig.MotorOutput.NeutralMode = Constants.Index.motorNeutralValue;
        /* Set the motor direction */
        indexMotorConfig.MotorOutput.Inverted = Constants.Index.motorOutputInverted;
        /* Config the peak outputs */
        indexMotorConfig.Voltage.PeakForwardVoltage = Constants.Index.peakForwardVoltage;
        indexMotorConfig.Voltage.PeakReverseVoltage = Constants.Index.peakReverseVoltage;
        /* Apply Index Motor Configs */
        motor.getConfigurator().apply(indexMotorConfig);
    }

    @Override
    public void periodic() {
        CoralState coralState = RobotState.getCoralState();

        if (coralState == CoralState.INTAKING || coralState == CoralState.FEEDING) {
            if (!intaking) {
                DogLog.log("Index/Control", "Intaking");
                motor.setControl(intakeControl);
                intaking = true;
            }
        } else {
            if (intaking) {
                DogLog.log("Index/Control", "Rejecting");
                motor.setControl(rejectControl);
                intaking = false;
            }
        }
    }
}