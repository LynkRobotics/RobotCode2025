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
import frc.robot.subsystems.RobotState.GamePieceState;

public class IndexSubsystem extends SubsystemBase {
    /* Devices */
    private final TalonFX motor;
    /* Control Requests */
    private final VoltageOut intakeControl = new VoltageOut(Constants.Index.intakeVoltage).withEnableFOC(true);
    private final VoltageOut rejectControl = new VoltageOut(Constants.Index.rejectVoltage).withEnableFOC(true);
    private final VoltageOut unjamControl = new VoltageOut(Constants.Index.unjamVoltage).withEnableFOC(true);
    private boolean intaking = false;

    private int stallCount = 0;

    public IndexSubsystem() {
        /* Devices */
        motor = new TalonFX(Constants.Index.motorID, Constants.Index.canBus);

        applyConfigs();
    }

    public void applyConfigs() {
        /* Configure the motor */
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
        GamePieceState gamePieceState = RobotState.getGamePieceState();
        double velocity = motor.getVelocity().getValueAsDouble();

        if (gamePieceState == GamePieceState.INTAKING_CORAL && Math.abs(velocity) < Constants.Index.minIntakeVelocity) {
            stallCount++;
            if (stallCount > Constants.Index.maxStallCount) {
                RobotState.UnjamCoral().schedule();
            }
        } else {
            stallCount = 0;
        }

        if (gamePieceState == GamePieceState.INTAKING_CORAL || gamePieceState == GamePieceState.FEEDING_CORAL) {
            if (!intaking) {
                DogLog.log("Index/Control", "Intaking");
                motor.setControl(intakeControl);
                intaking = true;
            }
        } else if (gamePieceState == GamePieceState.UNJAMMING_CORAL) {
            DogLog.log("Index/Control", "Unjamming");
            motor.setControl(unjamControl);
            intaking = false;
        } else {
            if (intaking) {
                DogLog.log("Index/Control", "Rejecting");
                motor.setControl(rejectControl);
                intaking = false;
            }
        }

        DogLog.log("Index/TorqueCurrent", motor.getTorqueCurrent().getValueAsDouble());
        DogLog.log("Index/Velocity", velocity);
    }
}