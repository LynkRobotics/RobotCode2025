// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.index;

import static frc.robot.Options.optIndexEnabled;
import static frc.robot.Options.optServiceMode;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.robotstate.RobotState;
import frc.robot.subsystems.robotstate.RobotState.GamePieceState;

public class IndexSubsystem extends SubsystemBase {
    /* Devices */
    private final TalonFX motor;
    /* Control Requests */
    private final VoltageOut intakeControl = new VoltageOut(IndexConstants.intakeVoltage).withEnableFOC(true);
    private final VoltageOut rejectControl = new VoltageOut(IndexConstants.rejectVoltage).withEnableFOC(true);
    private final VoltageOut unjamControl = new VoltageOut(IndexConstants.unjamVoltage).withEnableFOC(true);
    private boolean intaking = false;

    private int stallCount = 0;

    public IndexSubsystem() {
        /* Devices */
        motor = new TalonFX(IndexConstants.motorID, IndexConstants.canBus);

        applyConfigs();
    }

    public void applyConfigs() {
        /* Configure the motor */
        var indexMotorConfig = new TalonFXConfiguration();
        /* Set motor to brake control */
        indexMotorConfig.MotorOutput.NeutralMode = IndexConstants.motorNeutralValue;
        /* Set the motor direction */
        indexMotorConfig.MotorOutput.Inverted = IndexConstants.motorOutputInverted;
        /* Config the peak outputs */
        indexMotorConfig.Voltage.PeakForwardVoltage = IndexConstants.peakForwardVoltage;
        indexMotorConfig.Voltage.PeakReverseVoltage = IndexConstants.peakReverseVoltage;
        /* Apply Index Motor Configs */
        motor.getConfigurator().apply(indexMotorConfig);
    }

    @Override
    public void periodic() {
        Command currentCommand = getCurrentCommand();
        DogLog.log("Index/Current Command", currentCommand == null ? "None" : currentCommand.getName());

        GamePieceState gamePieceState = RobotState.getGamePieceState();
        double velocity = motor.getVelocity().getValueAsDouble();

        if (optServiceMode.get()) {
            if (velocity != 0.0) {
                motor.stopMotor();
            }
        } else if (optIndexEnabled.get()) {
            if (gamePieceState == GamePieceState.INTAKING_CORAL && Math.abs(velocity) < IndexConstants.minIntakeVelocity) {
                stallCount++;
                if (stallCount > IndexConstants.maxStallCount) {
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
        }

        DogLog.log("Index/TorqueCurrent", motor.getTorqueCurrent().getValueAsDouble());
        DogLog.log("Index/Velocity", velocity);
    }
}