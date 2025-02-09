// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.fasterxml.jackson.databind.cfg.CoercionAction;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.LoggedCommands;
import frc.robot.subsystems.RobotState.CoralState;

public class EndEffectorSubsystem extends SubsystemBase {
    /* Devices */
    private final TalonFX motor;
    /* Control Requests */
    private final VoltageOut feedControl = new VoltageOut(Constants.EndEffector.feedVoltage).withEnableFOC(true);
    private final VoltageOut advanceControl = new VoltageOut(Constants.EndEffector.advanceVoltage).withEnableFOC(true);
    private final VoltageOut scoreControl = new VoltageOut(Constants.EndEffector.scoreVoltage).withEnableFOC(true);
    
    CoralState lastState = CoralState.REJECTING;

    public EndEffectorSubsystem() {
        /* Devices */
        motor = new TalonFX(Constants.EndEffector.motorID, Constants.EndEffector.canBus);

        applyConfigs();
    }

    public void applyConfigs() {
        /* Configure the EndAffector Motor */
        var m_endAffectorConfiguration = new TalonFXConfiguration();
        /* Set EndAffector motor to Brake */
        m_endAffectorConfiguration.MotorOutput.NeutralMode = Constants.EndEffector.motorNeutralValue;
        /* Set the motor direction */
        m_endAffectorConfiguration.MotorOutput.Inverted = Constants.EndEffector.motorOutputInverted;
        /* Config the peak outputs */
        m_endAffectorConfiguration.Voltage.PeakForwardVoltage = Constants.EndEffector.peakForwardVoltage;
        m_endAffectorConfiguration.Voltage.PeakReverseVoltage = Constants.EndEffector.peakReverseVoltage;
        /* Apply Index Motor Configs */
        motor.getConfigurator().apply(m_endAffectorConfiguration);
    }

    @Override
    public void periodic() {
        CoralState coralState = RobotState.getCoralState();

        if (coralState != lastState) {
            if (coralState == CoralState.FEEDING) {
                DogLog.log("EndEffector/Control", "Feeding");
                motor.setControl(feedControl);
            } else if (coralState == CoralState.ADVANCING) {
                DogLog.log("EndEffector/Control", "Advancing");
                motor.setControl(advanceControl);
            } else if (coralState == CoralState.READY) {
                DogLog.log("EndEffector/Control", "Stopping (ready)");
                motor.stopMotor();
            } else if (coralState == CoralState.SCORING) {
                DogLog.log("EndEffector/Control", "Scoring");
                motor.setControl(scoreControl);
            } else {
                DogLog.log("EndEffector/Control", "Stopping");
                motor.stopMotor();
            }
        }
        
        lastState = coralState;
    }
}