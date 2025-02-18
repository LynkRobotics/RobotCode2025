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
import frc.robot.subsystems.RobotState.AlgaeState;
import frc.robot.subsystems.RobotState.CoralState;

public class EndEffectorSubsystem extends SubsystemBase {
    /* Devices */
    private final TalonFX motor;
    /* Control Requests */
    private final VoltageOut feedControl = new VoltageOut(Constants.EndEffector.feedVoltage).withEnableFOC(true);
    private final VoltageOut advanceControl = new VoltageOut(Constants.EndEffector.advanceVoltage).withEnableFOC(true);
    private final VoltageOut scoreControl = new VoltageOut(Constants.EndEffector.scoreVoltage).withEnableFOC(true);
    private final VoltageOut algaeControl = new VoltageOut(Constants.EndEffector.algaeVoltage).withEnableFOC(true);
    private final VoltageOut algaeOutControl = new VoltageOut(Constants.EndEffector.algaeOutVoltage).withEnableFOC(true);
    
    CoralState lastCoralState = CoralState.REJECTING;
    AlgaeState lastAlgaeState = AlgaeState.NONE;

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
        AlgaeState algaeState = RobotState.getAlgaeState();
        CoralState coralState = RobotState.getCoralState();

        if (algaeState != lastAlgaeState) {
            if (algaeState == AlgaeState.NONE) {
                DogLog.log("EndEffector/Control", "Stopping (no algae)");
                motor.stopMotor();
            } else if (algaeState == AlgaeState.INTAKING) {
                DogLog.log("EndEffector/Control", "Intaking algae");
                motor.setControl(algaeControl);
            } else if (algaeState == AlgaeState.HOLDING) {
                // No change currently
            } else if (algaeState == AlgaeState.SCORING) {
                motor.setControl(algaeOutControl);
                // TODO How to persist?
            }
            lastAlgaeState = algaeState;
        }
        if (coralState != lastCoralState || (algaeState != lastAlgaeState && algaeState == AlgaeState.NONE)) {
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
                DogLog.log("EndEffector/Control", "Stopping (coral state)");
                motor.stopMotor();
            }
            lastCoralState = coralState;
        }
        
        double current = motor.getTorqueCurrent().getValueAsDouble();
        DogLog.log("EndEffector/TorqueCurrent", motor.getTorqueCurrent().getValueAsDouble());

        if (algaeState != AlgaeState.NONE) {
            if (current > 80.0 && !RobotState.getFinalSensor()) {
                RobotState.setHaveAlgae();
            }
            if (current < 60.0) {
                RobotState.setNoAlgae();
            }
        }
    }
}