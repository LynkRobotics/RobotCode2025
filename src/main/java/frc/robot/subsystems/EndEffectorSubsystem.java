// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.LoggedAlert;
import frc.robot.Constants;
import frc.robot.subsystems.RobotState.GamePieceState;

public class EndEffectorSubsystem extends SubsystemBase {
    /* Devices */
    private final TalonFX motor;
    /* Control Requests */
    private final VoltageOut feedControl = new VoltageOut(Constants.EndEffector.feedVoltage).withEnableFOC(true);
    private final VoltageOut advanceControl = new VoltageOut(Constants.EndEffector.advanceVoltage).withEnableFOC(true);
    private final VoltageOut scoreControl = new VoltageOut(Constants.EndEffector.scoreVoltage).withEnableFOC(true);
    private final VoltageOut algaeControl = new VoltageOut(Constants.EndEffector.algaeVoltage).withEnableFOC(true);
    private final VoltageOut algaeOutControl = new VoltageOut(Constants.EndEffector.algaeOutVoltage).withEnableFOC(true);

    private GamePieceState lastState = GamePieceState.NONE;

    public EndEffectorSubsystem() {
        /* Devices */
        motor = new TalonFX(Constants.EndEffector.motorID, Constants.EndEffector.canBus);

        applyConfigs();

        SmartDashboard.putNumber("Algae voltage", Constants.EndEffector.algaeVoltage);
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
        GamePieceState gamePieceState = RobotState.getGamePieceState();
        double motorTemp = motor.getDeviceTemp().getValueAsDouble();

        if (gamePieceState != lastState) {
            if (gamePieceState == GamePieceState.NONE) {
                DogLog.log("EndEffector/Control", "Stopping (no game piece)");
                motor.stopMotor();
            } else if (gamePieceState == GamePieceState.INTAKING_ALGAE) {
                DogLog.log("EndEffector/Control", "Intaking algae");
                motor.setControl(algaeControl.withOutput(SmartDashboard.getNumber("Algae voltage", Constants.EndEffector.algaeVoltage)));
            } else if (gamePieceState == GamePieceState.HOLDING_ALGAE) {
                // No change currently, but issue it anyway, for kicks
                DogLog.log("EndEffector/Control", "Holding algae");
                motor.setControl(algaeControl.withOutput(SmartDashboard.getNumber("Algae voltage", Constants.EndEffector.algaeVoltage)));
            } else if (gamePieceState == GamePieceState.SCORING_ALGAE) {
                motor.setControl(algaeOutControl);
            } else if (gamePieceState == GamePieceState.INTAKING_CORAL) {
                DogLog.log("EndEffector/Control", "Stop (intaking coral)");
                motor.stopMotor();
            } else if (gamePieceState == GamePieceState.FEEDING_CORAL) {
                DogLog.log("EndEffector/Control", "Feeding coral");
                motor.setControl(feedControl);
            } else if (gamePieceState == GamePieceState.ADVANCING_CORAL) {
                DogLog.log("EndEffector/Control", "Advancing coral");
                motor.setControl(advanceControl);
            } else if (gamePieceState == GamePieceState.HOLDING_CORAL) {
                DogLog.log("EndEffector/Control", "Stopping (coral ready)");
                motor.stopMotor();
            } else if (gamePieceState == GamePieceState.SCORING_CORAL) {
                DogLog.log("EndEffector/Control", "Scoring coral");
                motor.setControl(scoreControl);
            } else {
                DogLog.log("EndEffector/Control", "Stopping (unhandled state)");
                motor.stopMotor();
            }
            lastState = gamePieceState;
        }
        
        double current = motor.getTorqueCurrent().getValueAsDouble();
        DogLog.log("EndEffector/TorqueCurrent", motor.getTorqueCurrent().getValueAsDouble());

        if (gamePieceState == GamePieceState.INTAKING_ALGAE && current > Constants.EndEffector.minimumAlgaeAcquireCurrent && !RobotState.getFinalSensor()) {
            RobotState.setHaveAlgae();
        }
        if (gamePieceState == GamePieceState.HOLDING_ALGAE && current < Constants.EndEffector.minimumAlgaeHoldCurrent) {
            RobotState.setNoAlgae();
        }

        DogLog.log("EndEffector/Motor Temp", motorTemp);

        if (motorTemp > 70){ //TODO tune and put in constants
            LoggedAlert.Warning("EndEffector", "TEMP WARNING", "End Effector Motor is approaching hot temperatures");
        }
        if (motorTemp > 90){
            LoggedAlert.Warning("EndEffector", "TEMP WARNING", "End Effector Motor is approaching temperature max");
        }

    }
}