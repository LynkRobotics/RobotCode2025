// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.endeffector;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.elevator.ElevatorConstants.Stop;
import frc.robot.subsystems.robotstate.RobotState;
import frc.robot.subsystems.robotstate.RobotState.GamePieceState;

public class EndEffectorSubsystem extends SubsystemBase {
    /* Devices */
    private final TalonFX motor;
    /* Control Requests */
    private final VoltageOut feedControl = new VoltageOut(EndEffectorConstants.feedVoltage).withEnableFOC(true);
    private final VoltageOut unjamControl = new VoltageOut(EndEffectorConstants.unjamVoltage).withEnableFOC(true);
    private final VoltageOut advanceControl = new VoltageOut(EndEffectorConstants.advanceVoltage).withEnableFOC(true);
    private final VoltageOut scoreL2L3Control = new VoltageOut(EndEffectorConstants.scoreL2L3Voltage).withEnableFOC(true);
    private final VoltageOut scoreL4Control = new VoltageOut(EndEffectorConstants.scoreL4Voltage).withEnableFOC(true);
    private final VoltageOut scoreL1Control = new VoltageOut(EndEffectorConstants.scoreL1Voltage).withEnableFOC(true);
    private final VoltageOut algaeControl = new VoltageOut(EndEffectorConstants.algaeVoltage).withEnableFOC(false);
    private final VoltageOut algaeOutControl = new VoltageOut(EndEffectorConstants.algaeOutVoltage).withEnableFOC(false);
    private final VoltageOut algaeHoldControl = new VoltageOut(EndEffectorConstants.algaeHoldVoltage).withEnableFOC(false);
    private final VoltageOut algaeBargeControl = new VoltageOut(EndEffectorConstants.algaeBargeVoltage).withEnableFOC(false);

    private GamePieceState lastState = GamePieceState.NONE;

    public EndEffectorSubsystem() {
        /* Devices */
        motor = new TalonFX(EndEffectorConstants.motorID, EndEffectorConstants.canBus);

        applyConfigs();
    }

    public void applyConfigs() {
        /* Configure the EndAffector Motor */
        var m_endAffectorConfiguration = new TalonFXConfiguration();
        /* Set EndAffector motor to Brake */
        m_endAffectorConfiguration.MotorOutput.NeutralMode = EndEffectorConstants.motorNeutralValue;
        /* Set the motor direction */
        m_endAffectorConfiguration.MotorOutput.Inverted = EndEffectorConstants.motorOutputInverted;
        /* Config the peak outputs */
        m_endAffectorConfiguration.Voltage.PeakForwardVoltage = EndEffectorConstants.peakForwardVoltage;
        m_endAffectorConfiguration.Voltage.PeakReverseVoltage = EndEffectorConstants.peakReverseVoltage;
        /* Apply Index Motor Configs */
        motor.getConfigurator().apply(m_endAffectorConfiguration);
    }

    @Override
    public void periodic() {
        Command currentCommand = getCurrentCommand();
        DogLog.log("EndEffector/Current Command", currentCommand == null ? "None" : currentCommand.getName());

        GamePieceState gamePieceState = RobotState.getGamePieceState();
        double motorTemp = motor.getDeviceTemp().getValueAsDouble();

        if (gamePieceState != lastState) {
            switch (gamePieceState) {
                case NONE:
                    DogLog.log("EndEffector/Control", "Stopping (no game piece)");
                    motor.stopMotor();
                    break;
                case INTAKING_ALGAE:
                    DogLog.log("EndEffector/Control", "Intaking algae");
                    motor.setControl(algaeControl);
                    break;
                case HOLDING_ALGAE:
                    DogLog.log("EndEffector/Control", "Holding algae");
                    motor.setControl(algaeHoldControl);
                    break;
                case SCORING_PROCESSOR_ALGAE:
                    DogLog.log("EndEffector/Control", "Scoring algae into processor");
                    motor.setControl(algaeOutControl);
                    break;
                case SCORING_BARGE_ALGAE:
                    DogLog.log("EndEffector/Control", "Scoring algae into barge");
                    motor.setControl(algaeBargeControl);
                    break;
                case INTAKING_CORAL:
                    DogLog.log("EndEffector/Control", "Stop (intaking coral)");
                    motor.stopMotor();
                    break;
                case UNJAMMING_CORAL:
                    DogLog.log("EndEffector/Control", "Unjamming");
                    motor.setControl(unjamControl);
                    break;
                case FEEDING_CORAL:
                    DogLog.log("EndEffector/Control", "Feeding coral");
                    motor.setControl(feedControl);
                    break;
                case ADVANCING_CORAL:
                    DogLog.log("EndEffector/Control", "Advancing coral");
                    motor.setControl(advanceControl);
                    break;
                case HOLDING_CORAL:
                    DogLog.log("EndEffector/Control", "Stopping (coral ready)");
                    motor.stopMotor();
                    break;
                case SCORING_CORAL:
                    Stop nextStop = RobotState.getNextStop();
                    if (nextStop == Stop.L1) {
                        DogLog.log("EndEffector/Control", "Scoring L1 coral");
                        motor.setControl(scoreL1Control);
                    } else if (nextStop == Stop.L2 || nextStop == Stop.L3) {
                        DogLog.log("EndEffector/Control", "Scoring L2/L4 coral");
                        motor.setControl(scoreL2L3Control);
                    } else {
                        DogLog.log("EndEffector/Control", "Scoring L4 coral");
                        motor.setControl(scoreL4Control);
                    }
                    break;
                default:
                    DogLog.log("EndEffector/Control", "Stopping (unhandled state)");
                    motor.stopMotor();
            }
            lastState = gamePieceState;
        }
        
        double current = motor.getStatorCurrent().getValueAsDouble();
        // double current = motor.getTorqueCurrent().getValueAsDouble();
        DogLog.log("EndEffector/TorqueCurrent", motor.getTorqueCurrent().getValueAsDouble());
        DogLog.log("EndEffector/StatorCurrent", motor.getStatorCurrent().getValueAsDouble());
        DogLog.log("EndEffector/Velocity", motor.getVelocity().getValueAsDouble());
        DogLog.log("EndEffector/RotorVelocity", motor.getRotorVelocity().getValueAsDouble());

        if (gamePieceState == GamePieceState.INTAKING_ALGAE && current > EndEffectorConstants.minimumAlgaeAcquireCurrent && !RobotState.getFinalSensor()) {
            RobotState.setHaveAlgae();
        }
        if (gamePieceState == GamePieceState.HOLDING_ALGAE && current < EndEffectorConstants.minimumAlgaeHoldCurrent && RobotState.getFinalSensor()) {
            RobotState.setNoAlgae();
        }

        DogLog.log("EndEffector/Motor Temp", motorTemp);

        // if (motorTemp > 70){ //TODO tune and put in constants
        //     LoggedAlert.Warning("EndEffector", "TEMP WARNING", "End Effector Motor is approaching hot temperatures");
        // }
        // if (motorTemp > 90){
        //     LoggedAlert.Warning("EndEffector", "TEMP WARNING", "End Effector Motor is approaching temperature max");
        // }
    }
}