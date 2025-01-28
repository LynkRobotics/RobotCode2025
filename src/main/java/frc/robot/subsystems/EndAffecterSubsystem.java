// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.LoggedCommands;

public class EndAffecterSubsystem extends SubsystemBase {
  /* Devices */
  private final TalonFX motor;
  private final DigitalInput sensor;
  /* Control Requests */
  private final DutyCycleOut indexSpeedDutyCycleOut;

  public EndAffecterSubsystem() {
    /* Devices */
    motor = new TalonFX(Constants.EndAffector.motorID, Constants.EndAffector.canBus);
    sensor = new DigitalInput(Constants.EndAffector.sensorID);
    /* Control Requests */
    indexSpeedDutyCycleOut = new DutyCycleOut(0).withEnableFOC(true);

    applyConfigs();
  }

  public void applyConfigs() {
    /* Configure the EndAffector Motor */
    var m_endAffectorConfiguration = new TalonFXConfiguration();
    /* Set EndAffector motor to Brake */
    m_endAffectorConfiguration.MotorOutput.NeutralMode = Constants.EndAffector.motorNeutralValue;
    /* Set the motor direction */
    m_endAffectorConfiguration.MotorOutput.Inverted = Constants.EndAffector.motorOutputInverted;
    /* Config the peak outputs */
    m_endAffectorConfiguration.Voltage.PeakForwardVoltage = Constants.EndAffector.peakForwardVoltage;
    m_endAffectorConfiguration.Voltage.PeakReverseVoltage = Constants.EndAffector.peakReverseVoltage;
    /* Apply Index Motor Configs */
    motor.getConfigurator().apply(m_endAffectorConfiguration);
  }

  public Command Intake(){
    return LoggedCommands.runOnce("End Affector Intake", 
    () -> {
      motor.setControl(indexSpeedDutyCycleOut.withOutput(Constants.EndAffector.intakeSpeed)); 
    }).until(this::haveGamePiece);
  }

  public Command Stop(){
    return LoggedCommands.runOnce("End Affector Stop", 
    () -> {
      motor.setControl(indexSpeedDutyCycleOut.withOutput(Constants.EndAffector.stopSpeed));
    });
  }

  public Command Outtake(){
    return LoggedCommands.runOnce("End Affector Outtake", 
    () -> {
      motor.setControl(indexSpeedDutyCycleOut.withOutput(Constants.EndAffector.outtakeSpeed)); 
    });
  }

  public boolean haveGamePiece() {
    return sensor.get();
  }

  @Override
  public void periodic() {
    
  }
}
