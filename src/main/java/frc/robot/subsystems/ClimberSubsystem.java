// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
  private final TalonFX motor;
  private final ClimberSelection which;
  private final VoltageOut voltageOut = new VoltageOut(0).withEnableFOC(true);
  private final PositionVoltage positionVoltage = new PositionVoltage(1.5).withEnableFOC(true);

  public enum ClimberSelection {
    LEFT,
    RIGHT
  }

  public ClimberSubsystem(ClimberSelection which) {
    this.which = which;
    motor = new TalonFX(which == ClimberSelection.LEFT ? Constants.Climber.leftID : Constants.Climber.rightID, Constants.Climber.CanBus);
    applyConfigs();
  }

  private void applyConfigs() {
    /* Configure the Shooter Motors */
    var m_ClimberMotorsConfiguration = new TalonFXConfiguration();
    /* Set Shooter motors to Brake */
    m_ClimberMotorsConfiguration.MotorOutput.NeutralMode = Constants.Climber.motorNeutralValue;
    /* Set the Shooters motor direction */
    m_ClimberMotorsConfiguration.MotorOutput.Inverted = Constants.Climber.motorOutputInverted;
    /* Config the peak outputs */
    m_ClimberMotorsConfiguration.Voltage.PeakForwardVoltage = Constants.Climber.peakForwardVoltage;
    m_ClimberMotorsConfiguration.Voltage.PeakReverseVoltage = Constants.Climber.peakReverseVoltage;

    // PID & FF configuration
    m_ClimberMotorsConfiguration.Slot0.kP = Constants.Climber.kP;
    m_ClimberMotorsConfiguration.Slot0.kI = Constants.Climber.kI;
    m_ClimberMotorsConfiguration.Slot0.kD = Constants.Climber.kD;
    m_ClimberMotorsConfiguration.Slot0.kS = Constants.Climber.kS;
    m_ClimberMotorsConfiguration.Slot0.kV = Constants.Climber.kV;
    m_ClimberMotorsConfiguration.Slot0.kA = Constants.Climber.kA;
    m_ClimberMotorsConfiguration.Slot0.kG = Constants.Climber.kG;

    // set Motion Magic settings
    var motionMagicConfigs = m_ClimberMotorsConfiguration.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = Constants.Climber.cruiseVelocity;
    motionMagicConfigs.MotionMagicAcceleration = Constants.Climber.acceleration;
    // motionMagicConfigs.MotionMagicJerk = Constants.Climber.jerk;

    /* Apply Shooters Motor Configs */
    motor.getConfigurator().apply(m_ClimberMotorsConfiguration);
  }

  public void applyVoltage(double voltage) {
    motor.setControl(voltageOut.withOutput(voltage));
  }

  public void setPosition(double position) {
    motor.setControl(positionVoltage.withPosition(position));
  }

  public double getPosition() {
    return motor.getPosition().getValueAsDouble();
  }

  public void stop() {
    motor.setControl(voltageOut.withOutput(0.0));
  }

  public void zero() {
    motor.setPosition(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("climber/" + which.toString() + " position", getPosition());
    SmartDashboard.putNumber("climber/" + which.toString() + " velocity", motor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("climber/" + which.toString() + " voltage", motor.getMotorVoltage().getValueAsDouble());
  }
}