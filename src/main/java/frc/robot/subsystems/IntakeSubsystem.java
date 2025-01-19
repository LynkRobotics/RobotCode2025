// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  private final TalonFX intakeMotor;
  private final DutyCycleOut intakeSpeedDutyCycleOut;
  static boolean intaking = false;

  public IntakeSubsystem() {
    intakeMotor = new TalonFX(Constants.Intake.intakeMotorID, Constants.Intake.intakeMotorCanBus);
    intakeSpeedDutyCycleOut = new DutyCycleOut(0);

    applyConfigs();
  }

  public void applyConfigs() {
    /* Configure the Intake Motor */
    var m_intakeConfiguration = new TalonFXConfiguration();
    /* Set Intake motor to Brake */
    m_intakeConfiguration.MotorOutput.NeutralMode = Constants.Intake.motorNeutralValue;
    /* Set the motor direction */
    m_intakeConfiguration.MotorOutput.Inverted = Constants.Intake.motorOutputInverted;
    /* Config the peak outputs */
    m_intakeConfiguration.Voltage.PeakForwardVoltage = Constants.Intake.peakForwardVoltage;
    m_intakeConfiguration.Voltage.PeakReverseVoltage = Constants.Intake.peakReverseVoltage;
    /* Apply Intake Motor Configs */
    intakeMotor.getConfigurator().apply(m_intakeConfiguration);
  }

  public void intake() {
    intakeMotor.setControl(intakeSpeedDutyCycleOut.withOutput(Constants.Intake.intakingSpeed));
  }

  public void eject() {
    intakeMotor.setControl(intakeSpeedDutyCycleOut.withOutput(Constants.Intake.ejectingSpeed));
  }

  public void stop() {
    intakeMotor.setControl(intakeSpeedDutyCycleOut.withOutput(Constants.Intake.stoppingSpeed));
  }


  @Override
  public void periodic() {
    double current = intakeMotor.getTorqueCurrent().getValueAsDouble();
    boolean active = (current > 20.0);

    if (active && !intaking) {
      DogLog.log("Intake/Status", "DEBUG: Intaking note detected");
    }
    SmartDashboard.putNumber("intake/torqueCurrent", current);

    DogLog.log("Intake/TorqueCurrent", current);
  }
}