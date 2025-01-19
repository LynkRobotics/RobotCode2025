// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.EnumMap;
import java.util.Map;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ElevatorSubsystem extends SubsystemBase {
  private final TalonFX leftMotor;
  private final TalonFX rightMotor;
  private final VoltageOut voltageOut = new VoltageOut(0).withEnableFOC(true);
  private final PositionVoltage positionVoltage = new PositionVoltage(1.5).withEnableFOC(true); //TODO 

  // Abstraction of the encoder positions for a defined elevator position
  private class ElevatorPosition {
    double leftMotorPosition;
    double rightMotorPosition;

    public ElevatorPosition(double left, double right) {
      leftMotorPosition = left;
      rightMotorPosition = right;
    }
  }

  public enum Position {
    ZERO,
    MAX,
    L1,
    L2,
    L3,
    L4
  };

  private final EnumMap<Position, ElevatorPosition> elevatorPositions = new EnumMap<>(Map.ofEntries(
    // All positions are left motor first, then second motor
    Map.entry(Position.ZERO, new ElevatorPosition(0, 0)),
    Map.entry(Position.MAX, new ElevatorPosition(125, 125)), //Theoretical example
    Map.entry(Position.L1, new ElevatorPosition(25, 25)),
    Map.entry(Position.L2, new ElevatorPosition(50, 50)),
    Map.entry(Position.L3, new ElevatorPosition(75, 75)),
    Map.entry(Position.L4, new ElevatorPosition(100, 100))
  ));

  public ElevatorSubsystem() {
    leftMotor = new TalonFX(0, "rio");
    rightMotor = new TalonFX(1, "rio");
    applyConfigs();
  }

  public void applyConfigs(){
    /* Configure the Elevator Motors */
    var m_ClimberMotorsConfiguration = new TalonFXConfiguration();
    /* Set Shooter motors to Brake */
    m_ClimberMotorsConfiguration.MotorOutput.NeutralMode = Constants.Elevator.motorNeutralValue;
    /* Set the Shooters motor direction */
    m_ClimberMotorsConfiguration.MotorOutput.Inverted = Constants.Elevator.motorOutputInverted;
    /* Config the peak outputs */
    m_ClimberMotorsConfiguration.Voltage.PeakForwardVoltage = Constants.Elevator.peakForwardVoltage;
    m_ClimberMotorsConfiguration.Voltage.PeakReverseVoltage = Constants.Elevator.peakReverseVoltage;

    // PID & FF configuration
    m_ClimberMotorsConfiguration.Slot0.kP = Constants.Elevator.kP;
    m_ClimberMotorsConfiguration.Slot0.kI = Constants.Elevator.kI;
    m_ClimberMotorsConfiguration.Slot0.kD = Constants.Elevator.kD;
    m_ClimberMotorsConfiguration.Slot0.kS = Constants.Elevator.kS;
    m_ClimberMotorsConfiguration.Slot0.kV = Constants.Elevator.kV;
    m_ClimberMotorsConfiguration.Slot0.kA = Constants.Elevator.kA;
    m_ClimberMotorsConfiguration.Slot0.kG = Constants.Elevator.kG;
  }

  public void applyVoltage(double voltage) {
    leftMotor.setControl(voltageOut.withOutput(voltage));
    rightMotor.setControl(voltageOut.withOutput(voltage));
  }

  public void setPosition(double position) {
    leftMotor.setControl(positionVoltage.withPosition(position));
    rightMotor.setControl(positionVoltage.withPosition(position));
  }

  public double getLeftPosition() {
    return leftMotor.getPosition().getValueAsDouble();
  }

  public double getRightPosition() {
    return rightMotor.getPosition().getValueAsDouble();
  }

  public void stop() {
    leftMotor.setControl(voltageOut.withOutput(0.0));
    rightMotor.setControl(voltageOut.withOutput(0.0));
  }

  public void zero() {
    leftMotor.setPosition(0.0);
    rightMotor.setPosition(0.0);
  }

  @Override
  public void periodic() {
    DogLog.log("Elevator/Left Motor Position", leftMotor.getPosition().getValueAsDouble());
    DogLog.log("Elevator/Right Motor Position", rightMotor.getPosition().getValueAsDouble());

  }
}
