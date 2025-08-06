// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.endeffector;

import com.ctre.phoenix6.hardware.TalonFX;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.LoggedCommands;
import frc.robot.subsystems.endeffector.EndEffectorConstants.EEPosition;

public class EndEffector extends SubsystemBase {
    public static final EndEffector instance = new EndEffector();
    
    private EEPosition desiredPosition = EEPosition.START;

    /* Devices */
    private final TalonFX positionMotor;
    private final TalonFX pieceMotor;

    /* Control Requests */
    // private final VoltageOut algaeControl = new VoltageOut(EndEffectorConstants.algaeVoltage).withEnableFOC(false);

    public EndEffector() {
        /* Devices */
        positionMotor = new TalonFX(EndEffectorConstants.positionMotorID, EndEffectorConstants.canBus);
        pieceMotor = new TalonFX(EndEffectorConstants.pieceMotorID, EndEffectorConstants.canBus);

        /* Configs */
        positionMotor.getConfigurator().apply(EndEffectorConstants.getPositionConfig());
        pieceMotor.getConfigurator().apply(EndEffectorConstants.getPieceConfig());
    }

    private boolean inPosition() {
        return false;
    }

    public Command ScoreL1Coral() {
        return LoggedCommands.print("Score L1 Coral", "TODO Implement Score L1 Coral");
    }

    public Command ScoreL23Coral() {
        return LoggedCommands.print("Score L2/L3 Coral", "TODO Implement Score L2/L3 Coral");
    }

    public Command ScoreL4Coral() {
        return LoggedCommands.print("Score L4 Coral", "TODO Implement Score L4 Coral");
    }

    @Override
    public void periodic() {
        Command currentCommand = getCurrentCommand();
        DogLog.log("EndEffector/Current Command", currentCommand == null ? "None" : currentCommand.getName());
        DogLog.log("EndEffector/Desired Position", desiredPosition.name());
        DogLog.log("EndEffector/Position Motor/TorqueCurrent", positionMotor.getTorqueCurrent().getValueAsDouble());
        DogLog.log("EndEffector/Position Motor/StatorCurrent", positionMotor.getStatorCurrent().getValueAsDouble());
        DogLog.log("EndEffector/Position Motor/Velocity", positionMotor.getVelocity().getValueAsDouble());
        DogLog.log("EndEffector/Position Motor/RotorVelocity", positionMotor.getRotorVelocity().getValueAsDouble());
        DogLog.log("EndEffector/Position Motor/Motor Temp", positionMotor.getDeviceTemp().getValueAsDouble());
        DogLog.log("EndEffector/Position Motor/Position", positionMotor.getPosition().getValueAsDouble());
        DogLog.log("EndEffector/Piece Motor/TorqueCurrent", pieceMotor.getTorqueCurrent().getValueAsDouble());
        DogLog.log("EndEffector/Piece Motor/StatorCurrent", pieceMotor.getStatorCurrent().getValueAsDouble());
        DogLog.log("EndEffector/Piece Motor/Velocity", pieceMotor.getVelocity().getValueAsDouble());
        DogLog.log("EndEffector/Piece Motor/RotorVelocity", pieceMotor.getRotorVelocity().getValueAsDouble());
        DogLog.log("EndEffector/Piece Motor/Motor Temp", pieceMotor.getDeviceTemp().getValueAsDouble());
    }
}