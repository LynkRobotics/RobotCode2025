// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.endeffector;

import java.util.concurrent.ThreadPoolExecutor.DiscardOldestPolicy;

import com.ctre.phoenix6.hardware.TalonFX;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.LoggedAlert;
import frc.lib.util.LoggedCommands;
import frc.robot.Field;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Elevator.ClearState;
import frc.robot.subsystems.endeffector.EndEffectorConstants.EEControl;
import frc.robot.subsystems.endeffector.EndEffectorConstants.EEPosition;

public class EndEffector extends SubsystemBase {
    public static final EndEffector instance = new EndEffector();

    public enum EEState {
        EMPTY,
        HAVE_ALGAE,
        HAVE_CORAL
    }
    public EEState state = EEState.EMPTY;

    public enum EEIntakeState {
        STOPPED,
        INTAKING_ALGAE,
        SCORING_ALGAE,
        INTAKING_CORAL,
        SCORING_CORAL
    }
    public EEIntakeState intakeState = EEIntakeState.STOPPED;
    
    private EEPosition desiredPosition = EEPosition.START;
    private boolean atDesiredPosition = true;
    private boolean waitingToPivot = false;

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

    public boolean inPosition() {
        return atDesiredPosition;
    }

    public Command ExpelCoral(Field.ReefLevel level) {
        EEControl control = switch (level) {
            case L4 -> EEControl.CORAL_L4;
            case L3 -> EEControl.CORAL_L3;
            case L2 -> EEControl.CORAL_L2;
            case L1 -> EEControl.CORAL_L1;
        };
        double postClearDelay = switch (level) {
            case L2, L3 -> 0.18;
            default -> 0.05;  
        };

        return LoggedCommands.sequence("Expel coral for " + level,
            Commands.runOnce(() -> pieceMotor.setControl(control.control), this),
            // TODO Wait for beam break
            Commands.waitSeconds(postClearDelay),
            Commands.runOnce(pieceMotor::stopMotor, this));
    }

    public Command WaitForState(EEState desiredState) {
        return LoggedCommands.waitUntil("Wait for EE state " + desiredState, () -> state == desiredState );
    }

    public Command StartAlgaeIntake() {
        return LoggedCommands.runOnce("Start algae intake",
            () ->{
                if (state != EEState.EMPTY) {
                    LoggedAlert.Error("End Effector", "Bad state", "End Effector state was not empty: " + state);
                    state = EEState.EMPTY;
                }
                intakeState = EEIntakeState.INTAKING_ALGAE;
                pieceMotor.setControl(EEControl.ALGAE_INTAKE.control);
            }, this);
    }

    public Command StartCoralIntake() {
        return LoggedCommands.runOnce("Start coral intake",
            () ->{
                if (state != EEState.EMPTY) {
                    LoggedAlert.Error("End Effector", "Bad state", "End Effector state was not empty: " + state);
                    state = EEState.EMPTY;
                }
                intakeState = EEIntakeState.INTAKING_CORAL;
                pieceMotor.setControl(EEControl.CORAL_INTAKE.control);
            }, this);
    }

    public void move(EEPosition position) {
        if (desiredPosition == position) {
            // No change
            return;
        }
        
        desiredPosition = position;
        atDesiredPosition = false;
        if (Elevator.instance.isClear(ClearState.CLEAR_LOW)) {
            waitingToPivot = false;
            // TODO positionMotor.setControl();
        } else {
            waitingToPivot = true;
        }
    }

    @Override
    public void periodic() {
        Command currentCommand = getCurrentCommand();
        double position = positionMotor.getPosition().getValueAsDouble();
        double epsilon = 0.5; // TODO How close until we say we're at the desired position

        DogLog.log("EndEffector/Current Command", currentCommand == null ? "None" : currentCommand.getName());
        DogLog.log("EndEffector/Desired Position", desiredPosition.name());
        DogLog.log("EndEffector/In Position", inPosition());
        DogLog.log("EndEffector/Position Motor/TorqueCurrent", positionMotor.getTorqueCurrent().getValueAsDouble());
        DogLog.log("EndEffector/Position Motor/StatorCurrent", positionMotor.getStatorCurrent().getValueAsDouble());
        DogLog.log("EndEffector/Position Motor/Velocity", positionMotor.getVelocity().getValueAsDouble());
        DogLog.log("EndEffector/Position Motor/RotorVelocity", positionMotor.getRotorVelocity().getValueAsDouble());
        DogLog.log("EndEffector/Position Motor/Motor Temp", positionMotor.getDeviceTemp().getValueAsDouble());
        DogLog.log("EndEffector/Position Motor/Position", position);
        DogLog.log("EndEffector/Piece Motor/TorqueCurrent", pieceMotor.getTorqueCurrent().getValueAsDouble());
        DogLog.log("EndEffector/Piece Motor/StatorCurrent", pieceMotor.getStatorCurrent().getValueAsDouble());
        DogLog.log("EndEffector/Piece Motor/Velocity", pieceMotor.getVelocity().getValueAsDouble());
        DogLog.log("EndEffector/Piece Motor/RotorVelocity", pieceMotor.getRotorVelocity().getValueAsDouble());
        DogLog.log("EndEffector/Piece Motor/Motor Temp", pieceMotor.getDeviceTemp().getValueAsDouble());

        DogLog.log("EndEffector/State", state.name());
        DogLog.log("EndEffector/Intake State", intakeState.name());

        if (!atDesiredPosition) {
            if (Math.abs(desiredPosition.position - position) < epsilon) {
                atDesiredPosition = true;
            }
        }
        if (waitingToPivot) {
            // TODO Do we need to worry about algae roller deploy?
            if (atDesiredPosition) {
                // Unclear how we'd get here, but if we are in position, there's no need to wait
                waitingToPivot = false;
            } else if (Elevator.instance.isClear(Elevator.ClearState.CLEAR_LOW)) {
                // If the elevator is clear, we can pivot
                waitingToPivot = false;
                // TODO positionMotor.setControl();
            }
        }
        
        if (intakeState == EEIntakeState.INTAKING_ALGAE) {
            if (false) { // TODO Detect algae grab; remember to use debounce; wait 0.2s after detection (velocity below 2000 degrees per second or beam break)
                state = EEState.HAVE_ALGAE;
                pieceMotor.setControl(EEControl.ALGAE_HOLD.control);
                intakeState = EEIntakeState.STOPPED;
            }
        } else if (intakeState == EEIntakeState.INTAKING_CORAL) {
            if (false) { // TODO Detect coral grab; remember to use debounce
                state = EEState.HAVE_CORAL;
                pieceMotor.setControl(EEControl.CORAL_HOLD.control);
                intakeState = EEIntakeState.STOPPED;
            }
        }
    }
}